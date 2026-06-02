# oscilloscope_gui.py
#
# Teensy 4.1 Dual-Channel Oscilloscope GUI
#
# Firmware binary format (4 bytes per sample pair):
#   [adc0_high][adc0_low][adc1_high][adc1_low]
#
# Pipeline:
#   binary bytes → uint16 ADC counts → float voltage (bias-corrected) → probe-scaled display

import collections
import csv
import sys
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QDoubleValidator
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


# ============================================================
# Constants
# ============================================================

BAUD_RATE               = 115200

# Sample rate window — GUI clamps its internal estimate to [MIN, MAX] so
# timestamps are always accurate regardless of momentary serial jitter.
MIN_SAMPLE_RATE_HZ      = 200_000       # 200 kHz — ER5 minimum requirement
TARGET_SAMPLE_RATE_HZ   = 300_000       # firmware delivers ~298 kHz at DELAY_US=3
MAX_SAMPLE_RATE_HZ      = 300_000       # 300 kHz upper bound

# Ring buffer depth in sample pairs.
# Sized for one full max display window at max rate:
#   MAX_DISPLAY_MS (1000) × MAX_SAMPLE_RATE_HZ (600 000) / 1000 = 600 000
# The deque evicts the oldest samples automatically — no unbounded growth.
RING_BUFFER_SAMPLES     = 300_000   # = MAX_DISPLAY_MS (1000) × MAX_SAMPLE_RATE_HZ (300 000) / 1000

SERIAL_UPDATE_MS        = 5             # how often to drain the serial buffer
ANALYSIS_UPDATE_MS      = 100           # peak-to-peak refresh interval
DISPLAY_REFRESH_RATE_HZ = 60

WINDOW_WIDTH            = 1600
WINDOW_HEIGHT           = 900

# Display window — minimum 100 ms guarantees ≥25 000 samples at 250 kHz.
DEFAULT_DISPLAY_MS      = 10
MIN_DISPLAY_MS          = 1
MAX_DISPLAY_MS          = 1_000

DEFAULT_VOLTAGE_SCALE   = 35.0          # updated: 10× probe × 3.5 V ADC swing
MIN_VOLTAGE_SCALE       = 0.1
MAX_VOLTAGE_SCALE       = 1_000.0

# Probe / voltage-divider scale. Set to your divider ratio so the Y-axis
# shows the real input voltage (e.g. 17.0 for a 17:1 divider, 10.0 for 10:1).
DEFAULT_PROBE_SCALE     = 10.0          # updated: 10:1 voltage divider
MIN_PROBE_SCALE         = 0.1
MAX_PROBE_SCALE         = 1_000.0

VREF                    = 3.3           # Teensy ADC reference voltage
ADC_MAX                 = 1023.0        # 10-bit ADC full scale

# Theoretical mid-scale bias used as a safe fallback before auto-zero completes.
ADC_BIAS_VOLTAGE        = VREF / 2.0   # 1.65 V

# Samples collected during auto-zero calibration (≈10 ms at 500 kHz).
CALIBRATION_SAMPLES     = 5_000

# If the measured rate shifts by more than this fraction the buffer is cleared
# and timestamps are reset to avoid a split timeline.
RATE_CHANGE_THRESHOLD   = 0.15         # 15 %


# ============================================================
# Main Window
# ============================================================

class OscilloscopeGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.serial_connection        = None
        self.teensy_test_mode_enabled = False
        self.auto_trigger_enabled     = False

        # Sample storage — three parallel deques always kept the same length.
        self.time_data        = collections.deque(maxlen=RING_BUFFER_SAMPLES)
        self.voltage_one_data = collections.deque(maxlen=RING_BUFFER_SAMPLES)
        self.voltage_two_data = collections.deque(maxlen=RING_BUFFER_SAMPLES)

        self.display_window_ms = DEFAULT_DISPLAY_MS
        self.voltage_scale     = DEFAULT_VOLTAGE_SCALE
        self.probe_scale_ch1   = DEFAULT_PROBE_SCALE
        self.probe_scale_ch2   = DEFAULT_PROBE_SCALE

        self._sample_count        = 0
        self._next_sample_time_ms = 0.0
        self._measured_rate_hz    = float(TARGET_SAMPLE_RATE_HZ)
        self._cmd_send_time       = None   # perf_counter() when last command was sent
        self._awaiting_cmd_response = False  # True until first data arrives post-command

        # Per-channel DC bias measured during auto-zero calibration.
        self._bias_ch1    = ADC_BIAS_VOLTAGE
        self._bias_ch2    = ADC_BIAS_VOLTAGE
        self._cal_acc_ch1 = 0.0
        self._cal_acc_ch2 = 0.0
        self._cal_count   = 0

        self.setWindowTitle("Teensy 4.1 Dual-Channel Oscilloscope GUI")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self._create_widgets()
        self._create_plot()
        self._build_layout()
        self._create_timers()
        self.refresh_ports()

    # --------------------------------------------------------
    # GUI construction
    # --------------------------------------------------------

    def _create_widgets(self):
        self.port_selector = QComboBox()

        self.refresh_button = QPushButton("Refresh Ports")
        self.refresh_button.clicked.connect(self.refresh_ports)

        self.connect_button = QPushButton("Connect to Device")
        self.connect_button.clicked.connect(self._toggle_connection)

        self.test_button = QPushButton("Start Teensy Test Signal")
        self.test_button.clicked.connect(self._toggle_teensy_test_mode)

        self.clear_button = QPushButton("Clear Plot")
        self.clear_button.clicked.connect(self.clear_plot)

        self.export_button = QPushButton("Export CSV")
        self.export_button.clicked.connect(self._export_csv)

        self.reset_display_button = QPushButton("Reset Display")
        self.reset_display_button.clicked.connect(self._reset_display)

        self.zero_cal_button = QPushButton("Zero Cal")
        self.zero_cal_button.setToolTip(
            "Re-measure the ADC idle level and re-zero both channels.\n"
            "Remove your signal first, then press."
        )
        self.zero_cal_button.clicked.connect(self._trigger_zero_cal)

        self.status_label      = QLabel("Status: Disconnected")
        self.sample_rate_label = QLabel("Received Rate: N/A")
        self.ptp_ch1_label       = QLabel("CH1 Peak-to-Peak: N/A")
        self.ptp_ch2_label       = QLabel("CH2 Peak-to-Peak: N/A")
        self.response_time_label = QLabel("CH1 Response Time: N/A")
        self.trigger_label     = QLabel("Trigger: Auto rising-edge")
        self.probe_hint_label  = QLabel("Set CH Probe spinboxes to match your voltage divider ratio")
        self.probe_hint_label.setStyleSheet("color: #ffcc44; font-size: 10px;")

        trigger_validator = QDoubleValidator()
        trigger_validator.setNotation(QDoubleValidator.Notation.StandardNotation)

        self.ch1_trigger_input = QLineEdit("0.0")
        self.ch1_trigger_input.setValidator(trigger_validator)
        self.ch1_trigger_input.setPlaceholderText("CH1 trigger voltage")
        self.ch1_trigger_input.editingFinished.connect(self._on_trigger_changed)

        self.ch2_trigger_input = QLineEdit("0.0")
        self.ch2_trigger_input.setValidator(trigger_validator)
        self.ch2_trigger_input.setPlaceholderText("CH2 trigger voltage")
        self.ch2_trigger_input.editingFinished.connect(self._on_trigger_changed)

        self.probe_ch1_input = QDoubleSpinBox()
        self.probe_ch1_input.setRange(MIN_PROBE_SCALE, MAX_PROBE_SCALE)
        self.probe_ch1_input.setDecimals(2)
        self.probe_ch1_input.setSingleStep(0.5)
        self.probe_ch1_input.setValue(DEFAULT_PROBE_SCALE)   # 10.0×
        self.probe_ch1_input.setSuffix(" ×")
        self.probe_ch1_input.setToolTip(
            "Voltage divider ratio for CH1.\n"
            "Example: 10:1 resistor divider → set to 10"
        )
        self.probe_ch1_input.valueChanged.connect(self._update_probe_scale_ch1)

        self.probe_ch2_input = QDoubleSpinBox()
        self.probe_ch2_input.setRange(MIN_PROBE_SCALE, MAX_PROBE_SCALE)
        self.probe_ch2_input.setDecimals(2)
        self.probe_ch2_input.setSingleStep(0.5)
        self.probe_ch2_input.setValue(DEFAULT_PROBE_SCALE)   # 10.0×
        self.probe_ch2_input.setSuffix(" ×")
        self.probe_ch2_input.setToolTip(
            "Voltage divider ratio for CH2.\n"
            "Example: 10:1 resistor divider → set to 10"
        )
        self.probe_ch2_input.valueChanged.connect(self._update_probe_scale_ch2)

        self.time_scale_input = QDoubleSpinBox()
        self.time_scale_input.setRange(MIN_DISPLAY_MS, MAX_DISPLAY_MS)
        self.time_scale_input.setDecimals(0)
        self.time_scale_input.setSingleStep(1.0)
        self.time_scale_input.setValue(DEFAULT_DISPLAY_MS)
        self.time_scale_input.setSuffix(" ms")
        self.time_scale_input.valueChanged.connect(self._update_time_scale)

        self.voltage_scale_input = QDoubleSpinBox()
        self.voltage_scale_input.setRange(MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE)
        self.voltage_scale_input.setDecimals(1)
        self.voltage_scale_input.setSingleStep(1.0)
        self.voltage_scale_input.setValue(DEFAULT_VOLTAGE_SCALE)   # 35.0 V
        self.voltage_scale_input.setSuffix(" V")
        self.voltage_scale_input.valueChanged.connect(self._update_voltage_scale)

        self.trigger_mode_selector = QComboBox()
        self.trigger_mode_selector.addItems(["Manual Trigger", "Auto Trigger"])
        self.trigger_mode_selector.currentIndexChanged.connect(self._on_trigger_changed)

    def _create_plot(self):
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground((15, 15, 25))
        self.plot_widget.setTitle("Dual-Channel Oscilloscope Waveform", color="w", size="14pt")
        self.plot_widget.setLabel("left",   "Voltage",           units="V",  color="#aaaaaa")
        self.plot_widget.setLabel("bottom", "Time After Trigger", units="ms", color="#aaaaaa")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self.plot_widget.setAntialiasing(True)
        self.plot_widget.setDefaultPadding(0.0)
        self.plot_widget.setXRange(0, self.display_window_ms)
        self.plot_widget.setYRange(-self.voltage_scale, self.voltage_scale)
        self.plot_widget.setLimits(yMin=-MAX_VOLTAGE_SCALE, yMax=MAX_VOLTAGE_SCALE)
        self.plot_widget.disableAutoRange()

        self.ch1_curve = self.plot_widget.plot(
            [], [], pen=pg.mkPen(color=(0, 220, 255), width=2), name="CH1", antialias=True,
        )
        self.ch2_curve = self.plot_widget.plot(
            [], [], pen=pg.mkPen(color=(255, 80, 80), width=2), name="CH2", antialias=True,
        )
        self.ch1_curve.setClipToView(True)
        self.ch1_curve.setDownsampling(auto=True, method="peak")
        self.ch2_curve.setClipToView(True)
        self.ch2_curve.setDownsampling(auto=True, method="peak")

        legend = self.plot_widget.addLegend(offset=(10, 10))
        legend.setLabelTextColor("w")

        dash = Qt.PenStyle.DashLine
        self.ch1_trigger_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color=(0, 220, 255), width=1, style=dash),
        )
        self.ch2_trigger_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color=(255, 80, 80), width=1, style=dash),
        )
        self.plot_widget.addItem(self.ch1_trigger_line)
        self.plot_widget.addItem(self.ch2_trigger_line)
        self.ch1_trigger_line.setValue(0.0)
        self.ch2_trigger_line.setValue(0.0)

    def _build_layout(self):
        side_layout = QVBoxLayout()
        side_layout.addWidget(self._build_connection_group())
        side_layout.addWidget(self._build_trigger_group())
        side_layout.addWidget(self._build_analysis_group())
        side_layout.addStretch()

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.plot_widget, stretch=4)
        main_layout.addLayout(side_layout,      stretch=1)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def _build_connection_group(self):
        group  = QGroupBox("Device Connection")
        layout = QGridLayout()

        layout.addWidget(QLabel("USB Serial Port:"),  0, 0)
        layout.addWidget(self.port_selector,          0, 1)
        layout.addWidget(self.refresh_button,         0, 2)

        layout.addWidget(self.connect_button,         1, 0)
        layout.addWidget(self.test_button,            1, 1)
        layout.addWidget(self.clear_button,           1, 2)

        layout.addWidget(self.export_button,          2, 0)
        layout.addWidget(self.reset_display_button,   2, 1)
        layout.addWidget(self.zero_cal_button,        2, 2)

        layout.addWidget(self.status_label,           3, 0, 1, 3)
        layout.addWidget(self.sample_rate_label,      4, 0, 1, 3)
        layout.addWidget(self.probe_hint_label,       5, 0, 1, 3)

        group.setLayout(layout)
        return group

    def _build_trigger_group(self):
        group  = QGroupBox("Channel Triggers & Probe Scale")
        layout = QGridLayout()

        layout.addWidget(QLabel("CH1 Level:"),     0, 0)
        layout.addWidget(self.ch1_trigger_input,   0, 1)
        layout.addWidget(QLabel("V"),              0, 2)

        layout.addWidget(QLabel("CH1 Probe:"),     1, 0)
        layout.addWidget(self.probe_ch1_input,     1, 1)

        layout.addWidget(QLabel("CH2 Level:"),     2, 0)
        layout.addWidget(self.ch2_trigger_input,   2, 1)
        layout.addWidget(QLabel("V"),              2, 2)

        layout.addWidget(QLabel("CH2 Probe:"),     3, 0)
        layout.addWidget(self.probe_ch2_input,     3, 1)

        layout.addWidget(QLabel("Time Scale:"),    4, 0)
        layout.addWidget(self.time_scale_input,    4, 1)

        layout.addWidget(QLabel("Voltage Scale:"), 5, 0)
        layout.addWidget(self.voltage_scale_input, 5, 1)

        layout.addWidget(QLabel("Trigger Mode:"),  6, 0)
        layout.addWidget(self.trigger_mode_selector, 6, 1)

        layout.addWidget(self.trigger_label,       7, 0, 1, 3)

        group.setLayout(layout)
        return group

    def _build_analysis_group(self):
        group  = QGroupBox("Measurements")
        layout = QGridLayout()
        layout.addWidget(self.ptp_ch1_label,       0, 0)
        layout.addWidget(self.ptp_ch2_label,       1, 0)
        layout.addWidget(self.response_time_label, 2, 0)
        group.setLayout(layout)
        return group

    def _create_timers(self):
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self._read_serial_data)
        self.serial_timer.start(SERIAL_UPDATE_MS)

        self.analysis_timer = QTimer()
        self.analysis_timer.timeout.connect(self._update_analysis)
        self.analysis_timer.start(ANALYSIS_UPDATE_MS)

        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self._update_plot)
        self.display_timer.start(int(1000 / DISPLAY_REFRESH_RATE_HZ))

        self.sample_rate_timer = QTimer()
        self.sample_rate_timer.timeout.connect(self._update_sample_rate)
        self.sample_rate_timer.start(1000)

    # --------------------------------------------------------
    # Sample-rate tracking
    # --------------------------------------------------------

    def _update_sample_rate(self):
        count = self._sample_count
        self._sample_count = 0

        if count == 0:
            self.sample_rate_label.setText("Received Rate: 0 sample pairs/sec")
            return

        clamped = max(MIN_SAMPLE_RATE_HZ, min(count, MAX_SAMPLE_RATE_HZ))

        prev  = self._measured_rate_hz
        ratio = clamped / max(prev, 1.0)
        if ratio < (1.0 - RATE_CHANGE_THRESHOLD) or ratio > (1.0 + RATE_CHANGE_THRESHOLD):
            self._measured_rate_hz = float(clamped)
            self._reset_timing()
            self.time_data.clear()
            self.voltage_one_data.clear()
            self.voltage_two_data.clear()
        else:
            smoothed = 0.8 * prev + 0.2 * clamped
            self._measured_rate_hz = max(float(MIN_SAMPLE_RATE_HZ),
                                         min(smoothed, float(MAX_SAMPLE_RATE_HZ)))

        self.sample_rate_label.setText(f"Received Rate: {count:,} sample pairs/sec")

    # --------------------------------------------------------
    # Serial connection
    # --------------------------------------------------------

    def refresh_ports(self):
        self.port_selector.clear()
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            self.port_selector.addItem(f"{port.device} - {port.description}", port.device)
        if self.port_selector.count() == 0:
            self.status_label.setText("Status: No USB serial ports found")
        else:
            self.status_label.setText("Status: USB serial port found")

    def _toggle_connection(self):
        if self._is_connected():
            self._disconnect_device()
        else:
            self._connect_device()

    def _connect_device(self):
        selected_port = self.port_selector.currentData()
        if not selected_port:
            text = self.port_selector.currentText()
            selected_port = text.split(" - ")[0] if text else None
        if not selected_port:
            self.status_label.setText("Status: No Teensy USB port selected")
            return

        try:
            if self.serial_connection:
                try:
                    self.serial_connection.close()
                except Exception:
                    pass
                self.serial_connection = None

            self.serial_connection = serial.Serial(
                port=selected_port, baudrate=BAUD_RATE, timeout=0.01,
            )
            self.serial_connection.reset_input_buffer()
            self._reset_timing()
            self._reset_calibration()
            self._measured_rate_hz = float(TARGET_SAMPLE_RATE_HZ)
            self._cmd_send_time = time.perf_counter()
            self._awaiting_cmd_response = True
            self.connect_button.setText("Disconnect Device")
            self.status_label.setText(f"Status: Connected to {selected_port} — zeroing…")

        except serial.SerialException as error:
            self.status_label.setText(f"Status: Connection failed: {error}")

    def _disconnect_device(self):
        self._stop_teensy_test_mode()
        if self.serial_connection:
            self.serial_connection.close()
        self.serial_connection = None
        self._reset_timing()
        self.connect_button.setText("Connect to Device")
        self.status_label.setText("Status: Disconnected")

    def _reset_timing(self):
        self._next_sample_time_ms = 0.0

    def _reset_calibration(self):
        self._bias_ch1    = ADC_BIAS_VOLTAGE
        self._bias_ch2    = ADC_BIAS_VOLTAGE
        self._cal_acc_ch1 = 0.0
        self._cal_acc_ch2 = 0.0
        self._cal_count   = 0

    def _is_connected(self):
        return self.serial_connection is not None and self.serial_connection.is_open

    def _send_command(self, command):
        if self._is_connected():
            self._cmd_send_time = time.perf_counter()
            self._awaiting_cmd_response = True
            self.serial_connection.write(f"{command}\n".encode("utf-8"))

    # --------------------------------------------------------
    # Teensy test signal
    # --------------------------------------------------------

    def _toggle_teensy_test_mode(self):
        if self.teensy_test_mode_enabled:
            self._stop_teensy_test_mode()
        else:
            self._start_teensy_test_mode()

    def _start_teensy_test_mode(self):
        if not self._is_connected():
            self.status_label.setText("Status: Connect to Teensy before starting test signal")
            return
        self.teensy_test_mode_enabled = True
        # Use the theoretical mid-scale bias for the test signal — the signal
        # is centred at 511 counts (1.65 V), not at the idle ADC level.
        self._bias_ch1 = ADC_BIAS_VOLTAGE
        self._bias_ch2 = ADC_BIAS_VOLTAGE
        self._cal_count = CALIBRATION_SAMPLES   # mark cal complete so it isn't re-run
        self._reset_timing()
        self.clear_plot()
        self._send_command("TEST_ON")
        self.test_button.setText("Stop Teensy Test Signal")
        self.status_label.setText("Status: Receiving Teensy test signal")

    def _stop_teensy_test_mode(self):
        if self.teensy_test_mode_enabled and self._is_connected():
            self._send_command("TEST_OFF")
        self.teensy_test_mode_enabled = False
        self.test_button.setText("Start Teensy Test Signal")

    # --------------------------------------------------------
    # Data ingestion
    # --------------------------------------------------------

    def _read_serial_data(self):
        if not self._is_connected():
            return
        try:
            available = self.serial_connection.in_waiting
            if available < 4:
                return
            raw = self.serial_connection.read(available - (available % 4))
            if raw:
                self._parse_binary_packets(raw)
        except serial.SerialException as error:
            self.status_label.setText(f"Status: Serial error: {error}")
            self._disconnect_device()

    def _parse_binary_packets(self, raw):
        if self._awaiting_cmd_response and self._cmd_send_time is not None:
            elapsed_ms = (time.perf_counter() - self._cmd_send_time) * 1000.0
            self.response_time_label.setText(f"Response Time: {elapsed_ms:.1f} ms")
            self._awaiting_cmd_response = False
        adc_values   = np.frombuffer(raw, dtype=">u2").reshape(-1, 2)
        packet_count = adc_values.shape[0]

        ch1_raw = adc_values[:, 0].astype(np.uint16, copy=False)
        ch2_raw = adc_values[:, 1].astype(np.uint16, copy=False)

        vsf = VREF / ADC_MAX   # voltage scale factor: counts → volts

        # Auto-zero calibration: accumulate idle ADC counts until we have
        # enough to compute a reliable per-channel DC bias.
        if self._cal_count < CALIBRATION_SAMPLES:
            needed = CALIBRATION_SAMPLES - self._cal_count
            chunk  = min(needed, packet_count)
            self._cal_acc_ch1 += float(ch1_raw[:chunk].sum())
            self._cal_acc_ch2 += float(ch2_raw[:chunk].sum())
            self._cal_count   += chunk
            if self._cal_count >= CALIBRATION_SAMPLES:
                self._bias_ch1 = (self._cal_acc_ch1 / CALIBRATION_SAMPLES) * vsf
                self._bias_ch2 = (self._cal_acc_ch2 / CALIBRATION_SAMPLES) * vsf
                self.status_label.setText("Status: Connected — zero-cal done")

        voltage_one = ch1_raw.astype(np.float32) * vsf - self._bias_ch1
        voltage_two = ch2_raw.astype(np.float32) * vsf - self._bias_ch2

        sample_period_ms  = 1000.0 / self._measured_rate_hz
        sample_times      = (self._next_sample_time_ms
                             + np.arange(packet_count, dtype=np.float64) * sample_period_ms)
        self._next_sample_time_ms = float(sample_times[-1] + sample_period_ms)

        self.time_data.extend(sample_times.tolist())
        self.voltage_one_data.extend(voltage_one.tolist())
        self.voltage_two_data.extend(voltage_two.tolist())
        self._sample_count += packet_count

    # --------------------------------------------------------
    # Probe scale
    # --------------------------------------------------------

    def _update_probe_scale_ch1(self):
        self._cmd_send_time = time.perf_counter()
        self._awaiting_cmd_response = True
        self.probe_scale_ch1 = float(self.probe_ch1_input.value())
        self._auto_voltage_scale()
        self._update_plot()

    def _update_probe_scale_ch2(self):
        self._cmd_send_time = time.perf_counter()
        self._awaiting_cmd_response = True
        self.probe_scale_ch2 = float(self.probe_ch2_input.value())
        self._auto_voltage_scale()
        self._update_plot()

    def _auto_voltage_scale(self):
        """Fit Y-axis to the largest possible swing given the current probe scales."""
        max_probe = max(self.probe_scale_ch1, self.probe_scale_ch2)
        new_scale = min(round(max_probe * (VREF / 2.0) * 1.05, 1), MAX_VOLTAGE_SCALE)
        self.voltage_scale = new_scale
        self.voltage_scale_input.blockSignals(True)
        self.voltage_scale_input.setValue(new_scale)
        self.voltage_scale_input.blockSignals(False)
        self.plot_widget.setYRange(-new_scale, new_scale)

    # --------------------------------------------------------
    # Plot rendering
    # --------------------------------------------------------

    def _update_plot(self):
        if len(self.time_data) < 3:
            self.ch1_curve.setData([], [])
            self.ch2_curve.setData([], [])
            return

        time_arr = np.asarray(self.time_data,        dtype=np.float64)
        v1_arr   = np.asarray(self.voltage_one_data, dtype=np.float32) * self.probe_scale_ch1
        v2_arr   = np.asarray(self.voltage_two_data, dtype=np.float32) * self.probe_scale_ch2

        if self.auto_trigger_enabled:
            ch1_level = self._auto_trigger_level(v1_arr, time_arr)
            ch2_level = self._auto_trigger_level(v2_arr, time_arr)
            self.trigger_label.setText(
                f"Trigger: Auto rising-edge  CH1 {ch1_level:.2f} V  CH2 {ch2_level:.2f} V"
            )
        else:
            ch1_level = self._parse_trigger(self.ch1_trigger_input)
            ch2_level = self._parse_trigger(self.ch2_trigger_input)
            self.trigger_label.setText("Trigger: Manual rising-edge")

        self.ch1_trigger_line.setValue(ch1_level)
        self.ch2_trigger_line.setValue(ch2_level)

        self._draw_channel(self.ch1_curve, v1_arr, time_arr,
                           self._find_trigger_index(ch1_level, v1_arr, time_arr))
        self._draw_channel(self.ch2_curve, v2_arr, time_arr,
                           self._find_trigger_index(ch2_level, v2_arr, time_arr))



    def _auto_trigger_level(self, voltage, time_arr):
        newest  = time_arr[-1]
        visible = voltage[time_arr >= (newest - self.display_window_ms)]
        if visible.size == 0:
            visible = voltage
        return float(np.median(visible))

    def _find_trigger_index(self, level, voltage, time_arr):
        if voltage.size < 2:
            return None
        edges = np.where((voltage[:-1] < level) & (voltage[1:] >= level))[0] + 1
        if edges.size == 0:
            return None
        valid = edges[(time_arr[-1] - time_arr[edges]) >= self.display_window_ms]
        if valid.size == 0:
            return None
        return int(valid[0])

    def _draw_channel(self, curve, voltage, time_arr, trigger_index):
        if time_arr.size == 0:
            curve.setData([], [])
            return
        if trigger_index is None:
            newest = time_arr[-1]
            mask   = time_arr >= (newest - self.display_window_ms)
            x = time_arr[mask] - newest + self.display_window_ms
            y = voltage[mask]
        else:
            t0   = time_arr[trigger_index]
            mask = (time_arr >= t0) & (time_arr <= t0 + self.display_window_ms)
            x    = time_arr[mask] - t0
            y    = voltage[mask]

        try:
            vb_width = int(self.plot_widget.plotItem.vb.width())
        except Exception:
            vb_width = 960
        max_pts = max(vb_width * 2, 400)
        if x.size > max_pts:
            step = int(np.ceil(x.size / max_pts))
            x, y = x[::step], y[::step]

        curve.setData(x, y)

    def _update_time_scale(self):
        self.display_window_ms = float(self.time_scale_input.value())
        step = 1.0 if self.display_window_ms < 10 else 10.0
        self.time_scale_input.blockSignals(True)
        self.time_scale_input.setSingleStep(step)
        self.time_scale_input.blockSignals(False)
        self.plot_widget.setXRange(0, self.display_window_ms)
        self._update_plot()

    def _update_voltage_scale(self):
        self.voltage_scale = float(self.voltage_scale_input.value())
        self.plot_widget.setYRange(-self.voltage_scale, self.voltage_scale)
        self._update_plot()

    def _on_trigger_changed(self):
        self._cmd_send_time = time.perf_counter()
        self._awaiting_cmd_response = True
        self._update_trigger_mode()

    def _update_trigger_mode(self):
        self.auto_trigger_enabled = (
            self.trigger_mode_selector.currentText() == "Auto Trigger"
        )
        self._update_plot()

    def _parse_trigger(self, line_edit):
        try:
            return float(line_edit.text())
        except ValueError:
            return 0.0

    # --------------------------------------------------------
    # Peak-to-peak analysis
    # --------------------------------------------------------

    def _update_analysis(self):
        if not self.time_data:
            return
        time_arr = np.asarray(self.time_data, dtype=np.float64)
        newest   = time_arr[-1]
        mask     = time_arr >= (newest - self.display_window_ms)

        v1 = np.asarray(self.voltage_one_data, dtype=np.float32) * self.probe_scale_ch1
        v2 = np.asarray(self.voltage_two_data, dtype=np.float32) * self.probe_scale_ch2

        vis1 = v1[mask] if mask.any() else v1
        vis2 = v2[mask] if mask.any() else v2

        self.ptp_ch1_label.setText(f"CH1 Peak-to-Peak: {float(np.ptp(vis1)):.4f} V")
        self.ptp_ch2_label.setText(f"CH2 Peak-to-Peak: {float(np.ptp(vis2)):.4f} V")

    # --------------------------------------------------------
    # User actions
    # --------------------------------------------------------

    def clear_plot(self):
        self._cmd_send_time = time.perf_counter()
        self._awaiting_cmd_response = True
        self.time_data.clear()
        self.voltage_one_data.clear()
        self.voltage_two_data.clear()
        self._reset_timing()
        self.ch1_curve.setData([], [])
        self.ch2_curve.setData([], [])
        self.ptp_ch1_label.setText("CH1 Peak-to-Peak: N/A")
        self.ptp_ch2_label.setText("CH2 Peak-to-Peak: N/A")
        self.trigger_label.setText(
            "Trigger: Auto rising-edge" if self.auto_trigger_enabled
            else "Trigger: Manual rising-edge"
        )

    def _reset_display(self):
        self.display_window_ms    = DEFAULT_DISPLAY_MS
        self.voltage_scale        = DEFAULT_VOLTAGE_SCALE
        self.auto_trigger_enabled = True
        self.probe_scale_ch1      = DEFAULT_PROBE_SCALE
        self.probe_scale_ch2      = DEFAULT_PROBE_SCALE

        self.ch1_trigger_input.setText("0.0")
        self.ch2_trigger_input.setText("0.0")

        for widget, value in [
            (self.time_scale_input,    DEFAULT_DISPLAY_MS),
            (self.voltage_scale_input, DEFAULT_VOLTAGE_SCALE),
            (self.probe_ch1_input,     DEFAULT_PROBE_SCALE),
            (self.probe_ch2_input,     DEFAULT_PROBE_SCALE),
        ]:
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)

        self.trigger_mode_selector.blockSignals(True)
        self.trigger_mode_selector.setCurrentText("Manual Trigger")
        self.trigger_mode_selector.blockSignals(False)

        self.plot_widget.setXRange(0, self.display_window_ms)
        self.plot_widget.setYRange(-self.voltage_scale, self.voltage_scale)
        self.ch1_trigger_line.setValue(0.0)
        self.ch2_trigger_line.setValue(0.0)
        self.trigger_label.setText("Display reset")
        self._update_plot()

    def _trigger_zero_cal(self):
        if not self._is_connected():
            self.status_label.setText("Status: Connect to device before zeroing")
            return
        self._reset_calibration()
        self.time_data.clear()
        self.voltage_one_data.clear()
        self.voltage_two_data.clear()
        self._reset_timing()
        self.status_label.setText(
            f"Status: Zero Cal in progress… collecting {CALIBRATION_SAMPLES:,} samples"
        )

    def _export_csv(self):
        if not self.voltage_one_data:
            QMessageBox.warning(self, "No Data", "There is no waveform data to export.")
            return

        file_name, _ = QFileDialog.getSaveFileName(
            self, "Save Waveform Data", "waveform_data.csv", "CSV Files (*.csv)",
        )
        if not file_name:
            return

        try:
            p1, p2 = self.probe_scale_ch1, self.probe_scale_ch2
            with Path(file_name).open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "time_ms",
                    f"ch1_voltage_V (probe {p1}x)",
                    f"ch2_voltage_V (probe {p2}x)",
                ])
                for t, v1, v2 in zip(self.time_data, self.voltage_one_data, self.voltage_two_data):
                    writer.writerow([t, v1 * p1, v2 * p2])
            QMessageBox.information(self, "Export Complete", "CSV file saved successfully.")
        except OSError as error:
            QMessageBox.critical(self, "Export Failed", str(error))

    def closeEvent(self, event):
        self._disconnect_device()
        event.accept()


# ============================================================
# Entry point
# ============================================================

def main():
    app = QApplication(sys.argv)
    window = OscilloscopeGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
