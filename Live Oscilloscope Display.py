# oscilloscope_gui.py
#
# Teensy 4.1 Dual-Channel Oscilloscope GUI
#
# Expected Teensy serial format:
# time_ms,adc1,voltage1,adc2,voltage2
#
# Commands sent to Teensy:
# TEST_ON
# TEST_OFF

import csv
import sys
from collections import deque
from pathlib import Path

import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QDoubleValidator
from PyQt6.QtWidgets import QApplication, QFileDialog
from PyQt6.QtWidgets import QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel
from PyQt6.QtWidgets import QLineEdit, QMainWindow, QMessageBox, QPushButton
from PyQt6.QtWidgets import QVBoxLayout, QWidget, QComboBox


# // ============================================================
# // Program Settings
# // ============================================================

BAUD_RATE = 115200
MAX_SAMPLES = 5000

SERIAL_UPDATE_MS = 20
ANALYSIS_UPDATE_MS = 100

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
DISPLAY_REFRESH_RATE_HZ = 60

DEFAULT_DISPLAY_WINDOW_MS = 1000
MIN_DISPLAY_WINDOW_MS = 10
MAX_DISPLAY_WINDOW_MS = 5000

MAX_SERIAL_LINES_PER_UPDATE = 100


# // ============================================================
# // Main GUI Class
# // ============================================================

class OscilloscopeGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.serial_connection = None
        self.test_mode_enabled = False

        self.time_data = deque(maxlen=MAX_SAMPLES)

        self.adc_one_data = deque(maxlen=MAX_SAMPLES)
        self.voltage_one_data = deque(maxlen=MAX_SAMPLES)

        self.adc_two_data = deque(maxlen=MAX_SAMPLES)
        self.voltage_two_data = deque(maxlen=MAX_SAMPLES)

        self.display_window_ms = DEFAULT_DISPLAY_WINDOW_MS

        self.setWindowTitle("Teensy 4.1 Dual-Channel Oscilloscope GUI")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.create_widgets()
        self.create_plot()
        self.build_layout()
        self.create_timers()
        self.refresh_ports()

    # // --------------------------------------------------------
    # // GUI Setup
    # // --------------------------------------------------------

    def create_widgets(self):
        self.port_selector = QComboBox()

        self.refresh_button = QPushButton("Refresh Ports")
        self.refresh_button.clicked.connect(self.refresh_ports)

        self.connect_button = QPushButton("Connect to Device")
        self.connect_button.clicked.connect(self.toggle_connection)

        self.test_button = QPushButton("Run Test Simulation")
        self.test_button.clicked.connect(self.toggle_test_mode)

        self.clear_button = QPushButton("Clear Plot")
        self.clear_button.clicked.connect(self.clear_plot)

        self.reset_display_button = QPushButton("Reset Display")
        self.reset_display_button.clicked.connect(self.reset_display)

        self.export_button = QPushButton("Export CSV")
        self.export_button.clicked.connect(self.export_csv)

        self.status_label = QLabel("Status: Disconnected")
        self.display_label = QLabel(
            f"Display| Resolution: {WINDOW_WIDTH} x {WINDOW_HEIGHT}, "
            f"Refresh rate: {DISPLAY_REFRESH_RATE_HZ} Hz |"
        )

        self.peak_to_peak_one_label = QLabel("CH1 Peak-to-Peak: N/A")
        self.peak_to_peak_two_label = QLabel("CH2 Peak-to-Peak: N/A")

        self.trigger_status_label = QLabel("Trigger: CH1 and CH2 manual rising-edge")

        trigger_validator = QDoubleValidator()
        trigger_validator.setNotation(QDoubleValidator.Notation.StandardNotation)

        self.channel_one_trigger_input = QLineEdit("0.0")
        self.channel_one_trigger_input.setValidator(trigger_validator)
        self.channel_one_trigger_input.setPlaceholderText("CH1 trigger voltage")
        self.channel_one_trigger_input.editingFinished.connect(self.update_plot)

        self.channel_two_trigger_input = QLineEdit("0.0")
        self.channel_two_trigger_input.setValidator(trigger_validator)
        self.channel_two_trigger_input.setPlaceholderText("CH2 trigger voltage")
        self.channel_two_trigger_input.editingFinished.connect(self.update_plot)

        self.time_scale_input = QDoubleSpinBox()
        self.time_scale_input.setRange(MIN_DISPLAY_WINDOW_MS, MAX_DISPLAY_WINDOW_MS)
        self.time_scale_input.setDecimals(0)
        self.time_scale_input.setSingleStep(100.0)
        self.time_scale_input.setValue(DEFAULT_DISPLAY_WINDOW_MS)
        self.time_scale_input.setSuffix(" ms")
        self.time_scale_input.valueChanged.connect(self.update_time_scale)

    def create_plot(self):
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground((20, 20, 20))
        self.plot_widget.setTitle("Dual-Channel Oscilloscope Waveform", color="w")
        self.plot_widget.setLabel("left", "Voltage", units="V", color="w")
        self.plot_widget.setLabel("bottom", "Time After Trigger", units="ms", color="w")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)

        self.plot_widget.setXRange(0, self.display_window_ms)
        self.plot_widget.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

        self.channel_one_curve = self.plot_widget.plot(
            [],
            [],
            pen=pg.mkPen(color=(0, 255, 255), width=2),
            name="CH1",
        )

        self.channel_two_curve = self.plot_widget.plot(
            [],
            [],
            pen=pg.mkPen(color=(255, 80, 80), width=2),
            name="CH2",
        )

        self.channel_one_trigger_line = pg.InfiniteLine(
            angle=0,
            movable=False,
            pen=pg.mkPen(color=(0, 255, 255), width=1, style=Qt.PenStyle.DashLine),
        )

        self.channel_two_trigger_line = pg.InfiniteLine(
            angle=0,
            movable=False,
            pen=pg.mkPen(color=(255, 80, 80), width=1, style=Qt.PenStyle.DashLine),
        )

        self.plot_widget.addItem(self.channel_one_trigger_line)
        self.plot_widget.addItem(self.channel_two_trigger_line)

        self.channel_one_trigger_line.setValue(self.get_channel_one_trigger_level())
        self.channel_two_trigger_line.setValue(self.get_channel_two_trigger_level())

    def build_layout(self):
        connection_group = self.build_connection_group()
        trigger_group = self.build_trigger_group()
        analysis_group = self.build_analysis_group()

        side_layout = QVBoxLayout()
        side_layout.addWidget(connection_group)
        side_layout.addWidget(trigger_group)
        side_layout.addWidget(analysis_group)
        side_layout.addStretch()

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.plot_widget, stretch=4)
        main_layout.addLayout(side_layout, stretch=1)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def build_connection_group(self):
        group = QGroupBox("Device Connection")
        layout = QGridLayout()

        layout.addWidget(QLabel("USB Serial Port:"), 0, 0)
        layout.addWidget(self.port_selector, 0, 1)
        layout.addWidget(self.refresh_button, 0, 2)

        layout.addWidget(self.connect_button, 1, 0)
        layout.addWidget(self.test_button, 1, 1)
        layout.addWidget(self.clear_button, 1, 2)

        layout.addWidget(self.export_button, 2, 0)
        layout.addWidget(self.reset_display_button, 2, 1)

        layout.addWidget(self.status_label, 3, 0, 1, 3)
        layout.addWidget(self.display_label, 4, 0, 1, 3)

        group.setLayout(layout)
        return group

    def build_trigger_group(self):
        group = QGroupBox("Channel Triggers")
        layout = QGridLayout()

        layout.addWidget(QLabel("CH1 Level:"), 0, 0)
        layout.addWidget(self.channel_one_trigger_input, 0, 1)
        layout.addWidget(QLabel("V"), 0, 2)

        layout.addWidget(QLabel("CH2 Level:"), 1, 0)
        layout.addWidget(self.channel_two_trigger_input, 1, 1)
        layout.addWidget(QLabel("V"), 1, 2)

        layout.addWidget(QLabel("Time Scale:"), 2, 0)
        layout.addWidget(self.time_scale_input, 2, 1)

        layout.addWidget(self.trigger_status_label, 3, 0, 1, 3)

        group.setLayout(layout)
        return group

    def build_analysis_group(self):
        group = QGroupBox("Peak-to-Peak Measurements")
        layout = QGridLayout()

        layout.addWidget(self.peak_to_peak_one_label, 0, 0)
        layout.addWidget(self.peak_to_peak_two_label, 1, 0)

        group.setLayout(layout)
        return group

    def create_timers(self):
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.read_serial_data)
        self.serial_timer.start(SERIAL_UPDATE_MS)

        self.analysis_timer = QTimer()
        self.analysis_timer.timeout.connect(self.update_analysis)
        self.analysis_timer.start(ANALYSIS_UPDATE_MS)

    # // --------------------------------------------------------
    # // Serial Connection
    # // --------------------------------------------------------

    def refresh_ports(self):
        self.port_selector.clear()

        ports = list(serial.tools.list_ports.comports())

        for port in ports:
            display_name = f"{port.device} - {port.description}"
            self.port_selector.addItem(display_name, port.device)

        if self.port_selector.count() == 0:
            self.status_label.setText("Status: No USB serial ports found")
        else:
            self.status_label.setText("Status: USB serial port found")

    def toggle_connection(self):
        if self.is_connected():
            self.disconnect_device()
        else:
            self.connect_device()

    def connect_device(self):
        selected_port = self.port_selector.currentData()

        if not selected_port:
            current_text = self.port_selector.currentText()
            if current_text:
                selected_port = current_text.split(" - ")[0]

        if not selected_port:
            self.status_label.setText("Status: No Teensy USB port selected")
            return

        try:
            self.serial_connection = serial.Serial(
                port=selected_port,
                baudrate=BAUD_RATE,
                timeout=0.01,
            )

            self.serial_connection.reset_input_buffer()
            self.connect_button.setText("Disconnect Device")
            self.status_label.setText(f"Status: Connected to {selected_port}")

        except serial.SerialException as error:
            self.status_label.setText(f"Status: Connection failed: {error}")

    def disconnect_device(self):
        self.stop_test_mode()

        if self.serial_connection:
            self.serial_connection.close()

        self.serial_connection = None
        self.connect_button.setText("Connect to Device")
        self.status_label.setText("Status: Disconnected")

    def is_connected(self):
        return self.serial_connection is not None and self.serial_connection.is_open

    def send_command(self, command):
        if self.is_connected():
            self.serial_connection.write(f"{command}\n".encode("utf-8"))

    # // --------------------------------------------------------
    # // Test Simulation Control
    # // --------------------------------------------------------

    def toggle_test_mode(self):
        if not self.is_connected():
            QMessageBox.warning(
                self,
                "Device Not Connected",
                "Connect to the Teensy 4.1 before running the test simulation.",
            )
            return

        if self.test_mode_enabled:
            self.stop_test_mode()
        else:
            self.start_test_mode()

    def start_test_mode(self):
        self.test_mode_enabled = True
        self.send_command("TEST_ON")
        self.test_button.setText("Stop Test Simulation")
        self.status_label.setText("Status: Test simulation running")

    def stop_test_mode(self):
        if self.is_connected():
            self.send_command("TEST_OFF")

        self.test_mode_enabled = False
        self.test_button.setText("Run Test Simulation")

    # // --------------------------------------------------------
    # // Data Reading and Parsing
    # // --------------------------------------------------------

    def read_serial_data(self):
        if not self.is_connected():
            return

        try:
            lines_read = 0

            while (
                self.serial_connection.in_waiting
                and lines_read < MAX_SERIAL_LINES_PER_UPDATE
            ):
                line = self.serial_connection.readline().decode("utf-8").strip()
                self.process_serial_line(line)
                lines_read += 1

        except serial.SerialException as error:
            self.status_label.setText(f"Status: Serial error: {error}")
            self.disconnect_device()

        except UnicodeDecodeError:
            return

    def process_serial_line(self, line):
        try:
            time_ms, adc_one, voltage_one, adc_two, voltage_two = self.parse_sample(line)
        except ValueError:
            return

        self.add_sample(time_ms, adc_one, voltage_one, adc_two, voltage_two)

    def parse_sample(self, line):
        parts = line.split(",")

        if len(parts) != 5:
            raise ValueError("Expected format: time_ms,adc1,voltage1,adc2,voltage2")

        time_ms = float(parts[0])
        adc_one = int(parts[1])
        voltage_one = float(parts[2])
        adc_two = int(parts[3])
        voltage_two = float(parts[4])

        return time_ms, adc_one, voltage_one, adc_two, voltage_two

    # // --------------------------------------------------------
    # // Plotting and Signal Analysis
    # // --------------------------------------------------------

    def add_sample(self, time_ms, adc_one, voltage_one, adc_two, voltage_two):
        self.time_data.append(time_ms)

        self.adc_one_data.append(adc_one)
        self.voltage_one_data.append(voltage_one)

        self.adc_two_data.append(adc_two)
        self.voltage_two_data.append(voltage_two)

        self.trim_old_samples()
        self.update_plot()

    def trim_old_samples(self):
        if not self.time_data:
            return

        newest_time = self.time_data[-1]
        keep_window_ms = self.display_window_ms * 3

        while self.time_data and (newest_time - self.time_data[0]) > keep_window_ms:
            self.time_data.popleft()

            self.adc_one_data.popleft()
            self.voltage_one_data.popleft()

            self.adc_two_data.popleft()
            self.voltage_two_data.popleft()

    def update_plot(self):
        if (
            not hasattr(self, "channel_one_curve")
            or not hasattr(self, "channel_two_curve")
        ):
            return

        if len(self.time_data) < 3:
            self.channel_one_curve.setData([], [])
            self.channel_two_curve.setData([], [])
            return

        channel_one_trigger_level = self.get_channel_one_trigger_level()
        channel_two_trigger_level = self.get_channel_two_trigger_level()

        self.channel_one_trigger_line.setValue(channel_one_trigger_level)
        self.channel_two_trigger_line.setValue(channel_two_trigger_level)

        channel_one_trigger_index = self.find_latest_trigger_index(
            channel_one_trigger_level,
            self.voltage_one_data,
        )
        channel_two_trigger_index = self.find_latest_trigger_index(
            channel_two_trigger_level,
            self.voltage_two_data,
        )

        self.plot_channel_window(
            self.channel_one_curve,
            self.voltage_one_data,
            channel_one_trigger_index,
        )
        self.plot_channel_window(
            self.channel_two_curve,
            self.voltage_two_data,
            channel_two_trigger_index,
        )

    def update_time_scale(self):
        self.display_window_ms = int(self.time_scale_input.value())

        if hasattr(self, "plot_widget"):
            self.plot_widget.setXRange(0, self.display_window_ms)
            self.trim_old_samples()
            self.update_plot()

    def get_channel_one_trigger_level(self):
        return self.get_trigger_level_from_input(self.channel_one_trigger_input)

    def get_channel_two_trigger_level(self):
        return self.get_trigger_level_from_input(self.channel_two_trigger_input)

    def get_trigger_level_from_input(self, trigger_input):
        try:
            return float(trigger_input.text())
        except ValueError:
            return 0.0

    def find_latest_trigger_index(self, trigger_level, voltage_data):
        time_values = list(self.time_data)
        voltage_values = list(voltage_data)

        for index in range(len(voltage_values) - 2, 0, -1):
            previous_voltage = voltage_values[index - 1]
            current_voltage = voltage_values[index]

            if previous_voltage < trigger_level <= current_voltage:
                trigger_time = time_values[index]
                newest_time = time_values[-1]

                if newest_time - trigger_time >= self.display_window_ms:
                    return index

        for index in range(1, len(voltage_values)):
            previous_voltage = voltage_values[index - 1]
            current_voltage = voltage_values[index]

            if previous_voltage < trigger_level <= current_voltage:
                return index

        return None

    def plot_channel_window(self, channel_curve, voltage_data, trigger_index):
        if trigger_index is None:
            self.plot_latest_channel_window(channel_curve, voltage_data)
            return

        time_values = list(self.time_data)
        voltage_values = list(voltage_data)

        trigger_time = time_values[trigger_index]

        x_values = []
        y_values = []

        for index in range(trigger_index, len(time_values)):
            relative_time = time_values[index] - trigger_time

            if relative_time > self.display_window_ms:
                break

            x_values.append(relative_time)
            y_values.append(voltage_values[index])

        channel_curve.setData(x_values, y_values)

    def plot_latest_channel_window(self, channel_curve, voltage_data):
        newest_time = self.time_data[-1]

        x_values = []
        y_values = []

        for time_ms, voltage in zip(self.time_data, voltage_data):
            relative_time = time_ms - newest_time + self.display_window_ms

            if 0 <= relative_time <= self.display_window_ms:
                x_values.append(relative_time)
                y_values.append(voltage)

        channel_curve.setData(x_values, y_values)

    def update_analysis(self):
        if not self.voltage_one_data or not self.voltage_two_data:
            return

        min_voltage_one = min(self.voltage_one_data)
        max_voltage_one = max(self.voltage_one_data)

        min_voltage_two = min(self.voltage_two_data)
        max_voltage_two = max(self.voltage_two_data)

        peak_to_peak_one = max_voltage_one - min_voltage_one
        peak_to_peak_two = max_voltage_two - min_voltage_two

        self.peak_to_peak_one_label.setText(
            f"CH1 Peak-to-Peak: {peak_to_peak_one:.4f} V"
        )
        self.peak_to_peak_two_label.setText(
            f"CH2 Peak-to-Peak: {peak_to_peak_two:.4f} V"
        )

        self.trigger_status_label.setText("Trigger: CH1 and CH2 manual rising-edge")

    # // --------------------------------------------------------
    # // User Actions
    # // --------------------------------------------------------

    def clear_plot(self):
        self.time_data.clear()

        self.adc_one_data.clear()
        self.voltage_one_data.clear()

        self.adc_two_data.clear()
        self.voltage_two_data.clear()

        self.channel_one_curve.setData([], [])
        self.channel_two_curve.setData([], [])

        self.peak_to_peak_one_label.setText("CH1 Peak-to-Peak: N/A")
        self.peak_to_peak_two_label.setText("CH2 Peak-to-Peak: N/A")

        self.trigger_status_label.setText("Trigger: CH1 and CH2 manual rising-edge")

    def reset_display(self):
        self.display_window_ms = DEFAULT_DISPLAY_WINDOW_MS

        self.channel_one_trigger_input.setText("0.0")
        self.channel_two_trigger_input.setText("0.0")

        self.time_scale_input.blockSignals(True)
        self.time_scale_input.setValue(DEFAULT_DISPLAY_WINDOW_MS)
        self.time_scale_input.blockSignals(False)

        self.plot_widget.setXRange(0, self.display_window_ms)
        self.plot_widget.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

        self.channel_one_trigger_line.setValue(0.0)
        self.channel_two_trigger_line.setValue(0.0)

        self.update_plot()
        self.trigger_status_label.setText("Display reset")

    def export_csv(self):
        if not self.voltage_one_data:
            QMessageBox.warning(self, "No Data", "There is no waveform data to export.")
            return

        file_name, _ = QFileDialog.getSaveFileName(
            self,
            "Save Waveform Data",
            "waveform_data.csv",
            "CSV Files (*.csv)",
        )

        if not file_name:
            return

        try:
            with Path(file_name).open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    ["time_ms", "adc1", "voltage1", "adc2", "voltage2"]
                )

                for time_ms, adc_one, voltage_one, adc_two, voltage_two in zip(
                    self.time_data,
                    self.adc_one_data,
                    self.voltage_one_data,
                    self.adc_two_data,
                    self.voltage_two_data,
                ):
                    writer.writerow(
                        [time_ms, adc_one, voltage_one, adc_two, voltage_two]
                    )

            QMessageBox.information(
                self,
                "Export Complete",
                "CSV file saved successfully.",
            )

        except OSError as error:
            QMessageBox.critical(self, "Export Failed", str(error))

    def closeEvent(self, event):
        self.disconnect_device()
        event.accept()


# // ============================================================
# // Program Startup
# // ============================================================

def main():
    app = QApplication(sys.argv)
    window = OscilloscopeGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()