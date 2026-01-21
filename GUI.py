import sys
import os
import time
import serial
from serial.tools import list_ports
import pandas as pd
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLineEdit, QPushButton, QLabel
)
from PyQt6.QtCore import QTimer
import pyqtgraph as pg
from collections import deque

def find_pico_port():
    for port in list_ports.comports():
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()

        if (
            "raspberry" in desc
            or "pico" in desc
            or "rp2040" in desc
            or "2e8a" in hwid
        ):
            return port.device

    return None


BAUDRATE = 115200

SERIAL_PORT = find_pico_port()
if SERIAL_PORT is None:
    raise RuntimeError("Raspberry Pi Pico not found on any COM port")

MAX_SPECTRUM_POINTS = 288

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)

class PicoGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TSL")
        self.resize(1200, 700)

        layout = QHBoxLayout(self)

        ctrl_layout = QVBoxLayout()
        layout.addLayout(ctrl_layout, 1)

        self.speed_input = QLineEdit()
        self.start_input = QLineEdit()
        self.stop_input = QLineEdit()
        self.exposition_input = QLineEdit()
        ctrl_layout.addWidget(QLabel("Heating Speed (°C/min)"))
        ctrl_layout.addWidget(self.speed_input)
        ctrl_layout.addWidget(QLabel("Start Temp (°C)"))
        ctrl_layout.addWidget(self.start_input)
        ctrl_layout.addWidget(QLabel("Stop Temp (°C)"))
        ctrl_layout.addWidget(self.stop_input)
        ctrl_layout.addWidget(QLabel("Exposition (s)"))
        ctrl_layout.addWidget(self.exposition_input)

        self.temp_label = QLabel("Temp: --- °C")
        self.setpoint_label = QLabel("PID_duty: --- °C")
        ctrl_layout.addWidget(self.temp_label)
        ctrl_layout.addWidget(self.setpoint_label)

        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.send_start_command)
        ctrl_layout.addWidget(self.start_btn)

        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.send_reset_command)
        ctrl_layout.addWidget(self.reset_btn)

        self.export_btn = QPushButton("Export Data")
        self.export_btn.clicked.connect(self.export_data)
        ctrl_layout.addWidget(self.export_btn)
        ctrl_layout.addStretch()

        plot_layout = QVBoxLayout()
        layout.addLayout(plot_layout, 3)

        self.spectrum_plot = pg.PlotWidget(title="Spectrum (Intensity vs nm)")
        self.spectrum_curve = self.spectrum_plot.plot(pen='y')
        plot_layout.addWidget(self.spectrum_plot)

        self.temp_plot = pg.PlotWidget(title="Temperature vs Time (s)")
        self.temp_curve = self.temp_plot.plot(pen='r', name="Temperature")
        self.pid_curve = self.temp_plot.plot(pen='g', name="PID Setpoint")
        plot_layout.addWidget(self.temp_plot)

        self.temp_data = deque(maxlen=600)
        self.pid_data = deque(maxlen=600)
        self.time_data = deque(maxlen=600)
        self.spectrum_data = [0] * MAX_SPECTRUM_POINTS
        self.nm_axis = list(range(MAX_SPECTRUM_POINTS))
        self.starzt_time = time.time()
        self.spectrum_records = []

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(200)
        
        QTimer.singleShot(500, self.send_reset_command)


    def send_start_command(self):
        try:
            speed = float(self.speed_input.text())
            start = float(self.start_input.text())
            stop = float(self.stop_input.text())
            exposition = float(self.exposition_input.text())
        except ValueError:
            print("Invalid input values")
            return
        cmd = f"1 {speed} {start} {stop} {exposition}\n"
        ser.write(cmd.encode())
        print("Sent command:", cmd.strip())
        self.start_time = time.time()
        self.temp_data.clear()
        self.pid_data.clear()
        self.time_data.clear()
        self.spectrum_records.clear()

    def send_reset_command(self):
        cmd = "0\n"
        ser.write(cmd.encode())
        print("Sent reset command")
        self.start_time = time.time()
        self.temp_data.clear()
        self.pid_data.clear()
        self.time_data.clear()
        self.spectrum_records.clear()

    def update_data(self):
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            if line.startswith("Temp:"):
                try:
                    parts = line.split("|")
                    temp_part = parts[0].strip()
                    setpoint_part = parts[1].strip()
                    pid_part = parts[2].strip()
                    duty = parts[3].strip()

                    temp_val = float(temp_part.split(":")[1].replace("C", ""))
                    next_setpoint = float(duty.split(":")[1])
                    pid_val = float(pid_part.split(":")[1].replace("C", ""))

                    t = time.time() - self.start_time if self.start_time else 0
                    self.temp_data.append(temp_val)
                    self.pid_data.append(pid_val)
                    self.time_data.append(t)

                    self.temp_label.setText(f"Temp: {temp_val:.2f} °C")
                    self.setpoint_label.setText(f"PID_duty: {next_setpoint:.2f}")

                except Exception as e:
                    print("Temp/Setpoint parse error:", e, line)

            elif line.startswith("Spectrum at Temp:"):
                try:
                    temp_str = line.split("Spectrum at Temp:")[1].split("C")[0]
                    spectrum_temp = float(temp_str)
                    parts = line.split("Data:")[1]
                    spectrum_values = list(map(int, parts.strip("[]").split(",")))
                    if len(spectrum_values) == MAX_SPECTRUM_POINTS:
                        self.spectrum_data = spectrum_values
                        self.spectrum_records.append({
                            "Temp": spectrum_temp,
                            "Spectrum": spectrum_values.copy()
                        })
                except Exception as e:
                    print("Error parsing spectrum:", e)

        self.temp_curve.setData(list(self.time_data), list(self.temp_data))
        self.pid_curve.setData(list(self.time_data), list(self.pid_data))
        self.spectrum_curve.setData(self.nm_axis, self.spectrum_data)

    def export_data(self):
        if not self.spectrum_records:
            print("No data to export")
            return
        rows = []
        for rec in self.spectrum_records:
            row = [rec["Temp"]] + rec["Spectrum"]
            rows.append(row)
        col_names = ["Temperature"] + [f"Spectrum_{i}" for i in range(MAX_SPECTRUM_POINTS)]
        df = pd.DataFrame(rows, columns=col_names)
        filename = os.path.expanduser("~/Desktop/ZPD/spectra.xlsx")
        try:
            df.to_excel(filename, index=False)
            print(f"Data exported to {filename}")
        except Exception as e:
            print("Failed to export data:", e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PicoGUI()
    window.show()
    sys.exit(app.exec())
