import sys
import time
import serial
import struct

import numpy as np
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg

from collections import defaultdict, deque

SERIAL_PORT = 'COM4'  # COM6 for UART, COM4 for USB
BAUD_RATE = 230400
RESOLUTION = 16

FRAME_SIZE = 2 + 1 + RESOLUTION*2 + RESOLUTION//8

PLOT_INDICES = [0, 5, 15]
UPDATE_HZ = 33  # visualization framerate (Hz)
STORE_ALL_DATA = True

class RunningAverage:
    def __init__(self, maxlen=30):
        self.values = deque(maxlen=maxlen)
        self.total = 0.0

    def add(self, x):
        if len(self.values) == self.values.maxlen:
            self.total -= self.values[0]
        self.values.append(x)
        self.total += x

    def average(self):
        if not self.values:
            return 0.0
        return self.total / len(self.values)


class SensorWindow(QtWidgets.QMainWindow):
    def __init__(self, sensor_ID):
        super().__init__()
        self.sensor_ID = sensor_ID
        self.setWindowTitle(f"Sensor {sensor_ID}")

        # Layout
        cw = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(cw)  # Vertical Split
        self.setCentralWidget(cw)

        # Heatmap
        self.view = pg.GraphicsLayoutWidget()
        self.img = pg.ImageItem()
        p = self.view.addPlot()
        p.addItem(self.img)
        p.setAspectLocked(True)
        lut = pg.colormap.get('viridis').getLookupTable(0.0, 1.0, 256)
        self.img.setLookupTable(lut)
        layout.addWidget(self.view)

        # Line Plot
        self.line_window = pg.GraphicsLayoutWidget()
        self.line_plot = self.line_window.addPlot(title=f"Sensor {sensor_ID} Distances")
        self.line_plot.showGrid(x=True, y=True)
        self.curves = {}
        self.max_points = 200
        self.data_buffer = {}
        for idx in PLOT_INDICES:
            curve = self.line_plot.plot(pen=pg.intColor(idx, hues=len(PLOT_INDICES)))
            self.curves[idx] = curve
            self.data_buffer[idx] = deque(maxlen=self.max_points)
        layout.addWidget(self.line_window)

        self.show()

    def update(self, distances):
        # Heatmap
        grid_size = int(np.sqrt(len(distances)))
        arr = distances.reshape((grid_size, grid_size))
        self.img.setImage(arr, autoLevels=False, levels=(0, 500), autoDownsample=True)

        # Line Plot
        for idx in PLOT_INDICES:
            self.data_buffer[idx].append(distances[idx])
            # if len(self.data_buffer[idx]) > self.max_points:
            #     self.data_buffer[idx].pop(0)
            self.curves[idx].setData(self.data_buffer[idx])


class SerialViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # Serial
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None)
        self.accumulated_frames = defaultdict(list)

        # Graphics
        self.windows = {}
        
        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_viewer)
        self.timer.start(UPDATE_HZ)

    def read_frame(self):
        """ Reads raw data from serial and decodes it into arrays of distances and statuses. Blocks until new frame is received.
        Maybe imlement max wait time?"""
        t0 = time.time()
        
        # sync
        n_skipped = 0
        while True:
            #print(self.ser.in_waiting)
            # if self.ser.in_waiting < (2 + 2*RESOLUTION + RESOLUTION // 8):
            #     print("buffer not full")
            #     time.sleep(0.001)
            #     continue
            if self.ser.read(1) == b'\xAA' and self.ser.read(1) == b'\x55':
                break
            n_skipped += 1
                
        if n_skipped:
            print(f"Skipped {n_skipped} bytes")
            pass
        
        dt = time.time() - t0
        time_ra.add(dt)
        print(time_ra.average())

        # read sensor ID
        raw = self.ser.read(1)
        sensor_ID = int.from_bytes(raw, byteorder='big')

        # read distances
        raw = self.ser.read(RESOLUTION * 2)
        distances = list(struct.unpack("<" + "H"*RESOLUTION, raw))


        # read statuses
        num_status_bytes = RESOLUTION // 8
        status_bytes = self.ser.read(num_status_bytes)
        statuses = []
        for i in range(RESOLUTION):
            byte_idx = i // 8
            bit_idx = i % 8
            statuses.append((status_bytes[byte_idx] >> bit_idx) & 1)  # extract one bit at a time

        return sensor_ID, np.array(distances), np.array(statuses)
        

    def update_viewer(self):
        # Keep only the most recent frame per sensor for the GUI
        latest_frames = {}
        
        # Drain buffer fully, keep only last frame of each sensor
        while self.ser.in_waiting >= FRAME_SIZE:
            sensor_ID, distances, statuses = self.read_frame()
            latest_frames[sensor_ID] = distances

        # Update those that have new data
        for sensor_ID, distances in latest_frames.items():
            if sensor_ID not in self.windows:
                self.windows[sensor_ID] = SensorWindow(sensor_ID)
            self.windows[sensor_ID].update(distances)

        # Accumulate all frames for logging
        if STORE_ALL_DATA:
            timestamp = time.time()
            self.accumulated_frames[sensor_ID].append((timestamp, distances.copy(), statuses.copy()))


    def save_data(self):
        import os, csv
        save_dir = "sensor_logs"
        os.makedirs(save_dir, exist_ok=True)
        for sensor_ID, frames in self.accumulated_frames.items():
            filename = os.path.join(save_dir, f"sensor_{sensor_ID}.csv")
            with open(filename, "w", newline="\n") as f:
                writer = csv.writer(f)
                header = ["timestamp"] + [f"dist_{i}" for i in range(RESOLUTION)] + [f"status_{i}" for i in range(RESOLUTION)]
                writer.writerow(header)
                for (ts, dists, stats) in frames:
                    row = [ts] + dists.tolist() + stats.tolist()
                    writer.writerow(row)
            print(f"Saved {len(frames)} frames for sensor {sensor_ID} -> {filename}")


    def closeEvent(self, event):
        self.ser.close()

        # Save accumulated data
        if STORE_ALL_DATA:
            self.save_data()

        super().closeEvent(event)
        print(f"\n\nUsed closeEvent() function\n\n")

    def close(self):
        self.ser.close()
        print(f"\n\nUsed close() function\n\n")


if __name__ == '__main__':
    time_ra = RunningAverage(maxlen=100)

    app = QtWidgets.QApplication(sys.argv)
    viewer = SerialViewer()
    app.aboutToQuit.connect(viewer.save_data)
    app.aboutToQuit.connect(viewer.close)
    sys.exit(app.exec())