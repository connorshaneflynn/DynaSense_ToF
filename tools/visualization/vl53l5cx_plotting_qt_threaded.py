import sys
import time
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg
from collections import defaultdict, deque

SERIAL_PORT = 'COM4' #'/dev/ttyACM1'  # Windows: COM4 for USB, COM6 for UART    Ubuntu: '/dev/ttyACM1'
BAUD_RATE = 230400
RESOLUTION = 16
FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION  # header + ID + distances + statuses

PLOT_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] # 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
MAX_DISTANCE = 1000
UPDATE_HZ = 60  # visualization framerate (Hz)
STORE_ALL_DATA = True


class SensorWindow(QtWidgets.QMainWindow):
    def __init__(self, sensor_ID):
        super().__init__()
        self.sensor_ID = sensor_ID
        self.setWindowTitle(f"Sensor {sensor_ID}")

        # Layout
        cw = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(cw)
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
        self.data_buffer = {}
        self.max_points = 200
        for idx in PLOT_INDICES:
            curve = self.line_plot.plot(pen=pg.intColor(idx, hues=len(PLOT_INDICES)))
            self.curves[idx] = curve
            self.data_buffer[idx] = deque(maxlen=self.max_points)
        self.line_plot.setYRange(-100, MAX_DISTANCE + 30, padding=0)
        layout.addWidget(self.line_window)

        self.show()

    def update_view(self, distances, filter_mask = None):
        # Heatmap
        if filter_mask is None:
            # treat everything as valid if no mask
            filter_mask = np.ones_like(distances, dtype=bool)

        grid_size = int(np.sqrt(len(distances)))

        norm_dist = np.clip(distances, 0, MAX_DISTANCE)/MAX_DISTANCE
        cmap = plt.get_cmap('viridis')
        colors = cmap(norm_dist.reshape((grid_size, grid_size), order= 'C')) # Returns (G, G, 4) RGBA
        mask_2d = filter_mask.reshape((grid_size, grid_size))
        colors[~mask_2d] = [1, 0, 0, 1]  # [R, G, B, Alpha] -> Red
        # colors[0,0] = [0, 1, 0, 1]
        # colors[0,3] = [0, 0, 1, 1]
        self.img.setImage(colors, autoDownsample=True)
            
        # Without filtering:
        # distances = np.clip(distances, 0, MAX_DISTANCE)
        # arr = distances.reshape((grid_size, grid_size))
        # self.img.setImage(arr, autoLevels=False, levels=(0, MAX_DISTANCE), autoDownsample=True)

        # Line Plot
        for idx in PLOT_INDICES:
            self.data_buffer[idx].append(distances[idx])
            self.curves[idx].setData(self.data_buffer[idx])



class SerialReader(QtCore.QThread):
    frame_received = QtCore.Signal(int, np.ndarray, np.ndarray)

    def __init__(self, port, baud, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=None)
        self._running = True

    def run(self):
        while self._running:
            sensor_ID, distances, statuses = self.read_frame()
            if sensor_ID is not None:
                self.frame_received.emit(sensor_ID, distances, statuses)

    def read_frame_bytes(self):
        """Reads the raw bytes from serial and packs them into the frame data.
        Returns sensor_ID, distances, and statuses."""
        # Read sensor ID
        raw = self.ser.read(1)
        if not raw:
            return None, None, None
        sensor_ID = int.from_bytes(raw, byteorder='big')

        # Read distances
        raw = self.ser.read(RESOLUTION * 2)
        if not raw:
            return None, None, None
        distances = np.array(struct.unpack("<" + "H" * RESOLUTION, raw))

        # Read statuses (1 byte each)
        raw = self.ser.read(RESOLUTION)
        if not raw:
            return None, None, None
        statuses = np.array(struct.unpack("<" + "B" * RESOLUTION, raw))

        # # when bitpacking is used, each status is 1 bit
        # num_status_bytes = RESOLUTION // 8
        # status_bytes = self.ser.read(num_status_bytes)
        # statuses = []
        # for i in range(RESOLUTION):
        #     byte_idx = i // 8
        #     bit_idx = i % 8
        #     statuses.append((status_bytes[byte_idx] >> bit_idx) & 1)
        # statuses = np.array(statuses)

        return sensor_ID, distances, statuses

    def read_frame(self):
        # Sync to header, repeat periodically if failed connection
        while True:
            try:
                if self.ser.read(1) == b'\xAA' and self.ser.read(1) == b'\x55':
                    break
            except serial.SerialException as e:
                print(f"Failed to read from '{SERIAL_PORT}'")
                time.sleep(1)
                # try to reconnect
                try:
                    self.ser = serial.Serial(self.port, self.baud)
                except serial.SerialException:
                    pass
        return self.read_frame_bytes()
        

    def stop(self):
        self._running = False
        self.ser.close()
        self.quit();
        self.wait()


class SerialViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.windows = {}
        self.latest_frames = {}
        self.accumulated_frames = defaultdict(list)

        # Serial reader thread
        try:
            self.reader = SerialReader(SERIAL_PORT, BAUD_RATE)
        except serial.SerialException:
            print(f"\nERROR: Could not open port '{SERIAL_PORT}'. Check if device is connected and running.\n")
            raise SystemExit()

        self.reader.frame_received.connect(self.store_frame)
        self.reader.start()
        print(f"Started listening on port '{SERIAL_PORT}'")

        # GUI update timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(int(1000 / UPDATE_HZ))

    def store_frame(self, sensor_ID, distances, statuses):
        # Keep only the most recent frame per sensor for the GUI
        self.latest_frames[sensor_ID] = (distances, statuses)

        # Accumulate all frames for logging
        if STORE_ALL_DATA:
            timestamp = time.time()
            self.accumulated_frames[sensor_ID].append((timestamp, distances.copy(), statuses.copy()))

    def update_visualization(self):
        # Update windows with latest available data
        for sensor_ID, (distances, statuses) in list(self.latest_frames.items()):
            if sensor_ID not in self.windows:
                self.windows[sensor_ID] = SensorWindow(sensor_ID)
            distances, filter_mask = self.filter_data(distances, statuses)
            self.windows[sensor_ID].update_view(distances, filter_mask)

    def filter_data(self, distances, statuses):
        """ Sets invalid distances (according to status) to -1000.
        Returns the filtered array and a mask for the filtered locations. """

        filter_statuses = [5, 9, 10]
        mask = np.isin(statuses, filter_statuses)
        distances_filt = np.where(mask, distances, -1000)
        return distances_filt, mask
        

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
        self.reader.stop()

        # Save accumulated data
        if STORE_ALL_DATA:
            self.save_data()

        event.accept()
        # super().closeEvent(event)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    viewer = SerialViewer()
    app.aboutToQuit.connect(viewer.save_data)
    sys.exit(app.exec())
