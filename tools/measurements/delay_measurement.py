import sys
import time
import serial
import struct
import numpy as np
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg
from collections import defaultdict, deque

SERIAL_PORT = 'COM4'  # COM4 for USB, COM6 for UART
BAUD_RATE = 230400
RESOLUTION = 16
FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION  # header + ID + distances + statuses

PLOT_INDICES = [5, 6, 9, 10]
MAX_DISTANCE = 1000
UPDATE_HZ = 120  # output framerate (Hz)

# start_time = 0


def read_frame(ser):
    # Sync to header, repeat periodically if failed connection
    while True:
        try:
            if ser.read(1) == b'\xAA' and ser.read(1) == b'\x55':
                break
        except serial.SerialException as e:
            print(f"Failed to read from '{SERIAL_PORT}'")
            time.sleep(1)
            # try to reconnect
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
            except serial.SerialException:
                pass
    # global start_time
    # start_time = time.time_ns()

    # Read sensor ID
    raw = ser.read(1)
    if not raw:
        return None, None, None
    sensor_ID = int.from_bytes(raw, byteorder='big')

    # Read distances
    raw = ser.read(RESOLUTION * 2)
    if not raw:
        return None, None, None
    distances = np.array(struct.unpack("<" + "H" * RESOLUTION, raw))

    # Read statuses (1 byte each)
    raw = ser.read(RESOLUTION)
    if not raw:
        return None, None, None
    statuses = np.array(struct.unpack("<" + "B" * RESOLUTION, raw))

    return sensor_ID, distances, statuses


def filter_data(data, statuses):
    data_filt = data[PLOT_INDICES]
    statuses_filt = statuses[PLOT_INDICES]

    valid_mask = np.isin(statuses_filt, [5, 9, 10])
    data_filt[~valid_mask] = -1

    return data_filt, statuses_filt


def output_data(data):
     print(data, flush=True)


if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
    except serial.SerialException:
            print(f"\nERROR: Could not open port '{SERIAL_PORT}'. Check if device is connected and running.\n")
            raise SystemExit()
    
    while True:
        try:
            _, data, statuses = read_frame(ser)

            data, statuses = filter_data(data, statuses)
            output_data(data)
            # delta_time = time.time_ns() - start_time
            # print(f"\t{delta_time/(1e6):03f}")

        except KeyboardInterrupt:
            ser.close()
            sys.exit()
