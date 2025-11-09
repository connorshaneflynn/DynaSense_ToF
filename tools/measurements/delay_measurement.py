import sys
import time
import serial
import hid
import struct
import numpy as np
from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg
from collections import defaultdict, deque

COM = 'CDC'  # HID or CDC

SERIAL_PORT = '/dev/ttyACM1'  # COM4 for USB, COM6 for UART, or /dev/ttyACM* on ubuntu
BAUD_RATE = 230400
RESOLUTION = 16
FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION  # header + ID + distances + statuses

PLOT_INDICES = [5, 6, 9, 10]
MAX_DISTANCE = 1000
UPDATE_HZ = 120  # output framerate (Hz)

# start_time = 0

def read_frame_bytes(packet):
        """Reads the raw bytes from serial and packs them into the frame data.
        Returns sensor_ID, distances, and statuses."""
        # Read sensor ID
        sensor_ID = packet[0]

        # Read distances
        # raw = input.read(RESOLUTION * 2)
        raw = packet[1:RESOLUTION*2+1]
        if not raw:
            return None, None, None
        distances = np.array(struct.unpack("<" + "H" * RESOLUTION, raw))

        # Read statuses (1 byte each)
        # raw = input.read(RESOLUTION)
        raw = packet[RESOLUTION*2+1:]
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

def CDC_read_frame(ser):
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
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
            except serial.SerialException:
                pass
    packet = ser.read(FRAME_SIZE - 2)
    return read_frame_bytes(packet)


def HID_read_frame(dev_handle):
    # No syncing as complete packet per frame

    packet = dev_handle.read(FRAME_SIZE)
    packet = packet[2:]
    return read_frame_bytes(packet)



def filter_data(data, statuses):
    data_filt = data[PLOT_INDICES]
    statuses_filt = statuses[PLOT_INDICES]

    valid_mask = np.isin(statuses_filt, [5, 9, 10])
    data_filt[~valid_mask] = -1

    return data_filt, statuses_filt


def output_data(data):
     print(data, flush=True, end="    ")


if __name__ == "__main__":
    # initialize COM
    if COM == 'CDC':
        try:
            dev = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
        except serial.SerialException:
                print(f"\nERROR: Could not open port '{SERIAL_PORT}'. Check if device is connected and running.\n")
                raise SystemExit()
    elif COM == 'HID':
        try:
            dev = hid.Device(0x0483, 0x5750)
        except hid.HIDException:
            print(f"Could not open HID device.")
            sys.exit()
    else:
        print("Error: Unknown COM")
        sys.exit()
    
    while True:
        try:
            t_start = time.perf_counter_ns()
            if COM == 'CDC':
                _, data, statuses = CDC_read_frame(dev)
            else:
                _, data, statuses = HID_read_frame(dev)
            t_delta = round((time.perf_counter_ns() - t_start) / 1e6, 1)
            print(t_delta)

            data, statuses = filter_data(data, statuses)
            output_data(data)
            # delta_time = time.time_ns() - start_time
            # print(f"\t{delta_time/(1e6):03f}")
            

        except KeyboardInterrupt:
            dev.close()
            sys.exit()
