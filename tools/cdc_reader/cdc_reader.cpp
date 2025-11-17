#include "cdc_reader.h"

#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <atomic>
#include <cstdint>

#include <libserialport.h>


/* CONFIGURATION */

constexpr const char* SERIAL_PORT = "/dev/ttyACM1";
constexpr int BAUD_RATE = 230400;
constexpr int RESOLUTION = 16;
constexpr int FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION;
constexpr int PLOT_INDICES[] = {5};

constexpr size_t DATA_N         = 16;   // Num measurements per frame
constexpr size_t NUM_DEVICES    = 2;    // TODO: change to auto detect

// TODO: it should auto detect number of MCUs and also number of sensors per MCU (can be different per MCU)


/* STRUCTS */

// Sensor Struct
struct SensorFrame {
    uint8_t sensor_ID;
    std::array<uint16_t, DATA_N> data{};
    std::array<uint8_t, DATA_N> status{};
    uint64_t seq = 0;                       // sequence counter, maybe remove (TODO)
};

// Sensor Struct with double buffer
struct SensorBuffer {
    SensorFrame frames[2];      // two frame buffers: one active, one being written
    std::atomic<int> active{0}; // index of currently active frame (0 or 1)
};

// Shared Data Struct for all sensors in one object for *internal* handling.
// No atomics, double buffering etc.
struct SharedData {
    std::array<SensorBuffer, NUM_DEVICES> devs;
};

// Data struct for all sensors in one object for *external* handling
struct Snapshot {
    std::array<SensorFrame, NUM_DEVICES> devs;
};

// Device Struct for each serial device
struct SerialDevice {
    std::string name;
    sp_port* port;
    bool valid = false;
};


/* SERIAL HANDLING */

// Opens the given serial port and sets it up.
// Returns a ptr to the port if successfull, else nullptr.
sp_port* open_serial_port(const char* port_name, int baud) {
    sp_port* port = nullptr;
    if (sp_get_port_by_name(port_name, &port) != SP_OK) {
        std::cerr << "ERROR: Port not found: " << port_name << std::endl;
        return nullptr;
    }
    if (sp_open(port,SP_MODE_READ_WRITE) != SP_OK) {
        std::cerr << "ERROR: Could not open port\n";
        sp_free_port(port);
        return nullptr;
    }
    sp_set_baudrate(port, baud);
    sp_set_bits(port, 8);
    sp_set_parity(port, SP_PARITY_NONE);
    sp_set_stopbits(port, 1);
    sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);

    std::cout << "Opened port " << port_name << " succesfully\n";

    return port;
}

// Reads n bytes into buf from port.
// Blocks until read n bytes.
bool read_exact(sp_port* port, uint8_t* buf, size_t n) {
    size_t total = 0;
    while (total < n) {
        int r = sp_blocking_read(port, buf + total, n - total, 100);
        if (r <= 0) return false;
        total += r;
    }
    return true;
}
//TODO: change this function. make nonblocking with sp_input_waiting and non blocking read


// Searches for starting header in stream of port.
// Blocks until found and returns true or false if no bytes available.
bool sync_to_header(sp_port* port) {
    uint8_t b1, b2;
    while (true) {
        if (!read_exact(port, &b1, 1)) return false;
        if (b1 == 0xAA) {
            if (!read_exact(port, &b2, 1)) return false;
            if (b2 == 0x55) return true;
        }
    }
}

// Reads a full frame from port and saves data in given buffers.
// Blocks until synced.
bool read_frame(sp_port* port, SensorFrame* frame_ptr) {
    if (!sync_to_header(port)) return false;

    std::vector<uint8_t> packet(FRAME_SIZE - 2);
    if (!read_exact(port, packet.data(), packet.size())) return false;

    frame_ptr->sensor_ID = packet[0];

    for (int i = 0; i < DATA_N; ++i) {
        frame_ptr->data[i] = packet[1 + i * 2] | (packet[1 + i * 2 + 1] << 8);
    }

    std::memcpy(frame_ptr->status.data(), &packet[1 + DATA_N * 2], DATA_N);
    return true;
}


/* OTHER */

// Independant update of each sensor data
void update_sensor(SensorBuffer& buf,
                   const uint16_t* newData,
                   const uint8_t* newStatus) {

    // Determine write index
    int cur = buf.active.load(std::memory_order_relaxed);
    int nxt = 1 - cur;

    // Write to inactive frame
    SensorFrame& f = buf.frames[nxt];
    std::memcpy(f.data.data(), newData, DATA_N * 2); // each value is 2 bytes
    std::memcpy(f.status.data(), newStatus, DATA_N);
    f.seq++;  // Optional, to keep track of updates

    // Publish new data
    buf.active.store(nxt, std::memory_order_release);
}


// Retrieve newest data.
// Only contains data and status per device, no double buffering implementations.
Snapshot get_latest_data(const SharedData& shared) {
    Snapshot snap;

    for (int i = 0; i < NUM_DEVICES; ++i) {
        const SensorBuffer& buf = shared.devs[i];

        // Extract active data buffer
        int idx = buf.active.load(std::memory_order_acquire);
        snap.devs[i] = buf.frames[idx];
    }
    return snap;
}


// Replaces invalid measurement values at PLOT_INDICES with -1.
void filter_data(std::array<uint16_t, DATA_N>& data, const std::array<uint8_t, DATA_N>& status) {
    for (int i = 0; i < 4; ++i) {
        int idx = PLOT_INDICES[i];
        if (!(status[idx] == 5 || status[idx] == 9 || status[idx] == 10)) {
            data[idx] = static_cast<uint16_t>(-1);
        }
    }
}
// TODO: check and change this function

// Read a full frame (blocking), filters and prints it.
void read_process_frame(sp_port* port, SensorFrame *frame_ptr) {
    auto t_start = std::chrono::high_resolution_clock::now();

    if (!read_frame(port, frame_ptr)) {
        std::cerr << "Failed to read frame\n";
        sp_drain(port);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
    }

    filter_data(frame_ptr->data, frame_ptr->status);

    for (int idx : PLOT_INDICES)
        std::cout << frame_ptr->data[idx] << " ";

    auto t_end = std::chrono::high_resolution_clock::now();
    double delta_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::cout << "     (" << delta_ms << " ms)\n";
}

std::atomic<bool> running{true};
// Iterates through all devices and reads blocking, then updates data in shared data struct.
// TODO: add check for sp_waiting and read nonblocking
void run_all_devices(std::array<SerialDevice, NUM_DEVICES>& devices, SharedData& shared) {
    while (running) {
        for (int i = 0; i < NUM_DEVICES; i++) {
            SensorFrame frame;
            if (!read_frame(devices[i].port, &frame)) continue;
            update_sensor(shared.devs[i], frame.data.data(), frame.status.data());
        }
    }
}


int main()
{
    // object to store serial devices
    std::array<SerialDevice, NUM_DEVICES> serial_devices;

    // serial paths to initialize devices
    constexpr std::array<const char*, NUM_DEVICES> serial_paths = {
        "/dev/ttyACM1",
        "/dev/ttyACM2"
    };

    // open devices and store the handles
    for (int i = 0; i < NUM_DEVICES; i++) {
        const char* path = serial_paths[i];
        sp_port* port = open_serial_port(path, BAUD_RATE);
        if (port) {
            serial_devices[i].port = port;
            serial_devices[i].name = path;
            serial_devices[i].valid = true;
        } else {
            std::cerr << "Failed to open serial device " << path << std::endl;
            return 1;
        }
    }

    // object to store sensor data
    SharedData shared;

    std::thread t(run_all_devices, std::ref(serial_devices), std::ref(shared));

    for (int i = 0; i < 500; i++) {
        // read snapshot
        Snapshot snap = get_latest_data(shared);

        // print some measurements
        for (int i = 0; i < NUM_DEVICES; i++) {
            std::cout << i << ":\t";
            for (int idx : PLOT_INDICES) {
                std::cout << snap.devs[i].data[idx] << "  ";
            }
        }
        std::cout << std::endl;

        // run loop at ~50hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    running = false;
    t.join();

    return 0;
}



// Open and store serial device(s) as array of SerialDevice
// create a SharedData object
// In a separate thread: iterate through serial devices
//      Check if new data
//      Read and publish new data
// In main loop: display/process data
//      Read in newest data
//      print some data or feedback
