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

constexpr int BAUD_RATE = 230400;
constexpr int RESOLUTION = 16;
constexpr int FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION;
constexpr std::array<int, 1> PLOT_INDICES = {5};

constexpr size_t DATA_N         = 16;   // Num measurements per frame
constexpr size_t NUM_DEVICES    = 1;    // TODO: change to auto detect

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

    /* All below is needed because atomic deletes copy ops. temporary: should not be needed without double buffering*/
    SensorBuffer() = default;

    // Move constructor
    SensorBuffer(SensorBuffer&& other) noexcept {
        frames[0] = other.frames[0];
        frames[1] = other.frames[1];
        active.store(other.active.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }

    // Move assignment
    SensorBuffer& operator=(SensorBuffer&& other) noexcept {
        if (this != &other) {
            frames[0] = other.frames[0];
            frames[1] = other.frames[1];
            active.store(other.active.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }
        return *this;
    }
};

// Shared Data Struct for all sensors in one object for *internal* handling.
struct SharedData {
    std::vector<SensorBuffer> devs;
};

// Data struct for all sensors in one object for *external* handling
// No atomics, double buffering etc.
struct Snapshot {
    std::vector<SensorFrame> sensor_frames; // TODO remove array specific code everywhere where used
};

// Device Struct for each serial device
struct SerialDevice {
    std::string name;
    sp_port* port;
    bool valid = false;
};


/* SERIAL HANDLING */

// Auto detects CDC devices per Product String (Default "STM32 ToF")
// Opens all detected devices and returns a vector of sp_port*
// Returns an empty vector if no devices were found
// Ports need to be closed before exiting program
std::vector<sp_port*> get_and_open_devices(std::string product_match = "STM32 ToF") {
    std::vector<sp_port*> found_devices;

    // Gather all ports
    sp_port **all_ports;
    if (sp_list_ports(&all_ports) != SP_OK) {
        std::cerr << "Could not find any USB ports\n";
        return found_devices;
    }

    // Find matching devices
    for (int i = 0; all_ports[i] != nullptr; i++) {
        const char* product = sp_get_port_usb_product(all_ports[i]);
        if (product && product == product_match) {
            // Save port
            sp_port* dev = nullptr;
            sp_copy_port(all_ports[i], &dev);
            // Open port
            if (sp_open(dev,SP_MODE_READ_WRITE) != SP_OK) {
                // free port
                std::cerr << "ERROR: Could not open port " << sp_get_port_name(dev) << std::endl;
                sp_free_port(dev);
                continue;
            }
            found_devices.push_back(dev);
        }
    }
    sp_free_port_list(all_ports);

    if (found_devices.empty()) {
        std::cout << "No USB ports found with product string: " << product_match << std::endl;
    } else {
        std::cout << "Found " << found_devices.size() << " devices\n";
    }
    return found_devices;
}

// Opens the given serial port and sets it up.
// Returns a ptr to the port if successfull, else nullptr.
sp_port* open_serial_port_by_name(const char* port_name, int baud) {
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
Snapshot get_latest_data(SharedData& shared) {
    Snapshot snap;
    snap.sensor_frames.resize(shared.devs.size());

    for (int i = 0; i < shared.devs.size(); i++) {
        const SensorBuffer& buf = shared.devs[i];

        // Extract active data buffer
        int idx = buf.active.load(std::memory_order_acquire);
        snap.sensor_frames[i] = buf.frames[idx];
    }
    return snap;
}


// Replaces invalid measurement values at PLOT_INDICES with -1.
void filter_data(std::array<uint16_t, DATA_N>& data, const std::array<uint8_t, DATA_N>& status) {
    for (int i = 0; i < PLOT_INDICES.size(); ++i) {
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
void run_all_devices(std::vector<SerialDevice>& devices, SharedData& shared) {
    while (running) {
        for (int i = 0; i < devices.size(); i++) {
            SensorFrame frame;
            if (!read_frame(devices[i].port, &frame)) continue;
            update_sensor(shared.devs[i], frame.data.data(), frame.status.data());
        }
    }
}


int main()
{
    // Find devices and store port handles
    std::vector<sp_port*> dev_ports = get_and_open_devices();
    if (dev_ports.empty()) return 1;

    // Store found devices
    std::vector<SerialDevice> serial_devices;
    for (int i = 0; i < dev_ports.size(); i++) {
        // SerialDevice field: name, port, valid
        SerialDevice dev {sp_get_port_name(dev_ports[i]), dev_ports[i], true};
        serial_devices.push_back(dev);  // (could also use emplace_back)
    }

    // object to store sensor data
    SharedData shared;
    shared.devs.resize(serial_devices.size());

    std::thread t(run_all_devices, std::ref(serial_devices), std::ref(shared));

    for (int i = 0; i < 500; i++) {
        // read snapshot
        Snapshot snap = get_latest_data(shared);

        // print some measurements
        for (int i = 0; i < NUM_DEVICES; i++) {
            std::cout << i << ":\t";
            for (int idx : PLOT_INDICES) {
                std::cout << snap.sensor_frames[i].data[idx] << "  ";
            }
        }
        std::cout << std::endl;

        // run loop at ~50hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    running = false;
    t.join();

    // close devices
    for (int i = 0; i < dev_ports.size(); i++) {
        sp_close(dev_ports[i]);
        sp_free_port(dev_ports[i]);
    }

    std::cout << "Closed all Ports\n";
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
