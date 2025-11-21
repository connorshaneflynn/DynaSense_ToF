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

constexpr std::array<int, 1> PLOT_INDICES = {5};

static constexpr size_t DATA_N         = 16;   // Num measurements per frame
static constexpr std::array<uint8_t, 2> header = {0xAA, 0x55};
static constexpr int FRAME_SIZE = header.size() + 1 + DATA_N * 2 + DATA_N;

// TODO: it should auto detect number of MCUs and also number of sensors per MCU (can be different per MCU)

/* STRUCTS */

// Sensor Struct
struct SensorFrame {
    uint8_t sensor_ID;
    std::array<uint16_t, DATA_N> data{};
    std::array<uint8_t, DATA_N> status{};
    uint64_t seq = 0;                       // sequence counter, maybe remove (TODO)
};

// Shared Data Struct for all sensors in one object for *internal* handling.
struct SharedData {
    std::vector<SensorFrame> sensors;
};

// Device Struct for each serial device
struct SerialDevice {
    std::string name;
    sp_port* port;
    bool valid = false;
    std::vector<uint8_t> rx_buf;
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

// // Reads n bytes into buf from port.
// // Blocks until read n bytes or timeout (1ms) reached.
// bool read_exact(sp_port* port, uint8_t* buf, size_t n) {
//     size_t total = 0;
//     while (total < n) {
//         int r = sp_blocking_read(port, buf + total, n - total, 10);
//         if (r <= 0) return false;
//         total += r;
//     }
//     return true;
// }
// //TODO: change this function. make nonblocking with sp_input_waiting and non blocking read


// // Searches for starting header in stream of port.
// // Blocks until found and returns true or false if no bytes available.
// bool sync_to_header(sp_port* port) {
//     uint8_t b1{0x00};
//     uint8_t b2{0x00};
//     int num_failed{0};
//     // while (true) {
//     //     if (b1 != 0xAA and !read_exact(port, &b1, 1)) return false;
//     //     if (b1 == 0xAA) {
//     //         if (!read_exact(port, &b2, 1)) return false;
//     //         if (b2 == 0x55) return true;
//     //         if (b2 == 0xAA) b1 = 0xAA;
//     //     }
//     // }

//     while (true) {
//         if (num_failed != 0) std::cout << "syncing " << num_failed << std::endl;
//         num_failed++;
        
//         if (!read_exact(port, &b1, 1)) return false;  // failed to read byte
//         if (b1 != 0xAA) continue;  // continue reading 1 byte at a time
        
//         if (!read_exact(port, &b2, 1)) return false; // failed to read byte
//         if (b2 == 0x55) return true; // found 0xAA, 0x55

//         if (b2 != 0xAA) continue;
//         while (true) {
//             if (!read_exact(port, &b2, 1)) return false; // failed to read byte
//             if (b2 == 0x55) return true; // found 0xAA, 0x55
//             if (b2 == 0xAA) continue;  // 0xAA, 0x55 still possible
//             // else not possible
//             break;  // continue search for 0xAA as b1
//         }
//     }
//     // TODO: check which logic to use and simplify
// }

// Drains the OS CDC buffer into a user-space device buffer
// This allows for better data parsing and consistency
// Returns false if no new data in stream
bool read_into_buffer(SerialDevice& dev) {
    // check for new data
    // int waiting = sp_input_waiting(dev.port);
    // std::cout << "CDC waiting:\t" << waiting << std::endl;
    // std::cout.flush();

    if (sp_input_waiting(dev.port) < FRAME_SIZE) return false;
    // temporary storage to read stream in chunks
    uint8_t tmp[256];  // in normal state max FRAME_SIZE (51) bytes will be waiting in buffer

    // read stream in tmp chunks and save to device buffer
    while (true) {
        int n = sp_nonblocking_read(dev.port, tmp, sizeof(tmp));
        if (n <= 0) break;  // no more bytes in stream
        dev.rx_buf.insert(dev.rx_buf.end(), tmp, tmp+n);
    }
    return true;
}

bool get_latest_frame(SerialDevice& dev, SensorFrame& frame) {
    std::vector<uint8_t>& rx = dev.rx_buf;

    // std::cout << "rx buf size:\t" << rx.size() << std::endl;
    // std::cout.flush();

    // check if enough bytes for a full frame
    if (rx.size() < FRAME_SIZE) return false;

    // most times last FRAME_SIZE bytes will be latest full frame
    // else search for last header
    size_t idx = rx.size() - FRAME_SIZE;
    if (!(rx[idx] == header[0] && rx[idx + 1] == header[1])) {
        auto it = std::find_end(rx.begin(), rx.end() - FRAME_SIZE,
                                header.begin(), header.end());
        if (it == rx.end() - FRAME_SIZE) return false; // no full frame found
        idx = it - rx.begin();
    }

    // copy data into frame
    idx += 2;  // header bytes
    frame.sensor_ID = rx[idx]; // ID byte
    idx += 1;
    for (int i = 0; i < DATA_N; ++i) {
        uint16_t lo = rx[idx + i*2];
        uint16_t hi = rx[idx + i*2 + 1];
        frame.data[i] = static_cast<uint16_t>(lo | (hi << 8));
    } // data bytes
    idx += 2 * DATA_N;
    std::memcpy(frame.status.data(), &rx[idx], DATA_N); // status bytes
    idx += DATA_N;
    // TODO: maybe into separate function

    rx.erase(rx.begin(), rx.begin() + idx);
    return true;
}



/* OTHER */

// Independant update of each sensor object
void update_sensor(SensorFrame& sensor_frame,
                   SensorFrame& new_frame,
                   std::mutex& mutex) {
                    
    std::lock_guard<std::mutex> lock(mutex);
    std::memcpy(sensor_frame.data.data(), new_frame.data.data(), DATA_N * 2); // each value is 2 bytes
    std::memcpy(sensor_frame.status.data(), new_frame.status.data(), DATA_N);
    sensor_frame.seq++;  // Optional, to keep track of updates
}
// TODO: just replace the whole frame instead of updating individual fields


// Retrieve newest data.
// Only contains data and status per device, no double buffering implementations.
void get_snapshot(SharedData& shared, SharedData& snapshot, std::vector<std::mutex> &mutexes) {
    // replaces old snapshot with newest data using deep copy
    // TODO: check that actualy real copy
    for (size_t i = 0; i < shared.sensors.size(); ++i) {
        std::lock_guard<std::mutex> lock(mutexes[i]);
        snapshot.sensors[i] = shared.sensors[i];
    }
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


std::atomic<bool> running{true};
// Iterates through all devices, reads in new data from stream, extracts the latest frame, then updates data in shared data struct.
void run_all_devices(std::vector<SerialDevice>& devices, SharedData& shared, std::vector<std::mutex>& mutexes) {
    // frame object to temp store data
    SensorFrame frame;

    // Flush CDC stream
    for (auto& dev: devices) {
        sp_flush(dev.port, SP_BUF_INPUT);
    }

    while (running) {
        for (int i = 0; i < devices.size(); i++) {
            // read OS CDC buffer
            if (!read_into_buffer(devices[i])) continue;  // no new data

            // extract latest frame and update sensor
            if (!get_latest_frame(devices[i], frame)) continue;  // no new full frame
            update_sensor(shared.sensors[i], frame, mutexes[i]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        SerialDevice dev {sp_get_port_name(dev_ports[i]), dev_ports[i], true};  // empty rx_buf
        dev.rx_buf.reserve(256);
        serial_devices.push_back(dev);  // (could also use emplace_back)
    }

    // create mutexes for sensor objects
    // TODO: this will be moved to a class property
    std::vector<std::mutex> sensor_mtxs(serial_devices.size());

    // object to store sensor data
    SharedData shared;
    shared.sensors.resize(serial_devices.size());

    // object to store snapshot of sensor data
    SharedData snapshot;
    snapshot.sensors.resize(serial_devices.size());

    std::thread t(run_all_devices, std::ref(serial_devices), std::ref(shared), std::ref(sensor_mtxs));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    for (int i = 0; i < 300; i++) {
        // read snapshot
        get_snapshot(shared, snapshot, sensor_mtxs);

        // print some measurements
        for (int i = 0; i < serial_devices.size(); i++) {
            std::cout << i << ":\t";
            for (int idx : PLOT_INDICES) {
                std::cout << snapshot.sensors[i].data[idx] << "  ";
            }
        }
        std::cout << std::endl;
        std::cout.flush();

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
