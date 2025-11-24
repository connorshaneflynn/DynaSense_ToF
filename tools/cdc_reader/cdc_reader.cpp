#include "cdc_reader.h"
#include <vector>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <deque>
#include <unordered_map>

#include <libserialport.h>

CDCReader::CDCReader(int16_t max_distance)
    : dist_threshold(max_distance),
      threshold_repl(max_distance)  // or 32'767: int16_t max value
{
    // TODO: include user defined mapping here
}


CDCReader::~CDCReader() {
    // shut down thread
    stop();

    // close devices
    for (int i = 0; i < serial_devices.size(); i++) {
        sp_close(serial_devices[i].port);
        sp_free_port(serial_devices[i].port);
    }
}

bool CDCReader::init() {
    // get and open ports
    std::vector<sp_port*> dev_ports = get_and_open_devices_();
    // TODO: maybe use unique ptr
    if (dev_ports.empty()) return false;
    
    // store ports as devices and add to mapping
    for (size_t i = 0; i < dev_ports.size(); i++) {
        sp_port* port = dev_ports[i];
        std::string SN = sp_get_port_usb_serial(port);
        std::string portname = sp_get_port_name(port);
        // add to UID to index map by using serial to UID bridge
        serial_devices.emplace_back(SerialDevice{
            SN, portname, port, true, {}
        });
        serial_devices.back().rx_buf.reserve(RX_BUFFER_SIZE);

        // add to ID map
        // Use defined map if key exists in map, else use port name
        std::string user_ID;
        if (user_id_map.count(SN)) {
            user_ID = user_id_map.at(SN);
        } else {
            user_ID = portname;
        }
        ID_mapping_[user_ID] = i;
    }

    // resize objects so an instance exists for each device
    for (size_t i = 0; i < serial_devices.size(); ++i) {
        sensor_mtxs.emplace_back();
    } // TODO: or use unique ptr and resize because it is movable
    shared.sensors.resize(serial_devices.size());
    snapshot.sensors.resize(serial_devices.size());

    return true;
}

void CDCReader::run() {
    if (running_) return;
    running_ = true;
    t_ = std::thread(&CDCReader::run_all_devices_, this);
}

void CDCReader::stop() {
    if (!running_) return;
    running_ = false;
    if (t_.joinable()) {
        t_.join();
    }
}

// Iterates through all devices, reads in new data from stream, extracts the latest frame, then updates data in shared data struct.
void CDCReader::run_all_devices_() {
    // frame object to temp store data
    SensorFrame frame;

    // Flush CDC stream
    for (auto& dev: serial_devices) {
        sp_flush(dev.port, SP_BUF_INPUT);
    }

    while (running_) {
        for (int i = 0; i < serial_devices.size(); i++) {
            // read OS CDC buffer
            if (!read_into_buffer_(serial_devices[i])) continue;  // no new data

            // extract latest frame and filter
            if (!get_latest_frame_(serial_devices[i], frame)) continue;  // no new full frame
            filter_data(frame);

            update_sensor_(shared.sensors[i], frame, sensor_mtxs[i]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// Auto detects CDC devices per Product String (Default "STM32 ToF")
// Opens all detected devices and returns a vector of sp_port*
// Returns an empty vector if no devices were found
// Ports need to be closed before exiting program
std::vector<sp_port*> CDCReader::get_and_open_devices_() {
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
        if (product && product == PRODUCT_NAME) {
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
        std::cout << "No USB ports found with product string: " << PRODUCT_NAME << std::endl;
    } else {
        std::cout << "Found " << found_devices.size() << " devices\n";
    }
    return found_devices;
}


// Drains the OS CDC buffer into a user-space device buffer
// This allows for better data parsing and consistency
// Returns false if no new data in stream
bool CDCReader::read_into_buffer_(SerialDevice& dev) {
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

// Extracts the latest frame from the device buffer and stores it in sensor object
bool CDCReader::get_latest_frame_(SerialDevice& dev, SensorFrame& frame) {
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
        int16_t lo = rx[idx + i*2];
        int16_t hi = rx[idx + i*2 + 1];
        frame.data[i] = static_cast<int16_t>(lo | (hi << 8));
    } // data bytes
    idx += 2 * DATA_N;
    std::memcpy(frame.status.data(), &rx[idx], DATA_N); // status bytes
    idx += DATA_N;
    // TODO: maybe into separate function

    rx.erase(rx.begin(), rx.begin() + idx);
    return true;
}

// This removes measurements above the distance threshold and with invalid status (not 5, 9, or 10)
void CDCReader::filter_data(SensorFrame& frame) {
    const auto is_valid_status = [](const uint8_t s) {
        return (s == 5 || s == 6 || s == 9 || s == 10);
    };

    for (size_t i = 0; i < frame.status.size(); i++) {
        const bool valid = is_valid_status(frame.status[i]);

        if (!valid) {
            frame.data[i] = static_cast<int16_t>(-1);
            // std::cout << static_cast<int>(frame.status[i]) << std::endl;
            // std::cout.flush();
            continue;
        }

        if (frame.data[i] > dist_threshold) {
            frame.data[i] = threshold_repl;
        }
    }
}


// Independant update of each sensor object
void CDCReader::update_sensor_(SensorFrame& sensor_frame,
                   SensorFrame& new_frame,
                   std::mutex& mutex) {
                    
    std::lock_guard<std::mutex> lock(mutex);
    std::memcpy(sensor_frame.data.data(), new_frame.data.data(), DATA_N * 2); // each value is 2 bytes
    std::memcpy(sensor_frame.status.data(), new_frame.status.data(), DATA_N);
    sensor_frame.seq++;  // Optional, to keep track of updates
}
// TODO: just replace the whole frame instead of updating individual fields


// Updates the CDCReader snapshot with the newest data in SharedData.
void CDCReader::update_snapshot() {
    // replaces old snapshot with newest data using deep copy
    // TODO: check that actualy real copy
    for (size_t i = 0; i < shared.sensors.size(); ++i) {
        std::lock_guard<std::mutex> lock(sensor_mtxs[i]);
        snapshot.sensors[i] = shared.sensors[i];
    }
}

// Returns a reference to the CDC object snapshot
// Same as accesing CDCReader::snapshot directly
const CDCReader::SharedData& CDCReader::get_snapshot_handle() {
    return snapshot;
}

// return specific sensor frame from snapshot using the user mapping
const CDCReader::SensorFrame& CDCReader::get_sensor(std::string string_ID) {
    return snapshot.sensors[ID_mapping_.at(string_ID)];
}
