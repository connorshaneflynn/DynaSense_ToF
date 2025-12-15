#include "cdc_reader.h"

#include <vector>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <algorithm>

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

// Gets and opens devices, store them, creates mapping and device buffer. Initializes objects.
bool CDCReader::init() {
    // get and open ports
    std::vector<sp_port*> dev_ports = get_and_open_devices_();
    // TODO: maybe use unique ptr
    if (dev_ports.empty()) return false;
    
    // store ports as devices and add to mapping
    store_devices_(dev_ports);

    // create mutex and sensor objects for each device
    for (size_t i = 0; i < serial_devices.size(); ++i) {
        sensor_mtxs.emplace_back();

        shared.sensors.emplace_back();
        shared.sensors.back().ID.device_ID = snapshot.device_names[i];

    } // TODO: or use unique ptr for mtx and resize because it is movable

    return true;
}

// Starts the thread and runs all devices (sleeps shortly before returning)
void CDCReader::run() {
    if (running_) return;
    running_ = true;
    t_ = std::thread(&CDCReader::run_all_devices_, this);
}

// Stops the thread
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
        using enum DeviceState;
        auto now = std::chrono::steady_clock::now();

        // iterate through devices
        for (int i = 0; i < serial_devices.size(); i++) {
            SerialDevice& dev = serial_devices[i];

            // read frame, skip if no new data yet
            if (dev.state == CONNECTED) {
                // read OS CDC buffer
                if (!read_into_buffer_(dev)) continue;  // no new data

                // extract latest frame and filter
                if (!get_latest_frame_(dev, frame)) continue;  // no new full frame
                
                filter_data(frame);

            } else {

                // ------ State handling ------
                switch (dev.state) {

                    // ignore
                    case DISCONNECTED:
                    case FAIL_STATE:
                        break;

                    // try reconnect
                    case ERROR:
                        reset_device_(dev);
                        [[fallthrough]];
                    case RETRY_WAIT:
                        if (now >= dev.next_retry) {
                            try_reconnect_(dev);
                        }
                        break;
                }
            }

            // fill invalid frame if device still not OK
            if (dev.state != CONNECTED) {
                make_invalid_frame(shared.sensors[i], frame);
            };

            update_sensor_(shared.sensors[i], frame, sensor_mtxs[i]);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // close all devices and free ports
    for (auto& dev : serial_devices) {
        sp_close(dev.port);
        sp_free_port(dev.port);
        dev.port = nullptr;
        dev.rx_buf.clear();
        dev.state = DeviceState::DISCONNECTED;
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
            sp_port* dev_port = nullptr;
            sp_copy_port(all_ports[i], &dev_port);
            // Open port
            if (sp_open(dev_port,SP_MODE_READ_WRITE) != SP_OK) {
                // free port
                std::cerr << "ERROR: Could not open port " << sp_get_port_name(dev_port) << std::endl;
                sp_free_port(dev_port);
                continue;
            }

            // set up port
            sp_set_flowcontrol(dev_port, SP_FLOWCONTROL_NONE);   // IMPORTANT: disables any processing in the tty layer
            sp_set_baudrate(dev_port, 115200);                   // any value, CDC ignores it, but tty sometimes expects it
            sp_set_bits(dev_port, 8);                            // these should not be needed but make it definitely safe
            sp_set_parity(dev_port, SP_PARITY_NONE);
            sp_set_stopbits(dev_port, 1);

            found_devices.push_back(dev_port);
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

// store ports as devices and add to mapping
void CDCReader::store_devices_(std::vector<sp_port*>& dev_ports) {
    for (size_t i = 0; i < dev_ports.size(); i++) {
        sp_port* port = dev_ports[i];
        std::string SN = sp_get_port_usb_serial(port);
        std::string portname = sp_get_port_name(port);

        serial_devices.emplace_back(SerialDevice{
            SN, portname, port, DeviceState::CONNECTED, {}, {}  // TODO: check that zero initializing works
        });
        serial_devices.back().rx_buf.reserve(RX_BUFFER_SIZE);

        // add to ID map
        // Use defined map if key exists in map, else use port name
        std::string user_ID;
        if (user_id_map.count(SN)) {
            user_ID = user_id_map.at(SN);
        } else {
            std::cout << "Found unmapped device\n";
            user_ID = portname;
        }
        ID_mapping_[i] = user_ID;
        snapshot.device_names.push_back(user_ID);
    }
    std::cout << user_id_map.size() << " devices specified in mapping.\n\n";
}

void CDCReader::reset_device_(SerialDevice& dev) {
    std::cout << "Reseting device:\t" << dev.portname << std::endl;
    sp_close(dev.port);
    sp_free_port(dev.port);
    dev.port = nullptr;
    dev.rx_buf.clear();
    dev.state = DeviceState::DISCONNECTED;
    std::cout << "Disconnected Device:\t" << dev.portname << std::endl;
}

bool CDCReader::try_reconnect_(SerialDevice& dev) {
    std::cout << "Reconnecting Device:\t" << dev.portname << std::endl;
    sp_port** ports;
    if (sp_list_ports(&ports) != SP_OK) {
        std::cerr << "ERROR: Failed to read any port\n";
        return false;
    }

    for (size_t i = 0; ports[i] != nullptr; ++i) {
        const char* SN = sp_get_port_usb_serial(ports[i]);
        if (SN != dev.serial_number) continue;

        // found device
        sp_copy_port(ports[i], &dev.port);
        if (sp_open(dev.port, SP_MODE_READ_WRITE) != SP_OK) {
            // free port
            std::cerr << "ERROR: Found port but unable to open port: " << dev.portname << std::endl;
            sp_free_port(dev.port);
            dev.port = nullptr;
            dev.state = DeviceState::FAIL_STATE;
            return false;
        }

        char* name = sp_get_port_name(dev.port);

        sp_set_flowcontrol(dev.port, SP_FLOWCONTROL_NONE);   // IMPORTANT: disables any processing in the tty layer
        sp_set_baudrate(dev.port, 115200);                   // any value, CDC ignores it, but tty sometimes expects it
        sp_set_bits(dev.port, 8);                            // these should not be needed but make it definitely safe
        sp_set_parity(dev.port, SP_PARITY_NONE);
        sp_set_stopbits(dev.port, 1);

        dev.state = DeviceState::CONNECTED;
        std::cout << "Connected Device:\t" << dev.portname << std::endl;

        sp_free_port_list(ports);
        return true;
    }

    // not found, retry later
    std::cout << "Failed. Port not found:\t" << dev.portname << std::endl;
    dev.next_retry = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    dev.state = DeviceState::RETRY_WAIT;

    sp_free_port_list((ports));
    return false;
}
// TODO: maybe combine with code for open devices at start
// TODO: check if portname changes

// fills a given SensorFrame with invalid (-1) and sets status to 100 (custom)
void CDCReader::make_invalid_frame(const SensorFrame& sensor_frame, SensorFrame& new_frame) {
    new_frame = sensor_frame;
    new_frame.data.fill(-1);
    new_frame.status.fill(100);
}

// Drains the OS CDC buffer into a user-space device buffer
// This allows for better data parsing and consistency
// Returns false if no new data in stream
bool CDCReader::read_into_buffer_(SerialDevice& dev) {
    // check for new data

    sp_return status = sp_input_waiting(dev.port);
    if (status < 0) {
        // failed to read
        // TODO: check that this doesn't happen during normal use
        dev.state = DeviceState::ERROR;
        std::cerr << "Device '" << dev.portname << "' in error state\n";
        return false;
    }
    if (status < FRAME_SIZE) return false;

    // temporary storage to read stream in chunks
    uint8_t tmp[256] {};  // in normal state max FRAME_SIZE (51) bytes will be waiting in buffer

    // read stream in tmp chunks and save to device buffer
    while (true) {
        int n = sp_nonblocking_read(dev.port, tmp, sizeof(tmp));
        if (n <= 0) break;  // no more bytes in stream
        dev.rx_buf.insert(dev.rx_buf.end(), tmp, tmp+n);
    }
    return true;
}

// Extracts the latest frame from the device buffer and stores it in sensor object
// Erases used device buffer
bool CDCReader::get_latest_frame_(SerialDevice& dev, SensorFrame& frame) {
    std::vector<uint8_t>& rx = dev.rx_buf;
    // TODO: is there a race condition here? do i need to use mutex or copy by value?
    // but: this function is called after buffer read by the same thread, so it should always be sequential

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
    // TODO: make header size variable

    // check frame, temporary until checksum
    if (rx.size() - 1 > idx + FRAME_SIZE) {
        // rx buf not exhausted
        uint8_t last_byte = rx[idx + FRAME_SIZE - 1];
        if (last_byte == header[0] || last_byte == header[1]) {
            // 1-3 bytes were lost and the header of the following frame would be consumed
            // std::cout << "WARNING: Byte slip\n";
            return false;
        }
    }
    // TODO: implement checksum to make this cleaner and more robust

    // copy data into frame
    idx += header.size();  // header bytes
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
    // idx now points one past last byte of frame
    // TODO: maybe into separate function

    rx.erase(rx.begin(), rx.begin() + idx);

    return true;
}

// This removes measurements above the distance threshold and with invalid status (not 5, 9, or 10)
// FIlters frame data in place
void CDCReader::filter_data(SensorFrame& frame) {
    const auto is_valid_status = [](const uint8_t s) {
        return (s == 5 || s == 6 || s == 9 || s == 10);
    };

    for (size_t i = 0; i < frame.status.size(); i++) {
        const bool valid = is_valid_status(frame.status[i]);

        if (!valid) {
            frame.data[i] = static_cast<int16_t>(-1);
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
    for (size_t i = 0; i < shared.sensors.size(); ++i) {
        std::lock_guard<std::mutex> lock(sensor_mtxs[i]);
        snapshot.sensors[ID_mapping_.at(i)] = shared.sensors[i];
    }
}
// TODO: add all names to snapshot

// Returns a reference to the CDC object snapshot
// Same as accesing CDCReader::snapshot directly
const CDCReader::Snapshot& CDCReader::get_snapshot_handle() {
    return snapshot;
}

// // return specific sensor frame from snapshot using the user mapping
// const CDCReader::SensorFrame& CDCReader::get_sensor(std::string string_ID) {
//     return snapshot.sensors[ID_mapping_.at(string_ID)];
// }
