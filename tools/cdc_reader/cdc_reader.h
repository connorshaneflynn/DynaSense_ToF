#pragma once

#include <cstdint>
#include <atomic>
#include <vector>
#include <array>
#include <thread>
#include <deque>
#include <unordered_map>
#include <mutex>

// temporary static ID mapping instead of separate json file
static const std::unordered_map<std::string, std::string> user_id_map = {
    {"209C35B54234", "FL"},
    {"205E35844234", "FR"},
    {"205435984234", "BR"},
    {"205C35B54234", "BL"}
};

// forward declaration from libserialport
struct sp_port;

class CDCReader {
public:
    /* constants */

    // These are firmware dependent values
    static constexpr uint8_t DATA_N = 16;                                               // Number of measurements
    static constexpr std::array<uint8_t, 2> header = {0xAA, 0x55};                      // For frame synchronization
    static constexpr uint8_t FRAME_SIZE = header.size() + 1 + DATA_N * 2 + DATA_N;      // Header, Sensor ID, Distance, Status
    static constexpr std::string_view PRODUCT_NAME = "STM32 ToF";                       // To identify CDC devices

    
    /* structs */

    // ID Struct
    struct unique_ID {
        std::string device_ID;
        uint8_t sensor_ID;
    };
    // TODO: not used currently but will when num sensors != num devices

    // Sensor Struct
    struct SensorFrame {
        unique_ID ID;                               // not yet used
        uint8_t sensor_ID;
        std::array<int16_t, DATA_N> data{};
        std::array<uint8_t, DATA_N> status{};
        uint64_t seq = 0;                           // not yet used, to keep track of updates
    };

    // Shared Data Struct for all sensors in one object
    struct SharedData {
        std::vector<SensorFrame> sensors;
    };

    // Snapshot struct that saves sensors as mapping so accessible through string ID
    struct Snapshot {
        std::unordered_map<std::string, SensorFrame> sensors;
        std::vector<std::string> device_names;            // Names of found sensors
    };

    // Device Struct for each serial device
    struct SerialDevice {
        std::string serial_number;
        std::string portname;
        sp_port* port;
        bool valid = false;                         // not yet used, for error handling
        std::vector<uint8_t> rx_buf;
    };


    /* constructor / destructor */

    explicit CDCReader(int16_t max_distance = 1000);
    ~CDCReader(); // includes stop thread, close and free devices

    bool init();  // includes get and open devices, store serial_devices, create mapping and buffer


    /* methods */

    void run();
    void stop();

    void update_snapshot();

    const Snapshot& get_snapshot_handle();

    // const SensorFrame& get_sensor(std::string string_ID);
    // NOTE: Not used anymore but could reimplement if wished.


    /* member variables and objects*/

    Snapshot snapshot;
    std::vector<SerialDevice> serial_devices;


private:
    /* methods */

    std::vector<sp_port*> get_and_open_devices_();

    void store_devices_(std::vector<sp_port*>& dev_ports);

    bool read_into_buffer_(SerialDevice& dev);

    bool get_latest_frame_(SerialDevice& dev, SensorFrame& frame);

    void filter_data(SensorFrame& frame);

    void update_sensor_(
        SensorFrame& sensor_frame,
        SensorFrame& new_frame,
        std::mutex& mutex
    );

    void run_all_devices_();


    /* member variables */
    static constexpr int RX_BUFFER_SIZE = 256;

    std::thread t_;
    std::atomic<bool> running_{false};

    int16_t dist_threshold;  // max distance in mm
    int16_t threshold_repl;  // value to replace overflow distance with

    std::deque<std::mutex> sensor_mtxs;

    std::unordered_map<size_t, std::string> ID_mapping_;  // maps internal index to user string ID

    SharedData shared;
};
