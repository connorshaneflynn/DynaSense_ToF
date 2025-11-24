#pragma once

#include <cstdint>
#include <atomic>
#include <vector>
#include <array>
#include <thread>
#include <deque>

// forward declaration from libserialport
struct sp_port;

class CDCReader {
public:
    /* constants */

    // These are firmware dependent values
    static constexpr uint8_t DATA_N = 16;
    static constexpr std::array<uint8_t, 2> header = {0xAA, 0x55};
    static constexpr uint8_t FRAME_SIZE = header.size() + 1 + DATA_N * 2 + DATA_N;
    static constexpr std::string_view PRODUCT_NAME = "STM32 ToF";

    
    /* structs */

    // Sensor Struct
    struct SensorFrame {
        uint8_t sensor_ID;
        std::array<int16_t, DATA_N> data{};
        std::array<uint8_t, DATA_N> status{};
        uint64_t seq = 0;
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


    /* constructor / destructor */

    explicit CDCReader(int16_t max_distance = 1000); // TODO: add params
    ~CDCReader(); // includes stop thread, close and free devices

    bool init();  // includes get and open devices, store ports in serial_devices, create buffer


    /* methods */

    void run();  // starts the thread and runs the function (sleeps shortly before returning)
    void stop(); // stops the thread

    void update_snapshot();

    const SharedData& get_snapshot_handle();


    /* member variables */

    SharedData snapshot;


private:
    /* methods */

    std::vector<sp_port*> get_and_open_devices_();

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

    std::vector<SerialDevice> serial_devices;
    std::deque<std::mutex> sensor_mtxs;

    SharedData shared;
};
