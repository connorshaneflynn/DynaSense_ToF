#include "cdc_reader.h"

#include <iostream>
#include <array>
#include <thread>
#include <chrono>

static constexpr std::array<uint8_t, 1> PLOT_INDICES {3};

// helper function to print several zones, specified in PLOT_INDICES
void print_snapshot(const CDCReader::SharedData& snap) {
    for (int i = 0; i < snap.sensors.size(); i++) {
        std::cout << i << ":  ";
        for (int idx : PLOT_INDICES) {
            std::cout << snap.sensors[i].data[idx] << "\t\t";
        }
    }
    std::cout << std::endl;
    std::cout.flush();
}

int main() {
    // Construct and initialize
    uint16_t distance_threshold = 1000;
    CDCReader reader(distance_threshold);
    if (!reader.init()) {
        std::cout << "ERROR: Failed to initialize the CDC reader\n";
        return 1;
    };

    // Start the reader thread
    reader.run();

    // Access data snapshot per reference
    const auto& snap = reader.get_snapshot_handle();
    reader.update_snapshot();  // populate snapshot map once before selecting an ID

    // Prefer the mapped ID "FL" if present; otherwise fall back to the first available
    std::string sensor_id = "FL";
    if (!snap.sensors.contains(sensor_id)) {
        if (snap.sensors.empty()) {
            std::cerr << "No sensors available in snapshot.\n";
            reader.stop();
            return 1;
        }
        sensor_id = snap.sensors.begin()->first;
        std::cout << "ID \"FL\" not found, using \"" << sensor_id << "\" instead.\n";
    } else {
        std::cout << "Using sensor ID \"" << sensor_id << "\"\n";
    }

    // Simulate main loop
    for (size_t i = 0; i < 3000; i++) {
        // Update snapshot with latest data
        reader.update_snapshot();

        // Some processing
        auto it = snap.sensors.find(sensor_id);
        if (it == snap.sensors.end()) {
            std::cerr << "Sensor \"" << sensor_id << "\" missing from snapshot.\n";
            break;
        }
        std::cout << it->second.data[5] << std::endl;

        // run loop at ~50hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // Stop the reader thread
    reader.stop();
    return 0;
}
