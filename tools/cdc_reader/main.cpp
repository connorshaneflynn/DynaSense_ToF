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
    auto& snap = reader.get_snapshot_handle();
    // or use:
    // const CDCReader::SharedData& snapshot = reader.snapshot;
    // NOTE: I can change how the snapshot is passed to the main loop if you want

    // Simulate main loop
    for (size_t i = 0; i < 300; i++) {
        // Update snapshot with latest data
        reader.update_snapshot();

        // Some processing

        // Here the first sensor is extracted from the snapshot to make it independent of naming
        const std::string& sensor_name = snap.names[0];
        std::cout << snap.sensors.at(sensor_name).data[5] << std::endl;

        // run loop at ~50hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // Stop the reader thread
    reader.stop();
    return 0;
}
