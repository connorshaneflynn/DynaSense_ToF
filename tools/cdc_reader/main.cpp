#include "cdc_reader.h"

#include <iostream>
#include <array>
#include <thread>
#include <chrono>

static constexpr std::array<uint8_t, 1> PLOT_INDICES {3};

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
    uint16_t distance_threshold = 1000;
    CDCReader reader;  //default threshold is max int16_t value
    if (!reader.init()) {
        std::cout << "ERROR: Failed to initialize the CDC reader\n";
        return 1;
    };
    reader.run();

    // reference to internal snapshot of sensor data
    const auto& snap = reader.get_snapshot_handle();
    // or use:
    // const CDCReader::SharedData& snapshot = reader.snapshot;
    // NOTE: I can change how the snapshot is passed to the main loop if you want

    for (size_t i = 0; i < 300; i++) {
        reader.update_snapshot();
        print_snapshot(snap);        

        // run loop at ~50hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    reader.stop();
    return 0;
}