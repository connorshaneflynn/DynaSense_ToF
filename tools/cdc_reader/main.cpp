#include "cdc_reader.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <vector>

static const std::vector<uint8_t> PLOT_INDICES {0, 3, 4, 7, 8, 11, 12, 15};

// helper function to print several zones, specified in PLOT_INDICES
void print_snapshot(const CDCReader::Snapshot& snapshot, std::vector<uint8_t> indices) {
    for (int i = 0; i < snapshot.sensors.size(); i++) {
        const std::string name = snapshot.device_names[i];
        CDCReader::SensorFrame frame = snapshot.sensors.at(snapshot.device_names[i]);

        std::cout << i << ":  ";
        for (int idx : indices) {
            std::cout << frame.data[idx] << ",  " << static_cast<uint16_t>(frame.status[idx]) << "\t\t";
        }

        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout.flush();
}


// helper function to observe the sequence numbers of all found sensors
void print_snapshot_seq(const CDCReader::Snapshot& snapshot) {
    static std::vector<int64_t> prev_seqs;
    if (prev_seqs.empty()) prev_seqs.resize(snapshot.sensors.size());

    uint64_t max_seq = 0;
    uint64_t min_seq = -1;  // sets to max value

    for (size_t i = 0; i < snapshot.sensors.size(); i++) {
        const std::string name = snapshot.device_names[i];
        CDCReader::SensorFrame frame = snapshot.sensors.at(snapshot.device_names[i]);

        int64_t seq = static_cast<int64_t>(frame.seq);

        std::cout << frame.ID.device_ID << ":\t" << seq << "   (" << seq-prev_seqs[i] << ")\t\t";
        
        if (frame.seq > max_seq) max_seq = frame.seq;
        if (frame.seq < min_seq) min_seq = frame.seq;

        prev_seqs[i] = seq;
    }

    if (max_seq > min_seq) {
        std::cout << "Max Diff:\t" << max_seq - min_seq;
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
    for (size_t i = 0; i < 500; i++) {
        // Update snapshot with latest data
        reader.update_snapshot();

        // Some processing

        // Here the first sensor is extracted from the snapshot to make it independent of naming
        // const std::string& sensor_name = snap.names[0];
        // std::cout << snap.sensors.at(sensor_name).data[5] << std::endl;

        // or using a user defined print function
        // print_snapshot(snap, PLOT_INDICES);
        print_snapshot_seq(snap);

        // run loop at ~50hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // Stop the reader thread
    reader.stop();
    return 0;
}
