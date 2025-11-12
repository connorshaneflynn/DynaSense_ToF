/*
Reads sensor data over HID from a single connected STM32.
*/

#include <iostream>
#include <vector>
#include <chrono>
#include <hidapi/hidapi.h>
#include <cstdint>
#include <cstring>

constexpr uint16_t VID = 0x0483;
constexpr uint16_t PID = 0x5750;
constexpr int RESOLUTION = 16;
constexpr int FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION; // same as Python
constexpr int PLOT_INDICES[] = {5, 6, 9, 10};

void filter_data(std::vector<uint16_t>& distances, std::vector<uint8_t>& statuses) {
    for (size_t i = 0; i < distances.size(); ++i) {
        if (!(statuses[i] == 5 || statuses[i] == 9 || statuses[i] == 10))
            distances[i] = static_cast<uint16_t>(-1);
    }
}

int main() {
    if (hid_init())
        return 1;

    hid_device* dev = hid_open(VID, PID, NULL);

    // struct hid_device_info* devs = hid_enumerate(VID, PID);
    // hid_device* dev = hid_open_path(devs->path);
    // hid_free_enumeration(devs);

    if (!dev) {
        std::cerr << "Could not open HID device.\n";
        return 1;
    }

    std::vector<unsigned char> packet(FRAME_SIZE);
    while (true) {
        auto t_start = std::chrono::high_resolution_clock::now();

        int n = hid_read(dev, packet.data(), FRAME_SIZE);
        if (n <= 0)
            continue;

        // remove header
        const unsigned char* p = packet.data() + 2;
        uint8_t sensor_ID = *p++;

        std::vector<uint16_t> distances(RESOLUTION);
        std::memcpy(distances.data(), p, RESOLUTION * 2);
        p += RESOLUTION * 2;

        std::vector<uint8_t> statuses(RESOLUTION);
        std::memcpy(statuses.data(), p, RESOLUTION);

        filter_data(distances, statuses);

        auto t_end = std::chrono::high_resolution_clock::now();
        double delta_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        for (int i : PLOT_INDICES)
            std::cout << distances[i] << " ";
        std::cout << "   (" << delta_ms << " ms)\n";
        std::cout.flush();
    }

    std::cout << "Stopping" << std::endl;

    hid_close(dev);
    hid_exit();
    return 0;
}
