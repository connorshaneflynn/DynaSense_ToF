/*
Reads data over HID and sends it back immediately. Extracts and prints latency measurements from data.
*/

#include <iostream>
#include <vector>
#include <hidapi/hidapi.h>
#include <cstdint>
#include <cstring>

constexpr uint16_t VID = 0x0483;
constexpr uint16_t PID = 0x5750;
constexpr int RESOLUTION = 16;
constexpr int FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION; // same as Python

int main() {
    if (hid_init())
    {
        std::cerr << "Could not initialize hid.\n";
        return 1;
    }
    
    hid_device* dev = hid_open(VID, PID, NULL);

    // struct hid_device_info* devs = hid_enumerate(VID, PID);
    // hid_device* dev = hid_open_path(devs->path);
    // hid_free_enumeration(devs);

    if (!dev) {
        std::cerr << "Could not open HID device.\n";
        return 1;
    } else {
        std::cout << "Found HID with VID " << VID << " and PID " << PID << std::endl;
    }

    std::vector<unsigned char> packet(FRAME_SIZE);
    std::vector<unsigned char> return_packet(FRAME_SIZE + 1);  // one additional byte for report ID
    return_packet[0] = 0x0;

    std::cout << "Starting Listening" << std::endl;
    while (true) {
        std::cout << "Reading\n";
        int n = hid_read(dev, packet.data(), FRAME_SIZE);
        if (n <= 0)
            continue;

        // send back same data immeadiately
        memcpy(&return_packet[1], &packet, FRAME_SIZE);
        hid_write(dev, return_packet.data(), return_packet.size());

        std::cout << "Returned Package" << std::endl;

        // extract first 4 bytes into time
        uint32_t latency;
        memcpy(&latency, packet.data(), sizeof(latency));

        // print time
        std::cout << "latency:\t" << latency << " ms\n\n";
        std::cout.flush();
    }

    std::cout << "Stopping" << std::endl;

    hid_close(dev);
    hid_exit();
    return 0;
}
