#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <libserialport.h>

constexpr const char* SERIAL_PORT = "/dev/ttyACM1";
constexpr int BAUD_RATE = 230400;
constexpr int RESOLUTION = 16;
constexpr int FRAME_SIZE = 2 + 1 + RESOLUTION * 2 + RESOLUTION;
constexpr int PLOT_INDICES[] = {5, 6, 9, 10};

sp_port* open_serial_port(const char* port_name, int baud) {
    sp_port* port = nullptr;
    if (sp_get_port_by_name(port_name, &port) != SP_OK) {
        std::cerr << "ERROR: Port not found: " << port_name << std::endl;
        return nullptr;
    }
    if (sp_open(port, SP_MODE_READ_WRITE) != SP_OK) {
        std::cerr << "ERROR: Could not open port\n";
        sp_free_port(port);
        return nullptr;
    }
    sp_set_baudrate(port, baud);
    sp_set_bits(port, 8);
    sp_set_parity(port, SP_PARITY_NONE);
    sp_set_stopbits(port, 1);
    sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);
    return port;
}

bool read_exact(sp_port* port, uint8_t* buf, size_t n) {
    size_t total = 0;
    while (total < n) {
        int r = sp_blocking_read(port, buf + total, n - total, 100);
        if (r <= 0) return false;
        total += r;
    }
    return true;
}

bool sync_to_header(sp_port* port) {
    uint8_t b1, b2;
    while (true) {
        if (!read_exact(port, &b1, 1)) return false;
        if (b1 == 0xAA) {
            if (!read_exact(port, &b2, 1)) return false;
            if (b2 == 0x55) return true;
        }
    }
}

bool read_frame(sp_port* port, uint8_t& sensor_ID,
                std::vector<uint16_t>& distances,
                std::vector<uint8_t>& statuses) {
    if (!sync_to_header(port)) return false;

    std::vector<uint8_t> packet(FRAME_SIZE - 2);
    if (!read_exact(port, packet.data(), packet.size())) return false;

    sensor_ID = packet[0];
    distances.resize(RESOLUTION);
    statuses.resize(RESOLUTION);

    for (int i = 0; i < RESOLUTION; ++i) {
        distances[i] = packet[1 + i * 2] | (packet[1 + i * 2 + 1] << 8);
    }

    std::memcpy(statuses.data(), &packet[1 + RESOLUTION * 2], RESOLUTION);
    return true;
}

void filter_data(std::vector<uint16_t>& data,
                 const std::vector<uint8_t>& statuses) {
    for (int i = 0; i < 4; ++i) {
        int idx = PLOT_INDICES[i];
        if (!(statuses[idx] == 5 || statuses[idx] == 9 || statuses[idx] == 10)) {
            data[idx] = static_cast<uint16_t>(-1);
        }
    }
}

int main() {
    sp_port* port = open_serial_port(SERIAL_PORT, BAUD_RATE);
    if (!port) return 1;

    uint8_t sensor_ID;
    std::vector<uint16_t> distances;
    std::vector<uint8_t> statuses;

    while (true) {
        auto t_start = std::chrono::high_resolution_clock::now();

        if (!read_frame(port, sensor_ID, distances, statuses)) {
            std::cerr << "Failed to read frame\n";
            sp_drain(port);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        filter_data(distances, statuses);

        for (int idx : PLOT_INDICES)
            std::cout << distances[idx] << " ";

        auto t_end = std::chrono::high_resolution_clock::now();
        double delta_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        std::cout << "     (" << delta_ms << " ms)\n";
    }

    sp_close(port);
    sp_free_port(port);
    return 0;
}
