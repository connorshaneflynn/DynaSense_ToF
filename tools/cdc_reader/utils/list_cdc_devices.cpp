#include <iostream>
#include <libserialport.h>

void print_port_info(sp_port *port)
{
    const char *port_name = sp_get_port_name(port);
    std::cout << "Port: " << (port_name ? port_name : "(unknown)") << "\n";

    // USB VID/PID
    int vid, pid;
    sp_get_port_usb_vid_pid(port, &vid, &pid);

    if (vid > 0 && pid > 0) {
        std::cout << "  USB VID: 0x" << std::hex << vid << std::dec << "\n";
        std::cout << "  USB PID: 0x" << std::hex << pid << std::dec << "\n";
    } else {
        std::cout << "  Not a USB device (VID/PID unavailable)\n";
        return;
    }

    // Manufacturer
    char *manufacturer = sp_get_port_usb_manufacturer(port);
    if (manufacturer)
        std::cout << "  Manufacturer: " << manufacturer << "\n";

    // Product
    char *product = sp_get_port_usb_product(port);
    if (product)
        std::cout << "  Product: " << product << "\n";

    // Serial number
    char *serial = sp_get_port_usb_serial(port);
    if (serial)
        std::cout << "  Serial: " << serial << "\n";

    // USB address (bus + hub + port path)
    int usb_bus;
    sp_get_port_usb_bus_address(port, &usb_bus, NULL);
    if (usb_bus)
        std::cout << "  USB Bus " << usb_bus << "\n";

    std::cout << "\n";
}

int main()
{
    struct sp_port **ports;

    enum sp_return result = sp_list_ports(&ports);
    if (result != SP_OK) {
        std::cerr << "sp_list_ports() failed\n";
        return 1;
    }

    std::cout << "Enumerating serial (CDC) devices...\n\n";

    for (int i = 0; ports[i] != NULL; i++) {
        sp_port *p = ports[i];
        print_port_info(p);
    }

    sp_free_port_list(ports);
    return 0;
}
