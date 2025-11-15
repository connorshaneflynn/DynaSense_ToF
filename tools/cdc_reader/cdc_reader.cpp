#include "cdc_reader.h"
#include <iostream>
#include <libserialport.h>


class CDCReader
{
    public:
};








int main()
{
    std::cout << constants::pi << std::endl;

    sp_port* port = nullptr;
    if (sp_get_port_by_name("/dev/ttyACM1", &port) != SP_OK) {
        std::cerr << "ERROR: Port not found: " << "/dev/ttyACM1" << std::endl;
    }

    return 0;
}
