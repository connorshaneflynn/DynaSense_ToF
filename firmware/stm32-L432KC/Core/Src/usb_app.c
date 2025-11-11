#include "usb_app.h"
#include "usb_config.h"  // TODO: fix import issue and remove this
#include <stdint.h>
#include <string.h>

volatile uint8_t hid_data_ready = 0;
uint8_t hid_rx_buffer[CUSTOM_HID_EPOUT_SIZE];

void on_hid_data_received(uint8_t *data, uint8_t len)
{
    memcpy(hid_rx_buffer, data, len);
    hid_data_ready = 1;
}