#ifndef USB_APP_H
#define USB_APP_H

#include "usb_config.h"
#include <stdint.h>

extern volatile uint8_t hid_data_ready;
extern uint8_t hid_rx_buffer[CUSTOM_HID_EPOUT_SIZE];

void on_hid_data_received(uint8_t *data, uint8_t len);

#endif