#ifndef USB_CONFIG_H
#define USB_CONFIG_H

/* Configure Custom HID */

// These sizes override the default size of 2 bytes
#define CUSTOM_HID_EPIN_SIZE  64U  // 64 bytes for IN
#define CUSTOM_HID_EPOUT_SIZE 64U  // 64 Bytes for OUT
// #define USBD_CUSTOMHID_REPORT_BUFFER_EVENT_ENABLED 1  // Used to override the EPIN and EPOUT sizes. Maybe not needed

// Default power is 100mA
#define USBD_MAX_POWER 0xFAU  // Request 500mA

#endif