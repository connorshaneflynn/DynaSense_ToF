Custom USB HID on Ubuntu:
Create a udev rule for access rights
Create file '/etc/udev/rules.d/99-hid.rules'.
Name and number in file name can be different.
File contents: 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE:="0666"'

Add packet sizes to main.h:
#define CUSTOM_HID_EPIN_SIZE  64U  // 64 bytes for IN
#define CUSTOM_HID_EPOUT_SIZE 32U  // 32 Bytes for OUT
