Custom USB HID on Ubuntu:
Create a udev rule for access rights
Create file '/etc/udev/rules.d/99-hid.rules'.
Name and number in file name can be different.
File contents: 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE:="0666"'