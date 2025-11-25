# Project Name

Small Class that reads sensor data from STM32 USB CDC devices using libserialport.

## Overview

This project provides a `CDCReader` C++ class for reading sensor data from USB CDC devices. It maintains a data structure with the newest sensor frames in a separate thread and provides the user with an independent snapshot of the latest data. It includes example code showing integration, utility helpers, and CMake build configuration.

## Features

- C++ class (`cdc_reader.h` / `cdc_reader.cpp`) implementing USB CDC data acquisition  
- Snapshot mechanism to safely access the latest sensor data  
- Example `main.cpp` demonstrating usage  
- Configuration file for named sensor access (`position_mapping.json`)  
- CMake-based build system  
- Utility files for listing CDC devices etc.
- Depends on **libserialport**

## Repository Structure

``` txt
cdc_reader/
│
├── cdc_reader.h          # Class interface
├── cdc_reader.cpp        # Class implementation
├── main.cpp              # Example usage
├── utils/                # Utility/helper modules
│   ├── ...
│
├── position_mapping.json # Device mapping configuration
├── CMakeLists.txt        # Build configuration
│
└── build/                # Build output directory
```

## Dependencies

- C++23 or newer (lower should be fine)
- CMake >= 3.10
- **libserialport**

Linux:

``` shell
sudo apt install libserialport-dev
```

## How to Use the Class

Minimal usage example:

```cpp
#include "cdc_reader.h"

int main() {
    CDCReader reader;
    reader.init();

    reader.start();

    const auto& snap = reader.get_snapshot_handle();
    snap.update_snapshot();
    // Access snapshot fields...

    reader.stop();
}
```

After class construction, init() must be called. Data is only accesible in snapshot once `update_snapshot()` has been called.

See `main.cpp` for more details. After building this can be run with `./build/cdc_reader`.

## Position Mapping

To access devices using the position (e.g. "FL"), a naming map must be supplied by the user. This maps the device serial number to the position string.

Currently the map is defined inside `cdc_reader.h`. Later I will add a separate JSON file for this.

Map Structure:

``` json
{"<serial number>", "<position string>"}
```

## Troubleshooting

- No devices found: ensure device connected, correct USB mode (CDC), correct udev rules/permissions, and that product_string matches the firmware product_name.
- Serial port busy: close other programs (e.g., serial monitors).
- Wrong frame parsing (no frames, frame skipping, high latency): verify firmware frame format (header and DATA_N) matches constants in cdc_reader.h.
