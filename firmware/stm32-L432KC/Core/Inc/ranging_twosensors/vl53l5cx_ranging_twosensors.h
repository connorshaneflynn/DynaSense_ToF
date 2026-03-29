#ifndef VL53L5CX_RANGING_TWOSENSORS_H
#define VL53L5CX_RANGING_TWOSENSORS_H

#include "vl53l5cx_common.h"

/**
 * @brief Initialize the VL53L5CX sensors and start ranging.
 * @return 0 on success, otherwise HAL/I2C error code.
 */
uint8_t initialize(void);

/**
 * @brief Main loop routine to read sensors and send data.
 */
void run(void);

#endif /* VL53L5CX_RANGING_TWOSENSORS_H */
