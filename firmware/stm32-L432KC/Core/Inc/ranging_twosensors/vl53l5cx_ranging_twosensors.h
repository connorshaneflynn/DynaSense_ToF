#ifndef VL53L5CX_RANGING_TWOSENSORS_H
#define VL53L5CX_RANGING_TWOSENSORS_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "vl53l5cx_api.h"


/* LED modes */
typedef enum {
    OFF,
    SLOW,
    FAST,
    VERYFAST
} LedMode;

/* Communication Modes*/

/* Public variables */

/* Sensor structures */

/* Functions */

/**
 * @brief Send a pre-formatted message via UART.
 */
void uartSend(void);

/**
 * @brief Update LED according to ledMode (non-blocking).
 */
void updateLED(void);

/**
 * @brief Scan the I2C bus and print found devices over UART.
 */
void i2c_scan(void);

/**
 * @brief Send ranging measurements over USB and UART.
 * @param distances Pointer to array of distances (mm).
 * @param statuses Pointer to array of target statuses.
 */
void send_data(uint8_t sensor_ID, VL53L5CX_ResultsData *results);

/**
 * @brief Initialize the VL53L5CX sensor and start ranging.
 * @return 0 on success, otherwise HAL/I2C error code.
 */
uint8_t initialize(void);

/**
 * @brief Main loop routine to read sensor and send data.
 */
void run(void);

#endif /* VL53L5CX_RANGING_H */
