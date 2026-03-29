#ifndef VL53L5CX_COMMON_H
#define VL53L5CX_COMMON_H

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

/* Current LED mode - extern so it can be set by ranging modules */
extern LedMode ledMode;

/* Sensor configuration */
#define RESOLUTION 16                                 // NxN
#define FREQUENCY  60                                 // hz

#define RANGING_MODE VL53L5CX_RANGING_MODE_AUTONOMOUS // _CONTINUOUS or _AUTONOMOUS
#define INTEGRATION_TIME 1                            // ms, only used if autonomous ranging
#define SHARPENER 0                                   // percent
#define TARGET_ORDER VL53L5CX_TARGET_ORDER_CLOSEST    // VL53L5CX_TARGET_ORDER_CLOSEST or VL53L5CX_TARGET_ORDER_STRONGEST

/* Communication settings */
#define COM_UART false
#define COM_USB true

/* General UART buffer - extern so modules can use it */
extern char msg[512];

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
 * @brief CDC transmit with timeout to avoid blocking.
 * @param buf Pointer to data buffer.
 * @param len Length of data.
 * @return USBD_OK on success, USBD_FAIL on timeout.
 */
uint8_t CDC_Transmit_WithTimeout(uint8_t *buf, uint16_t len);

/**
 * @brief Send ranging measurements over USB and/or UART.
 * @param sensor_ID Sensor identifier.
 * @param results Pointer to VL53L5CX results structure.
 */
void send_measurements(uint8_t sensor_ID, VL53L5CX_ResultsData *results);

#endif /* VL53L5CX_COMMON_H */
