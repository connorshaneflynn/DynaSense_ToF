#ifndef HID_LATENCY_TEST_H
#define HID_LATENCY_TEST_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
// #include "vl53l5cx_api.h"


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
 * @brief Update LED according to ledMode (non-blocking).
 */
void updateLED(void);


void update_timing_data(uint32_t time_ms);


/**
 * @brief Initialize the HID.
 * @return 0 on success.
 */
uint8_t initialize(void);

/**
 * @brief Main loop routine to measure HID latency.
 */
void run(void);

#endif /* HID_LATENCY_TEST_H */
