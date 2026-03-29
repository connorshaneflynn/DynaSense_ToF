/**
 * @file vl53l5cx_ranging.c
 * @brief Single-sensor VL53L5CX ToF ranging firmware.
 *
 * This firmware drives a single VL53L5CX time-of-flight sensor. It initializes
 * the sensor with the default I2C address (0x52), configures ranging parameters
 * (resolution, frequency, mode), and continuously streams distance measurements
 * over USB CDC.
 *
 * To compile this firmware, set the following in CMakeLists.txt:
 *     set(MODE "ranging_single")
 *
 * Sensor configuration (resolution, frequency, etc.) can be modified in
 * vl53l5cx_common.h.
 */

#include "vl53l5cx_ranging.h"
#include "vl53l5cx_common.h"

#include "main.h"

#include <stdio.h>

#include "vl53l5cx_api.h"

/* Sensor structures */
static VL53L5CX_Configuration Dev;
static VL53L5CX_ResultsData Results;
static uint8_t status, isReady;

/* This is the code that is called once at system start */
uint8_t initialize(void)
{
    /* Turn on LED to show active */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    /* Activate I2C communication */
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_SET);

    /* Initialize sensor with default address */
    Dev.platform.address = 0x52;

    status = vl53l5cx_init(&Dev);

    if (status)
    {
        sprintf(msg, "Failed to Init Sensor\r\n");
        uartSend();
        ledMode = VERYFAST;

        sprintf(msg, "Scanning I2C bus\r\n");
        uartSend();
        i2c_scan();
        return status;
    }

    ledMode = SLOW;

    sprintf(msg, "VL53L5CX Ready (Version: %s)\n", VL53L5CX_API_REVISION);
    uartSend();

    /* Configure sensor */
    status = vl53l5cx_set_target_order(&Dev, TARGET_ORDER);
    status = vl53l5cx_set_resolution(&Dev, RESOLUTION);
    status = vl53l5cx_set_ranging_frequency_hz(&Dev, FREQUENCY);
    status = vl53l5cx_set_ranging_mode(&Dev, RANGING_MODE);
    if (RANGING_MODE == VL53L5CX_RANGING_MODE_AUTONOMOUS) {
        status = vl53l5cx_set_integration_time_ms(&Dev, INTEGRATION_TIME);
    }
    status = vl53l5cx_set_sharpener_percent(&Dev, SHARPENER);
    status = vl53l5cx_start_ranging(&Dev);

    return 0;
}

/* This is the code that is called in each system loop */
void run(void)
{
    updateLED();

    status = vl53l5cx_check_data_ready(&Dev, &isReady);
    if (isReady)
    {
        vl53l5cx_get_ranging_data(&Dev, &Results);
        send_measurements(1, &Results);
    }
}
