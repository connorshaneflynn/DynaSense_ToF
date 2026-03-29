/**
 * @file vl53l5cx_ranging_twosensors.c
 * @brief Dual-sensor VL53L5CX ToF ranging firmware.
 *
 * This firmware drives two VL53L5CX time-of-flight sensors simultaneously.
 * At startup, it assigns unique I2C addresses to each sensor (0x10 and 0x52)
 * using the LPn pins for address isolation. Both sensors are configured with
 * identical ranging parameters and stream measurements over USB CDC with
 * sensor IDs (1 or 2) to distinguish the data source.
 *
 * To compile this firmware, set the following in CMakeLists.txt:
 *     set(MODE "ranging_double")
 *
 * Sensor configuration (resolution, frequency, etc.) can be modified in
 * vl53l5cx_common.h.
 */

#include "vl53l5cx_ranging_twosensors.h"
#include "vl53l5cx_common.h"

#include "main.h"

#include <stdio.h>

#include "vl53l5cx_api.h"

/* Sensor structures */
static VL53L5CX_Configuration Dev1, Dev2;
static VL53L5CX_ResultsData Results1, Results2;
static uint8_t status, isReady1, isReady2;

/* I2C addresses for two sensors */
static const uint16_t addr1 = 0x10;
static const uint16_t addr2 = 0x52;

static void set_I2C_addresses(void)
{
    /* Changes I2C addresses to use two sensors at once. */

    /* Set LPn_2 low to write address to sensor 1 */
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_RESET);
    status = vl53l5cx_set_i2c_address(&Dev1, addr1);
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_SET);

    /* Set LPn_1 low to write address to sensor 2 */
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_RESET);
    status = vl53l5cx_set_i2c_address(&Dev2, addr2);
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_SET);
}

/* This is the code that is called once at system start */
uint8_t initialize(void)
{
    /* Turn on LED to show active */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    /* Activate all I2C communication */
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_SET);

    /* Set individual I2C addresses */
    Dev1.platform.address = 0x52;  /* default address */
    Dev2.platform.address = 0x52;
    set_I2C_addresses();

    uint8_t status1 = vl53l5cx_init(&Dev1);
    uint8_t status2 = vl53l5cx_init(&Dev2);

    if (status1 && status2)
    {
        sprintf(msg, "Failed to Init Sensors\r\n");
        uartSend();
        ledMode = VERYFAST;

        sprintf(msg, "Scanning I2C bus\r\n");
        uartSend();
        i2c_scan();
        return status;
    }
    else if (status1 || status2)
    {
        sprintf(msg, "Failed to init both sensors. Running with single sensor.\r\n");
        uartSend();
        ledMode = FAST;

        sprintf(msg, "Scanning I2C bus\r\n");
        uartSend();
        i2c_scan();
    }
    else
    {
        ledMode = SLOW;
    }

    sprintf(msg, "VL53L5CX Ready (Version: %s)\n", VL53L5CX_API_REVISION);
    uartSend();

    /* Set up sensor 1 */
    if (!status1)
    {
        status = vl53l5cx_set_target_order(&Dev1, TARGET_ORDER);
        status = vl53l5cx_set_resolution(&Dev1, RESOLUTION);
        status = vl53l5cx_set_ranging_frequency_hz(&Dev1, FREQUENCY);
        status = vl53l5cx_set_ranging_mode(&Dev1, RANGING_MODE);
        if (RANGING_MODE == VL53L5CX_RANGING_MODE_AUTONOMOUS) {
            status = vl53l5cx_set_integration_time_ms(&Dev1, INTEGRATION_TIME);
        }
        status = vl53l5cx_set_sharpener_percent(&Dev1, SHARPENER);
        status = vl53l5cx_start_ranging(&Dev1);
    }

    /* Set up sensor 2 */
    if (!status2)
    {
        status = vl53l5cx_set_target_order(&Dev2, TARGET_ORDER);
        status = vl53l5cx_set_resolution(&Dev2, RESOLUTION);
        status = vl53l5cx_set_ranging_frequency_hz(&Dev2, FREQUENCY);
        status = vl53l5cx_set_ranging_mode(&Dev2, RANGING_MODE);
        if (RANGING_MODE == VL53L5CX_RANGING_MODE_AUTONOMOUS) {
            status = vl53l5cx_set_integration_time_ms(&Dev2, INTEGRATION_TIME);
        }
        status = vl53l5cx_set_sharpener_percent(&Dev2, SHARPENER);
        status = vl53l5cx_start_ranging(&Dev2);
    }

    return 0;
}

/* This is the code that is called in each system loop */
void run(void)
{
    updateLED();

    status = vl53l5cx_check_data_ready(&Dev1, &isReady1);
    if (isReady1)
    {
        vl53l5cx_get_ranging_data(&Dev1, &Results1);
        send_measurements(1, &Results1);
    }

    status = vl53l5cx_check_data_ready(&Dev2, &isReady2);
    if (isReady2)
    {
        vl53l5cx_get_ranging_data(&Dev2, &Results2);
        send_measurements(2, &Results2);
    }
}
