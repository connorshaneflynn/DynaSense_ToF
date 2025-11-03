#include "vl53l5cx_ranging.h"

#include "i2c.h"
#include "main.h"
#include "usart.h"
#include "usb_device.h"


#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "vl53l5cx_api.h"
// #include "../Drivers/VL53L4CD/Inc/VL53L4CD_api.h"

/* USER CODE BEGIN 0 */

/* Sensor structures */
static VL53L5CX_Configuration Dev;
static VL53L5CX_ResultsData Results;
static uint8_t status, isReady;

// Sensor setup
#define RESOLUTION 16                               // NxN
#define FREQUENCY 60                                // hz

#define INTEGRATION_TIME 10                         // ms
#define TARGET_ORDER VL53L5CX_TARGET_ORDER_CLOSEST  // VL53L5CX_TARGET_ORDER_CLOSEST or VL53L5CX_TARGET_ORDER_STRONGEST

// communication
#define COM_UART false
#define COM_USB true


// general UART buffer
char msg[512];
void uartSend(void)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

LedMode ledMode = OFF;
void updateLED(void)
{
  static uint32_t previousMillis = 0;
  static uint8_t ledState = 0;
  uint32_t interval = 0;

  switch(ledMode)
  {
    case OFF: interval = 0; HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); return;
    case SLOW: interval = 1000; break;
    case FAST: interval = 300; break;
    case VERYFAST: interval = 100; break;
  }

  uint32_t currentMillis = HAL_GetTick();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    ledState = 1 - ledState;
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}


void i2c_scan()
{
  char msg[64];

  sprintf(msg, "\r\nStarting I2C scan\r\n\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  for (uint8_t addr = 1; addr < 128; addr++)
  {
    sprintf(msg, "%d  ", addr);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)addr << 1, 3, 5) == HAL_OK)
    {
      sprintf(msg, "\r\nDevice Found at 0x%02X\r\n", addr);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }
  sprintf(msg, "\r\nScan Finished\r\n\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void send_measurements(uint8_t sensor_ID, VL53L5CX_ResultsData *Results)
{
  // all measurements are sent as a single frame over serial
  // distance is 2 bytes, status is one bit and will be packed into bytes
  // uses start bytes
  uint8_t buffer[2 + 1 + 2*RESOLUTION + RESOLUTION/8];  // start, distances, statuses
  int idx = 0;  // buffer index

  // Start Header
  buffer[idx++] = 0xAA;
  buffer[idx++] = 0x55;

  // ID Header
  buffer[idx++] = sensor_ID;

  // Distances
  for (int i = 0; i < RESOLUTION; i++)
  {
    buffer[idx++] = Results->distance_mm[i] & 0xFF;         // LSB
    buffer[idx++] = (Results->distance_mm[i] >> 8) & 0xFF;  // MSB
  }

  // Status Packing
  uint8_t status_bytes = 0;
  for (int i = 0; i < RESOLUTION; i++)
  {
    // status 5 is valid measurment
    // ToDo: check if other statuses also okay
    if (Results->target_status[i] == 5)
    {
      // set corresponding bit in byte to 1
      status_bytes |= (1 << (i % 8));
    }
    if ((i % 8) == 7 || i == RESOLUTION - 1)
    {
      // write full byte into buffer and reset
      buffer[idx++] = status_bytes;
      status_bytes = 0;
    }
  }

  // Send
  uint8_t status_transmit;
  // USB
  if (COM_USB)
  {
    status_transmit = CDC_Transmit_FS(buffer, sizeof(buffer));
  }
  // UART
  if (COM_UART)
  {
  HAL_UART_Transmit(&huart2, buffer, idx, HAL_MAX_DELAY);
  }
}


/* This is the code that is called once at system start*/
uint8_t initialize(void)
{
    /* USER CODE BEGIN 2*/
    // Activate I2C communication on sensor 1
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_RESET);

    Dev.platform.address = 0x52;
    // change address
    // status = vl53l5cx_set_i2c_address(&Dev, 0x20);
        

    status = vl53l5cx_init(&Dev);
    // status = 0 if success
    if (status)
    {
        sprintf(msg, "Failed to Init Sensor\r\n");
        uartSend();
        ledMode = FAST;

        sprintf(msg, "Scanning I2C bus\r\n");
        uartSend();

        i2c_scan();

        return status;
    }
    sprintf(msg, "VL53L5CX Ready (Version: %s)\n", VL53L5CX_API_REVISION);
    uartSend();
    ledMode = SLOW;

    status = vl53l5cx_set_target_order(&Dev, VL53L5CX_TARGET_ORDER_CLOSEST);  // or: VL53L5CX_TARGET_ORDER_STRONGEST
    status = vl53l5cx_set_resolution(&Dev, RESOLUTION);
    status = vl53l5cx_set_ranging_frequency_hz(&Dev, FREQUENCY);
    status = vl53l5cx_start_ranging(&Dev);
    status = vl53l5cx_set_integration_time_ms(&Dev, 10);

    uint8_t res, freq;
    uint32_t int_time;
    status = vl53l5cx_get_resolution(&Dev, &res);
    status = vl53l5cx_get_ranging_frequency_hz(&Dev, &freq);
    status = vl53l5cx_get_integration_time_ms(&Dev, &int_time);

    return 0;
}

/* This is the code that is called in each system loop*/
void run(void)
{
    /* USER CODE BEGIN 3 */

    updateLED();

    status = vl53l5cx_check_data_ready(&Dev, &isReady);
    if (isReady)
    {
      uint32_t start_millis = HAL_GetTick();
      vl53l5cx_get_ranging_data(&Dev, &Results);

      uint32_t mid_millis = HAL_GetTick() - start_millis;

      send_measurements(1, &Results);

      uint32_t end_millis = HAL_GetTick() - start_millis;
      uint8_t res = 1 + 2;
    }

}
