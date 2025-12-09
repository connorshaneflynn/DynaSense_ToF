#include "vl53l5cx_ranging_twosensors.h"

#include "main.h"
#include "usb_device.h"


#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "vl53l5cx_api.h"
#include <usart.h>
#include <i2c.h>
// #include "../Drivers/VL53L4CD/Inc/VL53L4CD_api.h"

/* USER CODE BEGIN 0 */

/* Sensor structures */
static VL53L5CX_Configuration Dev1, Dev2;
static VL53L5CX_ResultsData Results1, Results2;
static uint8_t status, isReady1, isReady2;

const uint16_t addr1 = 0x10;
const uint16_t addr2 = 0x52;

// Sensor setu
#define RESOLUTION 16                               // NxN
#define FREQUENCY 60                                // hz

#define INTEGRATION_TIME 3                          // ms
#define TARGET_ORDER VL53L5CX_TARGET_ORDER_CLOSEST  // VL53L5CX_TARGET_ORDER_CLOSEST or VL53L5CX_TARGET_ORDER_STRONGEST

// communication
#define COM_UART false
#define COM_USB true

#define CDC_TX_TIMEOUT_MS  3                        // wait #ms before skipping data frame if host USB is busy
#define FRAME_HEADER_LEN 2
static const uint8_t FRAME_HEADER[] = {0xAA, 0x55};

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


void set_I2C_addresses()
{
    /* Changes I2C addresses to use two sensors at once.*/

    // Set LPn_2 low to write address to sensor 1
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_RESET);
    status = vl53l5cx_set_i2c_address(&Dev1, addr1);
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_SET);
    // Set LPn_1 low to write address to sensor 2 (optional)
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_RESET);
    status = vl53l5cx_set_i2c_address(&Dev2, addr2);
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_SET);
}


uint8_t CDC_Transmit_WithTimeout(uint8_t *buf, uint16_t len)
{
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS(buf, len) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start) > CDC_TX_TIMEOUT_MS)
        {
            return USBD_FAIL;   // custom return code
        }
    }
    return USBD_OK;
}

void send_measurements(uint8_t sensor_ID, VL53L5CX_ResultsData *results) // int16_t *distances, uint8_t *statuses)
{
  // all measurements are sent as a single frame over serial
  // distance is 2 bytes, status is one bit and will be packed into bytes
  // uses start bytes
  uint8_t buffer[2 + 1 + 2*RESOLUTION + RESOLUTION];  // start, ID, distances, statuses
  int idx = 0;  // buffer index

  // Start Header
  for (size_t i = 0; i < FRAME_HEADER_LEN; i++)
  {
    buffer[idx++] = FRAME_HEADER[i];
  }

  // ID Header
  buffer[idx++] = sensor_ID;

  // Distances
  for (int i = 0; i < RESOLUTION; i++)
  {
    uint16_t dist = (uint16_t) results->distance_mm[i];
    buffer[idx++] = dist & 0xFF;         // LSB
    buffer[idx++] = (dist >> 8) & 0xFF;  // MSB
  }

//   // Status Packing
//   uint8_t status_bytes = 0;
//   for (int i = 0; i < RESOLUTION; i++)
//   {
//     // status 5 is valid measurment
//     // ToDo: check if other statuses also okay
//     if (results->target_status[i] == 5)
//     {
//       // set corresponding bit in byte to 1
//       status_bytes |= (1 << (i % 8));
//     }
//     if ((i % 8) == 7 || i == RESOLUTION - 1)
//     {
//       // write full byte into buffer and reset
//       buffer[idx++] = status_bytes;
//       status_bytes = 0;
//     }
//   }

  // Statuses
  for (int i = 0; i < RESOLUTION; i++)
  {
    buffer[idx++] = results->target_status[i];
  }

  // Send
  uint8_t status_transmit;
  // USB
  if (COM_USB)
  {
    status_transmit = CDC_Transmit_WithTimeout(buffer, sizeof(buffer));
  }
  int temp = 5;
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
    // Turn on LED to show active
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    // Activate all I2C communication
    HAL_GPIO_WritePin(LPn_1_GPIO_Port, LPn_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LPn_2_GPIO_Port, LPn_2_Pin, GPIO_PIN_SET);

    // Set individual I2C addresses
    Dev1.platform.address = 0x52;  // default address
    Dev2.platform.address = 0x52;
    set_I2C_addresses();
    // i2c_scan();

    uint8_t status1 = vl53l5cx_init(&Dev1);
    uint8_t status2 = vl53l5cx_init(&Dev2);
    // status = 0 if success
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

    // set up sensor 1
    if (!status1)
    {
    status = vl53l5cx_set_target_order(&Dev1, TARGET_ORDER);
    status = vl53l5cx_set_resolution(&Dev1, RESOLUTION);
    status = vl53l5cx_set_ranging_frequency_hz(&Dev1, FREQUENCY);
    status = vl53l5cx_set_ranging_mode(&Dev1, VL53L5CX_RANGING_MODE_CONTINUOUS); // or: VL53L5CX_RANGING_MODE_AUTONOMOUS
    // status = vl53l5cx_set_integration_time_ms(&Dev1, INTEGRATION_TIME);
    status = vl53l5cx_set_sharpener_percent(&Dev1, 14);  // ST default is 14
    status = vl53l5cx_start_ranging(&Dev1);
    }
    
    // set up sensor 2
    if (!status2)
    {
    status = vl53l5cx_set_target_order(&Dev2, TARGET_ORDER);
    status = vl53l5cx_set_resolution(&Dev2, RESOLUTION);
    status = vl53l5cx_set_ranging_frequency_hz(&Dev2, FREQUENCY);
    status = vl53l5cx_set_ranging_mode(&Dev2, VL53L5CX_RANGING_MODE_CONTINUOUS); // or: VL53L5CX_RANGING_MODE_AUTONOMOUS
    // status = vl53l5cx_set_integration_time_ms(&Dev2, INTEGRATION_TIME);
    status = vl53l5cx_set_sharpener_percent(&Dev2, 14);
    status = vl53l5cx_start_ranging(&Dev2);
    }
    
    return 0;
}

/* This is the code that is called in each system loop*/
void run(void)
{
    /* USER CODE BEGIN 3 */

    updateLED();

    status = vl53l5cx_check_data_ready(&Dev1, &isReady1);
    if (isReady1)
    {
      vl53l5cx_get_ranging_data(&Dev1, &Results1);

      // if (Results1.distance_mm[5] < 250 && Results1.target_status[5] == 5)
      // {
      //   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      // }
      // else
      // {
      //   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      // }

      send_measurements(1, &Results1);
    }
    status = vl53l5cx_check_data_ready(&Dev2, &isReady2);
    if (isReady2)
    {
      vl53l5cx_get_ranging_data(&Dev2, &Results2);

      // if (Results2.distance_mm[5] < 250 && Results2.target_status[5] == 5)
      // {
      //   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
      // }
      // else
      // {
      //   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      // }

      send_measurements(2, &Results2);
    }
}
