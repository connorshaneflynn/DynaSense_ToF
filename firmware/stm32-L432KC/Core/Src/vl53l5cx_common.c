#include "vl53l5cx_common.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdio.h>
#include <usart.h>
#include <i2c.h>

/* CDC timeout in ms */
#define CDC_TX_TIMEOUT_MS 3

/* Frame header for measurements */
#define FRAME_HEADER_LEN  2
static const uint8_t FRAME_HEADER[] = {0xAA, 0x55};

/* Global variables */
char msg[512];
LedMode ledMode = OFF;

void uartSend(void)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

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

void i2c_scan(void)
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

uint8_t CDC_Transmit_WithTimeout(uint8_t *buf, uint16_t len)
{
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS(buf, len) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start) > CDC_TX_TIMEOUT_MS)
        {
            return USBD_FAIL;
        }
    }
    return USBD_OK;
}

void send_measurements(uint8_t sensor_ID, VL53L5CX_ResultsData *results)
{
  uint8_t buffer[FRAME_HEADER_LEN + 1 + 2*RESOLUTION + RESOLUTION];
  int idx = 0;

  /* Start Header */
  for (size_t i = 0; i < FRAME_HEADER_LEN; i++)
  {
    buffer[idx++] = FRAME_HEADER[i];
  }

  /* ID Header */
  buffer[idx++] = sensor_ID;

  /* Distances */
  for (int i = 0; i < RESOLUTION; i++)
  {
    uint16_t dist = (uint16_t) results->distance_mm[i];
    buffer[idx++] = dist & 0xFF;
    buffer[idx++] = (dist >> 8) & 0xFF;
  }

  /* Statuses */
  for (int i = 0; i < RESOLUTION; i++)
  {
    buffer[idx++] = results->target_status[i];
  }

  /* Send via USB */
  if (COM_USB)
  {
    CDC_Transmit_WithTimeout(buffer, sizeof(buffer));
  }

  /* Send via UART */
  if (COM_UART)
  {
    HAL_UART_Transmit(&huart2, buffer, idx, HAL_MAX_DELAY);
  }
}
