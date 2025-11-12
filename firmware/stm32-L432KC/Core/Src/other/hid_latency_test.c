#include "hid_latency_test.h"

#include "main.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
// #include "usb_device.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// #include "usbd_def.h"
#include <usart.h>

#include "usb_app.h"

#include "usbd_customhid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

// data buffer
static uint8_t buffer[51];

// previous reflection time
static uint32_t delta_ms = 0;

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

void update_timing_data(uint32_t time_ms)
{
  // Store 32-bit value in first 4 bytes (little-endian)
  buffer[0] = (uint8_t)(time_ms & 0xFF);
  buffer[1] = (uint8_t)((time_ms >> 8) & 0xFF);
  buffer[2] = (uint8_t)((time_ms >> 16) & 0xFF);
  buffer[3] = (uint8_t)((time_ms >> 24) & 0xFF);
}


/* This is the code that is called once at system start*/
uint8_t initialize(void)
{
    /* USER CODE BEGIN 2*/
    // Turn on LED to show active
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    
    memset(buffer, 0xAA, sizeof(buffer));
    
    return 0;
}

/* This is the code that is called in each system loop*/
void run(void)
{
    /* USER CODE BEGIN 3 */
    ledMode = VERYFAST;
    updateLED();

    uint32_t start, end;

    start = HAL_GetTick();
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));

    // wait for return
    while (1)
    {
      if (hid_data_ready)
      {
        break;
      }
    }
    end = HAL_GetTick();
    
    // reset hid receive state
    hid_data_ready = 0;

    delta_ms = start - end;
    update_timing_data(delta_ms / 2);  // writes to buffer

    ledMode = OFF;
    updateLED();
    HAL_Delay(500);
}
