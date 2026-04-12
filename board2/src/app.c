#include "app.h"

#include "modules/can_control.h"
#include "modules/encoder.h"
#include "modules/led.h"

#include <stdio.h>

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

void init(void)
{
  led_set(LED_COLOR_RED, LED_STATE_OFF);
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);

  can_control_init(&hcan);
  encoder_init(&huart1);
  encoder_request_position();
}

void poll(void)
{
  uint16_t position = 0;

  if (encoder_get_position(&position)) {
    printf("Encoder Data: %u\n", position);
    if (!can_control_enqueue_encoder_position(position)) {
      Error_Handler();
    }
  }

  can_control_process_tx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  encoder_rx_complete_callback(huart);
  if (huart == &huart1) {
    encoder_request_position();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  encoder_error_callback(huart);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan_handle)
{
  (void)hcan_handle;
  Error_Handler();
}
