#include "app.h"

#include "modules/can_control.h"
#include "modules/dc_motor.h"
#include "modules/encoder.h"
#include "modules/led.h"
#include "modules/servo.h"

#include "debug_log.h"

extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

void init(void)
{
  led_set(LED_COLOR_RED, LED_STATE_OFF);
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);

  servo_init(&htim2);
  dc_motor_init(&htim3);
  can_control_init(&hcan, &htim3);
  encoder_init(&huart1);
  encoder_request_position();
}

void poll(void)
{
  uint16_t position = 0;

  if (encoder_get_position(&position)) {
    LOG("Encoder Data: %u\n", position);
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_handle)
{
  can_control_rx_callback(hcan_handle);
}
