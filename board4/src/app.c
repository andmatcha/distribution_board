#include "app.h"

#include "modules/can_control.h"
#include "modules/dc_motor.h"
#include "modules/encoder.h"
#include "modules/led.h"
#include "modules/servo.h"

#include <stdbool.h>
#include <stdio.h>

extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

#define ENCODER_CAN_SEND_INTERVAL_MS 20U
#define ENCODER_REQUEST_TIMEOUT_MS 10U

static volatile bool encoder_request_pending = false;
static volatile bool encoder_request_in_flight = false;
static uint32_t encoder_request_start_tick = 0U;
static bool encoder_has_last_position = false;
static uint16_t encoder_last_position = 0U;
static uint32_t encoder_last_can_queue_tick = 0U;
static bool encoder_has_sent_can = false;

static void start_encoder_request(void)
{
  encoder_request_start_tick = HAL_GetTick();
  encoder_request_in_flight = true;
  encoder_request_position();
}

static void process_encoder_request(void)
{
  bool should_request;
  bool in_flight;

  __disable_irq();
  should_request = encoder_request_pending;
  encoder_request_pending = false;
  in_flight = encoder_request_in_flight;
  __enable_irq();

  if (should_request ||
      (in_flight && (HAL_GetTick() - encoder_request_start_tick) >= ENCODER_REQUEST_TIMEOUT_MS)) {
    start_encoder_request();
  }
}

static void process_encoder_can_publish(void)
{
  uint32_t now_tick;

  if (!encoder_has_last_position) {
    return;
  }

  now_tick = HAL_GetTick();
  if (encoder_has_sent_can &&
      (now_tick - encoder_last_can_queue_tick) < ENCODER_CAN_SEND_INTERVAL_MS) {
    return;
  }

  if (can_control_enqueue_encoder_position(encoder_last_position)) {
    encoder_last_can_queue_tick = now_tick;
    encoder_has_sent_can = true;
  }
}

void init(void)
{
  led_set(LED_COLOR_RED, LED_STATE_OFF);
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);

  servo_init(&htim2);
  dc_motor_init(&htim3);
  can_control_init(&hcan, &htim3);
  encoder_init(&huart1);
  encoder_request_pending = false;
  encoder_request_in_flight = false;
  encoder_has_last_position = false;
  encoder_has_sent_can = false;
  encoder_last_can_queue_tick = 0U;
  start_encoder_request();
}

void poll(void)
{
  uint16_t position = 0;

  if (encoder_get_position(&position)) {
    printf("Encoder Data: %u\n", position);
    encoder_last_position = position;
    encoder_has_last_position = true;
  }

  process_encoder_can_publish();
  can_control_process_tx();
  process_encoder_request();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    encoder_rx_complete_callback(huart);
    encoder_request_in_flight = false;
    encoder_request_pending = true;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    encoder_error_callback(huart);
    encoder_request_in_flight = false;
    encoder_request_pending = true;
  }
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
