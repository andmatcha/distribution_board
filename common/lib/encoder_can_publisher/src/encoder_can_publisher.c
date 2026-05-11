/* Shared encoder polling and CAN publishing loop. */
#include "modules/encoder_can_publisher.h"
#include "board_config.h"
#include "modules/can_control.h"
#include "modules/encoder.h"

#include <stdio.h>

#ifndef BOARD_ENCODER_CAN_SEND_INTERVAL_MS
#define BOARD_ENCODER_CAN_SEND_INTERVAL_MS 20U
#endif

#ifndef BOARD_ENCODER_REQUEST_TIMEOUT_MS
#define BOARD_ENCODER_REQUEST_TIMEOUT_MS 10U
#endif

static UART_HandleTypeDef *encoder_can_publisher_huart = NULL;
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
      (in_flight && (HAL_GetTick() - encoder_request_start_tick) >= BOARD_ENCODER_REQUEST_TIMEOUT_MS)) {
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
      (now_tick - encoder_last_can_queue_tick) < BOARD_ENCODER_CAN_SEND_INTERVAL_MS) {
    return;
  }

  if (can_control_enqueue_encoder_position(encoder_last_position)) {
    encoder_last_can_queue_tick = now_tick;
    encoder_has_sent_can = true;
  }
}

void encoder_can_publisher_init(UART_HandleTypeDef *huart)
{
  encoder_can_publisher_huart = huart;
  encoder_init(huart);
  encoder_request_pending = false;
  encoder_request_in_flight = false;
  encoder_has_last_position = false;
  encoder_has_sent_can = false;
  encoder_last_can_queue_tick = 0U;
  start_encoder_request();
}

void encoder_can_publisher_process(void)
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

bool encoder_can_publisher_uart_rx_complete_callback(UART_HandleTypeDef *huart)
{
  if (encoder_can_publisher_huart == NULL ||
      huart->Instance != encoder_can_publisher_huart->Instance) {
    return false;
  }

  encoder_rx_complete_callback(huart);
  encoder_request_in_flight = false;
  encoder_request_pending = true;
  return true;
}

bool encoder_can_publisher_uart_error_callback(UART_HandleTypeDef *huart)
{
  if (encoder_can_publisher_huart == NULL ||
      huart->Instance != encoder_can_publisher_huart->Instance) {
    return false;
  }

  encoder_error_callback(huart);
  encoder_request_in_flight = false;
  encoder_request_pending = true;
  return true;
}
