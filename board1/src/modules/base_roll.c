#include "modules/base_roll.h"

#include "modules/base_can_scheduler.h"
#include "modules/encoder.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct
{
  bool has_last_can_position;
  uint16_t last_can_position;
  uint32_t last_can_send_tick;
  uint32_t last_log_tick;
} BaseRollState;

typedef enum
{
  BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION = 0,
  BASE_ROLL_ENCODER_PHASE_WAIT_POSITION
} BaseRollEncoderPhase;

static UART_HandleTypeDef *base_roll_encoder_uart = NULL;
static EncoderDevice base_roll_encoder_device;
static BaseRollState base_roll_state = {0};
static BaseRollEncoderPhase base_roll_encoder_phase = BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION;
static uint32_t base_roll_encoder_phase_start_tick = 0U;

#define BASE_ROLL_ENCODER_DE_PORT GPIOA
#define BASE_ROLL_ENCODER_DE_PIN GPIO_PIN_4
#define BASE_ROLL_CAN_STD_ID 0x301U
#define BASE_ROLL_CAN_KEEPALIVE_INTERVAL_MS 200U
#define BASE_ROLL_LOG_INTERVAL_MS 100U
#define BASE_ROLL_ENCODER_RESPONSE_TIMEOUT_MS 10U

static void base_roll_initialize_runtime(void);
static bool base_roll_read_position(uint16_t *position);
static bool base_roll_should_send_can(uint16_t position, uint32_t now_tick);
static bool base_roll_queue_position_can(uint16_t position);
static void base_roll_publish_position(uint16_t position);

static void base_roll_initialize_runtime(void)
{
  if (base_roll_encoder_uart == NULL) {
    return;
  }

  if (!encoder_device_init(&base_roll_encoder_device,
                           base_roll_encoder_uart,
                           BASE_ROLL_ENCODER_DE_PORT,
                           BASE_ROLL_ENCODER_DE_PIN)) {
    return;
  }

  printf("Base Roll initialization complete.\n");
}

static bool base_roll_read_position(uint16_t *position)
{
  uint32_t now_tick;
  uint8_t step_guard = 0U;

  if (position == NULL) {
    return false;
  }

  while (step_guard < 2U) {
    now_tick = HAL_GetTick();
    step_guard++;

    switch (base_roll_encoder_phase) {
      case BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION:
        encoder_device_request_data(&base_roll_encoder_device, ENCODER_CMD_POSITION);
        base_roll_encoder_phase_start_tick = now_tick;
        base_roll_encoder_phase = BASE_ROLL_ENCODER_PHASE_WAIT_POSITION;
        break;

      case BASE_ROLL_ENCODER_PHASE_WAIT_POSITION:
        if (encoder_device_get_position(&base_roll_encoder_device, position)) {
          base_roll_encoder_phase = BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION;
          return true;
        }
        if ((now_tick - base_roll_encoder_phase_start_tick) >= BASE_ROLL_ENCODER_RESPONSE_TIMEOUT_MS) {
          base_roll_encoder_phase = BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION;
          break;
        }
        return false;

      default:
        base_roll_encoder_phase = BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION;
        break;
    }
  }

  return false;
}

static bool base_roll_should_send_can(uint16_t position, uint32_t now_tick)
{
  if (!base_roll_state.has_last_can_position) {
    return true;
  }

  if (position != base_roll_state.last_can_position) {
    return true;
  }

  if ((now_tick - base_roll_state.last_can_send_tick) >= BASE_ROLL_CAN_KEEPALIVE_INTERVAL_MS) {
    return true;
  }

  return false;
}

static bool base_roll_queue_position_can(uint16_t position)
{
  uint8_t data[2];

  data[0] = (uint8_t)((position >> 8) & 0xFFU);
  data[1] = (uint8_t)(position & 0xFFU);

  return base_can_scheduler_stage(BASE_CAN_CHANNEL_ROLL,
                                  BASE_ROLL_CAN_STD_ID,
                                  data,
                                  2U);
}

static void base_roll_publish_position(uint16_t position)
{
  uint32_t now_tick;

  now_tick = HAL_GetTick();
  if (base_roll_should_send_can(position, now_tick) &&
      base_roll_queue_position_can(position)) {
    base_roll_state.last_can_position = position;
    base_roll_state.last_can_send_tick = now_tick;
    base_roll_state.has_last_can_position = true;
  }

  if ((now_tick - base_roll_state.last_log_tick) >= BASE_ROLL_LOG_INTERVAL_MS) {
    base_roll_state.last_log_tick = now_tick;
    printf("Base Roll Encoder: %u\n", position);
  }
}

void base_roll_init(UART_HandleTypeDef *encoder_uart)
{
  base_roll_encoder_uart = encoder_uart;
  base_roll_state = (BaseRollState){0};
  base_roll_encoder_phase = BASE_ROLL_ENCODER_PHASE_REQUEST_POSITION;
  base_roll_encoder_phase_start_tick = 0U;
  base_roll_initialize_runtime();
}

void base_roll_process(void)
{
  uint16_t position = 0U;

  if (base_roll_read_position(&position)) {
    base_roll_publish_position(position);
  }
}
