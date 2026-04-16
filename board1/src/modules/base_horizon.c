#include "modules/base_horizon.h"

#include "modules/base_can_scheduler.h"
#include "modules/encoder.h"
#include "modules/led.h"
#include "debug_log.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t pos;
  int16_t turns;
  int32_t total_counts;
  int16_t distance_tenths_mm;
} BaseHorizonSample;

typedef struct
{
  int16_t last_distance_tenths_mm;
  bool is_first_reading;
  bool runtime_ready;
  bool has_last_total_counts;
  int32_t last_total_counts;
  bool has_origin_position;
  uint16_t origin_position;
  bool has_pending_transition;
  int16_t pending_distance_tenths_mm;
  uint8_t pending_transition_count;
  bool has_last_can_distance;
  int16_t last_can_distance_tenths_mm;
  uint32_t last_can_send_tick;
  uint32_t last_encoder_log_tick;
  uint32_t last_error_log_tick;
  uint32_t next_origin_retry_tick;
} BaseHorizonState;

static UART_HandleTypeDef *base_horizon_encoder_uart = NULL;
static EncoderDevice base_horizon_encoder_device;
static BaseHorizonState base_horizon_state = {0};

#define BASE_HORIZON_ENCODER_DE_PORT GPIOA
#define BASE_HORIZON_ENCODER_DE_PIN GPIO_PIN_8
#define BASE_HORIZON_CAN_STD_ID 0x300U
#define BASE_HORIZON_CAN_KEEPALIVE_INTERVAL_MS 200U
#define BASE_HORIZON_LOG_INTERVAL_MS 100U
#define BASE_HORIZON_ERROR_LOG_INTERVAL_MS 200U
#define BASE_HORIZON_ENCODER_RESPONSE_TIMEOUT_MS 10U
#define BASE_HORIZON_ENCODER_RESET_SETTLE_MS 20U
#define BASE_HORIZON_ORIGIN_RETRY_INTERVAL_MS 100U
#define BASE_HORIZON_DIRECT_ACCEPT_TENTHS_MM 10
#define BASE_HORIZON_PENDING_CONFIRMATIONS 3U
#define BASE_HORIZON_PENDING_TOLERANCE_TENTHS_MM 10

static int32_t base_horizon_unwrap_counts(uint16_t pos, int32_t last_total_counts);
static bool base_horizon_queue_distance_can(int16_t distance_tenths_mm);
static bool base_horizon_should_log_error(uint32_t now_tick);
static bool base_horizon_initialize_runtime(void);
static bool base_horizon_capture_origin_position(void);
static bool base_horizon_prepare_origin_reference(void);
static uint16_t base_horizon_shift_position(uint16_t pos);
static bool base_horizon_build_sample(BaseHorizonSample *sample, uint16_t pos, int16_t turns);
static bool base_horizon_read_sample(BaseHorizonSample *sample);
static bool base_horizon_validate_distance(int16_t distance_tenths_mm);
static bool base_horizon_should_send_can(int16_t distance_tenths_mm, uint32_t now_tick);
static void base_horizon_publish_sample(const BaseHorizonSample *sample);

static int32_t base_horizon_unwrap_counts(uint16_t pos, int32_t last_total_counts)
{
  int32_t last_pos = last_total_counts & 0x3FFF;
  int32_t delta = (int32_t)(pos & 0x3FFFU) - last_pos;

  if (delta > 8192) {
    delta -= 16384;
  } else if (delta < -8192) {
    delta += 16384;
  }

  return last_total_counts + delta;
}

static bool base_horizon_queue_distance_can(int16_t distance_tenths_mm)
{
  uint8_t data[2];

  data[0] = (uint8_t)((distance_tenths_mm >> 8) & 0xFF);
  data[1] = (uint8_t)(distance_tenths_mm & 0xFF);

  return base_can_scheduler_stage(BASE_CAN_CHANNEL_HORIZON,
                                  BASE_HORIZON_CAN_STD_ID,
                                  data,
                                  2U);
}

static bool base_horizon_should_log_error(uint32_t now_tick)
{
  if ((now_tick - base_horizon_state.last_error_log_tick) < BASE_HORIZON_ERROR_LOG_INTERVAL_MS) {
    return false;
  }

  base_horizon_state.last_error_log_tick = now_tick;
  return true;
}

static bool base_horizon_initialize_runtime(void)
{
  if (base_horizon_encoder_uart == NULL) {
    return false;
  }

  if (!encoder_device_init(&base_horizon_encoder_device,
                           base_horizon_encoder_uart,
                           BASE_HORIZON_ENCODER_DE_PORT,
                           BASE_HORIZON_ENCODER_DE_PIN)) {
    LOG("Base Horizon encoder init failed.\n");
    return false;
  }

  led_set(LED_COLOR_RED, LED_STATE_ON);
  led_set(LED_COLOR_YELLOW, LED_STATE_ON);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);
  LOG("Base Horizon initialization complete.\n");
  return true;
}

static bool base_horizon_capture_origin_position(void)
{
  uint32_t start_tick;
  uint16_t origin_position = 0U;

  if (base_horizon_encoder_uart == NULL) {
    return false;
  }

  if (!encoder_device_request_data(&base_horizon_encoder_device, ENCODER_CMD_POSITION)) {
    if (base_horizon_should_log_error(HAL_GetTick())) {
      LOG("Base Horizon origin request failed: uart_err=%lu checksum_err=%lu\n",
          (unsigned long)encoder_device_get_uart_error_count(&base_horizon_encoder_device),
          (unsigned long)encoder_device_get_checksum_error_count(&base_horizon_encoder_device));
    }
    return false;
  }

  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < BASE_HORIZON_ENCODER_RESPONSE_TIMEOUT_MS) {
    if (encoder_device_get_position(&base_horizon_encoder_device, &origin_position)) {
      base_horizon_state.origin_position = (uint16_t)(origin_position & 0x3FFFU);
      base_horizon_state.has_origin_position = true;
      base_horizon_state.next_origin_retry_tick = 0U;
      LOG("Base Horizon origin captured: pos=%u\n", base_horizon_state.origin_position);
      return true;
    }
  }

  if (base_horizon_should_log_error(HAL_GetTick())) {
    LOG("Base Horizon origin timeout: uart_err=%lu checksum_err=%lu\n",
        (unsigned long)encoder_device_get_uart_error_count(&base_horizon_encoder_device),
        (unsigned long)encoder_device_get_checksum_error_count(&base_horizon_encoder_device));
    encoder_device_log_timeout_state(&base_horizon_encoder_device, "Base Horizon origin timeout detail");
  }

  return false;
}

static bool base_horizon_prepare_origin_reference(void)
{
  uint32_t now_tick;

  if (!base_horizon_state.runtime_ready) {
    return false;
  }

  encoder_device_stop_polling(&base_horizon_encoder_device);

  now_tick = HAL_GetTick();
  if (!encoder_device_reset(&base_horizon_encoder_device)) {
    if (base_horizon_should_log_error(now_tick)) {
      LOG("Base Horizon encoder reset failed: uart_err=%lu checksum_err=%lu\n",
          (unsigned long)encoder_device_get_uart_error_count(&base_horizon_encoder_device),
          (unsigned long)encoder_device_get_checksum_error_count(&base_horizon_encoder_device));
    }
    base_horizon_state.next_origin_retry_tick = now_tick + BASE_HORIZON_ORIGIN_RETRY_INTERVAL_MS;
    return false;
  }

  HAL_Delay(BASE_HORIZON_ENCODER_RESET_SETTLE_MS);
  if (!base_horizon_capture_origin_position()) {
    base_horizon_state.next_origin_retry_tick = HAL_GetTick() + BASE_HORIZON_ORIGIN_RETRY_INTERVAL_MS;
    return false;
  }

  base_horizon_state.is_first_reading = true;
  base_horizon_state.has_last_total_counts = false;
  base_horizon_state.has_pending_transition = false;
  base_horizon_state.has_last_can_distance = false;

  if (!encoder_device_start_polling(&base_horizon_encoder_device, ENCODER_POLL_MODE_POSITION_TURNS)) {
    if (base_horizon_should_log_error(HAL_GetTick())) {
      LOG("Base Horizon polling start failed: uart_err=%lu checksum_err=%lu\n",
          (unsigned long)encoder_device_get_uart_error_count(&base_horizon_encoder_device),
          (unsigned long)encoder_device_get_checksum_error_count(&base_horizon_encoder_device));
    }
    base_horizon_state.has_origin_position = false;
    base_horizon_state.next_origin_retry_tick = HAL_GetTick() + BASE_HORIZON_ORIGIN_RETRY_INTERVAL_MS;
    return false;
  }

  return true;
}

static uint16_t base_horizon_shift_position(uint16_t pos)
{
  uint16_t raw_pos = (uint16_t)(pos & 0x3FFFU);

  if (!base_horizon_state.has_origin_position) {
    return raw_pos;
  }

  return (uint16_t)((raw_pos - base_horizon_state.origin_position) & 0x3FFFU);
}

static bool base_horizon_build_sample(BaseHorizonSample *sample, uint16_t pos, int16_t turns)
{
  int32_t raw_total_counts;
  int32_t total_counts;
  int32_t predicted_total_counts;
  int32_t turns_mismatch;
  uint16_t shifted_pos;

  if (sample == NULL) {
    return false;
  }

  shifted_pos = base_horizon_shift_position(pos);
  raw_total_counts = ((int32_t)turns * 16384) + (int32_t)shifted_pos;
  total_counts = raw_total_counts;

  if (base_horizon_state.has_last_total_counts) {
    predicted_total_counts = base_horizon_unwrap_counts(shifted_pos, base_horizon_state.last_total_counts);
    turns_mismatch = raw_total_counts - predicted_total_counts;

    if (turns_mismatch >= 8192 || turns_mismatch <= -8192) {
      total_counts = predicted_total_counts;
    }
  }

  sample->pos = shifted_pos;
  sample->turns = turns;
  sample->total_counts = total_counts;
  sample->distance_tenths_mm = (int16_t)roundf((float)total_counts * -50.0f / 16384.0f);
  return true;
}

static bool base_horizon_read_sample(BaseHorizonSample *sample)
{
  uint16_t pos = 0U;
  int16_t turns = 0;

  if (sample == NULL) {
    return false;
  }

  if (!encoder_device_get_sample(&base_horizon_encoder_device, &pos, &turns)) {
    return false;
  }

  return base_horizon_build_sample(sample, pos, turns);
}

static bool base_horizon_validate_distance(int16_t distance_tenths_mm)
{
  int16_t diff;
  int16_t pending_diff;

  if (base_horizon_state.is_first_reading) {
    base_horizon_state.is_first_reading = false;
    base_horizon_state.has_pending_transition = false;
    return true;
  }

  diff = distance_tenths_mm - base_horizon_state.last_distance_tenths_mm;
  if (diff < 0) {
    diff = -diff;
  }

  if (diff <= BASE_HORIZON_DIRECT_ACCEPT_TENTHS_MM) {
    base_horizon_state.has_pending_transition = false;
    return true;
  }

  if (base_horizon_state.has_pending_transition) {
    pending_diff = distance_tenths_mm - base_horizon_state.pending_distance_tenths_mm;
    if (pending_diff < 0) {
      pending_diff = -pending_diff;
    }

    if (pending_diff <= BASE_HORIZON_PENDING_TOLERANCE_TENTHS_MM) {
      base_horizon_state.pending_transition_count++;
    } else {
      base_horizon_state.pending_distance_tenths_mm = distance_tenths_mm;
      base_horizon_state.pending_transition_count = 1U;
    }
  } else {
    base_horizon_state.has_pending_transition = true;
    base_horizon_state.pending_distance_tenths_mm = distance_tenths_mm;
    base_horizon_state.pending_transition_count = 1U;
  }

  if (base_horizon_state.pending_transition_count >= BASE_HORIZON_PENDING_CONFIRMATIONS) {
    base_horizon_state.has_pending_transition = false;
    return true;
  }

  return false;
}

static bool base_horizon_should_send_can(int16_t distance_tenths_mm, uint32_t now_tick)
{
  if (!base_horizon_state.has_last_can_distance) {
    return true;
  }

  if (distance_tenths_mm != base_horizon_state.last_can_distance_tenths_mm) {
    return true;
  }

  if ((now_tick - base_horizon_state.last_can_send_tick) >= BASE_HORIZON_CAN_KEEPALIVE_INTERVAL_MS) {
    return true;
  }

  return false;
}

static void base_horizon_publish_sample(const BaseHorizonSample *sample)
{
  uint32_t now_tick;

  if (sample == NULL) {
    return;
  }

  base_horizon_state.last_distance_tenths_mm = sample->distance_tenths_mm;
  base_horizon_state.last_total_counts = sample->total_counts;
  base_horizon_state.has_last_total_counts = true;

  now_tick = HAL_GetTick();
  if (base_horizon_should_send_can(sample->distance_tenths_mm, now_tick)) {
    if (!base_horizon_queue_distance_can(sample->distance_tenths_mm)) {
      if (base_horizon_should_log_error(now_tick)) {
        LOG("Base Horizon CAN stage failed.\n");
      }
      return;
    }

    base_horizon_state.last_can_distance_tenths_mm = sample->distance_tenths_mm;
    base_horizon_state.last_can_send_tick = now_tick;
    base_horizon_state.has_last_can_distance = true;
  }

  if ((now_tick - base_horizon_state.last_encoder_log_tick) >= BASE_HORIZON_LOG_INTERVAL_MS) {
    int32_t abs_distance_tenths_mm;

    base_horizon_state.last_encoder_log_tick = now_tick;
    abs_distance_tenths_mm = sample->distance_tenths_mm;
    if (abs_distance_tenths_mm < 0) {
      abs_distance_tenths_mm = -abs_distance_tenths_mm;
      LOG("Base Horizon Encoder: %d turns, %u pos -> Distance: -%ld.%01ld mm (counts=%ld)\n",
          sample->turns,
          sample->pos,
          (long)(abs_distance_tenths_mm / 10),
          (long)(abs_distance_tenths_mm % 10),
          (long)sample->total_counts);
    } else {
      LOG("Base Horizon Encoder: %d turns, %u pos -> Distance: %ld.%01ld mm (counts=%ld)\n",
          sample->turns,
          sample->pos,
          (long)(abs_distance_tenths_mm / 10),
          (long)(abs_distance_tenths_mm % 10),
          (long)sample->total_counts);
    }
  }
}

void base_horizon_init(UART_HandleTypeDef *encoder_uart)
{
  base_horizon_encoder_uart = encoder_uart;
  base_horizon_state = (BaseHorizonState){0};
  base_horizon_state.is_first_reading = true;
  base_horizon_state.runtime_ready = base_horizon_initialize_runtime();
  (void)base_horizon_prepare_origin_reference();
}

void base_horizon_process(void)
{
  BaseHorizonSample sample;
  uint32_t now_tick;

  if (!base_horizon_state.runtime_ready) {
    return;
  }

  encoder_device_process(&base_horizon_encoder_device);

  if (!base_horizon_state.has_origin_position) {
    now_tick = HAL_GetTick();
    if ((int32_t)(now_tick - base_horizon_state.next_origin_retry_tick) >= 0) {
      (void)base_horizon_prepare_origin_reference();
    }
    return;
  }

  if (base_horizon_read_sample(&sample) &&
      base_horizon_validate_distance(sample.distance_tenths_mm)) {
    base_horizon_publish_sample(&sample);
  }
}
