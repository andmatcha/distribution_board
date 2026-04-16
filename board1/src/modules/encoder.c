/**
 ******************************************************************************
 * @file           : encoder.c
 * @brief          : Encoder reading module implementation
 ******************************************************************************
 */

#include "modules/encoder.h"

#include "debug_log.h"

#define ENCODER_BUFFER_SIZE 2U
#define ENCODER_MAX_DEVICES 4U
#define ENCODER_DEFAULT_DE_PIN GPIO_PIN_8
#define ENCODER_DEFAULT_DE_PORT GPIOA
#define ENCODER_UART_TX_TIMEOUT_MS 10U
#define ENCODER_RESPONSE_TIMEOUT_MS 15U

static EncoderDevice default_encoder_device;
static EncoderDevice *encoder_devices[ENCODER_MAX_DEVICES];

static uint8_t encoder_calculate_checksum(uint16_t w);
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position);
static bool encoder_decode_turns(uint8_t lsb, uint8_t msb, int16_t *turns);
static bool encoder_register_device(EncoderDevice *device);
static EncoderDevice *encoder_find_device(UART_HandleTypeDef *huart);
static bool encoder_is_valid_cmd(uint8_t cmd);
static uint8_t encoder_poll_start_cmd(EncoderPollMode mode);
static void encoder_clear_uart_state(UART_HandleTypeDef *huart);
static void encoder_cancel_io(EncoderDevice *device);
static bool encoder_submit_request(EncoderDevice *device, uint8_t cmd);
static bool encoder_restart_polling(EncoderDevice *device);
#if DEBUG_LOG_ENABLED
static const char *encoder_uart_name(const UART_HandleTypeDef *huart);
static uint32_t encoder_dma_remaining(const UART_HandleTypeDef *huart);
#endif
static void encoder_tx_enable(const EncoderDevice *device);
static void encoder_rx_enable(const EncoderDevice *device);

static uint8_t encoder_calculate_checksum(uint16_t w)
{
  uint8_t cs = 0x3U;

  for (int i = 0; i < 14; i += 2) {
    cs ^= (uint8_t)((w >> i) & 0x3U);
  }

  return cs;
}

static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position)
{
  uint16_t w;

  if (position == NULL) {
    return false;
  }

  w = (uint16_t)lsb | ((uint16_t)msb << 8);
  if (encoder_calculate_checksum(w) == (uint8_t)(w >> 14)) {
    *position = (w & 0x3FFFU);
    return true;
  }

  return false;
}

static bool encoder_decode_turns(uint8_t lsb, uint8_t msb, int16_t *turns)
{
  uint16_t w;
  uint16_t val;

  if (turns == NULL) {
    return false;
  }

  w = (uint16_t)lsb | ((uint16_t)msb << 8);
  if (encoder_calculate_checksum(w) == (uint8_t)(w >> 14)) {
    val = (w & 0x3FFFU);
    if ((val & 0x2000U) != 0U) {
      *turns = (int16_t)(val | 0xC000U);
    } else {
      *turns = (int16_t)val;
    }
    return true;
  }

  return false;
}

static bool encoder_register_device(EncoderDevice *device)
{
  uint32_t index;

  if (device == NULL) {
    return false;
  }

  for (index = 0U; index < ENCODER_MAX_DEVICES; index++) {
    if (encoder_devices[index] == device) {
      return true;
    }
  }

  for (index = 0U; index < ENCODER_MAX_DEVICES; index++) {
    if (encoder_devices[index] == NULL) {
      encoder_devices[index] = device;
      return true;
    }
  }

  return false;
}

static EncoderDevice *encoder_find_device(UART_HandleTypeDef *huart)
{
  uint32_t index;

  if (huart == NULL) {
    return NULL;
  }

  for (index = 0U; index < ENCODER_MAX_DEVICES; index++) {
    if (encoder_devices[index] != NULL &&
        encoder_devices[index]->huart != NULL &&
        encoder_devices[index]->huart->Instance == huart->Instance) {
      return encoder_devices[index];
    }
  }

  return NULL;
}

static bool encoder_is_valid_cmd(uint8_t cmd)
{
  return (cmd == ENCODER_CMD_POSITION) || (cmd == ENCODER_CMD_TURNS);
}

static uint8_t encoder_poll_start_cmd(EncoderPollMode mode)
{
  if (mode == ENCODER_POLL_MODE_POSITION || mode == ENCODER_POLL_MODE_POSITION_TURNS) {
    return ENCODER_CMD_POSITION;
  }

  return 0U;
}

static void encoder_clear_uart_state(UART_HandleTypeDef *huart)
{
  if (huart == NULL) {
    return;
  }

  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)huart->Instance->DR;
  huart->RxState = HAL_UART_STATE_READY;
}

static void encoder_cancel_io(EncoderDevice *device)
{
  if (device == NULL || device->huart == NULL) {
    return;
  }

  HAL_UART_DMAStop(device->huart);
  encoder_clear_uart_state(device->huart);
  encoder_rx_enable(device);
  device->response_pending = false;
}

static bool encoder_submit_request(EncoderDevice *device, uint8_t cmd)
{
  uint32_t start_tick;

  if (device == NULL || device->huart == NULL || !encoder_is_valid_cmd(cmd)) {
    return false;
  }

  if (device->response_pending || device->huart->RxState != HAL_UART_STATE_READY) {
    encoder_cancel_io(device);
  }
  device->pending_cmd = cmd;

  encoder_tx_enable(device);
  if (HAL_UART_Transmit(device->huart, &cmd, 1U, 10U) != HAL_OK) {
    device->uart_error_count++;
    LOG("Encoder request tx error: uart=%s cmd=0x%02X err=0x%08lX count=%lu\n",
        encoder_uart_name(device->huart),
        cmd,
        (unsigned long)device->huart->ErrorCode,
        (unsigned long)device->uart_error_count);
    encoder_rx_enable(device);
    return false;
  }

  start_tick = HAL_GetTick();
  while ((device->huart->Instance->SR & UART_FLAG_TC) == 0U) {
    if ((HAL_GetTick() - start_tick) >= ENCODER_UART_TX_TIMEOUT_MS) {
      device->uart_error_count++;
      LOG("Encoder request timeout: uart=%s cmd=0x%02X count=%lu\n",
          encoder_uart_name(device->huart),
          cmd,
          (unsigned long)device->uart_error_count);
      encoder_rx_enable(device);
      return false;
    }
  }

  encoder_rx_enable(device);
  if (HAL_UART_Receive_DMA(device->huart, device->rx_buf, ENCODER_BUFFER_SIZE) != HAL_OK) {
    encoder_cancel_io(device);
    if (HAL_UART_Receive_DMA(device->huart, device->rx_buf, ENCODER_BUFFER_SIZE) == HAL_OK) {
      device->request_tick = HAL_GetTick();
      device->response_pending = true;
      return true;
    }
    device->uart_error_count++;
    LOG("Encoder rx dma start error: uart=%s cmd=0x%02X err=0x%08lX count=%lu\n",
        encoder_uart_name(device->huart),
        cmd,
        (unsigned long)device->huart->ErrorCode,
        (unsigned long)device->uart_error_count);
    return false;
  }

  device->request_tick = HAL_GetTick();
  device->response_pending = true;
  return true;
}

static bool encoder_restart_polling(EncoderDevice *device)
{
  uint8_t cmd;

  if (device == NULL) {
    return false;
  }

  cmd = encoder_poll_start_cmd(device->poll_mode);
  if (cmd == 0U) {
    return false;
  }

  return encoder_submit_request(device, cmd);
}

#if DEBUG_LOG_ENABLED
static const char *encoder_uart_name(const UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance == NULL) {
    return "unknown";
  }

  if (huart->Instance == USART1) {
    return "USART1";
  }

  if (huart->Instance == USART2) {
    return "USART2";
  }

  return "UART";
}
#endif

#if DEBUG_LOG_ENABLED
static uint32_t encoder_dma_remaining(const UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->hdmarx == NULL || huart->hdmarx->Instance == NULL) {
    return 0U;
  }

  return huart->hdmarx->Instance->CNDTR;
}
#endif

static void encoder_tx_enable(const EncoderDevice *device)
{
  if (device == NULL || device->de_port == NULL || device->de_pin == 0U) {
    return;
  }

  device->de_port->BSRR = device->de_pin;
}

static void encoder_rx_enable(const EncoderDevice *device)
{
  if (device == NULL || device->de_port == NULL || device->de_pin == 0U) {
    return;
  }

  device->de_port->BRR = device->de_pin;
}

bool encoder_device_init(EncoderDevice *device,
                         UART_HandleTypeDef *huart,
                         GPIO_TypeDef *de_port,
                         uint16_t de_pin)
{
  if (device == NULL || huart == NULL || de_port == NULL || de_pin == 0U) {
    return false;
  }

  device->huart = huart;
  device->de_port = de_port;
  device->de_pin = de_pin;
  device->rx_buf[0] = 0U;
  device->rx_buf[1] = 0U;
  device->position = 0U;
  device->turns = 0;
  device->sample_position = 0U;
  device->sample_turns = 0;
  device->request_tick = 0U;
  device->pending_cmd = 0U;
  device->poll_mode = ENCODER_POLL_MODE_NONE;
  device->position_ready = false;
  device->turns_ready = false;
  device->sample_ready = false;
  device->response_pending = false;
  device->uart_error_count = 0U;
  device->checksum_error_count = 0U;

  if (!encoder_register_device(device)) {
    return false;
  }

  encoder_rx_enable(device);
  return true;
}

bool encoder_device_reset(EncoderDevice *device)
{
  uint8_t cmds[] = {ENCODER_CMD_RESET_1, ENCODER_CMD_RESET_2};
  uint32_t start_tick;

  if (device == NULL || device->huart == NULL) {
    return false;
  }

  encoder_device_stop_polling(device);

  encoder_tx_enable(device);
  if (HAL_UART_Transmit(device->huart, cmds, 2U, 10U) != HAL_OK) {
    device->uart_error_count++;
    LOG("Encoder reset tx error: uart=%s err=0x%08lX count=%lu\n",
        encoder_uart_name(device->huart),
        (unsigned long)device->huart->ErrorCode,
        (unsigned long)device->uart_error_count);
    encoder_rx_enable(device);
    return false;
  }

  start_tick = HAL_GetTick();
  while ((device->huart->Instance->SR & UART_FLAG_TC) == 0U) {
    if ((HAL_GetTick() - start_tick) >= ENCODER_UART_TX_TIMEOUT_MS) {
      device->uart_error_count++;
      LOG("Encoder reset timeout: uart=%s count=%lu\n",
          encoder_uart_name(device->huart),
          (unsigned long)device->uart_error_count);
      encoder_rx_enable(device);
      return false;
    }
  }

  encoder_rx_enable(device);
  return true;
}

bool encoder_device_request_data(EncoderDevice *device, uint8_t cmd)
{
  if (device == NULL) {
    return false;
  }

  encoder_device_stop_polling(device);
  device->position_ready = false;
  device->turns_ready = false;
  device->sample_ready = false;
  return encoder_submit_request(device, cmd);
}

bool encoder_device_start_polling(EncoderDevice *device, EncoderPollMode mode)
{
  if (device == NULL || mode == ENCODER_POLL_MODE_NONE) {
    return false;
  }

  encoder_device_stop_polling(device);
  device->poll_mode = mode;
  device->position_ready = false;
  device->turns_ready = false;
  device->sample_ready = false;
  return encoder_restart_polling(device);
}

void encoder_device_stop_polling(EncoderDevice *device)
{
  if (device == NULL) {
    return;
  }

  device->poll_mode = ENCODER_POLL_MODE_NONE;
  device->pending_cmd = 0U;
  device->position_ready = false;
  device->turns_ready = false;
  device->sample_ready = false;
  encoder_cancel_io(device);
}

void encoder_device_process(EncoderDevice *device)
{
  uint32_t now_tick;

  if (device == NULL || device->huart == NULL || !device->response_pending) {
    return;
  }

  now_tick = HAL_GetTick();
  if ((now_tick - device->request_tick) < ENCODER_RESPONSE_TIMEOUT_MS) {
    return;
  }

  device->uart_error_count++;
  LOG("Encoder response timeout: uart=%s cmd=0x%02X count=%lu\n",
      encoder_uart_name(device->huart),
      device->pending_cmd,
      (unsigned long)device->uart_error_count);
  encoder_device_log_timeout_state(device, "Encoder timeout detail");

  encoder_cancel_io(device);
  if (device->poll_mode != ENCODER_POLL_MODE_NONE && !encoder_restart_polling(device)) {
    LOG("Encoder polling restart failed: uart=%s count=%lu\n",
        encoder_uart_name(device->huart),
        (unsigned long)device->uart_error_count);
  }
}

bool encoder_device_get_position(EncoderDevice *device, uint16_t *position)
{
  bool ready = false;

  if (device == NULL || position == NULL) {
    return false;
  }

  __disable_irq();
  if (device->position_ready) {
    device->position_ready = false;
    *position = device->position;
    ready = true;
  }
  __enable_irq();

  return ready;
}

bool encoder_device_get_turns(EncoderDevice *device, int16_t *turns)
{
  bool ready = false;

  if (device == NULL || turns == NULL) {
    return false;
  }

  __disable_irq();
  if (device->turns_ready) {
    device->turns_ready = false;
    *turns = device->turns;
    ready = true;
  }
  __enable_irq();

  return ready;
}

bool encoder_device_get_sample(EncoderDevice *device, uint16_t *position, int16_t *turns)
{
  bool ready = false;

  if (device == NULL || position == NULL || turns == NULL) {
    return false;
  }

  __disable_irq();
  if (device->sample_ready) {
    device->sample_ready = false;
    *position = device->sample_position;
    *turns = device->sample_turns;
    ready = true;
  }
  __enable_irq();

  return ready;
}

uint32_t encoder_device_get_checksum_error_count(const EncoderDevice *device)
{
  if (device == NULL) {
    return 0U;
  }

  return device->checksum_error_count;
}

uint32_t encoder_device_get_uart_error_count(const EncoderDevice *device)
{
  if (device == NULL) {
    return 0U;
  }

  return device->uart_error_count;
}

void encoder_init(UART_HandleTypeDef *huart)
{
  (void)encoder_device_init(&default_encoder_device,
                            huart,
                            ENCODER_DEFAULT_DE_PORT,
                            ENCODER_DEFAULT_DE_PIN);
}

bool encoder_reset(void)
{
  return encoder_device_reset(&default_encoder_device);
}

bool encoder_request_data(uint8_t cmd)
{
  return encoder_device_request_data(&default_encoder_device, cmd);
}

bool encoder_get_position(uint16_t *position)
{
  return encoder_device_get_position(&default_encoder_device, position);
}

bool encoder_get_turns(int16_t *turns)
{
  return encoder_device_get_turns(&default_encoder_device, turns);
}

uint32_t encoder_get_checksum_error_count(void)
{
  return encoder_device_get_checksum_error_count(&default_encoder_device);
}

uint32_t encoder_get_uart_error_count(void)
{
  return encoder_device_get_uart_error_count(&default_encoder_device);
}

void encoder_rx_complete_callback(UART_HandleTypeDef *huart)
{
  EncoderDevice *device;
  uint8_t completed_cmd;
  bool decoded = false;

  device = encoder_find_device(huart);
  if (device == NULL) {
    return;
  }

  completed_cmd = device->pending_cmd;
  HAL_UART_DMAStop(huart);
  device->response_pending = false;

  if (completed_cmd == ENCODER_CMD_POSITION) {
    uint16_t pos = 0U;

    if (encoder_decode_position(device->rx_buf[0], device->rx_buf[1], &pos)) {
      device->position = pos;
      device->sample_position = pos;
      device->position_ready = true;
      decoded = true;
    }
  } else if (completed_cmd == ENCODER_CMD_TURNS) {
    int16_t turns = 0;

    if (encoder_decode_turns(device->rx_buf[0], device->rx_buf[1], &turns)) {
      device->turns = turns;
      device->sample_turns = turns;
      device->turns_ready = true;
      if (device->poll_mode == ENCODER_POLL_MODE_POSITION_TURNS) {
        device->sample_ready = true;
      }
      decoded = true;
    }
  }

  if (!decoded) {
    device->checksum_error_count++;
    LOG("Encoder checksum error: uart=%s cmd=0x%02X data=%02X %02X count=%lu\n",
        encoder_uart_name(device->huart),
        completed_cmd,
        device->rx_buf[0],
        device->rx_buf[1],
        (unsigned long)device->checksum_error_count);
    if (device->poll_mode != ENCODER_POLL_MODE_NONE) {
      (void)encoder_restart_polling(device);
    }
    return;
  }

  if (device->poll_mode == ENCODER_POLL_MODE_POSITION) {
    (void)encoder_submit_request(device, ENCODER_CMD_POSITION);
  } else if (device->poll_mode == ENCODER_POLL_MODE_POSITION_TURNS) {
    if (completed_cmd == ENCODER_CMD_POSITION) {
      (void)encoder_submit_request(device, ENCODER_CMD_TURNS);
    } else {
      (void)encoder_submit_request(device, ENCODER_CMD_POSITION);
    }
  }
}

void encoder_error_callback(UART_HandleTypeDef *huart)
{
  EncoderDevice *device;

  device = encoder_find_device(huart);
  if (device == NULL) {
    return;
  }

  device->uart_error_count++;
  encoder_cancel_io(device);
  LOG("Encoder uart error: uart=%s code=0x%08lX count=%lu\n",
      encoder_uart_name(huart),
      (unsigned long)huart->ErrorCode,
      (unsigned long)device->uart_error_count);

  if (device->poll_mode != ENCODER_POLL_MODE_NONE && !encoder_restart_polling(device)) {
    LOG("Encoder polling restart failed after error: uart=%s count=%lu\n",
        encoder_uart_name(huart),
        (unsigned long)device->uart_error_count);
  }
}

void encoder_device_log_timeout_state(const EncoderDevice *device, const char *context)
{
#if DEBUG_LOG_ENABLED
  const char *label = context;
  uint32_t sr;
  uint32_t cr1;
  uint32_t cr3;
  uint32_t dma_remaining;

  if (device == NULL || device->huart == NULL || device->huart->Instance == NULL) {
    return;
  }

  if (label == NULL) {
    label = "Encoder timeout";
  }

  sr = device->huart->Instance->SR;
  cr1 = device->huart->Instance->CR1;
  cr3 = device->huart->Instance->CR3;
  dma_remaining = encoder_dma_remaining(device->huart);
  LOG("%s: uart=%s cmd=0x%02X sr=0x%08lX cr1=0x%08lX cr3=0x%08lX rx_state=%lu dma_remaining=%lu rx_buf=%02X %02X\n",
      label,
      encoder_uart_name(device->huart),
      device->pending_cmd,
      (unsigned long)sr,
      (unsigned long)cr1,
      (unsigned long)cr3,
      (unsigned long)device->huart->RxState,
      (unsigned long)dma_remaining,
      device->rx_buf[0],
      device->rx_buf[1]);
#else
  (void)device;
  (void)context;
#endif
}
