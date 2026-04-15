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

static EncoderDevice default_encoder_device;
static EncoderDevice *encoder_devices[ENCODER_MAX_DEVICES];

static uint8_t encoder_calculate_checksum(uint16_t w);
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position);
static bool encoder_decode_turns(uint8_t lsb, uint8_t msb, int16_t *turns);
static bool encoder_register_device(EncoderDevice *device);
static EncoderDevice *encoder_find_device(UART_HandleTypeDef *huart);
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
  device->last_cmd = ENCODER_CMD_POSITION;
  device->is_data_ready = false;
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

  HAL_UART_DMAStop(device->huart);
  __HAL_UART_CLEAR_OREFLAG(device->huart);

  encoder_tx_enable(device);
  if (HAL_UART_Transmit(device->huart, cmds, 2U, 10U) != HAL_OK) {
    device->uart_error_count++;
    LOG("Encoder reset tx error: uart=%s err=0x%08lX count=%lu\n",
        encoder_uart_name(device->huart),
        (unsigned long)device->huart->ErrorCode,
        (unsigned long)device->uart_error_count);
    return false;
  }

  start_tick = HAL_GetTick();
  while ((device->huart->Instance->SR & UART_FLAG_TC) == 0U) {
    if ((HAL_GetTick() - start_tick) >= ENCODER_UART_TX_TIMEOUT_MS) {
      device->uart_error_count++;
      LOG("Encoder reset timeout: uart=%s count=%lu\n",
          encoder_uart_name(device->huart),
          (unsigned long)device->uart_error_count);
      return false;
    }
  }

  encoder_rx_enable(device);
  return true;
}

bool encoder_device_request_data(EncoderDevice *device, uint8_t cmd)
{
  uint32_t start_tick;

  if (device == NULL || device->huart == NULL) {
    return false;
  }

  if (cmd != ENCODER_CMD_POSITION && cmd != ENCODER_CMD_TURNS) {
    return false;
  }

  device->last_cmd = cmd;
  device->is_data_ready = false;

  HAL_UART_DMAStop(device->huart);
  __HAL_UART_CLEAR_FEFLAG(device->huart);
  __HAL_UART_CLEAR_NEFLAG(device->huart);
  __HAL_UART_CLEAR_OREFLAG(device->huart);
  (void)device->huart->Instance->DR;

  device->huart->RxState = HAL_UART_STATE_READY;
  if (HAL_UART_Receive_DMA(device->huart, device->rx_buf, ENCODER_BUFFER_SIZE) != HAL_OK) {
    device->uart_error_count++;
    LOG("Encoder rx dma start error: uart=%s cmd=0x%02X err=0x%08lX count=%lu\n",
        encoder_uart_name(device->huart),
        cmd,
        (unsigned long)device->huart->ErrorCode,
        (unsigned long)device->uart_error_count);
    return false;
  }

  encoder_tx_enable(device);
  if (HAL_UART_Transmit(device->huart, &cmd, 1U, 2U) != HAL_OK) {
    HAL_UART_DMAStop(device->huart);
    device->uart_error_count++;
    LOG("Encoder request tx error: uart=%s cmd=0x%02X err=0x%08lX count=%lu\n",
        encoder_uart_name(device->huart),
        cmd,
        (unsigned long)device->huart->ErrorCode,
        (unsigned long)device->uart_error_count);
    return false;
  }

  start_tick = HAL_GetTick();
  while ((device->huart->Instance->SR & UART_FLAG_TC) == 0U) {
    if ((HAL_GetTick() - start_tick) >= ENCODER_UART_TX_TIMEOUT_MS) {
      HAL_UART_DMAStop(device->huart);
      device->uart_error_count++;
      LOG("Encoder request timeout: uart=%s cmd=0x%02X count=%lu\n",
          encoder_uart_name(device->huart),
          cmd,
          (unsigned long)device->uart_error_count);
      return false;
    }
  }

  encoder_rx_enable(device);
  return true;
}

bool encoder_device_get_position(EncoderDevice *device, uint16_t *position)
{
  bool ready = false;

  if (device == NULL || position == NULL) {
    return false;
  }

  __disable_irq();
  if (device->is_data_ready && device->last_cmd == ENCODER_CMD_POSITION) {
    device->is_data_ready = false;
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
  if (device->is_data_ready && device->last_cmd == ENCODER_CMD_TURNS) {
    device->is_data_ready = false;
    *turns = device->turns;
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

  device = encoder_find_device(huart);
  if (device == NULL) {
    return;
  }

  HAL_UART_DMAStop(huart);

  if (device->last_cmd == ENCODER_CMD_POSITION) {
    uint16_t pos = 0U;

    if (encoder_decode_position(device->rx_buf[0], device->rx_buf[1], &pos)) {
      device->position = pos;
      device->is_data_ready = true;
    } else {
      device->checksum_error_count++;
      LOG("Encoder checksum error: uart=%s cmd=0x%02X data=%02X %02X count=%lu\n",
          encoder_uart_name(device->huart),
          device->last_cmd,
          device->rx_buf[0],
          device->rx_buf[1],
          (unsigned long)device->checksum_error_count);
    }
  } else if (device->last_cmd == ENCODER_CMD_TURNS) {
    int16_t turns = 0;

    if (encoder_decode_turns(device->rx_buf[0], device->rx_buf[1], &turns)) {
      device->turns = turns;
      device->is_data_ready = true;
    } else {
      device->checksum_error_count++;
      LOG("Encoder checksum error: uart=%s cmd=0x%02X data=%02X %02X count=%lu\n",
          encoder_uart_name(device->huart),
          device->last_cmd,
          device->rx_buf[0],
          device->rx_buf[1],
          (unsigned long)device->checksum_error_count);
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
  HAL_UART_DMAStop(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)huart->Instance->DR;
  huart->RxState = HAL_UART_STATE_READY;
  device->is_data_ready = false;
  LOG("Encoder uart error: uart=%s code=0x%08lX count=%lu\n",
      encoder_uart_name(huart),
      (unsigned long)huart->ErrorCode,
      (unsigned long)device->uart_error_count);
  (void)huart;
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
      device->last_cmd,
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
