/**
 ******************************************************************************
 * @file           : encoder.c
 * @brief          : Encoder reading module implementation
 ******************************************************************************
 */

#include "modules/encoder.h"

/* Private defines -----------------------------------------------------------*/
#define ENCODER_CMD 0x54
#define ENCODER_BUFFER_SIZE 2
#define ENCODER_RS485_DE_PIN GPIO_PIN_8
#define ENCODER_RS485_DE_PORT GPIOA
#define RS485_TX_EN()   (GPIOA->BSRR = GPIO_PIN_8)
#define RS485_RX_EN()   (GPIOA->BRR = GPIO_PIN_8)

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *encoder_huart = NULL;
static uint8_t encoder_rx_buf[ENCODER_BUFFER_SIZE];
static uint16_t encoder_position = 0;
static volatile bool encoder_data_ready = false;
static volatile uint32_t encoder_uart_error_count = 0;
static volatile uint32_t encoder_checksum_error_count = 0;

static void encoder_clear_uart_errors(void);
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position);

static void encoder_clear_uart_errors(void)
{
  if (encoder_huart == NULL) {
    return;
  }

  __HAL_UART_CLEAR_FEFLAG(encoder_huart);
  __HAL_UART_CLEAR_NEFLAG(encoder_huart);
  __HAL_UART_CLEAR_OREFLAG(encoder_huart);
  __HAL_UART_CLEAR_PEFLAG(encoder_huart);
  (void)encoder_huart->Instance->DR;
}

static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position)
{
  uint16_t w;
  uint16_t cs;

  if (position == NULL) {
    return false;
  }

  w = (uint16_t)lsb | ((uint16_t)msb << 8);
  cs = 0x3U;
  cs ^= (w >> 0) & 0x3U;
  cs ^= (w >> 2) & 0x3U;
  cs ^= (w >> 4) & 0x3U;
  cs ^= (w >> 6) & 0x3U;
  cs ^= (w >> 8) & 0x3U;
  cs ^= (w >> 10) & 0x3U;
  cs ^= (w >> 12) & 0x3U;

  if (cs != (w >> 14)) {
    return false;
  }

  *position = (uint16_t)(w & 0x3FFFU);
  return true;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize encoder module
 */
void encoder_init(UART_HandleTypeDef *huart)
{
  if (huart == NULL) {
    return;
  }

  encoder_huart = huart;
  encoder_data_ready = false;
  encoder_response_pending = false;
  encoder_request_tick = 0U;
  encoder_position = 0;
  encoder_uart_error_count = 0;
  encoder_checksum_error_count = 0;

  /* Ensure RX mode */
  RS485_RX_EN();
}

/**
 * @brief Request encoder position data
 */
void encoder_request_position(void)
{
  static uint8_t cmd = ENCODER_CMD;
  uint32_t start_tick;

  if (encoder_huart == NULL) {
    return;
  }

  HAL_UART_DMAStop(encoder_huart);
  encoder_clear_uart_errors();

  RS485_TX_EN();
  if (HAL_UART_Transmit(encoder_huart, &cmd, 1, 10) != HAL_OK) {
    encoder_uart_error_count++;
    RS485_RX_EN();
    return;
  }

  start_tick = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(encoder_huart, UART_FLAG_TC) == RESET) {
    if ((HAL_GetTick() - start_tick) > 2U) {
      encoder_uart_error_count++;
      break;
    }
  }

  RS485_RX_EN();
  encoder_huart->RxState = HAL_UART_STATE_READY;
  if (HAL_UART_Receive_DMA(encoder_huart, encoder_rx_buf, ENCODER_BUFFER_SIZE) != HAL_OK) {
    encoder_uart_error_count++;
  }
}

/**
 * @brief Get latest encoder position data
 */
bool encoder_get_position(uint16_t *position)
{
  bool ready = false;

  if (position == NULL) {
    return false;
  }

  __disable_irq();
  if (encoder_data_ready) {
    encoder_data_ready = false;
    *position = encoder_position;
    ready = true;
  }
  __enable_irq();

  return ready;
}

/**
 * @brief UART RX complete callback
 */
void encoder_rx_complete_callback(UART_HandleTypeDef *huart)
{
  uint16_t position = 0;

  if (encoder_huart == NULL || huart->Instance != encoder_huart->Instance) {
    return;
  }

  HAL_UART_DMAStop(huart);

  if (encoder_decode_position(encoder_rx_buf[0], encoder_rx_buf[1], &position)) {
    encoder_position = position;
    encoder_data_ready = true;
  } else {
    encoder_checksum_error_count++;
  }
}

/**
 * @brief UART error callback
 */
void encoder_error_callback(UART_HandleTypeDef *huart)
{
  if (encoder_huart == NULL || huart->Instance != encoder_huart->Instance) {
    return;
  }

  encoder_uart_error_count++;
  encoder_data_ready = false;

  HAL_UART_DMAStop(huart);
  encoder_clear_uart_errors();
  RS485_RX_EN();
}

/**
 * @brief Get checksum error count
 */
uint32_t encoder_get_checksum_error_count(void)
{
  return encoder_checksum_error_count;
}

/**
 * @brief Get UART error count
 */
uint32_t encoder_get_uart_error_count(void)
{
  return encoder_uart_error_count;
}
