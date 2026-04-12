/**
 ******************************************************************************
 * @file           : encoder.c
 * @brief          : Encoder reading module implementation
 ******************************************************************************
 */

#include "modules/encoder.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define ENCODER_CMD 0x54
#define ENCODER_BUFFER_SIZE 2
#define ENCODER_RESPONSE_TIMEOUT_MS 10U
#define ENCODER_RS485_DE_PIN GPIO_PIN_8
#define ENCODER_RS485_DE_PORT GPIOA
#define RS485_TX_EN()   (GPIOA->BSRR = GPIO_PIN_8)
#define RS485_RX_EN()   (GPIOA->BRR = GPIO_PIN_8)

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *encoder_huart = NULL;
static uint8_t encoder_rx_buf[ENCODER_BUFFER_SIZE];
static uint16_t encoder_position = 0;
static volatile bool encoder_data_ready = false;
static volatile bool encoder_response_pending = false;
static volatile uint32_t encoder_request_tick = 0;
static volatile uint32_t encoder_uart_error_count = 0;
static volatile uint32_t encoder_checksum_error_count = 0;

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

  RS485_TX_EN();
  if (HAL_UART_Transmit(encoder_huart, &cmd, 1, 10) != HAL_OK) {
    encoder_uart_error_count++;
    Error_Handler();
  }
  start_tick = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(encoder_huart, UART_FLAG_TC) == RESET) {
    if ((HAL_GetTick() - start_tick) >= ENCODER_RESPONSE_TIMEOUT_MS) {
      encoder_uart_error_count++;
      Error_Handler();
    }
  }
  RS485_RX_EN();
  if (HAL_UART_Receive_DMA(encoder_huart, encoder_rx_buf, ENCODER_BUFFER_SIZE) != HAL_OK) {
    encoder_uart_error_count++;
    Error_Handler();
  }
  encoder_request_tick = HAL_GetTick();
  encoder_response_pending = true;
}

/**
 * @brief Get latest encoder position data
 */
bool encoder_get_position(uint16_t *position)
{
  if (position == NULL) {
    return false;
  }

  if (encoder_response_pending &&
      (HAL_GetTick() - encoder_request_tick) >= ENCODER_RESPONSE_TIMEOUT_MS) {
    Error_Handler();
  }

  if (encoder_data_ready) {
    encoder_data_ready = false;
    *position = encoder_position;
    return true;
  }
  return false;
}

/**
 * @brief UART RX complete callback
 */
void encoder_rx_complete_callback(UART_HandleTypeDef *huart)
{
  HAL_UART_DMAStop(huart);
  uint16_t w;
  if (huart->Instance == encoder_huart->Instance) {
    w = (encoder_rx_buf[0] | encoder_rx_buf[1] << 8);

    uint16_t cs = 0x3;
    cs ^= (w >> 0) & 0x3;
    cs ^= (w >> 2) & 0x3;
    cs ^= (w >> 4) & 0x3;
    cs ^= (w >> 6) & 0x3;
    cs ^= (w >> 8) & 0x3;
    cs ^= (w >> 10) & 0x3;
    cs ^= (w >> 12) & 0x3;

    if (cs == (w >> 14)) {
      encoder_position = (w & 0x3FFF);
    } else {
      encoder_checksum_error_count++;
      Error_Handler();
      return;
    }

    encoder_response_pending = false;
    encoder_data_ready = true;
  }
}

/**
 * @brief UART error callback
 */
void encoder_error_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != encoder_huart->Instance) {
    return;
  }

  uint32_t err = HAL_UART_GetError(huart);
  encoder_uart_error_count++;
  printf("[USART1] Error: 0x%08lX\n", (unsigned long)err);
  Error_Handler();
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
