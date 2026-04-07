/**
 ******************************************************************************
 * @file           : encoder.c
 * @brief          : Encoder reading module implementation
 ******************************************************************************
 */

#include "encoder.h"

/* Private defines -----------------------------------------------------------*/
#define ENCODER_BUFFER_SIZE 2
#define ENCODER_RS485_DE_PIN GPIO_PIN_8
#define ENCODER_RS485_DE_PORT GPIOA

/* High speed GPIO macros */
#define ENCODER_TX_ENABLE()  (ENCODER_RS485_DE_PORT->BSRR = ENCODER_RS485_DE_PIN)
#define ENCODER_RX_ENABLE()  (ENCODER_RS485_DE_PORT->BRR = ENCODER_RS485_DE_PIN)

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *encoder_huart = NULL;
static uint8_t encoder_rx_buf[ENCODER_BUFFER_SIZE];
static uint16_t encoder_position = 0;
static int16_t encoder_turns = 0;
static uint8_t encoder_last_cmd = ENCODER_CMD_POSITION;
static volatile bool is_encoder_data_ready = false;
static volatile uint32_t encoder_uart_error_count = 0;
static volatile uint32_t encoder_checksum_error_count = 0;

/* Private function prototypes -----------------------------------------------*/
static uint8_t encoder_calculate_checksum(uint16_t w);
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position);
static bool encoder_decode_turns(uint8_t lsb, uint8_t msb, int16_t *turns);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Calculate 2-bit checksum for encoder data
 * @param w 16-bit raw data word (14-bit data + 2-bit checksum)
 * @retval Calculated 2-bit checksum
 */
static uint8_t encoder_calculate_checksum(uint16_t w)
{
  uint8_t cs = 0x3;
  for (int i = 0; i < 14; i += 2) {
    cs ^= (uint8_t)((w >> i) & 0x3U);
  }
  return cs;
}

/**
 * @brief Decode encoder position from 2 bytes with checksum validation
 * @param lsb LSB byte
 * @param msb MSB byte
 * @param position Pointer to store decoded 14-bit position
 * @retval true if checksum is valid, false otherwise
 */
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position)
{
  if (position == NULL) {
    return false;
  }

  uint16_t w = (uint16_t)lsb | ((uint16_t)msb << 8);

  /* Verify checksum */
  if (encoder_calculate_checksum(w) == (uint8_t)(w >> 14)) {
    *position = (w & 0x3FFFU);
    return true;
  }

  return false;
}

/**
 * @brief Decode encoder turns from 2 bytes with checksum validation
 * @param lsb LSB byte
 * @param msb MSB byte
 * @param turns Pointer to store decoded 14-bit signed turns
 * @retval true if checksum is valid, false otherwise
 */
static bool encoder_decode_turns(uint8_t lsb, uint8_t msb, int16_t *turns)
{
  if (turns == NULL) {
    return false;
  }

  uint16_t w = (uint16_t)lsb | ((uint16_t)msb << 8);

  /* Verify checksum */
  if (encoder_calculate_checksum(w) == (uint8_t)(w >> 14)) {
    uint16_t val = (w & 0x3FFFU);
    /* Sign extend 14-bit to 16-bit */
    if (val & 0x2000U) {
      *turns = (int16_t)(val | 0xC000U);
    } else {
      *turns = (int16_t)val;
    }
    return true;
  }

  return false;
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
  is_encoder_data_ready = false;
  encoder_position = 0;
  encoder_turns = 0;
  encoder_last_cmd = ENCODER_CMD_POSITION;
  encoder_uart_error_count = 0;
  encoder_checksum_error_count = 0;

  /* Ensure RX mode */
  ENCODER_RX_ENABLE();
}

/**
 * @brief Reset encoder position/turns
 */
void encoder_reset(void)
{
  if (encoder_huart == NULL) {
    return;
  }

  uint8_t cmds[] = {ENCODER_CMD_RESET_1, ENCODER_CMD_RESET_2};

  /* Ensure UART state is clean */
  HAL_UART_DMAStop(encoder_huart);
  __HAL_UART_CLEAR_OREFLAG(encoder_huart);

  /* Switch to TX mode and send command */
  ENCODER_TX_ENABLE();
  HAL_UART_Transmit(encoder_huart, cmds, 2, 10);

  /* Wait for transmission complete BEFORE switching to RX */
  while (!(encoder_huart->Instance->SR & UART_FLAG_TC)) {}
  ENCODER_RX_ENABLE();
}

/**
 * @brief Request encoder data
 */
void encoder_request_data(uint8_t cmd)
{
  if (encoder_huart == NULL) {
    return;
  }

  if (cmd != ENCODER_CMD_POSITION && cmd != ENCODER_CMD_TURNS) {
    return;
  }

  /* Remember last command and clear completion flag BEFORE starting new transfer */
  encoder_last_cmd = cmd;
  is_encoder_data_ready = false;

  /* Ensure UART state is clean */
  HAL_UART_DMAStop(encoder_huart);
  __HAL_UART_CLEAR_FEFLAG(encoder_huart);
  __HAL_UART_CLEAR_NEFLAG(encoder_huart);
  __HAL_UART_CLEAR_OREFLAG(encoder_huart);
  (void)encoder_huart->Instance->DR;

  /* Switch to TX mode and send command */
  ENCODER_TX_ENABLE();
  HAL_UART_Transmit(encoder_huart, &cmd, 1, 2);

  /* Wait for transmission complete BEFORE switching to RX */
  while (!(encoder_huart->Instance->SR & UART_FLAG_TC)) {}
  ENCODER_RX_ENABLE();

  /* Clear any echo/noise from the line before starting DMA reception */
  __HAL_UART_CLEAR_OREFLAG(encoder_huart);

  /* Now start DMA reception safely */
  encoder_huart->RxState = HAL_UART_STATE_READY;
  HAL_UART_Receive_DMA(encoder_huart, encoder_rx_buf, ENCODER_BUFFER_SIZE);
}

/**
 * @brief Get latest encoder position data
 */
bool encoder_get_position(uint16_t *position)
{
  if (position == NULL) {
    return false;
  }

  bool ready = false;
  __disable_irq();
  if (is_encoder_data_ready && encoder_last_cmd == ENCODER_CMD_POSITION) {
    is_encoder_data_ready = false;
    *position = encoder_position;
    ready = true;
  }
  __enable_irq();
  return ready;
}

/**
 * @brief Get latest encoder turns data
 */
bool encoder_get_turns(int16_t *turns)
{
  if (turns == NULL) {
    return false;
  }

  bool ready = false;
  __disable_irq();
  if (is_encoder_data_ready && encoder_last_cmd == ENCODER_CMD_TURNS) {
    is_encoder_data_ready = false;
    *turns = encoder_turns;
    ready = true;
  }
  __enable_irq();
  return ready;
}

/**
 * @brief UART RX complete callback (HAL)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (encoder_huart == NULL || huart->Instance != encoder_huart->Instance) {
    return;
  }

  /* Stop DMA immediately to finalize the transfer */
  HAL_UART_DMAStop(huart);

  if (encoder_last_cmd == ENCODER_CMD_POSITION) {
    uint16_t pos = 0;
    if (encoder_decode_position(encoder_rx_buf[0], encoder_rx_buf[1], &pos)) {
      encoder_position = pos;
      is_encoder_data_ready = true;
    } else {
      encoder_checksum_error_count++;
    }
  } else if (encoder_last_cmd == ENCODER_CMD_TURNS) {
    int16_t turns = 0;
    if (encoder_decode_turns(encoder_rx_buf[0], encoder_rx_buf[1], &turns)) {
      encoder_turns = turns;
      is_encoder_data_ready = true;
    } else {
      encoder_checksum_error_count++;
    }
  }
}

/**
 * @brief UART error callback (HAL)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (encoder_huart == NULL || huart->Instance != encoder_huart->Instance) {
    return;
  }

  encoder_uart_error_count++;

  /* Clear all error flags and flush DR */
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  (void)huart->Instance->DR;

  HAL_UART_DMAStop(huart);
  ENCODER_RX_ENABLE();
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
