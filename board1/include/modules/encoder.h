/**
 ******************************************************************************
 * @file           : encoder.h
 * @brief          : Encoder reading module header
 ******************************************************************************
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define ENCODER_CMD_POSITION 0x54
#define ENCODER_CMD_TURNS 0x55
#define ENCODER_CMD_RESET_1 0x56
#define ENCODER_CMD_RESET_2 0x75

typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t rx_buf[2];
  uint16_t position;
  int16_t turns;
  uint8_t last_cmd;
  volatile bool is_data_ready;
  volatile uint32_t uart_error_count;
  volatile uint32_t checksum_error_count;
} EncoderDevice;

/**
 * @brief Initialize a specific encoder device
 * @param device Encoder device storage
 * @param huart UART handle for the encoder
 * @param de_port GPIO port for RS485 direction control
 * @param de_pin GPIO pin for RS485 direction control
 * @retval true if initialized successfully, false otherwise
 */
bool encoder_device_init(EncoderDevice *device,
                         UART_HandleTypeDef *huart,
                         GPIO_TypeDef *de_port,
                         uint16_t de_pin);

/**
 * @brief Reset a specific encoder position/turns
 * @param device Encoder device
 * @note This function sends 0x56 and 0x75 consecutively
 * @retval None
 */
bool encoder_device_reset(EncoderDevice *device);

/**
 * @brief Request data from a specific encoder
 * @param device Encoder device
 * @param cmd Command byte (ENCODER_CMD_POSITION or ENCODER_CMD_TURNS)
 * @retval None
 */
bool encoder_device_request_data(EncoderDevice *device, uint8_t cmd);

/**
 * @brief Get latest position from a specific encoder
 * @param device Encoder device
 * @param position Pointer to store the 14-bit position value
 * @retval true if new data is available, false otherwise
 */
bool encoder_device_get_position(EncoderDevice *device, uint16_t *position);

/**
 * @brief Get latest turns from a specific encoder
 * @param device Encoder device
 * @param turns Pointer to store the 14-bit signed turns value
 * @retval true if new data is available, false otherwise
 */
bool encoder_device_get_turns(EncoderDevice *device, int16_t *turns);

/**
 * @brief Get checksum error count for a specific encoder
 * @param device Encoder device
 * @retval Number of checksum errors
 */
uint32_t encoder_device_get_checksum_error_count(const EncoderDevice *device);

/**
 * @brief Get UART error count for a specific encoder
 * @param device Encoder device
 * @retval Number of UART errors
 */
uint32_t encoder_device_get_uart_error_count(const EncoderDevice *device);

/**
 * @brief Log low-level UART/DMA state for timeout diagnosis
 * @param device Encoder device
 * @param context Short log context
 * @retval None
 */
void encoder_device_log_timeout_state(const EncoderDevice *device, const char *context);

/**
 * @brief Initialize the default encoder module
 * @details Uses the original Base Horizon wiring on USART1 + PA8.
 * @param huart Pointer to UART handle for encoder communication
 * @retval None
 */
void encoder_init(UART_HandleTypeDef *huart);

/**
 * @brief Reset encoder position/turns
 * @note This function sends 0x56 and 0x75 consecutively
 * @retval None
 */
bool encoder_reset(void);

/**
 * @brief Request encoder data
 * @param cmd Command byte (ENCODER_CMD_POSITION or ENCODER_CMD_TURNS)
 * @note This function sends command to encoder and switches to RX mode
 * @retval None
 */
bool encoder_request_data(uint8_t cmd);

/**
 * @brief Get latest encoder position data
 * @param position Pointer to store the 14-bit position value
 * @retval true if new data is available, false otherwise
 */
bool encoder_get_position(uint16_t *position);

/**
 * @brief Get latest encoder turns data
 * @param turns Pointer to store the 14-bit signed turns value
 * @retval true if new data is available, false otherwise
 */
bool encoder_get_turns(int16_t *turns);

/**
 * @brief Get checksum error count
 * @retval Number of checksum errors
 */
uint32_t encoder_get_checksum_error_count(void);

/**
 * @brief Get UART error count
 * @retval Number of UART errors
 */
uint32_t encoder_get_uart_error_count(void);

/**
 * @brief UART RX complete callback helper
 * @param huart Pointer to UART handle
 * @retval None
 */
void encoder_rx_complete_callback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback helper
 * @param huart Pointer to UART handle
 * @retval None
 */
void encoder_error_callback(UART_HandleTypeDef *huart);

#endif /* ENCODER_H */
