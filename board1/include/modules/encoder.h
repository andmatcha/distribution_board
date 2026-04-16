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

typedef enum
{
  ENCODER_POLL_MODE_NONE = 0,
  ENCODER_POLL_MODE_POSITION,
  ENCODER_POLL_MODE_POSITION_TURNS
} EncoderPollMode;

typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t rx_buf[2];
  uint16_t position;
  int16_t turns;
  uint16_t sample_position;
  int16_t sample_turns;
  uint32_t request_tick;
  uint8_t pending_cmd;
  EncoderPollMode poll_mode;
  volatile bool position_ready;
  volatile bool turns_ready;
  volatile bool sample_ready;
  volatile bool response_pending;
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
 * @retval true if reset command completed successfully, false otherwise
 */
bool encoder_device_reset(EncoderDevice *device);

/**
 * @brief Request a single response from a specific encoder
 * @param device Encoder device
 * @param cmd Command byte (ENCODER_CMD_POSITION or ENCODER_CMD_TURNS)
 * @retval true if the request was sent successfully, false otherwise
 */
bool encoder_device_request_data(EncoderDevice *device, uint8_t cmd);

/**
 * @brief Start continuous polling for a specific encoder
 * @param device Encoder device
 * @param mode Polling mode
 * @retval true if polling started successfully, false otherwise
 */
bool encoder_device_start_polling(EncoderDevice *device, EncoderPollMode mode);

/**
 * @brief Stop continuous polling for a specific encoder
 * @param device Encoder device
 * @retval None
 */
void encoder_device_stop_polling(EncoderDevice *device);

/**
 * @brief Supervise timeouts and recover polling when needed
 * @param device Encoder device
 * @retval None
 */
void encoder_device_process(EncoderDevice *device);

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
 * @brief Get the latest matched position/turns pair from a specific encoder
 * @param device Encoder device
 * @param position Pointer to store the 14-bit position value
 * @param turns Pointer to store the 14-bit signed turns value
 * @retval true if a new matched pair is available, false otherwise
 */
bool encoder_device_get_sample(EncoderDevice *device, uint16_t *position, int16_t *turns);

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
 * @retval true if reset command completed successfully, false otherwise
 */
bool encoder_reset(void);

/**
 * @brief Request encoder data once
 * @param cmd Command byte (ENCODER_CMD_POSITION or ENCODER_CMD_TURNS)
 * @retval true if the request was sent successfully, false otherwise
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
