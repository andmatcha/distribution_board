/**
 ******************************************************************************
 * @file           : encoder.h
 * @brief          : Encoder reading module header
 ******************************************************************************
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define ENCODER_CMD_POSITION 0x54
#define ENCODER_CMD_TURNS 0x55
#define ENCODER_CMD_RESET_1 0x56
#define ENCODER_CMD_RESET_2 0x75

/**
 * @brief Initialize encoder module
 * @param huart Pointer to UART handle for encoder communication
 * @retval None
 */
void encoder_init(UART_HandleTypeDef *huart);

/**
 * @brief Reset encoder position/turns
 * @note This function sends 0x56 and 0x75 consecutively
 * @retval None
 */
void encoder_reset(void);

/**
 * @brief Request encoder data
 * @param cmd Command byte (ENCODER_CMD_POSITION or ENCODER_CMD_TURNS)
 * @note This function sends command to encoder and switches to RX mode
 * @retval None
 */
void encoder_request_data(uint8_t cmd);

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

#endif /* ENCODER_H */
