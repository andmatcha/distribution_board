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

/**
 * @brief Initialize encoder module
 * @param huart Pointer to UART handle for encoder communication
 * @retval None
 */
void encoder_init(UART_HandleTypeDef *huart);

/**
 * @brief Request encoder position data
 * @note This function sends command to encoder and switches to RX mode
 * @retval None
 */
void encoder_request_position(void);

/**
 * @brief Get latest encoder position data
 * @param position Pointer to store the 14-bit position value
 * @retval true if new data is available, false otherwise
 */
bool encoder_get_position(uint16_t *position);

/**
 * @brief UART RX complete callback (called from HAL)
 * @param huart Pointer to UART handle
 * @retval None
 */
void encoder_rx_complete_callback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback (called from HAL)
 * @param huart Pointer to UART handle
 * @retval None
 */
void encoder_error_callback(UART_HandleTypeDef *huart);

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
