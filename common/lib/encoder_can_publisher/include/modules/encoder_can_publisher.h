/* Encoder-to-CAN publisher public API */
#ifndef ENCODER_CAN_PUBLISHER_H
#define ENCODER_CAN_PUBLISHER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

void encoder_can_publisher_init(UART_HandleTypeDef *huart);
void encoder_can_publisher_process(void);
bool encoder_can_publisher_uart_rx_complete_callback(UART_HandleTypeDef *huart);
bool encoder_can_publisher_uart_error_callback(UART_HandleTypeDef *huart);

#endif /* ENCODER_CAN_PUBLISHER_H */
