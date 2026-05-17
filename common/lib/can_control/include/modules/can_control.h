/* CAN control public API */
#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

#include "stm32f1xx_hal.h"
#include "board_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef BOARD_CAN_ID_ENCODER
#error "BOARD_CAN_ID_ENCODER must be defined in board_config.h"
#endif

#ifndef BOARD_CAN_CONTROL_ENABLE_RX
#define BOARD_CAN_CONTROL_ENABLE_RX 0
#endif

// CAN ID definitions
#define CAN_ID_ENCODER BOARD_CAN_ID_ENCODER

#if BOARD_CAN_CONTROL_ENABLE_RX
#ifndef BOARD_CAN_ID_DC
#error "BOARD_CAN_ID_DC must be defined when BOARD_CAN_CONTROL_ENABLE_RX is enabled"
#endif
#ifndef BOARD_CAN_ID_SERVO
#error "BOARD_CAN_ID_SERVO must be defined when BOARD_CAN_CONTROL_ENABLE_RX is enabled"
#endif
#define CAN_ID_DC BOARD_CAN_ID_DC
#define CAN_ID_SERVO BOARD_CAN_ID_SERVO
#endif

/**
 * @brief CAN initialize
 * @param hcan CAN handle
 */
void can_control_init(CAN_HandleTypeDef *hcan);

/**
 * @brief Queue encoder position for CAN transmission
 * @param position 16-bit position value
 * @retval true if queued, false if buffer full
 */
bool can_control_enqueue_encoder_position(uint16_t position);

/**
 * @brief Process queued CAN transmissions
 * @note Non-blocking. Sends only while TX mailboxes are available.
 * @retval None
 */
void can_control_process_tx(void);

#if BOARD_CAN_CONTROL_ENABLE_RX
/**
 * @brief Process queued CAN RX commands outside the interrupt context
 * @retval None
 */
void can_control_process_rx(void);

/**
 * @brief CAN RX callback helper
 * @param hcan CAN handle
 * @note Intended to be called from HAL_CAN_RxFifo0MsgPendingCallback.
 */
void can_control_rx_callback(CAN_HandleTypeDef *hcan);
#endif

#endif /* CAN_CONTROL_H */
