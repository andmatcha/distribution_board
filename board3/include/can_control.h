/* CAN control public API */
#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// CAN ID定義
#define CAN_ID_ENCODER 0x303U  // エンコーダー値送信

/**
 * @brief CAN送信初期化
 * @param hcan CANハンドル
 */
void can_control_init(CAN_HandleTypeDef *hcan);

/**
 * @brief エンコーダー値送信用バッファへ積む
 * @param position 送信する16bit値
 * @retval true if queued, false if buffer full
 */
bool can_control_enqueue_encoder_position(uint16_t position);

/**
 * @brief 送信バッファからCAN送信を進める
 * @note 非ブロッキングで、空きメールボックス分だけ送信する
 * @retval None
 */
void can_control_process_tx(void);

#endif /* CAN_CONTROL_H */
