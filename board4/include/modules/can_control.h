/* CAN control public API */
#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// CAN ID定義
#define CAN_ID_DC   0x208  // ニョッキDCモーター
#define CAN_ID_SERVO  0x1FF  // 把持サーボモーター
#define CAN_ID_ENCODER 0x304U  // エンコーダー値送信

/**
 * @brief CAN受信初期化
 * @param hcan CANハンドル
 * @param htim TIM3ハンドル (モーター制御用)
 */
void can_control_init(CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim);

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

/**
 * @brief CAN受信コールバック (HAL_CAN_RxFifo0MsgPendingCallbackから呼ばれる)
 * @param hcan CANハンドル
 */
void can_control_rx_callback(CAN_HandleTypeDef *hcan);

#endif /* CAN_CONTROL_H */
