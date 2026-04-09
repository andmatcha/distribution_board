#ifndef BASE_HORIZON_H
#define BASE_HORIZON_H

#include "main.h"

/**
 * @brief Base Horizon 機能を初期化する
 * @details エンコーダと内部状態を初期化する
 * @param[in] encoder_uart Base Horizon 用エンコーダに接続されたUART
 * @retval なし
 */
void base_horizon_init(UART_HandleTypeDef *encoder_uart);

/**
 * @brief Base Horizon の1周期分の処理を実行する
 * @details エンコーダ取得、距離計算、異常値判定、必要に応じたCAN送信を行う
 * @param なし
 * @retval なし
 */
void base_horizon_process(void);

#endif /* BASE_HORIZON_H */
