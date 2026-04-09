#ifndef BASE_ROLL_H
#define BASE_ROLL_H

#include "main.h"

/**
 * @brief Base Roll 機能を初期化する
 * @param[in] encoder_uart Base Roll 用エンコーダに接続されたUART
 * @retval なし
 */
void base_roll_init(UART_HandleTypeDef *encoder_uart);

/**
 * @brief Base Roll の1周期分の処理を実行する
 * @details エンコーダ取得と必要に応じたCAN送信を行う
 * @retval なし
 */
void base_roll_process(void);

#endif /* BASE_ROLL_H */
