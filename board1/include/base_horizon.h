#ifndef BASE_HORIZON_H
#define BASE_HORIZON_H

#include "main.h"

/**
 * @brief Base Horizon 機能を初期化する
 * @details エンコーダ、CAN送信ヘッダ、内部状態を初期化する
 * @param[in] encoder_uart Base Horizon 用エンコーダに接続されたUART
 * @param[in] can_handle Base Horizon の結果送信に使うCANハンドル
 * @retval なし
 */
void base_horizon_init(UART_HandleTypeDef *encoder_uart, CAN_HandleTypeDef *can_handle);

/**
 * @brief Base Horizon の1周期分の処理を実行する
 * @details エンコーダ取得、距離計算、異常値判定、必要に応じたCAN送信を行う
 * @param なし
 * @retval なし
 */
void base_horizon_process(void);

/**
 * @brief Base Horizon 用の CAN mailbox0 送信中止通知を処理する
 * @param[in] can_handle 割り込み元のCANハンドル
 * @retval なし
 */
void base_horizon_handle_can_tx_mailbox0_abort(CAN_HandleTypeDef *can_handle);

/**
 * @brief Base Horizon 用の CAN mailbox1 送信中止通知を処理する
 * @param[in] can_handle 割り込み元のCANハンドル
 * @retval なし
 */
void base_horizon_handle_can_tx_mailbox1_abort(CAN_HandleTypeDef *can_handle);

/**
 * @brief Base Horizon 用の CAN mailbox2 送信中止通知を処理する
 * @param[in] can_handle 割り込み元のCANハンドル
 * @retval なし
 */
void base_horizon_handle_can_tx_mailbox2_abort(CAN_HandleTypeDef *can_handle);

/**
 * @brief Base Horizon 用の CAN エラー通知を処理する
 * @param[in] can_handle 割り込み元のCANハンドル
 * @retval なし
 */
void base_horizon_handle_can_error(CAN_HandleTypeDef *can_handle);

#endif /* BASE_HORIZON_H */
