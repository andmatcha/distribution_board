/* DC motor control public API */
#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

// モーター番号
typedef enum {
  DC_MOTOR_1 = 0,  // TIM3_CH1(PB4) + PB6
  DC_MOTOR_2 = 1   // TIM3_CH2(PB5) + PB7
} DcMotorId;

// モーター方向
typedef enum {
  DC_MOTOR_DIR_STOP    = 0,  // 停止
  DC_MOTOR_DIR_FORWARD = 1,  // 正転
  DC_MOTOR_DIR_REVERSE = 2,  // 逆転
  DC_MOTOR_DIR_BRAKE   = 3   // ブレーキ
} DcMotorDirection;

/**
 * @brief DCモーター初期化
 * @param htim TIM3ハンドル
 */
void dc_motor_init(TIM_HandleTypeDef *htim);

/**
 * @brief DCモーター制御
 * @param motor_id モーター番号 (DC_MOTOR_1 or DC_MOTOR_2)
 * @param direction 方向 (STOP/FORWARD/REVERSE/BRAKE)
 * @param duty_percent デューティ比 (0〜100%)
 */
void dc_motor_set(DcMotorId motor_id, DcMotorDirection direction, uint8_t duty_percent);

/**
 * @brief キーボードニョッキ動作の周期処理
 */
void dc_motor_process(void);

/**
 * @brief キーボードニョッキ動作を開始要求する (非ブロッキング)
 * DC_MOTOR_1を使用して、dc_motor_process() 内で正転→逆転の動作を実行
 */
void dc_motor_push(void);

/**
 * @brief キーボードニョッキ動作が要求中または実行中かどうか
 */
bool dc_motor_push_is_active(void);

#endif /* DC_MOTOR_H */
