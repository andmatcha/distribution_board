/* Servo motor control public API */
#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"

typedef enum {
  SERVO_DIR_OPEN = 0,   // サーボ引っ込む
  SERVO_DIR_CLOSE = 1,  // サーボ出っ張る
  SERVO_DIR_STOP = 2    // サーボ停止
} ServoDirection;

typedef enum {
  SERVO_MODE_NORMAL = 0,   // 通常モード
  SERVO_MODE_FAST = 1      // 高速モード
} ServoMode;

/**
 * @brief servo motor control
 *
 *
 * @param direction 方向 (OPEN or CLOSE)
 * @param mode モード (NORMAL or FAST)
 */
void servo_control(ServoDirection direction, ServoMode mode);

/**
 * @brief サーボモーター初期化
 * @param htim TIM3ハンドル
 */
void servo_init(TIM_HandleTypeDef *htim);

/**
 * @brief サーボモーター角度設定
 * @param angle 角度 (0〜270度)
 */
void servo_set_angle(uint16_t angle);

#endif /* SERVO_H */
