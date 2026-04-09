/* Servo motor control implementation */
#include "servo.h"
#include <stdio.h>

// サーボモーター用タイマーハンドル
static TIM_HandleTypeDef *htim_servo = NULL;
static const uint32_t servo_channel = TIM_CHANNEL_2;

// サーボPWM周波数: 50Hz (20ms周期)
// TIM2設定: 32MHz / (prescaler+1) / (period+1) = PWM周波数
// 現在の設定: prescaler=31, period=19999 → 32MHz/32/20000 = 50Hz
//
// サーボモーター仕様:
// - 制御角: 270度
// - パルス幅範囲: 0.5ms ～ 2.5ms
//
// 50Hz PWMの場合、1周期 = 20ms
// サーボ制御パルス幅:
// - 0度:   0.5ms → CCR = (0.5ms / 20ms) × 20000 = 500
// - 135度: 1.5ms → CCR = (1.5ms / 20ms) × 20000 = 1500
// - 270度: 2.5ms → CCR = (2.5ms / 20ms) × 20000 = 2500

#define SERVO_MIN_CCR  500   // 0.5ms パルス幅 (0度)
#define SERVO_MAX_CCR  2500  // 2.5ms パルス幅 (270度)
#define ANGLE_MIN     0     // 最小角度 (度)
#define ANGLE_MAX     270   // 最大角度 (度)

uint16_t current_angle = 0; // 初期角度

void servo_set_angle(uint16_t angle);

void servo_init(TIM_HandleTypeDef *htim) {
  htim_servo = htim;

  printf("[SERVO_INIT] TIM Instance: 0x%08lX\n", (uint32_t)htim->Instance);
  printf("[SERVO_INIT] TIM prescaler: %lu\n", htim->Instance->PSC);
  printf("[SERVO_INIT] TIM ARR: %lu\n", htim->Instance->ARR);
  printf("[SERVO_INIT] TIM CR1: 0x%04lX\n", htim->Instance->CR1);

  // TIM2 CH2 (PA1)をPWM出力として開始
  // HAL_TIM_PWM_Startはタイマーを自動的に有効化する
  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(htim_servo, servo_channel);
  printf("[SERVO_INIT] HAL_TIM_PWM_Start status: %d (0=OK)\n", status);
  printf("[SERVO_INIT] TIM CR1 after PWM_Start: 0x%04lX (bit0 should be 1 for counter enable)\n", htim_servo->Instance->CR1);

  // 初期位置
  servo_set_angle(current_angle);
}

void servo_set_angle(uint16_t angle) {
  if (htim_servo == NULL) {
    // printf("[SERVO] ERROR: htim_servo is NULL!\n");
    return;
  }

  // 角度範囲チェック (0〜270)
  if (angle < ANGLE_MIN) {
    angle = ANGLE_MIN;
  }
  if (angle > ANGLE_MAX) {
    angle = ANGLE_MAX;
  }

  // CCR値計算: 角度に応じたデューティ比
  // angleが0〜270の範囲を、CCRのSERVO_MIN_CCR〜SERVO_MAX_CCRに変換
  uint32_t ccr = SERVO_MIN_CCR + ((angle * (SERVO_MAX_CCR - SERVO_MIN_CCR)) / ANGLE_MAX);

  // CH2のCCR値を設定
  __HAL_TIM_SET_COMPARE(htim_servo, servo_channel, ccr);

  uint32_t pulse_width_us = ccr;
  uint32_t timer_enabled = (htim_servo->Instance->CR1 & TIM_CR1_CEN) ? 1U : 0U;
  uint32_t channel_enabled = (htim_servo->Instance->CCER & TIM_CCER_CC2E) ? 1U : 0U;
  printf("[SERVO_PWM] TIM2_CH2 enabled=%lu channel=%lu angle=%u ccr=%lu pulse=%luus\n",
         timer_enabled,
         channel_enabled,
         angle,
         ccr,
         pulse_width_us);
}

// サーボモーター制御 呼ばれるたびに角度を少しずつ変化させる
void servo_control(ServoDirection direction, ServoMode mode) {
  uint16_t angle_step = (mode == SERVO_MODE_FAST) ? 10 : 4; // 高速モード : 通常モード

  if (direction == SERVO_DIR_OPEN) {
    current_angle = current_angle + angle_step;
    if (current_angle > ANGLE_MAX) {
      current_angle = ANGLE_MAX;
    }
  } else if (direction == SERVO_DIR_CLOSE) {
    if (current_angle < angle_step) {
      current_angle = ANGLE_MIN;
    } else {
      current_angle = current_angle - angle_step;
    }
  } else {
    // SERVO_DIR_STOP: 現在の角度を保持（何もしない）
    return;
  }

  servo_set_angle(current_angle);
}
