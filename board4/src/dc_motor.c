/* DC motor control implementation */
#include "dc_motor.h"
#include <stdio.h>

// DCモーター用タイマーハンドル
static TIM_HandleTypeDef *htim_motor = NULL;

// TB67H450FNGドライバー制御:
// IN1: PWM入力 (TIM3_CH1=PB4 for Motor1, TIM3_CH2=PB5 for Motor2)
// IN2: GPIO出力 (PB6 for Motor1, PB7 for Motor2)
//
// 制御ロジック:
// IN1=L, IN2=L → 停止 (1ms後スタンバイ)
// IN1=H, IN2=L → 正転
// IN1=L, IN2=H → 逆転
// IN1=H, IN2=H → ブレーキ
//
// スタンバイ復帰: IN1またはIN2をHにすると復帰 (最大30us)

// モーター1: PB6 (IN2)
#define MOTOR1_IN2_PORT  GPIOB
#define MOTOR1_IN2_PIN   GPIO_PIN_6

// モーター2: PB7 (IN2)
#define MOTOR2_IN2_PORT  GPIOB
#define MOTOR2_IN2_PIN   GPIO_PIN_7

// TIM3のARR値 (Period = 19999, カウント0〜19999で20000段階)
// PWM周波数: 50Hz
#define TIM3_PERIOD  19999

void dc_motor_init(TIM_HandleTypeDef *htim) {
  htim_motor = htim;

  // TIM3カウンターが既に起動しているか確認
  uint32_t ccer_before = htim_motor->Instance->CCER;

  // TIM3 CH1 (PB4) - Motor1 IN1 (PWM)
  // カウンターは既にサーボ初期化で起動済みの想定
  // チャンネル出力のみを有効化（CCERレジスタのCC1Eビット）
  if (!(ccer_before & TIM_CCER_CC1E)) {
    HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_1);
  }

  // TIM3 CH2 (PB5) - Motor2 IN1 (PWM)
  if (!(ccer_before & TIM_CCER_CC2E)) {
    HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_2);
  }

  // 初期状態: 両モーター停止
  dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0);
  dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_STOP, 0);
}

void dc_motor_set(DcMotorId motor_id, DcMotorDirection direction, uint8_t duty_percent) {
  if (htim_motor == NULL) {
    return;
  }

  // デューティ比範囲チェック
  if (duty_percent > 100) {
    duty_percent = 100;
  }

  // CCR値計算 (0〜100% → 0〜999)
  uint32_t ccr = (duty_percent * TIM3_PERIOD) / 100;

  // モーター選択
  uint32_t tim_channel;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;

  if (motor_id == DC_MOTOR_1) {
    tim_channel = TIM_CHANNEL_1;
    in2_port = MOTOR1_IN2_PORT;
    in2_pin = MOTOR1_IN2_PIN;
  } else {
    tim_channel = TIM_CHANNEL_2;
    in2_port = MOTOR2_IN2_PORT;
    in2_pin = MOTOR2_IN2_PIN;
  }

  // 方向制御
  switch (direction) {
    case DC_MOTOR_DIR_STOP:
      // IN1=L, IN2=L → 停止
      __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, 0);
      HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
      break;

    case DC_MOTOR_DIR_FORWARD:
      // IN1=H(PWM), IN2=L → 正転
      HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
      // スタンバイ復帰待機 (30us)
      for (volatile int i = 0; i < 100; i++);
      __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, ccr);
      break;

    case DC_MOTOR_DIR_REVERSE:
      // IN1=L, IN2=H → 逆転
      __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, 0);
      HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
      // スタンバイ復帰待機 (30us)
      for (volatile int i = 0; i < 100; i++);
      break;

    case DC_MOTOR_DIR_BRAKE:
      // IN1=H, IN2=H → ブレーキ
      HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, TIM3_PERIOD);
      break;

    default:
      // デフォルトは停止
      __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, 0);
      HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
      break;
  }
}

void dc_motor_push() {
  // キーボードニョッキ動作: 出っ張る→引っ込む
  const uint8_t speed = 100;        // モーター速度 (%)
  const uint32_t push_time = 1000;  // 出っ張る時間 (ms)
  const uint32_t pull_time = 1000;  // 引っ込む時間 (ms)

  // 正転: 出っ張る
  dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_FORWARD, speed);
  HAL_Delay(push_time);

  // 逆転: 引っ込む
  dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_REVERSE, speed);
  HAL_Delay(pull_time);

  // 停止
  dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0);
}
