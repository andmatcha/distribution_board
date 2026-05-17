#include "modules/dc_motor.h"

#define MOTOR1_IN2_PORT GPIOB
#define MOTOR1_IN2_PIN GPIO_PIN_6
#define MOTOR2_IN2_PORT GPIOB
#define MOTOR2_IN2_PIN GPIO_PIN_7

#define PUSH_SPEED_PERCENT 100U
#define PUSH_FORWARD_TIME_MS 2000U
#define PUSH_COAST_TIME_MS 100U
#define PUSH_REVERSE_TIME_MS 2000U

typedef enum {
  DC_MOTOR_PUSH_STATE_IDLE = 0,
  DC_MOTOR_PUSH_STATE_FORWARD,
  DC_MOTOR_PUSH_STATE_COAST,
  DC_MOTOR_PUSH_STATE_REVERSE
} DcMotorPushState;

static TIM_HandleTypeDef *htim_motor = NULL;
static volatile bool push_requested = false;
static volatile DcMotorPushState push_state = DC_MOTOR_PUSH_STATE_IDLE;
static uint32_t push_phase_started_tick = 0U;

static uint32_t dc_motor_pwm_period(void)
{
  uint32_t period;

  if (htim_motor == NULL) {
    return 0U;
  }

  period = __HAL_TIM_GET_AUTORELOAD(htim_motor);
  return (period == 0U) ? 1U : period;
}

void dc_motor_init(TIM_HandleTypeDef *htim)
{
  uint32_t ccer_before;

  htim_motor = htim;
  push_requested = false;
  push_state = DC_MOTOR_PUSH_STATE_IDLE;
  push_phase_started_tick = 0U;

  if (htim_motor == NULL) {
    return;
  }

  ccer_before = htim_motor->Instance->CCER;

  if ((ccer_before & TIM_CCER_CC1E) == 0U) {
    (void)HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_1);
  }

  if ((ccer_before & TIM_CCER_CC2E) == 0U) {
    (void)HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_2);
  }

  dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0U);
  dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_STOP, 0U);
}

void dc_motor_set(DcMotorId motor_id, DcMotorDirection direction, uint8_t duty_percent)
{
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  uint32_t tim_channel;
  uint32_t period;
  uint32_t ccr;

  if (htim_motor == NULL) {
    return;
  }

  if (duty_percent > 100U) {
    duty_percent = 100U;
  }

  period = dc_motor_pwm_period();
  ccr = ((uint32_t)duty_percent * period) / 100U;

  if (motor_id == DC_MOTOR_1) {
    tim_channel = TIM_CHANNEL_1;
    in2_port = MOTOR1_IN2_PORT;
    in2_pin = MOTOR1_IN2_PIN;
  } else {
    tim_channel = TIM_CHANNEL_2;
    in2_port = MOTOR2_IN2_PORT;
    in2_pin = MOTOR2_IN2_PIN;
  }

  switch (direction) {
  case DC_MOTOR_DIR_STOP:
    __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, 0U);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    break;

  case DC_MOTOR_DIR_FORWARD:
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 100; i++) {
    }
    __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, ccr);
    break;

  case DC_MOTOR_DIR_REVERSE:
    __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, 0U);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 100; i++) {
    }
    break;

  case DC_MOTOR_DIR_BRAKE:
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, period);
    break;

  default:
    __HAL_TIM_SET_COMPARE(htim_motor, tim_channel, 0U);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    break;
  }
}

void dc_motor_process(void)
{
  uint32_t now_tick;

  if (htim_motor == NULL) {
    return;
  }

  now_tick = HAL_GetTick();

  switch (push_state) {
  case DC_MOTOR_PUSH_STATE_IDLE:
    if (!push_requested) {
      return;
    }
    push_state = DC_MOTOR_PUSH_STATE_FORWARD;
    push_requested = false;
    push_phase_started_tick = now_tick;
    dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_REVERSE, PUSH_SPEED_PERCENT);
    break;

  case DC_MOTOR_PUSH_STATE_FORWARD:
    if ((uint32_t)(now_tick - push_phase_started_tick) < PUSH_FORWARD_TIME_MS) {
      return;
    }
    push_phase_started_tick = now_tick;
    push_state = DC_MOTOR_PUSH_STATE_COAST;
    dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0U);
    break;

  case DC_MOTOR_PUSH_STATE_COAST:
    if ((uint32_t)(now_tick - push_phase_started_tick) < PUSH_COAST_TIME_MS) {
      return;
    }
    push_phase_started_tick = now_tick;
    push_state = DC_MOTOR_PUSH_STATE_REVERSE;
    dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_FORWARD, PUSH_SPEED_PERCENT);
    break;

  case DC_MOTOR_PUSH_STATE_REVERSE:
    if ((uint32_t)(now_tick - push_phase_started_tick) < PUSH_REVERSE_TIME_MS) {
      return;
    }
    dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0U);
    push_state = DC_MOTOR_PUSH_STATE_IDLE;
    break;

  default:
    dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0U);
    push_requested = false;
    push_state = DC_MOTOR_PUSH_STATE_IDLE;
    break;
  }
}

void dc_motor_push(void)
{
  if (htim_motor == NULL) {
    return;
  }

  if (push_state == DC_MOTOR_PUSH_STATE_IDLE) {
    push_requested = true;
  }
}

bool dc_motor_push_is_active(void)
{
  return push_requested || (push_state != DC_MOTOR_PUSH_STATE_IDLE);
}
