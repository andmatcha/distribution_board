/* Continuous rotation servo control implementation */
#include "modules/servo.h"

#include "board_config.h"
#include "debug_log.h"
#include "main.h"

static TIM_HandleTypeDef *htim_servo = NULL;
static const uint32_t servo_channel = TIM_CHANNEL_2;

/*
 * TIM2 CH2 outputs a 50 Hz PWM signal on PA1.
 * A continuous rotation servo interprets the pulse width as speed:
 *   - shorter than neutral: reverse rotation
 *   - neutral: stop
 *   - longer than neutral: forward rotation
 *
 * The board_config values select a fixed medium speed in each direction.
 */
static void servo_set_pulse_width(uint16_t pulse_width_us)
{
  if (htim_servo == NULL) {
    return;
  }

  __HAL_TIM_SET_COMPARE(htim_servo, servo_channel, pulse_width_us);

#if DEBUG_LOG_ENABLED
  uint32_t timer_enabled = (htim_servo->Instance->CR1 & TIM_CR1_CEN) ? 1U : 0U;
  uint32_t channel_enabled = (htim_servo->Instance->CCER & TIM_CCER_CC2E) ? 1U : 0U;
  LOG("[SHOVEL_SERVO_PWM] TIM2_CH2 enabled=%lu channel=%lu pulse=%uus\n",
      timer_enabled,
      channel_enabled,
      pulse_width_us);
#endif
}

void servo_init(TIM_HandleTypeDef *htim)
{
  htim_servo = htim;

  LOG("[SHOVEL_SERVO_INIT] TIM Instance: 0x%08lX\n", (uint32_t)htim->Instance);
  LOG("[SHOVEL_SERVO_INIT] TIM prescaler: %lu\n", htim->Instance->PSC);
  LOG("[SHOVEL_SERVO_INIT] TIM ARR: %lu\n", htim->Instance->ARR);

  /* Set neutral before enabling PWM so the shovel does not move at startup. */
  servo_set_pulse_width(BOARD_SHOVEL_SERVO_STOP_PULSE_US);

  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(htim_servo, servo_channel);
  LOG("[SHOVEL_SERVO_INIT] HAL_TIM_PWM_Start status: %d (0=OK)\n", status);
  if (status != HAL_OK) {
    Error_Handler();
  }
}

void servo_control(ServoDirection direction)
{
  switch (direction) {
    case SERVO_DIR_FORWARD:
      servo_set_pulse_width(BOARD_SHOVEL_SERVO_FORWARD_PULSE_US);
      break;
    case SERVO_DIR_REVERSE:
      servo_set_pulse_width(BOARD_SHOVEL_SERVO_REVERSE_PULSE_US);
      break;
    case SERVO_DIR_STOP:
    default:
      servo_set_pulse_width(BOARD_SHOVEL_SERVO_STOP_PULSE_US);
      break;
  }
}
