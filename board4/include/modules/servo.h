/* Continuous rotation servo control public API */
#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"

typedef enum {
  SERVO_DIR_FORWARD = 0,
  SERVO_DIR_REVERSE = 1,
  SERVO_DIR_STOP = 2
} ServoDirection;

/**
 * @brief Control the shovel's continuous rotation servo at a fixed speed.
 * @param direction Forward, reverse, or stop.
 */
void servo_control(ServoDirection direction);

/**
 * @brief Initialize the shovel servo and start it at the neutral pulse width.
 * @param htim TIM2 handle.
 */
void servo_init(TIM_HandleTypeDef *htim);

#endif /* SERVO_H */
