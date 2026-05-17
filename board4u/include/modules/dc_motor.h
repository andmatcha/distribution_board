#ifndef BOARD4U_MODULES_DC_MOTOR_H
#define BOARD4U_MODULES_DC_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef enum {
  DC_MOTOR_1 = 0,
  DC_MOTOR_2 = 1
} DcMotorId;

typedef enum {
  DC_MOTOR_DIR_STOP = 0,
  DC_MOTOR_DIR_FORWARD = 1,
  DC_MOTOR_DIR_REVERSE = 2,
  DC_MOTOR_DIR_BRAKE = 3
} DcMotorDirection;

void dc_motor_init(TIM_HandleTypeDef *htim);
void dc_motor_set(DcMotorId motor_id, DcMotorDirection direction, uint8_t duty_percent);
void dc_motor_process(void);
void dc_motor_push(void);
bool dc_motor_push_is_active(void);

#endif /* BOARD4U_MODULES_DC_MOTOR_H */
