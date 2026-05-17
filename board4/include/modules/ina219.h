/* INA219 current/voltage monitor public API */
#ifndef INA219_H
#define INA219_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t bus_voltage_mv;
  int32_t shunt_voltage_uv;
  int32_t current_ua;
  uint32_t power_uw;
} Ina219Measurement;

void ina219_init(I2C_HandleTypeDef *hi2c);
void ina219_process(void);
bool ina219_read(Ina219Measurement *measurement);

#endif /* INA219_H */
