/* LED control public API */
#ifndef LED_H
#define LED_H

#include "stm32f1xx_hal.h"

typedef enum {
  LED_COLOR_GREEN = 0,
  LED_COLOR_YELLOW,
  LED_COLOR_RED
} LedColor;

typedef enum {
  LED_STATE_OFF = 0,
  LED_STATE_ON  = 1
} LedState;

void led_set(LedColor color, LedState state);

#endif /* LED_H */


