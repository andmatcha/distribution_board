// LED control implementation shared by the distribution boards.
#include "modules/led.h"
#include "board_config.h"

#ifndef BOARD_LED_GPIO_PORT
#define BOARD_LED_GPIO_PORT GPIOA
#endif

#ifndef BOARD_LED_GREEN_PIN
#define BOARD_LED_GREEN_PIN GPIO_PIN_5
#endif

#ifndef BOARD_LED_YELLOW_PIN
#define BOARD_LED_YELLOW_PIN GPIO_PIN_6
#endif

#ifndef BOARD_LED_RED_PIN
#define BOARD_LED_RED_PIN GPIO_PIN_7
#endif

static uint16_t led_color_to_pin(LedColor color) {
  switch (color) {
  case LED_COLOR_GREEN:
    return BOARD_LED_GREEN_PIN;
  case LED_COLOR_YELLOW:
    return BOARD_LED_YELLOW_PIN;
  case LED_COLOR_RED:
    return BOARD_LED_RED_PIN;
  default:
    return 0;
  }
}

void led_set(LedColor color, LedState state) {
  uint16_t pin = led_color_to_pin(color);
  if (pin == 0) {
    return;
  }
  GPIO_PinState pin_state = (state == LED_STATE_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, pin, pin_state);
}
