// LED制御API（列挙型）と文字列入力ラッパー
#include "led.h"

static uint16_t led_color_to_pin(LedColor color) {
  switch (color) {
  case LED_COLOR_GREEN:
    return GPIO_PIN_5;
  case LED_COLOR_YELLOW:
    return GPIO_PIN_6;
  case LED_COLOR_RED:
    return GPIO_PIN_7;
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
  HAL_GPIO_WritePin(GPIOA, pin, pin_state);
}


