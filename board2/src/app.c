#include "app.h"

#include "modules/can_control.h"
#include "modules/encoder_can_publisher.h"
#include "modules/led.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

void init(void)
{
  led_set(LED_COLOR_RED, LED_STATE_OFF);
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);

  can_control_init(&hcan);
  encoder_can_publisher_init(&huart1);
}

void poll(void)
{
  encoder_can_publisher_process();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)encoder_can_publisher_uart_rx_complete_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  (void)encoder_can_publisher_uart_error_callback(huart);
}
