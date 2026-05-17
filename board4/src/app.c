#include "app.h"

#include "modules/can_control.h"
#include "modules/encoder_can_publisher.h"
#include "modules/ina219.h"
#include "modules/led.h"
#include "modules/servo.h"

extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

void init(void)
{
  led_set(LED_COLOR_RED, LED_STATE_OFF);
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);

  servo_init(&htim2);
  ina219_init(&hi2c1);
  can_control_init(&hcan);
  encoder_can_publisher_init(&huart1);
}

void poll(void)
{
  can_control_process_rx();
  encoder_can_publisher_process();
  ina219_process();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)encoder_can_publisher_uart_rx_complete_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  (void)encoder_can_publisher_uart_error_callback(huart);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_handle)
{
  can_control_rx_callback(hcan_handle);
}
