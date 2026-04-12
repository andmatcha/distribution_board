#include "app.h"

#include "modules/base_can_scheduler.h"
#include "modules/base_horizon.h"
#include "modules/base_roll.h"
#include "modules/encoder.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void init(void)
{
  base_can_scheduler_init(&hcan);
  base_horizon_init(&huart1);
  base_roll_init(&huart2);
}

void poll(void)
{
  base_horizon_process();
  base_roll_process();
  base_can_scheduler_process();
  HAL_Delay(1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  encoder_rx_complete_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  encoder_error_callback(huart);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan_handle)
{
  (void)hcan_handle;
  Error_Handler();
}
