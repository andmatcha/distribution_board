#ifndef BASE_CAN_SCHEDULER_H
#define BASE_CAN_SCHEDULER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  BASE_CAN_CHANNEL_HORIZON = 0,
  BASE_CAN_CHANNEL_ROLL,
  BASE_CAN_CHANNEL_COUNT
} BaseCanChannel;

/**
 * @brief Shared CAN scheduler for Base Horizon / Base Roll
 * @param can_handle CAN handle used for transmission
 * @retval None
 */
void base_can_scheduler_init(CAN_HandleTypeDef *can_handle);

/**
 * @brief Stage the latest frame for a channel
 * @param channel Logical source channel
 * @param std_id CAN standard ID
 * @param data Payload bytes
 * @param dlc Payload length
 * @retval true if staged successfully, false otherwise
 */
bool base_can_scheduler_stage(BaseCanChannel channel,
                              uint32_t std_id,
                              const uint8_t *data,
                              uint8_t dlc);

/**
 * @brief Advance CAN transmission by one frame
 * @details Alternates between Horizon and Roll when both are pending.
 * @retval None
 */
void base_can_scheduler_process(void);

/**
 * @brief Handle asynchronous CAN error notifications
 * @param can_handle CAN handle that raised the error
 * @retval None
 */
void base_can_scheduler_handle_error(CAN_HandleTypeDef *can_handle);

#endif /* BASE_CAN_SCHEDULER_H */
