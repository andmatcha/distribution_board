#include "base_can_scheduler.h"

typedef struct
{
  bool pending;
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
} BaseCanFrameSlot;

static CAN_HandleTypeDef *base_can_scheduler_handle = NULL;
static BaseCanFrameSlot base_can_slots[BASE_CAN_CHANNEL_COUNT];
static BaseCanChannel base_can_next_channel = BASE_CAN_CHANNEL_HORIZON;
static uint32_t base_can_bus_off_until_tick = 0U;

#define BASE_CAN_BUS_OFF_COOLDOWN_MS 1000U

static void base_can_scheduler_try_recover(void);
static BaseCanChannel base_can_scheduler_other_channel(BaseCanChannel channel);
static bool base_can_scheduler_send(BaseCanChannel channel);

static void base_can_scheduler_try_recover(void)
{
  HAL_CAN_StateTypeDef state;

  if (base_can_scheduler_handle == NULL) {
    return;
  }

  state = HAL_CAN_GetState(base_can_scheduler_handle);
  if (state == HAL_CAN_STATE_ERROR || state == HAL_CAN_STATE_READY) {
    if (HAL_CAN_Stop(base_can_scheduler_handle) == HAL_OK) {
      (void)HAL_CAN_ResetError(base_can_scheduler_handle);
      (void)HAL_CAN_Start(base_can_scheduler_handle);
    }
  }
}

static BaseCanChannel base_can_scheduler_other_channel(BaseCanChannel channel)
{
  if (channel == BASE_CAN_CHANNEL_HORIZON) {
    return BASE_CAN_CHANNEL_ROLL;
  }

  return BASE_CAN_CHANNEL_HORIZON;
}

static bool base_can_scheduler_send(BaseCanChannel channel)
{
  uint32_t tx_mailbox;
  uint32_t error_code;
  HAL_StatusTypeDef status;

  status = HAL_CAN_AddTxMessage(base_can_scheduler_handle,
                                &base_can_slots[channel].header,
                                base_can_slots[channel].data,
                                &tx_mailbox);
  if (status != HAL_OK) {
    error_code = HAL_CAN_GetError(base_can_scheduler_handle);
    if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
      base_can_bus_off_until_tick = HAL_GetTick() + BASE_CAN_BUS_OFF_COOLDOWN_MS;
    }

    base_can_scheduler_try_recover();
    return false;
  }

  base_can_slots[channel].pending = false;
  return true;
}

void base_can_scheduler_init(CAN_HandleTypeDef *can_handle)
{
  uint32_t channel_index;

  base_can_scheduler_handle = can_handle;
  base_can_next_channel = BASE_CAN_CHANNEL_HORIZON;
  base_can_bus_off_until_tick = 0U;

  for (channel_index = 0U; channel_index < BASE_CAN_CHANNEL_COUNT; channel_index++) {
    base_can_slots[channel_index].pending = false;
  }

  if (base_can_scheduler_handle != NULL && HAL_CAN_Start(base_can_scheduler_handle) != HAL_OK) {
    base_can_scheduler_try_recover();
  }
}

bool base_can_scheduler_stage(BaseCanChannel channel,
                              uint32_t std_id,
                              const uint8_t *data,
                              uint8_t dlc)
{
  uint32_t data_index;

  if (base_can_scheduler_handle == NULL ||
      channel >= BASE_CAN_CHANNEL_COUNT ||
      data == NULL ||
      dlc > 8U) {
    return false;
  }

  base_can_slots[channel].header.StdId = std_id;
  base_can_slots[channel].header.ExtId = 0U;
  base_can_slots[channel].header.IDE = CAN_ID_STD;
  base_can_slots[channel].header.RTR = CAN_RTR_DATA;
  base_can_slots[channel].header.DLC = dlc;
  base_can_slots[channel].header.TransmitGlobalTime = DISABLE;

  for (data_index = 0U; data_index < dlc; data_index++) {
    base_can_slots[channel].data[data_index] = data[data_index];
  }

  base_can_slots[channel].pending = true;
  return true;
}

void base_can_scheduler_process(void)
{
  BaseCanChannel selected_channel;
  BaseCanChannel alternate_channel;
  uint32_t now_tick;

  if (base_can_scheduler_handle == NULL) {
    return;
  }

  now_tick = HAL_GetTick();
  if ((int32_t)(now_tick - base_can_bus_off_until_tick) < 0) {
    return;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(base_can_scheduler_handle) == 0U) {
    return;
  }

  selected_channel = base_can_next_channel;
  alternate_channel = base_can_scheduler_other_channel(selected_channel);

  if (!base_can_slots[selected_channel].pending) {
    if (!base_can_slots[alternate_channel].pending) {
      return;
    }
    selected_channel = alternate_channel;
  }

  if (base_can_scheduler_send(selected_channel)) {
    base_can_next_channel = base_can_scheduler_other_channel(selected_channel);
  }
}
