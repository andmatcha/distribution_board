/* CAN control implementation */
#include "modules/can_control.h"
#include "main.h"
#include <stdio.h>

#define CAN_TX_QUEUE_SIZE 8U
#define CAN_BUS_OFF_COOLDOWN_MS 1000U

static CAN_HandleTypeDef *hcan_ctrl = NULL;
static volatile uint16_t encoder_tx_queue[CAN_TX_QUEUE_SIZE];
static volatile uint8_t encoder_tx_head = 0;
static volatile uint8_t encoder_tx_tail = 0;
static uint32_t can_bus_off_until_tick = 0U;

static bool can_control_start(void);
static void can_control_handle_error(void);
static bool can_tx_queue_peek(uint16_t *position);
static void can_tx_queue_drop(void);

// CAN送信初期化
void can_control_init(CAN_HandleTypeDef *hcan) {
  hcan_ctrl = hcan;
  encoder_tx_head = 0U;
  encoder_tx_tail = 0U;
  can_bus_off_until_tick = 0U;
  (void)can_control_start();
}

bool can_control_enqueue_encoder_position(uint16_t position) {
  uint8_t next_head = (uint8_t)((encoder_tx_head + 1U) % CAN_TX_QUEUE_SIZE);

  if (next_head == encoder_tx_tail) {
    return false;
  }

  encoder_tx_queue[encoder_tx_head] = position;
  encoder_tx_head = next_head;
  return true;
}

void can_control_process_tx(void) {
  CAN_TxHeaderTypeDef tx_header = {0};
  uint16_t position;
  uint32_t error_code;
  uint32_t now_tick;

  if (hcan_ctrl == NULL) {
    return;
  }

  now_tick = HAL_GetTick();
  if ((int32_t)(now_tick - can_bus_off_until_tick) < 0) {
    return;
  }

  if (!can_control_start()) {
    return;
  }

  error_code = HAL_CAN_GetError(hcan_ctrl);
  if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
    can_control_handle_error();
    return;
  }

  tx_header.StdId = CAN_ID_ENCODER;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 2;
  tx_header.TransmitGlobalTime = DISABLE;

  while ((HAL_CAN_GetTxMailboxesFreeLevel(hcan_ctrl) > 0U) && can_tx_queue_peek(&position)) {
    uint8_t tx_data[2];
    uint32_t tx_mailbox;

    tx_data[0] = (uint8_t)(position >> 8);
    tx_data[1] = (uint8_t)(position & 0xFFU);

    if (HAL_CAN_AddTxMessage(hcan_ctrl, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
      can_control_handle_error();
      break;
    }

    can_tx_queue_drop();
  }
}

static bool can_control_start(void) {
  HAL_CAN_StateTypeDef state;

  if (hcan_ctrl == NULL) {
    return false;
  }

  state = HAL_CAN_GetState(hcan_ctrl);
  if (state == HAL_CAN_STATE_LISTENING) {
    return true;
  }

  if (state == HAL_CAN_STATE_RESET || state == HAL_CAN_STATE_ERROR) {
    (void)HAL_CAN_Init(hcan_ctrl);
  }

  if (HAL_CAN_GetState(hcan_ctrl) == HAL_CAN_STATE_READY) {
    if (HAL_CAN_Start(hcan_ctrl) == HAL_OK) {
      return true;
    }
  }

  return (HAL_CAN_GetState(hcan_ctrl) == HAL_CAN_STATE_LISTENING);
}

static void can_control_handle_error(void) {
  uint32_t error_code;

  if (hcan_ctrl == NULL) {
    return;
  }

  error_code = HAL_CAN_GetError(hcan_ctrl);
  if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
    can_bus_off_until_tick = HAL_GetTick() + CAN_BUS_OFF_COOLDOWN_MS;
  }

  (void)HAL_CAN_Stop(hcan_ctrl);
  (void)HAL_CAN_Init(hcan_ctrl);
  (void)HAL_CAN_ResetError(hcan_ctrl);
  (void)can_control_start();
}

static bool can_tx_queue_peek(uint16_t *position) {
  if ((position == NULL) || (encoder_tx_tail == encoder_tx_head)) {
    return false;
  }

  *position = encoder_tx_queue[encoder_tx_tail];
  return true;
}

static void can_tx_queue_drop(void) {
  if (encoder_tx_tail == encoder_tx_head) {
    return;
  }

  encoder_tx_tail = (uint8_t)((encoder_tx_tail + 1U) % CAN_TX_QUEUE_SIZE);
}
