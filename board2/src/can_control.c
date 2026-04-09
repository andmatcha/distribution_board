/* CAN control implementation */
#include "can_control.h"

#define CAN_TX_QUEUE_SIZE 8U

static CAN_HandleTypeDef *hcan_ctrl = NULL;
static volatile uint16_t encoder_tx_queue[CAN_TX_QUEUE_SIZE];
static volatile uint8_t encoder_tx_head = 0;
static volatile uint8_t encoder_tx_tail = 0;

static bool can_tx_queue_pop(uint16_t *position);

// CAN送信初期化
void can_control_init(CAN_HandleTypeDef *hcan) {
  hcan_ctrl = hcan;
  HAL_CAN_Start(hcan_ctrl);
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

  if (hcan_ctrl == NULL) {
    return;
  }

  tx_header.StdId = CAN_ID_ENCODER;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 2;
  tx_header.TransmitGlobalTime = DISABLE;

  while ((HAL_CAN_GetTxMailboxesFreeLevel(hcan_ctrl) > 0U) && can_tx_queue_pop(&position)) {
    uint8_t tx_data[2];
    uint32_t tx_mailbox;

    tx_data[0] = (uint8_t)(position >> 8);
    tx_data[1] = (uint8_t)(position & 0xFFU);

    if (HAL_CAN_AddTxMessage(hcan_ctrl, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
      break;
    }
  }
}

static bool can_tx_queue_pop(uint16_t *position) {
  if ((position == NULL) || (encoder_tx_tail == encoder_tx_head)) {
    return false;
  }

  *position = encoder_tx_queue[encoder_tx_tail];
  encoder_tx_tail = (uint8_t)((encoder_tx_tail + 1U) % CAN_TX_QUEUE_SIZE);
  return true;
}
