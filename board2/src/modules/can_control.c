/* CAN control implementation */
#include "modules/can_control.h"
#include "debug_log.h"
#include "main.h"

#define CAN_TX_QUEUE_SIZE 8U
#define CAN_ERROR_NOTIFICATION_MASK \
  (CAN_IT_ERROR | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE)

static CAN_HandleTypeDef *hcan_ctrl = NULL;
static volatile uint16_t encoder_tx_queue[CAN_TX_QUEUE_SIZE];
static volatile uint8_t encoder_tx_head = 0;
static volatile uint8_t encoder_tx_tail = 0;

static bool can_tx_queue_pop(uint16_t *position);

// CAN送信初期化
void can_control_init(CAN_HandleTypeDef *hcan) {
  hcan_ctrl = hcan;
  if ((hcan_ctrl != NULL) && (HAL_CAN_Start(hcan_ctrl) != HAL_OK)) {
    LOG("CAN start error: 0x%08lX\n", (unsigned long)HAL_CAN_GetError(hcan_ctrl));
    Error_Handler();
  }
  if ((hcan_ctrl != NULL) &&
      (HAL_CAN_ActivateNotification(hcan_ctrl, CAN_ERROR_NOTIFICATION_MASK) != HAL_OK)) {
    LOG("CAN notification error: 0x%08lX\n", (unsigned long)HAL_CAN_GetError(hcan_ctrl));
    Error_Handler();
  }
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
      LOG("CAN send error: id=0x%03lX err=0x%08lX\n",
          (unsigned long)tx_header.StdId,
          (unsigned long)HAL_CAN_GetError(hcan_ctrl));
      Error_Handler();
    }

    LOG("CAN send ok: id=0x%03lX data=%02X %02X mailbox=%lu\n",
        (unsigned long)tx_header.StdId,
        tx_data[0],
        tx_data[1],
        (unsigned long)tx_mailbox);
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
