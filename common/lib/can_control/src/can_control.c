/* CAN control implementation */
#include "modules/can_control.h"
#include "debug_log.h"

#if BOARD_CAN_CONTROL_ENABLE_RX
#include "modules/dc_motor.h"
#include "modules/led.h"
#include "modules/servo.h"
#endif

#ifndef BOARD_CAN_RX_DC_SPEED
#define BOARD_CAN_RX_DC_SPEED 100
#endif

#define CAN_TX_QUEUE_SIZE 8U
#define CAN_BUS_OFF_COOLDOWN_MS 1000U

static CAN_HandleTypeDef *hcan_ctrl = NULL;
static volatile uint16_t encoder_tx_queue[CAN_TX_QUEUE_SIZE];
static volatile uint8_t encoder_tx_head = 0;
static volatile uint8_t encoder_tx_tail = 0;
static uint32_t can_bus_off_until_tick = 0U;

static bool can_control_start(void);
static void can_control_handle_error(void);
#if BOARD_CAN_CONTROL_ENABLE_RX
static void can_filter_config(void);
#endif
static bool can_tx_queue_peek(uint16_t *position);
static void can_tx_queue_drop(void);

// CAN initialize
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
    uint8_t tx_data[8] = {0};
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

#if BOARD_CAN_CONTROL_ENABLE_RX
// CAN filter setting for board-specific RX IDs.
static void can_filter_config(void) {
  CAN_FilterTypeDef filter_config;
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDLIST;
  filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
  filter_config.FilterIdHigh = (CAN_ID_DC << 5);
  filter_config.FilterIdLow = (CAN_ID_SERVO << 5);
  filter_config.FilterMaskIdHigh = 0;
  filter_config.FilterMaskIdLow = 0;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan_ctrl, &filter_config);
}
#endif

static bool can_control_start(void) {
  HAL_CAN_StateTypeDef state;

  if (hcan_ctrl == NULL) {
    return false;
  }

  state = HAL_CAN_GetState(hcan_ctrl);
  if (state == HAL_CAN_STATE_LISTENING) {
#if BOARD_CAN_CONTROL_ENABLE_RX
    (void)HAL_CAN_ActivateNotification(hcan_ctrl, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
    return true;
  }

  if (state == HAL_CAN_STATE_RESET || state == HAL_CAN_STATE_ERROR) {
    (void)HAL_CAN_Init(hcan_ctrl);
  }

  if (HAL_CAN_GetState(hcan_ctrl) == HAL_CAN_STATE_READY) {
#if BOARD_CAN_CONTROL_ENABLE_RX
    can_filter_config();
#endif
    if (HAL_CAN_Start(hcan_ctrl) == HAL_OK) {
#if BOARD_CAN_CONTROL_ENABLE_RX
      (void)HAL_CAN_ActivateNotification(hcan_ctrl, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
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

#if BOARD_CAN_CONTROL_ENABLE_RX
// CAN receive callback
void can_control_rx_callback(CAN_HandleTypeDef *hcan) {
  led_set(LED_COLOR_YELLOW, LED_STATE_ON);
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // Get message from FIFO0.
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    return;
  }

  // Branch by received ID.
  if (rx_header.StdId == CAN_ID_DC) {
    // DC motor command (0x208)
    if (rx_header.DLC < 5) {
      return;
    }

    if (rx_data[0] == 1) {
      dc_motor_push();
    } else if (!dc_motor_push_is_active()) {
      dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0);
    }

    if (rx_data[3] == 1) {
      dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_FORWARD, BOARD_CAN_RX_DC_SPEED);
    } else if (rx_data[4] == 1) {
      dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_REVERSE, BOARD_CAN_RX_DC_SPEED);
    } else {
      dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_STOP, 0);
    }
  } else if (rx_header.StdId == CAN_ID_SERVO) {
    // Servo command (0x1FF)
    if (rx_header.DLC < 6) {
      return;
    }

    int16_t data = (int16_t)(rx_data[4]<<8 | rx_data[5]);
    if (data < 0) {
      servo_control(SERVO_DIR_OPEN, SERVO_MODE_NORMAL);
      LOG("SERVO OPEN\n");
    } else if (data == 0) {
      servo_control(SERVO_DIR_STOP, SERVO_MODE_NORMAL);
      LOG("SERVO STOP\n");
    } else {
      servo_control(SERVO_DIR_CLOSE, SERVO_MODE_NORMAL);
      LOG("SERVO CLOSE\n");
    }
  }
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
}
#endif
