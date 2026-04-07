#include "base_horizon.h"

#include "encoder.h"
#include "led.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief Base Horizon の1回分のエンコーダ取得結果をまとめた構造体
 */
typedef struct
{
  uint16_t pos;           /* 1回転内の位置カウント */
  int16_t turns;          /* 回転数 */
  int32_t total_counts;   /* 回転数を含めた総カウント */
  int16_t distance_tenths_mm; /* 0.1mm単位へ換算した距離 */
} BaseHorizonSample;

/**
 * @brief Base Horizon の状態を保持する構造体
 */
typedef struct
{
  int16_t last_distance_tenths_mm;    /* 最後に採用した距離[0.1mm] */
  bool is_first_reading;              /* 初回サンプルかどうか */
  bool has_last_total_counts;         /* 総カウントの基準値があるか */
  int32_t last_total_counts;          /* 最後に採用した総カウント */
  bool has_pending_transition;        /* 距離遷移の確認中か */
  int16_t pending_distance_tenths_mm; /* 確認中の候補距離[0.1mm] */
  int32_t pending_total_counts;       /* 確認中候補の総カウント */
  uint8_t pending_transition_count;   /* 候補距離が連続した回数 */
  bool has_last_can_distance;         /* 最後に送信したCAN距離があるか */
  int16_t last_can_distance_tenths_mm;/* 最後に送信したCAN距離[0.1mm] */
  uint32_t last_can_send_tick;        /* 最後にCAN送信した時刻 */
  uint32_t last_encoder_log_tick;     /* 最後にエンコーダログを出した時刻 */
} BaseHorizonState;

/**
 * @brief Base Horizon の CAN送信判定結果
 */
typedef enum
{
  BASE_HORIZON_CAN_DECISION_SEND = 0,      /* CAN送信する */
  BASE_HORIZON_CAN_DECISION_SKIP_SAME,     /* 値が変わらず keepalive 前なので送らない */
  BASE_HORIZON_CAN_DECISION_SKIP_BUS_OFF   /* bus-off 抑止中なので送らない */
} BaseHorizonCanDecision;

static uint32_t base_horizon_get_pending_can_mailboxes(void); /* 送信待ちmailbox取得 */

/**
 * @brief Base Horizon のエンコーダ取得状態
 */
typedef enum
{
  BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION = 0, /* position要求を送る段階 */
  BASE_HORIZON_ENCODER_PHASE_WAIT_POSITION,        /* position応答待ち */
  BASE_HORIZON_ENCODER_PHASE_REQUEST_TURNS,        /* turns要求を送る段階 */
  BASE_HORIZON_ENCODER_PHASE_WAIT_TURNS            /* turns応答待ち */
} BaseHorizonEncoderPhase;

static UART_HandleTypeDef *base_horizon_encoder_uart = NULL;   /* Base Horizon 用UART */
static CAN_HandleTypeDef *base_horizon_can_handle = NULL;      /* Base Horizon 用CAN */
static CAN_TxHeaderTypeDef base_horizon_can_tx_header;         /* Base Horizon 用CANヘッダ */
static uint8_t base_horizon_can_tx_data[8];                    /* Base Horizon 用CANデータ */
static uint32_t base_horizon_can_tx_mailbox;                   /* 直近に使ったmailbox */
static BaseHorizonState base_horizon_state = {0};              /* Base Horizon の内部状態 */
static BaseHorizonEncoderPhase base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION; /* 取得状態 */
static uint32_t base_horizon_encoder_phase_start_tick = 0;     /* 現在の取得段階に入った時刻 */
static uint16_t base_horizon_pending_pos = 0;                  /* position応答の一時保持 */
static int16_t base_horizon_pending_turns = 0;                 /* turns応答の一時保持 */
static volatile uint32_t base_horizon_can_bus_off_until_tick = 0;    /* bus-off抑止終了時刻 */

#define BASE_HORIZON_CAN_STD_ID 0x300U                   /* Base Horizon のCAN標準ID */
#define BASE_HORIZON_BUS_OFF_COOLDOWN_MS 1000U           /* bus-off後に送信を止める時間[ms] */
#define BASE_HORIZON_CAN_KEEPALIVE_INTERVAL_MS 200U      /* 値が変わらなくても送る周期[ms] */
#define BASE_HORIZON_LOG_INTERVAL_MS 100U                /* エンコーダログの最小出力周期[ms] */
#define BASE_HORIZON_ENCODER_RESPONSE_TIMEOUT_MS 10U     /* encoder応答待ちのタイムアウト[ms] */
#define BASE_HORIZON_DIRECT_ACCEPT_TENTHS_MM 10          /* 即時採用する距離差[0.1mm] */
#define BASE_HORIZON_PENDING_CONFIRMATIONS 3U            /* 新しい距離を確定する連続回数 */
#define BASE_HORIZON_PENDING_TOLERANCE_TENTHS_MM 10      /* 確認中候補として許す距離差[0.1mm] */

/**
 * @brief Base Horizon の位置カウントから前回値に近い総カウントを復元する
 * @param[in] pos 今回の1回転内位置カウント
 * @param[in] last_total_counts 前回採用した総カウント
 * @retval 復元した総カウント
 */
static int32_t base_horizon_unwrap_counts(uint16_t pos, int32_t last_total_counts)
{
  int32_t last_pos = last_total_counts & 0x3FFF;
  int32_t delta = (int32_t)(pos & 0x3FFFU) - last_pos;

  if (delta > 8192) {
    delta -= 16384;
  } else if (delta < -8192) {
    delta += 16384;
  }

  return last_total_counts + delta;
}

/**
 * @brief Base Horizon 用CANの異常状態から復旧を試みる
 * @param[in] reason 復旧を試す理由
 * @retval なし
 */
static void base_horizon_try_recover_can(const char *reason)
{
  HAL_CAN_StateTypeDef state;
  (void)reason;

  if (base_horizon_can_handle == NULL) {
    return;
  }

  state = HAL_CAN_GetState(base_horizon_can_handle);
  if (state == HAL_CAN_STATE_ERROR || state == HAL_CAN_STATE_READY) {
    if (HAL_CAN_Stop(base_horizon_can_handle) == HAL_OK) {
      (void)HAL_CAN_ResetError(base_horizon_can_handle);
      if (HAL_CAN_Start(base_horizon_can_handle) == HAL_OK) {
      }
    }
  }
}

/**
 * @brief 送信待ちのCAN mailbox をビット集合で返す
 * @param なし
 * @retval 送信待ち mailbox のビットフラグ
 */
static uint32_t base_horizon_get_pending_can_mailboxes(void)
{
  uint32_t pending = 0U;

  if (base_horizon_can_handle == NULL) {
    return 0U;
  }

  if (HAL_CAN_IsTxMessagePending(base_horizon_can_handle, CAN_TX_MAILBOX0) != 0U) {
    pending |= CAN_TX_MAILBOX0;
  }
  if (HAL_CAN_IsTxMessagePending(base_horizon_can_handle, CAN_TX_MAILBOX1) != 0U) {
    pending |= CAN_TX_MAILBOX1;
  }
  if (HAL_CAN_IsTxMessagePending(base_horizon_can_handle, CAN_TX_MAILBOX2) != 0U) {
    pending |= CAN_TX_MAILBOX2;
  }

  return pending;
}

/**
 * @brief Base Horizon の距離値をCANへ送信する
 * @param[in] distance_tenths_mm 送信する距離[0.1mm]
 * @retval true 送信要求をCANへ登録できた
 * @retval false 送信できなかった
 */
static bool base_horizon_send_distance_can(int16_t distance_tenths_mm)
{
  uint32_t now;
  uint32_t free_level;
  uint32_t pending_mailboxes;
  uint32_t error_code;
  HAL_StatusTypeDef status;

  if (base_horizon_can_handle == NULL) {
    return false;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - base_horizon_can_bus_off_until_tick) < 0) {
    return false;
  }

  free_level = HAL_CAN_GetTxMailboxesFreeLevel(base_horizon_can_handle);
  if (free_level == 0U) {
    pending_mailboxes = base_horizon_get_pending_can_mailboxes();

    if (pending_mailboxes != 0U) {
      if (HAL_CAN_AbortTxRequest(base_horizon_can_handle, pending_mailboxes) == HAL_OK) {
      } else {
        base_horizon_try_recover_can("mailbox full abort failure");
        return false;
      }

      free_level = HAL_CAN_GetTxMailboxesFreeLevel(base_horizon_can_handle);
      if (free_level == 0U) {
        base_horizon_try_recover_can("mailbox full after abort");
        return false;
      }
    } else {
      base_horizon_try_recover_can("mailbox full without pending flag");
      return false;
    }
  }

  base_horizon_can_tx_data[0] = (uint8_t)(distance_tenths_mm & 0xFF);
  base_horizon_can_tx_data[1] = (uint8_t)((distance_tenths_mm >> 8) & 0xFF);

  status = HAL_CAN_AddTxMessage(base_horizon_can_handle,
                                &base_horizon_can_tx_header,
                                base_horizon_can_tx_data,
                                &base_horizon_can_tx_mailbox);
  if (status != HAL_OK) {
    error_code = HAL_CAN_GetError(base_horizon_can_handle);

    if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
      base_horizon_can_bus_off_until_tick = HAL_GetTick() + BASE_HORIZON_BUS_OFF_COOLDOWN_MS;
    }

    base_horizon_try_recover_can("tx failure");
    return false;
  }

  return true;
}

/**
 * @brief Base Horizon 用の周辺機能を初期化する
 * @param なし
 * @retval なし
 */
static void base_horizon_initialize_runtime(void)
{
  if (base_horizon_encoder_uart == NULL) {
    return;
  }

  encoder_init(base_horizon_encoder_uart);
  encoder_reset();
  HAL_Delay(10);

  base_horizon_can_tx_header.StdId = BASE_HORIZON_CAN_STD_ID;
  base_horizon_can_tx_header.ExtId = 0;
  base_horizon_can_tx_header.IDE = CAN_ID_STD;
  base_horizon_can_tx_header.RTR = CAN_RTR_DATA;
  base_horizon_can_tx_header.DLC = 2;
  base_horizon_can_tx_header.TransmitGlobalTime = DISABLE;

  for (uint8_t i = 0; i < 8; i++) {
    base_horizon_can_tx_data[i] = 0;
  }

  led_set(LED_COLOR_RED, LED_STATE_ON);
  led_set(LED_COLOR_YELLOW, LED_STATE_ON);
  led_set(LED_COLOR_GREEN, LED_STATE_ON);
  printf("Base Horizon initialization complete.\n");
}

/**
 * @brief Base Horizon の pos / turns から距離まで計算したサンプルを作る
 * @param[out] sample 計算結果の格納先
 * @param[in] pos 1回転内位置カウント
 * @param[in] turns 回転数
 * @retval true サンプル作成成功
 * @retval false 引数不正
 */
static bool base_horizon_build_sample(BaseHorizonSample *sample, uint16_t pos, int16_t turns)
{
  int32_t raw_total_counts;
  int32_t total_counts;
  int32_t predicted_total_counts;
  int32_t turns_mismatch;

  if (sample == NULL) {
    return false;
  }

  raw_total_counts = ((int32_t)turns * 16384) + (int32_t)(pos & 0x3FFF);
  total_counts = raw_total_counts;

  /* 位置と回転数は別時刻に読まれるため、前回値に近い総カウントへ補正する。 */
  if (base_horizon_state.has_last_total_counts) {
    predicted_total_counts = base_horizon_unwrap_counts(pos, base_horizon_state.last_total_counts);
    turns_mismatch = raw_total_counts - predicted_total_counts;

    if (turns_mismatch >= 8192 || turns_mismatch <= -8192) {
      total_counts = predicted_total_counts;
    }
  }

  sample->pos = pos;
  sample->turns = turns;
  sample->total_counts = total_counts;
  sample->distance_tenths_mm = (int16_t)roundf((float)total_counts * 50.0f / 16384.0f);
  return true;
}

/**
 * @brief Base Horizon エンコーダを非ブロッキングで進め、サンプル完成時だけ返す
 * @param[out] sample 読み出したサンプルの格納先
 * @retval true サンプル作成成功
 * @retval false まだ取得途中、または今回の取得に失敗
 */
static bool base_horizon_read_sample(BaseHorizonSample *sample)
{
  uint32_t now_tick;
  uint8_t step_guard = 0;

  if (sample == NULL) {
    return false;
  }

  while (step_guard < 4U) {
    now_tick = HAL_GetTick();
    step_guard++;

    switch (base_horizon_encoder_phase) {
      case BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION:
        encoder_request_data(ENCODER_CMD_POSITION);
        base_horizon_encoder_phase_start_tick = now_tick;
        base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_WAIT_POSITION;
        break;

      case BASE_HORIZON_ENCODER_PHASE_WAIT_POSITION:
        if (encoder_get_position(&base_horizon_pending_pos)) {
          base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_TURNS;
          break;
        }
        if ((now_tick - base_horizon_encoder_phase_start_tick) >= BASE_HORIZON_ENCODER_RESPONSE_TIMEOUT_MS) {
          base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION;
          break;
        }
        return false;

      case BASE_HORIZON_ENCODER_PHASE_REQUEST_TURNS:
        encoder_request_data(ENCODER_CMD_TURNS);
        base_horizon_encoder_phase_start_tick = now_tick;
        base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_WAIT_TURNS;
        break;

      case BASE_HORIZON_ENCODER_PHASE_WAIT_TURNS:
        if (encoder_get_turns(&base_horizon_pending_turns)) {
          base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION;
          return base_horizon_build_sample(sample, base_horizon_pending_pos, base_horizon_pending_turns);
        }
        if ((now_tick - base_horizon_encoder_phase_start_tick) >= BASE_HORIZON_ENCODER_RESPONSE_TIMEOUT_MS) {
          base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION;
          break;
        }
        return false;

      default:
        base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION;
        break;
    }
  }

  return false;
}

/**
 * @brief Base Horizon の距離変化を採用するか保留するかを判定する
 * @param[in] distance_tenths_mm 今回の距離[0.1mm]
 * @retval true 採用する
 * @retval false まだ保留する
 */
static bool base_horizon_validate_distance(int16_t distance_tenths_mm)
{
  int16_t diff;
  int16_t pending_diff;

  if (base_horizon_state.is_first_reading) {
    base_horizon_state.is_first_reading = false;
    base_horizon_state.has_pending_transition = false;
    return true;
  }

  diff = distance_tenths_mm - base_horizon_state.last_distance_tenths_mm;
  if (diff < 0) {
    diff = -diff;
  }

  if (diff <= BASE_HORIZON_DIRECT_ACCEPT_TENTHS_MM) {
    base_horizon_state.has_pending_transition = false;
    return true;
  }

  if (base_horizon_state.has_pending_transition) {
    pending_diff = distance_tenths_mm - base_horizon_state.pending_distance_tenths_mm;
    if (pending_diff < 0) {
      pending_diff = -pending_diff;
    }

    if (pending_diff <= BASE_HORIZON_PENDING_TOLERANCE_TENTHS_MM) {
      base_horizon_state.pending_transition_count++;
    } else {
      base_horizon_state.pending_distance_tenths_mm = distance_tenths_mm;
      base_horizon_state.pending_total_counts = base_horizon_state.last_total_counts;
      base_horizon_state.pending_transition_count = 1U;
    }
  } else {
    base_horizon_state.has_pending_transition = true;
    base_horizon_state.pending_distance_tenths_mm = distance_tenths_mm;
    base_horizon_state.pending_total_counts = base_horizon_state.last_total_counts;
    base_horizon_state.pending_transition_count = 1U;
  }

  if (base_horizon_state.pending_transition_count >= BASE_HORIZON_PENDING_CONFIRMATIONS) {
    base_horizon_state.has_pending_transition = false;
    return true;
  }

  return false;
}

/**
 * @brief Base Horizon の距離をCAN送信すべきかを判定する
 * @param[in] distance_tenths_mm 今回の距離[0.1mm]
 * @param[in] now_tick 現在時刻[tick]
 * @retval BASE_HORIZON_CAN_DECISION_SEND CAN送信する
 * @retval BASE_HORIZON_CAN_DECISION_SKIP_SAME 値が変わらないのでCAN送信しない
 * @retval BASE_HORIZON_CAN_DECISION_SKIP_BUS_OFF bus-off抑止中なのでCAN送信しない
 */
static BaseHorizonCanDecision base_horizon_should_send_can(int16_t distance_tenths_mm, uint32_t now_tick)
{
  if ((int32_t)(now_tick - base_horizon_can_bus_off_until_tick) < 0) {
    return BASE_HORIZON_CAN_DECISION_SKIP_BUS_OFF;
  }

  if (!base_horizon_state.has_last_can_distance) {
    return BASE_HORIZON_CAN_DECISION_SEND;
  }

  if (distance_tenths_mm != base_horizon_state.last_can_distance_tenths_mm) {
    return BASE_HORIZON_CAN_DECISION_SEND;
  }

  if ((now_tick - base_horizon_state.last_can_send_tick) >= BASE_HORIZON_CAN_KEEPALIVE_INTERVAL_MS) {
    return BASE_HORIZON_CAN_DECISION_SEND;
  }

  return BASE_HORIZON_CAN_DECISION_SKIP_SAME;
}

/**
 * @brief Base Horizon の採用サンプルを状態へ反映し、必要な出力を行う
 * @param[in] sample 採用済みサンプル
 * @retval なし
 */
static void base_horizon_publish_sample(const BaseHorizonSample *sample)
{
  uint32_t now_tick;
  BaseHorizonCanDecision can_decision;

  if (sample == NULL) {
    return;
  }

  base_horizon_state.last_distance_tenths_mm = sample->distance_tenths_mm;
  base_horizon_state.last_total_counts = sample->total_counts;
  base_horizon_state.has_last_total_counts = true;

  now_tick = HAL_GetTick();
  can_decision = base_horizon_should_send_can(sample->distance_tenths_mm, now_tick);
  if (can_decision == BASE_HORIZON_CAN_DECISION_SEND &&
      base_horizon_send_distance_can(sample->distance_tenths_mm)) {
    base_horizon_state.last_can_distance_tenths_mm = sample->distance_tenths_mm;
    base_horizon_state.last_can_send_tick = now_tick;
    base_horizon_state.has_last_can_distance = true;
  }

  if ((now_tick - base_horizon_state.last_encoder_log_tick) >= BASE_HORIZON_LOG_INTERVAL_MS) {
    int32_t abs_distance_tenths_mm;

    base_horizon_state.last_encoder_log_tick = now_tick;
    abs_distance_tenths_mm = sample->distance_tenths_mm;
    if (abs_distance_tenths_mm < 0) {
      abs_distance_tenths_mm = -abs_distance_tenths_mm;
      printf("Base Horizon Encoder: %d turns, %u pos -> Distance: -%ld.%01ld mm (counts=%ld)\n",
             sample->turns,
             sample->pos,
             (long)(abs_distance_tenths_mm / 10),
             (long)(abs_distance_tenths_mm % 10),
             (long)sample->total_counts);
    } else {
      printf("Base Horizon Encoder: %d turns, %u pos -> Distance: %ld.%01ld mm (counts=%ld)\n",
             sample->turns,
             sample->pos,
             (long)(abs_distance_tenths_mm / 10),
             (long)(abs_distance_tenths_mm % 10),
             (long)sample->total_counts);
    }
  }
}

void base_horizon_init(UART_HandleTypeDef *encoder_uart, CAN_HandleTypeDef *can_handle)
{
  base_horizon_encoder_uart = encoder_uart;
  base_horizon_can_handle = can_handle;
  base_horizon_state = (BaseHorizonState){0};
  base_horizon_state.is_first_reading = true;
  base_horizon_encoder_phase = BASE_HORIZON_ENCODER_PHASE_REQUEST_POSITION;
  base_horizon_encoder_phase_start_tick = 0;
  base_horizon_pending_pos = 0;
  base_horizon_pending_turns = 0;
  base_horizon_can_bus_off_until_tick = 0;
  base_horizon_initialize_runtime();
}

void base_horizon_process(void)
{
  BaseHorizonSample sample;

  if (base_horizon_read_sample(&sample)) {
    if (base_horizon_validate_distance(sample.distance_tenths_mm)) {
      base_horizon_publish_sample(&sample);
    }
  }
}

void base_horizon_handle_can_tx_mailbox0_abort(CAN_HandleTypeDef *can_handle)
{
  if (base_horizon_can_handle == NULL || can_handle->Instance != base_horizon_can_handle->Instance) {
    return;
  }
}

void base_horizon_handle_can_tx_mailbox1_abort(CAN_HandleTypeDef *can_handle)
{
  if (base_horizon_can_handle == NULL || can_handle->Instance != base_horizon_can_handle->Instance) {
    return;
  }
}

void base_horizon_handle_can_tx_mailbox2_abort(CAN_HandleTypeDef *can_handle)
{
  if (base_horizon_can_handle == NULL || can_handle->Instance != base_horizon_can_handle->Instance) {
    return;
  }
}

void base_horizon_handle_can_error(CAN_HandleTypeDef *can_handle)
{
  uint32_t error_code;

  if (base_horizon_can_handle == NULL || can_handle->Instance != base_horizon_can_handle->Instance) {
    return;
  }

  error_code = HAL_CAN_GetError(can_handle);

  if ((error_code & HAL_CAN_ERROR_BOF) != 0U) {
    base_horizon_can_bus_off_until_tick = HAL_GetTick() + BASE_HORIZON_BUS_OFF_COOLDOWN_MS;
    base_horizon_try_recover_can("bus-off");
  }
}
