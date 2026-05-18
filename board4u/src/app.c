#include "app.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "modules/dc_motor.h"
#include "modules/usb_storage_reader.h"

#define CAN_USB_READ_REQUEST_ID 0x208U
#define CAN_USB_DATA_RESPONSE_BASE_ID 0x220U
#define CAN_USB_DATA_RESPONSE_ID_COUNT 4U
#define CAN_USB_TRANSFER_COMPLETE_ID 0x225U
#define CAN_USB_READ_REQUEST_INDEX 2U
#define CAN_DC_MOTOR_COMMAND_MIN_DLC 5U
#define CAN_DC_MOTOR_SPEED_PERCENT 100U
#define CAN2_FILTER_BANK 14U
#define CAN2_SLAVE_START_FILTER_BANK 14U
#define USB_CAN_CHUNK_SIZE 8U
#define USB_CAN_MAX_FRAME_COUNT 128U
#define USB_CAN_MAX_DATA_BYTES (USB_CAN_CHUNK_SIZE * USB_CAN_MAX_FRAME_COUNT)
#define USB_CAN_SEND_INTERVAL_MS 10U
#define USB_CAN_REPEAT_COUNT 10U

extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;

static uint8_t usb_can_tx_buffer[USB_CAN_MAX_DATA_BYTES];
static size_t usb_can_tx_bytes;
static size_t usb_can_tx_offset;
static uint16_t usb_can_tx_frame_count;
static uint8_t usb_can_tx_repeat_count;
static uint32_t usb_can_next_send_tick;
static uint8_t usb_can_completion_pending;
static uint8_t usb_can_tx_active;

static uint8_t tick_reached(uint32_t now, uint32_t target)
{
    return (uint8_t)(((int32_t)(now - target)) >= 0);
}

static uint32_t usb_can_response_id_for_frame(uint16_t frame_count)
{
    return CAN_USB_DATA_RESPONSE_BASE_ID +
           ((uint32_t)frame_count % CAN_USB_DATA_RESPONSE_ID_COUNT);
}

static HAL_StatusTypeDef can_send_usb_data_chunk(uint32_t response_id,
                                                 const uint8_t *data,
                                                 size_t data_length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[USB_CAN_CHUNK_SIZE] = {0};
    uint32_t tx_mailbox;

    if (data_length > USB_CAN_CHUNK_SIZE) {
        data_length = USB_CAN_CHUNK_SIZE;
    }

    tx_header.StdId = response_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = USB_CAN_CHUNK_SIZE;
    tx_header.TransmitGlobalTime = DISABLE;

    if (data_length > 0U) {
        memcpy(tx_data, data, data_length);
    }

    return HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
}

static void usb_can_transfer_start(void)
{
    size_t bytes_read = 0U;

    if (usb_can_tx_active != 0U) {
        printf("[can] read_usb request ignored; 0x%03lX-0x%03lX transfer is still active; "
               "0x%03lX completion is pending or not yet sent\r\n",
               (unsigned long)CAN_USB_DATA_RESPONSE_BASE_ID,
               (unsigned long)(CAN_USB_DATA_RESPONSE_BASE_ID + CAN_USB_DATA_RESPONSE_ID_COUNT - 1U),
               (unsigned long)CAN_USB_TRANSFER_COMPLETE_ID);
        return;
    }

    if (usb_storage_reader_read_data(usb_can_tx_buffer,
                                     sizeof(usb_can_tx_buffer),
                                     &bytes_read) == 0U) {
        printf("[can] ERROR: USB file data read failed; 0x%03lX was not sent\r\n",
               (unsigned long)CAN_USB_DATA_RESPONSE_BASE_ID);
        return;
    }

    usb_can_tx_bytes = bytes_read;
    usb_can_tx_offset = 0U;
    usb_can_tx_frame_count = 0U;
    usb_can_tx_repeat_count = 0U;
    usb_can_next_send_tick = HAL_GetTick();
    usb_can_completion_pending = 0U;
    usb_can_tx_active = 1U;

    printf("[can] starting 0x%03lX-0x%03lX USB data transfer: %lu bytes, max %u frames, %u repeats/frame\r\n",
           (unsigned long)CAN_USB_DATA_RESPONSE_BASE_ID,
           (unsigned long)(CAN_USB_DATA_RESPONSE_BASE_ID + CAN_USB_DATA_RESPONSE_ID_COUNT - 1U),
           (unsigned long)usb_can_tx_bytes,
           (unsigned int)USB_CAN_MAX_FRAME_COUNT,
           (unsigned int)USB_CAN_REPEAT_COUNT);
}

static void usb_can_transfer_process(void)
{
    HAL_StatusTypeDef status;
    size_t remaining;
    size_t chunk_length;
    uint32_t response_id;
    const uint32_t now = HAL_GetTick();

    if (usb_can_tx_active == 0U) {
        return;
    }

    if (tick_reached(now, usb_can_next_send_tick) == 0U) {
        return;
    }

    if (usb_can_completion_pending != 0U) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U) {
            return;
        }

        status = can_send_usb_data_chunk(CAN_USB_TRANSFER_COMPLETE_ID, NULL, 0U);
        if (status != HAL_OK) {
            printf("[can] failed to send 0x%03lX USB transfer complete: HAL_Status=%d\r\n",
                   (unsigned long)CAN_USB_TRANSFER_COMPLETE_ID,
                   (int)status);
            return;
        }

        usb_can_tx_repeat_count++;
        if (usb_can_tx_repeat_count >= USB_CAN_REPEAT_COUNT) {
            usb_can_completion_pending = 0U;
            usb_can_tx_active = 0U;
            printf("[can] completed 0x%03lX-0x%03lX USB data transfer: %lu bytes in %u frames; sent 0x%03lX %u times\r\n",
                   (unsigned long)CAN_USB_DATA_RESPONSE_BASE_ID,
                   (unsigned long)(CAN_USB_DATA_RESPONSE_BASE_ID + CAN_USB_DATA_RESPONSE_ID_COUNT - 1U),
                   (unsigned long)usb_can_tx_offset,
                   (unsigned int)usb_can_tx_frame_count,
                   (unsigned long)CAN_USB_TRANSFER_COMPLETE_ID,
                   (unsigned int)USB_CAN_REPEAT_COUNT);
            return;
        }

        usb_can_next_send_tick = now + USB_CAN_SEND_INTERVAL_MS;
        return;
    }

    if ((usb_can_tx_offset >= usb_can_tx_bytes) ||
        (usb_can_tx_frame_count >= USB_CAN_MAX_FRAME_COUNT)) {
        usb_can_completion_pending = 1U;
        usb_can_tx_repeat_count = 0U;
        return;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U) {
        return;
    }

    remaining = usb_can_tx_bytes - usb_can_tx_offset;
    chunk_length = (remaining > USB_CAN_CHUNK_SIZE) ? USB_CAN_CHUNK_SIZE : remaining;
    response_id = usb_can_response_id_for_frame(usb_can_tx_frame_count);
    status = can_send_usb_data_chunk(response_id,
                                     &usb_can_tx_buffer[usb_can_tx_offset],
                                     chunk_length);
    if (status != HAL_OK) {
        printf("[can] failed to send 0x%03lX USB chunk %u: HAL_Status=%d\r\n",
               (unsigned long)response_id,
               (unsigned int)(usb_can_tx_frame_count + 1U),
               (int)status);
        return;
    }

    usb_can_tx_repeat_count++;
    if (usb_can_tx_repeat_count >= USB_CAN_REPEAT_COUNT) {
        usb_can_tx_repeat_count = 0U;
        usb_can_tx_offset += chunk_length;
        usb_can_tx_frame_count++;

        if ((usb_can_tx_offset >= usb_can_tx_bytes) ||
            (usb_can_tx_frame_count >= USB_CAN_MAX_FRAME_COUNT)) {
            usb_can_completion_pending = 1U;
        }
    }

    usb_can_next_send_tick = now + USB_CAN_SEND_INTERVAL_MS;
}

static void can_handle_usb_read_request(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *rx_data)
{
    if ((rx_header->IDE != CAN_ID_STD) ||
        (rx_header->RTR != CAN_RTR_DATA) ||
        (rx_header->StdId != CAN_USB_READ_REQUEST_ID) ||
        (rx_header->DLC <= CAN_USB_READ_REQUEST_INDEX) ||
        (rx_data[CAN_USB_READ_REQUEST_INDEX] != 1U)) {
        return;
    }

    printf("[can] read_usb request received on 0x%03lX\r\n",
           (unsigned long)CAN_USB_READ_REQUEST_ID);
    usb_can_transfer_start();
}

static void can_handle_dc_motor_command(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *rx_data)
{
    if ((rx_header->IDE != CAN_ID_STD) ||
        (rx_header->RTR != CAN_RTR_DATA) ||
        (rx_header->StdId != CAN_USB_READ_REQUEST_ID) ||
        (rx_header->DLC < CAN_DC_MOTOR_COMMAND_MIN_DLC)) {
        return;
    }

    if (rx_data[0] == 1U) {
        dc_motor_push();
    } else if (!dc_motor_push_is_active()) {
        dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0U);
    }

    if (rx_data[3] == 1U) {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_FORWARD, CAN_DC_MOTOR_SPEED_PERCENT);
    } else if (rx_data[4] == 1U) {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_REVERSE, CAN_DC_MOTOR_SPEED_PERCENT);
    } else {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_STOP, 0U);
    }
}

static void can_poll(void)
{
    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            printf("[can] failed to read RX FIFO0\r\n");
            return;
        }

        can_handle_dc_motor_command(&rx_header, rx_data);
        dc_motor_process();
        can_handle_usb_read_request(&rx_header, rx_data);
    }
}

static void can_init(void)
{
    CAN_FilterTypeDef filter_config;

    filter_config.FilterBank = CAN2_FILTER_BANK;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = (uint32_t)(CAN_USB_READ_REQUEST_ID << 5);
    filter_config.FilterIdLow = 0U;
    filter_config.FilterMaskIdHigh = (uint32_t)(0x7FFU << 5);
    filter_config.FilterMaskIdLow = 0U;
    filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = CAN2_SLAVE_START_FILTER_BANK;

    if (HAL_CAN_ConfigFilter(&hcan2, &filter_config) != HAL_OK) {
        printf("[can] filter configuration failed\r\n");
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        printf("[can] start failed\r\n");
        Error_Handler();
    }

    printf("[can] listening for 0x%03lX read_usb request data[%u] == 1 and DC motor commands\r\n",
           (unsigned long)CAN_USB_READ_REQUEST_ID,
           (unsigned int)CAN_USB_READ_REQUEST_INDEX);
}

void init(void)
{
    dc_motor_init(&htim3);
    usb_storage_reader_init();
    can_init();
}

void poll(void)
{
    usb_storage_reader_poll();
    can_poll();
    usb_can_transfer_process();
    dc_motor_process();
}
