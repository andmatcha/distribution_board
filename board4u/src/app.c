#include "app.h"

#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "modules/usb_storage_reader.h"

#define CAN_COORDINATE_REQUEST_ID 0x208U
#define CAN_COORDINATE_RESPONSE_ID 0x209U
#define CAN_COORDINATE_REQUEST_INDEX 2U
#define CAN2_FILTER_BANK 14U
#define CAN2_SLAVE_START_FILTER_BANK 14U

extern CAN_HandleTypeDef hcan2;

static void store_i32_le(uint8_t *destination, int32_t value)
{
    const uint32_t raw_value = (uint32_t)value;

    destination[0] = (uint8_t)(raw_value & 0xFFU);
    destination[1] = (uint8_t)((raw_value >> 8) & 0xFFU);
    destination[2] = (uint8_t)((raw_value >> 16) & 0xFFU);
    destination[3] = (uint8_t)((raw_value >> 24) & 0xFFU);
}

static void can_send_position(int32_t latitude_e7, int32_t longitude_e7)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    HAL_StatusTypeDef status;

    tx_header.StdId = CAN_COORDINATE_RESPONSE_ID;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    store_i32_le(&tx_data[0], latitude_e7);
    store_i32_le(&tx_data[4], longitude_e7);

    status = HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &tx_mailbox);
    if (status != HAL_OK) {
        printf("[can] failed to send 0x%03lX: HAL_Status=%d\r\n",
               (unsigned long)CAN_COORDINATE_RESPONSE_ID,
               (int)status);
        return;
    }

    printf("[can] sent 0x%03lX lat=%ld lon=%ld\r\n",
           (unsigned long)CAN_COORDINATE_RESPONSE_ID,
           (long)latitude_e7,
           (long)longitude_e7);
}

static void read_and_send_position(const char *source)
{
    int32_t latitude_e7;
    int32_t longitude_e7;

    if (usb_storage_reader_read_next_position(&latitude_e7, &longitude_e7) == 0U) {
        printf("[%s] ERROR: coordinate read failed; 0x%03lX was not sent\r\n",
               source,
               (unsigned long)CAN_COORDINATE_RESPONSE_ID);
        return;
    }

    printf("[%s] read coordinate: latitude_e7=%ld longitude_e7=%ld\r\n",
           source,
           (long)latitude_e7,
           (long)longitude_e7);
    can_send_position(latitude_e7, longitude_e7);
}

static void can_handle_coordinate_request(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *rx_data)
{
    if ((rx_header->IDE != CAN_ID_STD) ||
        (rx_header->RTR != CAN_RTR_DATA) ||
        (rx_header->StdId != CAN_COORDINATE_REQUEST_ID) ||
        (rx_header->DLC <= CAN_COORDINATE_REQUEST_INDEX) ||
        (rx_data[CAN_COORDINATE_REQUEST_INDEX] != 1U)) {
        return;
    }

    printf("[can] coordinate request received on 0x%03lX\r\n",
           (unsigned long)CAN_COORDINATE_REQUEST_ID);
    read_and_send_position("can");
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

        can_handle_coordinate_request(&rx_header, rx_data);
    }
}

static void can_init(void)
{
    CAN_FilterTypeDef filter_config;

    filter_config.FilterBank = CAN2_FILTER_BANK;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = (uint32_t)(CAN_COORDINATE_REQUEST_ID << 5);
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

    printf("[can] listening for 0x%03lX, data[%u] == 1\r\n",
           (unsigned long)CAN_COORDINATE_REQUEST_ID,
           (unsigned int)CAN_COORDINATE_REQUEST_INDEX);
}

void init(void)
{
    usb_storage_reader_init();
    can_init();
}

void poll(void)
{
    usb_storage_reader_poll();
    can_poll();
}
