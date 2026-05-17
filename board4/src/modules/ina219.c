/* TI INA219 current/voltage monitor implementation */
#include "modules/ina219.h"

#include "board_config.h"
#include "debug_log.h"

#ifndef BOARD_SERVO_INA219_ENABLE
#define BOARD_SERVO_INA219_ENABLE 1
#endif

#ifndef BOARD_SERVO_INA219_I2C_ADDRESS_7BIT
#define BOARD_SERVO_INA219_I2C_ADDRESS_7BIT 0x40U
#endif

#ifndef BOARD_SERVO_INA219_SHUNT_MILLIOHM
#define BOARD_SERVO_INA219_SHUNT_MILLIOHM 5U
#endif

#ifndef BOARD_SERVO_INA219_CURRENT_LSB_UA
#define BOARD_SERVO_INA219_CURRENT_LSB_UA 200U
#endif

#ifndef BOARD_SERVO_INA219_LOG_INTERVAL_MS
#define BOARD_SERVO_INA219_LOG_INTERVAL_MS 500U
#endif

#ifndef BOARD_SERVO_INA219_I2C_TIMEOUT_MS
#define BOARD_SERVO_INA219_I2C_TIMEOUT_MS 10U
#endif

#ifndef BOARD_SERVO_INA219_CONFIG_REG
#define BOARD_SERVO_INA219_CONFIG_REG 0x399FU
#endif

#define INA219_REG_CONFIG 0x00U
#define INA219_REG_SHUNT_VOLTAGE 0x01U
#define INA219_REG_BUS_VOLTAGE 0x02U
#define INA219_REG_POWER 0x03U
#define INA219_REG_CURRENT 0x04U
#define INA219_REG_CALIBRATION 0x05U

#define INA219_I2C_ADDRESS (BOARD_SERVO_INA219_I2C_ADDRESS_7BIT << 1U)
#define INA219_CALIBRATION_VALUE \
  (40960000U / (BOARD_SERVO_INA219_CURRENT_LSB_UA * BOARD_SERVO_INA219_SHUNT_MILLIOHM))
#define INA219_POWER_LSB_UW (20U * BOARD_SERVO_INA219_CURRENT_LSB_UA)

#if BOARD_SERVO_INA219_ENABLE
#if BOARD_SERVO_INA219_SHUNT_MILLIOHM == 0
#error "BOARD_SERVO_INA219_SHUNT_MILLIOHM must be greater than zero"
#endif

#if BOARD_SERVO_INA219_CURRENT_LSB_UA == 0
#error "BOARD_SERVO_INA219_CURRENT_LSB_UA must be greater than zero"
#endif

#if INA219_CALIBRATION_VALUE == 0 || INA219_CALIBRATION_VALUE > 65535U
#error "INA219 calibration value is out of range; adjust shunt or current LSB"
#endif
#endif

static I2C_HandleTypeDef *ina219_hi2c = NULL;
static bool ina219_configured = false;
static uint32_t ina219_last_process_tick = 0U;
static HAL_StatusTypeDef ina219_last_status = HAL_OK;

static HAL_StatusTypeDef ina219_write_register(uint8_t reg, uint16_t value)
{
  uint8_t data[2];

  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)(value & 0xFFU);

  ina219_last_status = HAL_I2C_Mem_Write(ina219_hi2c,
                                         INA219_I2C_ADDRESS,
                                         reg,
                                         I2C_MEMADD_SIZE_8BIT,
                                         data,
                                         sizeof(data),
                                         BOARD_SERVO_INA219_I2C_TIMEOUT_MS);
  return ina219_last_status;
}

static HAL_StatusTypeDef ina219_read_register(uint8_t reg, uint16_t *value)
{
  uint8_t data[2] = {0U, 0U};

  ina219_last_status = HAL_I2C_Mem_Read(ina219_hi2c,
                                        INA219_I2C_ADDRESS,
                                        reg,
                                        I2C_MEMADD_SIZE_8BIT,
                                        data,
                                        sizeof(data),
                                        BOARD_SERVO_INA219_I2C_TIMEOUT_MS);
  if (ina219_last_status != HAL_OK) {
    return ina219_last_status;
  }

  *value = ((uint16_t)data[0] << 8) | data[1];
  return HAL_OK;
}

static bool ina219_configure(void)
{
  if (ina219_hi2c == NULL) {
    return false;
  }

  ina219_last_status = HAL_I2C_IsDeviceReady(ina219_hi2c,
                                             INA219_I2C_ADDRESS,
                                             2U,
                                             BOARD_SERVO_INA219_I2C_TIMEOUT_MS);
  if (ina219_last_status != HAL_OK) {
    return false;
  }

  if (ina219_write_register(INA219_REG_CONFIG, BOARD_SERVO_INA219_CONFIG_REG) != HAL_OK) {
    return false;
  }

  if (ina219_write_register(INA219_REG_CALIBRATION, (uint16_t)INA219_CALIBRATION_VALUE) != HAL_OK) {
    return false;
  }

  ina219_configured = true;
  LOG("[INA219] ready addr=0x%02lX shunt=%lumohm current_lsb=%luuA power_lsb=%luuW cal=%lu\n",
      (unsigned long)BOARD_SERVO_INA219_I2C_ADDRESS_7BIT,
      (unsigned long)BOARD_SERVO_INA219_SHUNT_MILLIOHM,
      (unsigned long)BOARD_SERVO_INA219_CURRENT_LSB_UA,
      (unsigned long)INA219_POWER_LSB_UW,
      (unsigned long)INA219_CALIBRATION_VALUE);
  return true;
}

#if DEBUG_LOG_ENABLED
static uint32_t ina219_abs_i32(int32_t value)
{
  if (value < 0) {
    return (uint32_t)(-(value + 1)) + 1U;
  }

  return (uint32_t)value;
}
#endif

static void ina219_log_measurement(const Ina219Measurement *measurement)
{
#if DEBUG_LOG_ENABLED
  uint32_t shunt_uv_abs = ina219_abs_i32(measurement->shunt_voltage_uv);
  uint32_t current_ua_abs = ina219_abs_i32(measurement->current_ua);

  LOG("[INA219] servo bus=%lu.%03luV shunt=%s%lu.%03lumV current=%s%lu.%03lumA power=%lu.%03luW\n",
      (unsigned long)(measurement->bus_voltage_mv / 1000U),
      (unsigned long)(measurement->bus_voltage_mv % 1000U),
      (measurement->shunt_voltage_uv < 0) ? "-" : "",
      (unsigned long)(shunt_uv_abs / 1000U),
      (unsigned long)(shunt_uv_abs % 1000U),
      (measurement->current_ua < 0) ? "-" : "",
      (unsigned long)(current_ua_abs / 1000U),
      (unsigned long)(current_ua_abs % 1000U),
      (unsigned long)(measurement->power_uw / 1000000U),
      (unsigned long)((measurement->power_uw / 1000U) % 1000U));
#else
  (void)measurement;
#endif
}

void ina219_init(I2C_HandleTypeDef *hi2c)
{
#if BOARD_SERVO_INA219_ENABLE
  ina219_hi2c = hi2c;
  ina219_configured = false;
  ina219_last_process_tick = HAL_GetTick();

  if (!ina219_configure()) {
    LOG("[INA219] init failed status=%d error=0x%08lX\n",
        ina219_last_status,
        (ina219_hi2c != NULL) ? (unsigned long)ina219_hi2c->ErrorCode : 0UL);
  }
#else
  (void)hi2c;
#endif
}

bool ina219_read(Ina219Measurement *measurement)
{
#if BOARD_SERVO_INA219_ENABLE
  uint16_t shunt_raw = 0U;
  uint16_t bus_raw = 0U;
  uint16_t current_raw = 0U;
  uint16_t power_raw = 0U;

  if (measurement == NULL) {
    return false;
  }

  if (!ina219_configured && !ina219_configure()) {
    return false;
  }

  if (ina219_read_register(INA219_REG_SHUNT_VOLTAGE, &shunt_raw) != HAL_OK) {
    ina219_configured = false;
    return false;
  }

  if (ina219_read_register(INA219_REG_BUS_VOLTAGE, &bus_raw) != HAL_OK) {
    ina219_configured = false;
    return false;
  }

  if (ina219_read_register(INA219_REG_CURRENT, &current_raw) != HAL_OK) {
    ina219_configured = false;
    return false;
  }

  if (ina219_read_register(INA219_REG_POWER, &power_raw) != HAL_OK) {
    ina219_configured = false;
    return false;
  }

  measurement->shunt_voltage_uv = (int32_t)((int16_t)shunt_raw) * 10;
  measurement->bus_voltage_mv = ((uint32_t)(bus_raw >> 3) * 4U);
  measurement->current_ua = (int32_t)((int16_t)current_raw) * (int32_t)BOARD_SERVO_INA219_CURRENT_LSB_UA;
  measurement->power_uw = (uint32_t)power_raw * INA219_POWER_LSB_UW;
  return true;
#else
  (void)measurement;
  return false;
#endif
}

void ina219_process(void)
{
#if BOARD_SERVO_INA219_ENABLE
  uint32_t now_tick = HAL_GetTick();
  Ina219Measurement measurement;

  if ((uint32_t)(now_tick - ina219_last_process_tick) < BOARD_SERVO_INA219_LOG_INTERVAL_MS) {
    return;
  }

  ina219_last_process_tick = now_tick;

  if (!ina219_read(&measurement)) {
    LOG("[INA219] read failed status=%d error=0x%08lX\n",
        ina219_last_status,
        (ina219_hi2c != NULL) ? (unsigned long)ina219_hi2c->ErrorCode : 0UL);
    return;
  }

  ina219_log_measurement(&measurement);
#endif
}
