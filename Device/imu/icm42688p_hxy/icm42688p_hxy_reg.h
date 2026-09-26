#pragma once

#include <stdint.h>

#define ICM42688P_REG_WHO_AM_I           0x01U
#define ICM42688P_REG_COM_CFG            0x05U
#define ICM42688P_REG_INT_CFG1           0x06U
#define ICM42688P_REG_DATA_STAT          0x0BU
#define ICM42688P_REG_ACC_XH             0x0CU
#define ICM42688P_REG_GYR_XH             0x12U
#define ICM42688P_REG_TEMP_H             0x22U
#define ICM42688P_REG_ACC_CONF           0x40U
#define ICM42688P_REG_ACC_RANGE          0x41U
#define ICM42688P_REG_GYR_CONF           0x42U
#define ICM42688P_REG_GYR_RANGE          0x43U
#define ICM42688P_REG_SOFT_RST           0x4AU
#define ICM42688P_REG_PWR_CTRL           0x7DU

#define ICM42688P_WHO_AM_I_VALUE         0x6AU
#define ICM42688P_READ_FLAG              0x80U
#define ICM42688P_COM_CFG_DEFAULT        0x50U
#define ICM42688P_SOFT_RST_VALUE         0xA5U
#define ICM42688P_PWR_ALL_ON             0x0EU
#define ICM42688P_ACC_CONF_1600HZ        0xACU
#define ICM42688P_GYR_CONF_1600HZ        0xACU
#define ICM42688P_ACC_RANGE_16G          0x03U
#define ICM42688P_GYR_RANGE_2000DPS      0x00U
#define ICM42688P_INT1_DRDY_ACCEL        0x01U
#define ICM42688P_DATA_STAT_GYR_CONF_ERR (1U << 5)
#define ICM42688P_DATA_STAT_ACC_CONF_ERR (1U << 4)

#define ICM42688P_SENSOR_FRAME_SIZE      12U
#define ICM42688P_TEMP_FRAME_SIZE        2U
#define ICM42688P_RAW_FRAME_SIZE         14U

#define ICM42688P_ACCEL_LSB_PER_G        2048.0F
#define ICM42688P_GYRO_LSB_PER_DPS       16.4F
#define ICM42688P_TEMPERATURE_OFFSET_C   23.0F
#define ICM42688P_TEMPERATURE_LSB_PER_C  512.0F

struct Icm42688pRawFrame {
    uint8_t accel_be[6];
    uint8_t gyro_be[6];
    uint8_t temperature_be[2];
};
