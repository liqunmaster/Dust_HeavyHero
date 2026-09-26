#include "icm42688p_hxy.hpp"

#include <errno.h>
#include <stdio.h>

#include <zephyr/sys/byteorder.h>

#include "icm42688p_hxy_reg.h"

static constexpr float kRadiansPerDegree = 0.017453292519943295F;
static constexpr float kGravity = 9.80665F;

K_THREAD_STACK_DEFINE(imu_sample_stack, 2048);

static_assert(sizeof(Icm42688pRawFrame) == ICM42688P_RAW_FRAME_SIZE);

int icm42688phxy::init(const icm42688phxy_config &config)
{
    int ret = bsp_spi_init(&spi_, config.spi_device, config.spi_peripheral, config.spi_frequency, SPI_MODE_CPOL | SPI_MODE_CPHA);
    if (ret != 0) return ret;

    uint8_t initial_frame[ICM42688P_RAW_FRAME_SIZE]{};
    double_buffer_init(&raw_buffer_, raw_storage_[0], raw_storage_[1],
                       sizeof(initial_frame));
    double_buffer_reset(&raw_buffer_, initial_frame);
    k_sem_init(&data_ready_sem_, 0, 1);

    k_busy_wait(3000U);
    init_register_ = ICM42688P_REG_SOFT_RST;
    ret = write_register(ICM42688P_REG_SOFT_RST,
                         ICM42688P_SOFT_RST_VALUE);
    if (ret != 0) return ret;
    k_sleep(K_MSEC(50));

    ret = read_registers(ICM42688P_REG_WHO_AM_I, &who_am_i_, 1U);
    if (ret != 0) return ret;
    if (who_am_i_ != ICM42688P_WHO_AM_I_VALUE) return -ENODEV;

    ret = configure_register(ICM42688P_REG_PWR_CTRL,
                             ICM42688P_PWR_ALL_ON, 10U);
    if (ret != 0) return ret;
    ret = configure_register(ICM42688P_REG_COM_CFG,
                             ICM42688P_COM_CFG_DEFAULT, 1U);
    if (ret != 0) return ret;
    ret = configure_register(ICM42688P_REG_ACC_CONF,
                             ICM42688P_ACC_CONF_1600HZ, 1U);
    if (ret != 0) return ret;
    ret = configure_register(ICM42688P_REG_ACC_RANGE,
                             ICM42688P_ACC_RANGE_16G, 1U);
    if (ret != 0) return ret;
    ret = configure_register(ICM42688P_REG_GYR_CONF,
                             ICM42688P_GYR_CONF_1600HZ, 1U);
    if (ret != 0) return ret;
    ret = configure_register(ICM42688P_REG_GYR_RANGE,
                             ICM42688P_GYR_RANGE_2000DPS, 1U);
    if (ret != 0) return ret;
    ret = configure_register(ICM42688P_REG_INT_CFG1,
                             ICM42688P_INT1_DRDY_ACCEL, 1U);
    if (ret != 0) return ret;

    init_register_ = ICM42688P_REG_DATA_STAT;
    ret = read_registers(ICM42688P_REG_DATA_STAT, &data_status_, 1U);
    if (ret != 0) return ret;
    init_readback_ = data_status_;
    if ((data_status_ & (ICM42688P_DATA_STAT_GYR_CONF_ERR |
                         ICM42688P_DATA_STAT_ACC_CONF_ERR)) != 0U) {
        return -EIO;
    }

    ret = bsp_gpio_input_irq_init(&data_ready_gpio_, config.gpio_device,
                                  config.data_ready_pin,
                                  GPIO_INT_LEVEL_ACTIVE,
                                  data_ready_callback, this);
    if (ret != 0) return ret;

    ret = bsp_gpio_input_irq_enable(&data_ready_gpio_);
    if (ret != 0) return ret;

    ret = sample_once();
    if (ret != 0) return ret;

    k_tid_t tid = k_thread_create(
        &worker_, imu_sample_stack, K_THREAD_STACK_SIZEOF(imu_sample_stack),
        worker_entry, this, nullptr, nullptr, K_PRIO_PREEMPT(2), 0,
        K_NO_WAIT);
    if (tid == nullptr) return -ENOMEM;
    (void)k_thread_name_set(tid, "imu_sample");
    init_register_ = 0U;
    init_readback_ = 0U;
    return 0;
}

void icm42688phxy::worker_entry(void *instance, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    auto *imu = static_cast<icm42688phxy *>(instance);
    while (true) {
        (void)k_sem_take(&imu->data_ready_sem_, K_FOREVER);
        (void)imu->sample_once();
        (void)bsp_gpio_input_irq_enable(&imu->data_ready_gpio_);
    }
}

void icm42688phxy::data_ready_callback(void *user_data)
{
    auto *imu = static_cast<icm42688phxy *>(user_data);
    if (imu != nullptr) {
        (void)bsp_gpio_input_irq_disable(&imu->data_ready_gpio_);
        k_sem_give(&imu->data_ready_sem_);
    }
}

int icm42688phxy::sample_once()
{
    auto *write_bytes = static_cast<uint8_t *>(
        double_buffer_write_array(&raw_buffer_));
    if (write_bytes == nullptr) return -EINVAL;
    auto *write_sample = reinterpret_cast<Icm42688pRawFrame *>(write_bytes);
    int ret = read_registers(ICM42688P_REG_ACC_XH,
                             write_sample->accel_be,
                             ICM42688P_SENSOR_FRAME_SIZE);
    if (ret != 0) return ret;
    ret = read_registers(ICM42688P_REG_TEMP_H,
                         write_sample->temperature_be,
                         ICM42688P_TEMP_FRAME_SIZE);
    if (ret != 0) return ret;
    double_buffer_publish(&raw_buffer_);
    const auto *raw = static_cast<const Icm42688pRawFrame *>(
        double_buffer_read_array(&raw_buffer_));
    if (raw == nullptr) return -EINVAL;

    ImuSample sample{};
    for (size_t axis = 0U; axis < 3U; ++axis) {
        const int16_t accel_raw = static_cast<int16_t>(
            sys_get_be16(&raw->accel_be[axis * 2U]));
        const int16_t gyro_raw = static_cast<int16_t>(
            sys_get_be16(&raw->gyro_be[axis * 2U]));
        sample.accel_m_s2[axis] = static_cast<float>(accel_raw) *
            (kGravity / ICM42688P_ACCEL_LSB_PER_G);
        sample.gyro_rad_s[axis] = static_cast<float>(gyro_raw) *
            (kRadiansPerDegree / ICM42688P_GYRO_LSB_PER_DPS);
    }
    const int16_t temperature_raw = static_cast<int16_t>(
        sys_get_be16(raw->temperature_be));
    sample.temperature_c = ICM42688P_TEMPERATURE_OFFSET_C +
        static_cast<float>(temperature_raw) / ICM42688P_TEMPERATURE_LSB_PER_C;
    sample.timestamp_ms = k_uptime_get_32();
    publish_sample(sample);
    return 0;
}

int icm42688phxy::read_registers(uint8_t address, uint8_t *data,
                                 size_t length)
{
    if (data == nullptr || length == 0U || length > 16U) return -EINVAL;
    uint8_t tx[17]{};
    uint8_t rx[17]{};
    tx[0] = static_cast<uint8_t>(address | ICM42688P_READ_FLAG);
    const int ret = bsp_spi_transceive(&spi_, tx, rx, length + 1U);
    if (ret != 0) return ret;
    for (size_t index = 0U; index < length; ++index) {
        data[index] = rx[index + 1U];
    }
    return 0;
}

int icm42688phxy::write_register(uint8_t address, uint8_t value)
{
    const uint8_t tx[2] = {
        static_cast<uint8_t>(address & 0x7FU),
        value,
    };
    uint8_t discarded[sizeof(tx)]{};
    return bsp_spi_transceive(&spi_, tx, discarded, sizeof(tx));
}

int icm42688phxy::configure_register(uint8_t address, uint8_t value,
                                     uint32_t settle_ms)
{
    init_register_ = address;
    init_readback_ = 0U;
    int ret = write_register(address, value);
    if (ret != 0) return ret;
    if (settle_ms != 0U) k_sleep(K_MSEC(settle_ms));
    ret = read_registers(address, &init_readback_, 1U);
    if (ret != 0) return ret;
    return init_readback_ == value ? 0 : -EIO;
}
