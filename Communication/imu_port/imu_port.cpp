#include "imu_port.hpp"

#define IMU_NODE DT_ALIAS(imu_spi)

namespace {

    struct imu_state {
        struct k_spinlock lock;
        imu_sample sample{};
        atomic_t feedback_count{};
        bool has_sample{false};
    };

    imu_state state{};
    bsp_spi imu_spi{};
    bsp_gpio_irq imu_int1_irq{};
    const gpio_dt_spec imu_int1 = GPIO_DT_SPEC_GET(IMU_NODE, int1_gpios);
    const gpio_dt_spec imu_int2 = GPIO_DT_SPEC_GET(IMU_NODE, int2_gpios);

    K_SEM_DEFINE(imu_sem, 0, 1);
    K_THREAD_STACK_DEFINE(imu_stack, 2048);
    struct k_thread imu_thread;

    int read_registers(uint8_t address, uint8_t *data, size_t length)
    {
        if (data == nullptr || length == 0U || length + 1U > BSP_SPI_BUFFER_SIZE ||
            address > 0x7FU) {
            return -EINVAL;
        }

        uint8_t tx[BSP_SPI_BUFFER_SIZE]{};
        uint8_t rx[BSP_SPI_BUFFER_SIZE]{};
        tx[0] = static_cast<uint8_t>(address | ICM42688PHXY_SPI_READ_BIT);
        const int ret = bsp_spi_transceive(&imu_spi, tx, rx, length + 1U);
        if (ret != 0) {
            return ret;
        }
        for (size_t i = 0U; i < length; ++i) {
            data[i] = rx[i + 1U];
        }
        return 0;
    }

    int write_register(uint8_t address, uint8_t value)
    {
        if (address > 0x7FU) {
            return -EINVAL;
        }
        const uint8_t tx[2] = {address, value};
        uint8_t rx[sizeof(tx)]{};
        return bsp_spi_transceive(&imu_spi, tx, rx, sizeof(tx));
    }

    int configure_register(uint8_t address, uint8_t value, uint32_t settle_ms)
    {
        int ret = write_register(address, value);
        if (ret != 0) {
            return ret;
        }
        if (settle_ms != 0U) {
            k_sleep(K_MSEC(settle_ms));
        }

        uint8_t readback = 0U;
        ret = read_registers(address, &readback, 1U);
        return ret != 0 ? ret : (readback == value ? 0 : -EIO);
    }

    int init_icm42688p_hxy()
    {
        if (!device_is_ready(imu_int1.port) || !device_is_ready(imu_int2.port)) {
            return -ENODEV;
        }

        const int ret = bsp_spi_init(&imu_spi, DEVICE_DT_GET(DT_BUS(IMU_NODE)), static_cast<uint16_t>(DT_REG_ADDR(IMU_NODE)), DT_PROP(IMU_NODE, spi_max_frequency), SPI_MODE_CPOL | SPI_MODE_CPHA);
        if (ret != 0) {
            return ret;
        }

        k_busy_wait(3000U);
        if (write_register(ICM42688PHXY_REG_SOFT_RST, ICM42688PHXY_SOFT_RST_VALUE) != 0) {
            return -EIO;
        }
        k_sleep(K_MSEC(50));

        uint8_t who_am_i = 0U;
        if (read_registers(ICM42688PHXY_REG_WHO_AM_I, &who_am_i, 1U) != 0 || who_am_i != ICM42688PHXY_WHO_AM_I_VALUE) {
            return -ENODEV;
        }

        if (configure_register(ICM42688PHXY_REG_PWR_CTRL, ICM42688PHXY_PWR_ALL_ON, 10U)         != 0 || 
            configure_register(ICM42688PHXY_REG_COM_CFG, ICM42688PHXY_COM_CFG_BDU_AUTO_INC, 1U) != 0 ||
            configure_register(ICM42688PHXY_REG_ACC_CONF, ICM42688PHXY_ACC_CONF_1600HZ, 1U)     != 0 ||
            configure_register(ICM42688PHXY_REG_ACC_RANGE, ICM42688PHXY_ACC_RANGE_16G, 1U)      != 0 ||
            configure_register(ICM42688PHXY_REG_GYR_CONF, ICM42688PHXY_GYR_CONF_1600HZ, 1U)     != 0 ||
            configure_register(ICM42688PHXY_REG_GYR_RANGE, ICM42688PHXY_GYR_RANGE_2000DPS, 1U)  != 0) {
            return -EIO;
        }

        uint8_t status = 0U;
        if (read_registers(ICM42688PHXY_REG_DATA_STAT, &status, 1U) != 0 ||
            (status & (ICM42688PHXY_DATA_STAT_GYR_CONF_ERR | ICM42688PHXY_DATA_STAT_ACC_CONF_ERR)) != 0U) {
            return -EIO;
        }

        const int irq_ret = bsp_gpio_exti_init(&imu_int1_irq, &imu_int1, GPIO_INT_EDGE_TO_ACTIVE, [](void *) { k_sem_give(&imu_sem); }, nullptr);
        if (irq_ret != 0) {
            return irq_ret;
        }
        return configure_register(ICM42688PHXY_REG_INT_CFG1, ICM42688PHXY_INT1_DRDY_ACCEL, 1U);
    }

    int read_imu_frame()
    {
        uint8_t status = 0U;
        int ret = read_registers(ICM42688PHXY_REG_DATA_STAT, &status, 1U);
        if (ret != 0) {
            return ret;
        }
        if ((status & (ICM42688PHXY_DATA_STAT_GYR_CONF_ERR | ICM42688PHXY_DATA_STAT_ACC_CONF_ERR)) != 0U) {
            return -EIO;
        }
        if ((status & ICM42688PHXY_DATA_STAT_ACC_READY) == 0U) {
            return -EAGAIN;
        }

        uint8_t frame[icm42688phxy::frame_size]{};
        ret = read_registers(ICM42688PHXY_REG_ACC_XH, frame, ICM42688PHXY_SENSOR_FRAME_SIZE);
        if (ret != 0) {
            return ret;
        }
        ret = read_registers(ICM42688PHXY_REG_TEMP_H, &frame[ICM42688PHXY_SENSOR_FRAME_SIZE], ICM42688PHXY_TEMP_FRAME_SIZE);
        if (ret != 0) {
            return ret;
        }

        atomic_inc(&state.feedback_count);

        imu_sample decoded{};
        if (icm42688phxy::decode_frame(frame, sizeof(frame), decoded) == 0) {
            const k_spinlock_key_t key = k_spin_lock(&state.lock);
            state.sample = decoded;
            state.has_sample = true;
            k_spin_unlock(&state.lock, key);
        }

        static int n = 0;

        return 0;
    }

    void imu_port_process()
    {
        (void)read_imu_frame();
    }

    void imu_thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&imu_sem, K_FOREVER);
            imu_port_process();
        }
    }

}

int imu_port_init()
{
    const int ret = init_icm42688p_hxy();
    if (ret != 0) {
        return ret;
    }

    k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack),
                    imu_thread_entry, nullptr, nullptr, nullptr,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    return 0;
}

int imu_port_get_sample(imu_sample &sample)
{
    const k_spinlock_key_t key = k_spin_lock(&state.lock);
    if (!state.has_sample) {
        k_spin_unlock(&state.lock, key);
        return -ENODATA;
    }
    sample = state.sample;
    k_spin_unlock(&state.lock, key);
    return 0;
}

uint32_t imu_port_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&state.feedback_count));
}
