#include "imu_chan.hpp"

#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DEFINE(imu_data_channel, ImuTopicData, nullptr, nullptr,
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT(.quaternion = {1.0F, 0.0F, 0.0F, 0.0F},
                               .valid = false));

int imu_topic_publish(const ImuTopicData &data)
{
    return zbus_chan_pub(&imu_data_channel, &data, K_NO_WAIT);
}

int imu_topic_read(ImuTopicData &data)
{
    return zbus_chan_read(&imu_data_channel, &data, K_NO_WAIT);
}
