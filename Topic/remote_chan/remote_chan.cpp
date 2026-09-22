#include "remote_chan.hpp"

#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DEFINE(remote_data_channel, RemoteTopicData, nullptr, nullptr,
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT(.forward = 0.0F, .lateral = 0.0F,
                               .rotation = 0.0F, .valid = false));

int remote_topic_publish(const RemoteTopicData &data)
{
    return zbus_chan_pub(&remote_data_channel, &data, K_NO_WAIT);
}

int remote_topic_read(RemoteTopicData &data)
{
    return zbus_chan_read(&remote_data_channel, &data, K_NO_WAIT);
}
