#include "input.hpp"

#include <errno.h>

#include "remote_chan.hpp"
#if defined(CONFIG_REMOTE_DEVICE_DT7)
#include "dt7.hpp"
#elif defined(CONFIG_REMOTE_DEVICE_VT02)
#include "vt02.hpp"
#elif defined(CONFIG_REMOTE_DEVICE_VT03)
#include "vt03.hpp"
#endif

static constexpr int32_t kChannelCenter = 1024;
static constexpr float kChannelSpan = 660.0F;
static constexpr int32_t kDeadZone = 20;

static float normalize_channel(uint16_t value)
{
    int32_t centered = static_cast<int32_t>(value) - kChannelCenter;
    if (centered > -kDeadZone && centered < kDeadZone) centered = 0;
    float normalized = static_cast<float>(centered) / kChannelSpan;
    if (normalized > 1.0F) normalized = 1.0F;
    if (normalized < -1.0F) normalized = -1.0F;
    return normalized;
}

int InputModule::init()
{
    RemoteSample initial{};
    double_buffer_init(&sample_buffer_, &sample_storage_[0],
                       &sample_storage_[1], sizeof(initial));
    double_buffer_reset(&sample_buffer_, &initial);
    k_sem_init(&sample_ready_, 0, 1);
    const RemoteProtocol *protocol = nullptr;
#if defined(CONFIG_REMOTE_DEVICE_DT7)
    protocol = &dt7_protocol();
#elif defined(CONFIG_REMOTE_DEVICE_VT02)
    protocol = &vt02_protocol();
#elif defined(CONFIG_REMOTE_DEVICE_VT03)
    protocol = &vt03_protocol();
#endif
    if (protocol == nullptr) return -ENODEV;
    int ret = receiver_.init(*protocol);
    if (ret != 0) return ret;
    receiver_.set_sample_callback(sample_callback, this);
    k_tid_t tid = k_thread_create(
        &worker_, worker_stack_, K_KERNEL_STACK_SIZEOF(worker_stack_),
        worker_entry, this, nullptr, nullptr, K_PRIO_PREEMPT(6), 0,
        K_NO_WAIT);
    if (tid == nullptr) return -ENOMEM;
    (void)k_thread_name_set(tid, "input");
    ret = receiver_.start();
    if (ret != 0) { k_thread_abort(tid); return ret; }
    return ret;
}

void InputModule::sample_callback(const RemoteSample *sample, void *user_data)
{
    if (sample == nullptr || user_data == nullptr) return;
    auto *input = static_cast<InputModule *>(user_data);
    auto *destination = static_cast<RemoteSample *>(
        double_buffer_write_array(&input->sample_buffer_));
    if (destination == nullptr) return;
    *destination = *sample;
    double_buffer_publish(&input->sample_buffer_);
    k_sem_give(&input->sample_ready_);
}

void InputModule::worker_entry(void *instance, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    auto *input = static_cast<InputModule *>(instance);
    while (true) {
        (void)k_sem_take(&input->sample_ready_, K_FOREVER);
        RemoteSample sample{};
        if (double_buffer_snapshot(&input->sample_buffer_, &sample)) {
            input->process_sample(sample);
        }
    }
}

void InputModule::process_sample(const RemoteSample &sample)
{
    RemoteTopicData topic{};
    if (sample.type == RemoteType::Vt02) {
        const bool w = (sample.keyboard & (1U << 0)) != 0U;
        const bool s = (sample.keyboard & (1U << 1)) != 0U;
        const bool a = (sample.keyboard & (1U << 2)) != 0U;
        const bool d = (sample.keyboard & (1U << 3)) != 0U;
        topic.forward = static_cast<float>(int(w) - int(s));
        topic.lateral = static_cast<float>(int(d) - int(a));
        topic.rotation = static_cast<float>(sample.mouse_x) / 32767.0F;
        if (topic.rotation > 1.0F) topic.rotation = 1.0F;
        if (topic.rotation < -1.0F) topic.rotation = -1.0F;
    } else {
        topic.forward = normalize_channel(sample.channel[1]);
        topic.lateral = normalize_channel(sample.channel[0]);
        topic.rotation = normalize_channel(sample.channel[2]);
    }
    topic.switch_left = sample.switch_left;
    topic.switch_right = sample.switch_right;
    topic.timestamp_ms = sample.timestamp_ms;
    topic.valid = true;
    (void)remote_topic_publish(topic);
}
