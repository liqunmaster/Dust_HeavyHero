#include "ins.hpp"

#include <errno.h>

#include "imu_chan.hpp"

int InsModule::init(ImuDevice &imu)
{
    ImuSample initial{};
    double_buffer_init(&sample_buffer_, &sample_storage_[0],
                       &sample_storage_[1], sizeof(initial));
    double_buffer_reset(&sample_buffer_, &initial);
    k_sem_init(&sample_ready_, 0, 1);
    estimator_.configure(0.001F, 0.003F, 0.03F);
    k_tid_t tid = k_thread_create(
        &worker_, worker_stack_, K_KERNEL_STACK_SIZEOF(worker_stack_),
        worker_entry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0,
        K_NO_WAIT);
    if (tid == nullptr) return -ENOMEM;
    (void)k_thread_name_set(tid, "ins");
    imu.set_sample_callback(sample_callback, this);
    return 0;
}

void InsModule::sample_callback(const ImuSample *sample, void *user_data)
{
    if (sample == nullptr || user_data == nullptr) return;
    auto *ins = static_cast<InsModule *>(user_data);
    auto *destination = static_cast<ImuSample *>(
        double_buffer_write_array(&ins->sample_buffer_));
    if (destination == nullptr) return;
    *destination = *sample;
    double_buffer_publish(&ins->sample_buffer_);
    k_sem_give(&ins->sample_ready_);
}

void InsModule::worker_entry(void *instance, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    auto *ins = static_cast<InsModule *>(instance);
    while (true) {
        (void)k_sem_take(&ins->sample_ready_, K_FOREVER);
        ImuSample sample{};
        if (double_buffer_snapshot(&ins->sample_buffer_, &sample)) {
            ins->process_sample(sample);
        }
    }
}

void InsModule::process_sample(const ImuSample &sample)
{
    float dt = 0.001F;
    if (last_timestamp_ms_ != 0U) {
        dt = static_cast<float>(sample.timestamp_ms - last_timestamp_ms_) *
            0.001F;
        if (dt <= 0.0F || dt > 0.05F) dt = 0.001F;
    }
    last_timestamp_ms_ = sample.timestamp_ms;

    const AttitudeState &attitude = estimator_.update(
        sample.accel_m_s2, sample.gyro_rad_s, dt);
    ImuTopicData topic{};
    topic.quaternion[0] = attitude.orientation.w;
    topic.quaternion[1] = attitude.orientation.x;
    topic.quaternion[2] = attitude.orientation.y;
    topic.quaternion[3] = attitude.orientation.z;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        topic.accel_m_s2[axis] = sample.accel_m_s2[axis];
        topic.gyro_rad_s[axis] = sample.gyro_rad_s[axis];
    }
    topic.roll_rad = attitude.roll_rad;
    topic.pitch_rad = attitude.pitch_rad;
    topic.yaw_rad = attitude.yaw_rad;
    topic.temperature_c = sample.temperature_c;
    topic.timestamp_ms = sample.timestamp_ms;
    topic.valid = true;
    (void)imu_topic_publish(topic);
}
