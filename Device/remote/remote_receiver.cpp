#include "remote_receiver.hpp"
#include <errno.h>
#include <string.h>

int RemoteReceiver::init(const RemoteProtocol &protocol)
{
    if (protocol.uart_device == nullptr || protocol.frame_size == 0U ||
        protocol.frame_size > sizeof(frame_) || protocol.validate == nullptr ||
        protocol.decode == nullptr) return -EINVAL;
    protocol_ = protocol;
    ring_buffer_init(&byte_ring_, byte_ring_storage_, sizeof(byte_ring_storage_));
    k_sem_init(&data_ready_, 0, 1);
    atomic_set(&rx_overrun_, 0);
    int ret = bsp_uart_init(&uart_, protocol_.uart_device, &protocol_.uart_config);
    if (ret == 0) initialized_ = true;
    return ret;
}

int RemoteReceiver::start()
{
    if (!initialized_) return -EPERM;
    if (started_) return 0;
    k_tid_t tid = k_thread_create(&worker_, worker_stack_,
        K_KERNEL_STACK_SIZEOF(worker_stack_), worker_entry, this, nullptr,
        nullptr, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
    if (tid == nullptr) return -ENOMEM;
    (void)k_thread_name_set(tid, "remote_rx");
    int ret = bsp_uart_rx_attach(&uart_, rx_bytes, this);
    if (ret != 0) { k_thread_abort(tid); return ret; }
    started_ = true;
    return 0;
}

void RemoteReceiver::rx_bytes(const uint8_t *data, size_t length, void *user)
{
    auto *self = static_cast<RemoteReceiver *>(user);
    if (self == nullptr || data == nullptr || length == 0U ||
        atomic_get(&self->rx_overrun_) != 0) return;
    if (ring_buffer_free(&self->byte_ring_) < length ||
        ring_buffer_write(&self->byte_ring_, data, length) != length)
        atomic_set(&self->rx_overrun_, 1);
    k_sem_give(&self->data_ready_);
}

void RemoteReceiver::worker_entry(void *instance, void *unused1, void *unused2)
{
    ARG_UNUSED(unused1); ARG_UNUSED(unused2);
    auto *self = static_cast<RemoteReceiver *>(instance);
    uint8_t byte;
    while (true) {
        (void)k_sem_take(&self->data_ready_, K_FOREVER);
        if (atomic_cas(&self->rx_overrun_, 1, 2)) {
            ring_buffer_clear(&self->byte_ring_);
            self->frame_length_ = 0U;
            atomic_set(&self->rx_overrun_, 0);
            continue;
        }
        while (ring_buffer_read(&self->byte_ring_, &byte, 1U) == 1U)
            self->process_byte(byte);
    }
}

void RemoteReceiver::process_byte(uint8_t byte)
{
    if (frame_length_ == protocol_.frame_size)
        memmove(frame_, frame_ + 1, --frame_length_);
    frame_[frame_length_++] = byte;
    if (frame_length_ < protocol_.frame_size ||
        !protocol_.validate(frame_, frame_length_)) return;
    RemoteSample sample{};
    if (protocol_.decode(frame_, frame_length_, sample)) publish_sample(sample);
    frame_length_ = 0U;
}
