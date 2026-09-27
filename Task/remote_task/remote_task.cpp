#include "remote_task.hpp"

namespace {
    struct remote_state {
        struct k_spinlock lock;
        remote_sample sample;
        atomic_t feedback_count;
        bool has_sample;
    };

    remote_state state{};

    K_SEM_DEFINE(remote_sem, 0, 1);

    K_THREAD_STACK_DEFINE(remote_stack, 1024);

    struct k_thread remote_thread;
    
    struct k_timer remote_timer;

    void store_sample(const remote_sample &sample)
    {
        const k_spinlock_key_t key = k_spin_lock(&state.lock);
        state.sample = sample;
        state.has_sample = true;
        k_spin_unlock(&state.lock, key);
    }

    #ifdef CONFIG_REMOTE_DEVICE_DT7
        static constexpr size_t remote_frame_size = dt7::frame_size;
        static const struct device *const remote_uart = DEVICE_DT_GET(DT_ALIAS(dt7_uart));
        uint8_t frame[remote_frame_size];
        size_t frame_length = 0U;

        int init_remote_uart()
        {
            if (!device_is_ready(remote_uart)) {
                return -ENODEV;   
            }

            struct uart_config config{};
            int ret = uart_config_get(remote_uart, &config);
            if (ret != 0) {
                return ret;
            }
            config.parity = static_cast<uart_config_parity>(DT_ENUM_IDX(DT_ALIAS(dt7_uart), parity));
            ret = uart_configure(remote_uart, &config);
            if (ret != 0)  {
                return ret;
            }
            return bsp_uart_init(remote_uart);
        }

        void process_remote_byte(uint8_t byte)
        {
            frame[frame_length++] = byte;
            if (frame_length != remote_frame_size) {
                return;
            }
            frame_length = 0U;
            atomic_inc(&state.feedback_count);

            remote_sample decoded{};
            if (dt7::decode_frame(frame, sizeof(frame), decoded) == 0) {
                store_sample(decoded);
            }
        }

    #elif defined(CONFIG_REMOTE_DEVICE_VT03)
        static constexpr size_t remote_frame_size = vt03::frame_size;
        static const struct device *const remote_uart = DEVICE_DT_GET(DT_ALIAS(vt03_uart));
        uint8_t frame[remote_frame_size];
        size_t frame_length = 0U;

        int init_remote_uart()
        {
            if (!device_is_ready(remote_uart)) return -ENODEV;
            return bsp_uart_init(remote_uart);
        }

        void process_remote_byte(uint8_t byte)
        {
            if (frame_length == 0U && byte != 0xA9U) return;
            if (frame_length == 1U && byte != 0x53U) {
                frame_length = byte == 0xA9U ? 1U : 0U;
                return;
            }

            frame[frame_length++] = byte;
            if (frame_length != remote_frame_size) {
                return;
            }
            frame_length = 0U;
            atomic_inc(&state.feedback_count);

            remote_sample decoded{};
            if (vt03::decode_frame(frame, sizeof(frame), decoded) == 0) {
                store_sample(decoded);
            }
        }
    #endif

    void remote_timer_callback(struct k_timer *)
    {
        k_sem_give(&remote_sem);
    }

    void remote_task()
    {
        uint8_t bytes[remote_frame_size * 4U];
        int count;
        while ((count = bsp_uart_receive(bytes, sizeof(bytes))) > 0) {
            for (int i = 0; i < count; ++i) {
                process_remote_byte(bytes[i]);
            }
        }
        if (count < 0) {
            frame_length = 0U;
        }
    }

    void remote_thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&remote_sem, K_FOREVER);
            remote_task();
        }
    }
}

int remote_init()
{
    const int result = init_remote_uart();
    if (result != 0) {
        return result;
    }

    k_timer_init(&remote_timer, remote_timer_callback, nullptr);
    k_thread_create(&remote_thread, remote_stack, K_THREAD_STACK_SIZEOF(remote_stack),
                    remote_thread_entry, nullptr, nullptr, nullptr,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_timer_start(&remote_timer, K_MSEC(1), K_MSEC(1));
    return 0;
}

int remote_task_get_sample(remote_sample &sample)
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

uint32_t remote_task_feedback_count()
{
    return static_cast<uint32_t>(atomic_get(&state.feedback_count));
}
