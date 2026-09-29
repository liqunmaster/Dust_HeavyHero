#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

class cpu_usage final {
public:
    int sample(float &percent)
    {
#if defined(CONFIG_SCHED_THREAD_USAGE_ALL)
        k_thread_runtime_stats current{};
        const int ret = k_thread_runtime_stats_all_get(&current);
        if (ret != 0) return ret;

        if (!has_previous_) {
            previous_ = current;
            has_previous_ = true;
            return -EAGAIN;
        }

        const uint64_t total = current.execution_cycles - previous_.execution_cycles;
        const uint64_t idle  = current.idle_cycles - previous_.idle_cycles;
        previous_ = current;
        if (total == 0U) return -EAGAIN;

        const uint64_t bounded_idle = idle < total ? idle : total;
        percent = 100.0f * (1.0f - static_cast<float>(bounded_idle) / static_cast<float>(total));
        return 0;
#else
        (void)percent;
        return -ENOTSUP;
#endif
    }

private:
#if defined(CONFIG_SCHED_THREAD_USAGE_ALL)
    k_thread_runtime_stats previous_{};
    bool has_previous_{false};
#endif
};
