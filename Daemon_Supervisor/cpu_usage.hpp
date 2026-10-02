#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "ws2812b.hpp"


class cpu_usage final {
    public:
    

    int display_on_ws2812b() {
        float percent = 0.0f;
        const int ret = sample(percent);
        if (ret != 0) {
            (void)ws2812b_off();
            return ret;
        }

        constexpr float brightness = 126.0f;
        if (percent < 0.0f) {
            percent = 0.0f;
        }
        if (percent > 100.0f) {
            percent = 100.0f;
        }

        const uint8_t red   = static_cast<uint8_t>(percent * brightness / 100.0f);
        const uint8_t green = static_cast<uint8_t>((100.0f - percent) * brightness / 100.0f);
        return ws2812b_set_color({red, green, 0U});
    }

    

    int sample(float &percent) {
#if defined(CONFIG_SCHED_THREAD_USAGE_ALL)
        k_thread_runtime_stats current{};
        const int ret = k_thread_runtime_stats_all_get(&current);
        if (ret != 0) {
            return ret;
        }

        if (!has_previous_) {
            previous_     = current;
            has_previous_ = true;
            return -EAGAIN;
        }

        const uint64_t total = current.execution_cycles - previous_.execution_cycles;
        const uint64_t idle  = current.idle_cycles - previous_.idle_cycles;
        previous_            = current;
        if (total == 0U) {
            return -EAGAIN;
        }

        const uint64_t bounded_idle = idle < total ? idle : total;
        percent                     = 100.0f * (1.0f - static_cast<float>(bounded_idle) / static_cast<float>(total));
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



inline void cpu_usage_supervisor() {
    static cpu_usage usage;
    static bool initialized = false;
    if (!initialized) {
        (void)usage.display_on_ws2812b();
        initialized = true;
    }

    k_sleep(K_MSEC(500));

    (void)usage.display_on_ws2812b();
}



inline void cpu_survival_supervisor() {
    ws2812b_set_color({255,0,0});
    k_sleep(K_MSEC(1000));
    ws2812b_set_color({0,0,0});
    k_sleep(K_MSEC(1000));
    ws2812b_set_color({0,255,0});
    k_sleep(K_MSEC(1000));
    ws2812b_set_color({0,0,0});
    k_sleep(K_MSEC(1000));
    ws2812b_set_color({0,0,255});
    k_sleep(K_MSEC(1000));
    ws2812b_set_color({0,0,0});
    k_sleep(K_MSEC(1000));
}
