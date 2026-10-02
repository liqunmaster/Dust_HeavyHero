#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter {

    
    class FirstOrderLpf final {
        public:

        

        void Init(float cutoff_hz, float sample_rate_hz) {
            if (cutoff_hz > 0.0F && sample_rate_hz > 0.0F) {
                const float rc = 1.0F / (math::two_pi * cutoff_hz);
                const float dt = 1.0F / sample_rate_hz;
                alpha_         = dt / (rc + dt);
            } else {
                alpha_ = 1.0F;
            }
            Reset();
        }

        

        float Update(float input) {
            if (!initialized_) {
                value_       = input;
                initialized_ = true;
            } else {
                value_ += alpha_ * (input - value_);
            }
            return value_;
        }

        

        void Reset() {
            value_       = 0.0F;
            initialized_ = false;
        }

        

        void Reset(float initial_value) {
            value_       = initial_value;
            initialized_ = true;
        }

        private:

        float alpha_      = 1.0F;

        float value_      = 0.0F;

        bool initialized_ = false;
    };

    
    class LowPassFilter final {
        public:

        

        void Init(float cutoff_hz, float sample_rate_hz, uint8_t order = 1U) {
            order_ = order >= 1U && order <= kMaxOrder ? order : 1U;
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].Init(cutoff_hz, sample_rate_hz);
            }
        }

        

        float Update(float input) {
            for (uint8_t i = 0; i < order_; ++i) {
                input = stages_[i].Update(input);
            }
            return input;
        }

        

        void Reset() {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].Reset();
            }
        }

        

        void Reset(float initial_value) {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].Reset(initial_value);
            }
        }

        private:

        static constexpr uint8_t kMaxOrder = 10U;

        FirstOrderLpf stages_[kMaxOrder]{};

        uint8_t order_ = 1U;
    };
}
