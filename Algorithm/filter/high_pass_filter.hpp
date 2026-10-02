#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter {

    
    class FirstOrderHpf final {
        public:
        

        void Init(float cutoff_hz, float sample_rate_hz) {
            enabled_ = cutoff_hz > 0.0F && sample_rate_hz > 0.0F;
            if (enabled_) {
                const float rc = 1.0F / (math::two_pi * cutoff_hz);
                const float dt = 1.0F / sample_rate_hz;
                alpha_         = rc / (rc + dt);
            }
            Reset();
        }

        

        float Update(float input) {
            if (!enabled_) {
                return input;
            }
            if (!initialized_) {
                previous_input_ = input;
                initialized_    = true;
                return 0.0F;
            }
            value_          = alpha_ * (value_ + input - previous_input_);
            previous_input_ = input;
            return value_;
        }

        

        void Reset() {
            value_          = 0.0F;
            previous_input_ = 0.0F;
            initialized_    = false;
        }

        

        void Reset(float initial_input) {
            value_          = 0.0F;
            previous_input_ = initial_input;
            initialized_    = true;
        }

        private:
        float alpha_          = 1.0F;

        float value_          = 0.0F;

        float previous_input_ = 0.0F;

        bool initialized_     = false;

        bool enabled_         = false;
    };

    
    class HighPassFilter final {
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

        

        void Reset(float initial_input) {
            Reset();
            Update(initial_input);
        }

        private:

        static constexpr uint8_t kMaxOrder = 10U;
        
        FirstOrderHpf stages_[kMaxOrder]{};

        uint8_t order_ = 1U;
    };

}
