#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>


class kalman_axis final {
    public:

    kalman_axis() = default;

    kalman_axis(float process_noise, float measurement_noise);

    void configure(float process_noise, float measurement_noise);

    void reset(float value = 0.0F);

    float update(float measurement);

    private:

    float process_noise_{0.001F};

    float measurement_noise_{0.05F};

    float estimate_{0.0F};

    float covariance_{1.0F};

    bool initialized_{false};
};


class kalman_vector3 final {
    public:

    kalman_vector3() = default;

    kalman_vector3(float process_noise, float measurement_noise);

    void configure(float process_noise, float measurement_noise);

    void reset(const float value[3] = nullptr);

    void update(const float measurement[3], float filtered[3]);

    private:

    kalman_axis axis_[3];
};

namespace alg::filter {

    template <int NX, int NZ, int NU>
    class ExtendedKalman final {
        public:
        using State       = Eigen::Matrix<float, NX, 1>;
        using Cov         = Eigen::Matrix<float, NX, NX>;
        using Obs         = Eigen::Matrix<float, NZ, 1>;
        using ObsCov      = Eigen::Matrix<float, NZ, NZ>;
        using Ctrl        = Eigen::Matrix<float, NU, 1>;
        using ObsMat      = Eigen::Matrix<float, NZ, NX>;
        using SystemFunc  = void (*)(const State&, const Ctrl&, State&, Cov&);
        using ObserveFunc = void (*)(const State&, Obs&, ObsMat&);

        void Init(const State& initial_state, const Cov& initial_covariance);

        

        void SetSystemFunc(SystemFunc function) {
            system_func_ = function;
        }

        

        void SetObserveFunc(ObserveFunc function) {
            observe_func_ = function;
        }

        

        void SetState(const State& state) {
            state_ = state; predicted_state_ = state;
        }
        void SetCovariance(const Cov& covariance) {
            covariance_ = covariance; predicted_covariance_ = covariance;
        }

        void SetGainRowScale(int row, float scale);
        void SetCorrectionLimit(int row, float limit);

        bool Predict(const Ctrl& control, const Cov& process_noise);
        
        bool Update(const Obs& measurement, const ObsCov& measurement_noise, float max_chi2 = std::numeric_limits<float>::infinity(), float gain_scale = 1.0F);

        void UsePrediction();

        void FadePredictedVariance(int index, float lambda, float max_variance);

        

        const State& GetX() const {
            return state_;
        }
        

        const Cov& GetP() const {
            return covariance_;
        }
        

        float GetChi2() const {
            return chi2_;
        }

        private:

        SystemFunc system_func_{nullptr};

        ObserveFunc observe_func_{nullptr};

        State state_{State::Zero()};

        State predicted_state_{State::Zero()};

        Cov covariance_{Cov::Identity()};

        Cov predicted_covariance_{Cov::Identity()};

        State gain_row_scale_{State::Ones()};

        State correction_limit_{State::Constant(std::numeric_limits<float>::infinity())};
        
        float chi2_{0.0F};
    };

    extern template class ExtendedKalman<6, 3, 4>;

}
