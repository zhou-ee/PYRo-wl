/**
 * @file pyro_algo_linear_kf.h
 * @brief 通用 KF 核心的可选线性模型适配器。
 *        Optional linear-model adapter over the common KF core.
 */
#ifndef PYRO_ALGO_LINEAR_KF_H
#define PYRO_ALGO_LINEAR_KF_H

#include "pyro_algo_kf.h"

namespace pyro
{

/**
 * @brief 在线性模型层计算 F*x+B*u 和 z-H*x；数值核心使用统一接口。
 *        Compute F*x+B*u and z-H*x in the linear layer, using the common core.
 * @tparam N 状态维度 / State dimension.
 * @tparam Z 最大观测维度 / Maximum observation dimension.
 * @tparam U 控制维度，0 表示无控制 / Control dimension; 0 means no control.
 */
template <std::size_t N, std::size_t Z, std::size_t U = 0>
class linear_kalman_filter_t : public kalman_filter_t<N, Z>
{
    using base_t = kalman_filter_t<N, Z>;
    static_assert(U <= 65535, "Invalid CMSIS control dimension");

  public:
    using typename base_t::covariance_t;
    using typename base_t::measurement_mask_t;
    using typename base_t::measurement_noise_t;
    using typename base_t::measurement_t;
    using typename base_t::observation_t;
    using typename base_t::state_t;
    using typename base_t::transition_t;
    using control_t =
        std::array<float, U>; ///< 控制向量 u，U×1 / Control vector.
    using control_matrix_t =
        std::array<float, N * U>; ///< B，N×U / Control matrix.

    /// 线性预测 x_pred=F*x。 / Linear prediction x_pred=F*x.
    status_t predict(const transition_t &F, const covariance_t &Q_d)
    {
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, kf_result_t::NOT_INITIALIZED);
        if (!this->_finite(F))
            return this->_report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
        state_t x_pred{};
        if (!this->_check(this->_multiply(
                F.data(), N, N, this->get_state().data(), 1, x_pred.data())))
            return PYRO_ERROR;
        if (!this->_finite(x_pred))
            return this->_report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
        return base_t::predict(x_pred, F, Q_d);
    }

    /// 带控制预测 x_pred=F*x+B*u，要求 U>0。 / Controlled prediction, U>0.
    status_t predict(const transition_t &F, const covariance_t &Q_d,
                     const control_matrix_t &B, const control_t &u)
    {
        static_assert(U > 0, "Use predict(F,Q_d) when U is zero");
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, kf_result_t::NOT_INITIALIZED);
        if (!this->_finite(F) || !this->_finite(B) || !this->_finite(u))
            return this->_report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
        state_t x_pred{}, dx{};
        if (!this->_check(this->_multiply(
                F.data(), N, N, this->get_state().data(), 1, x_pred.data())) ||
            !this->_check(
                this->_multiply(B.data(), N, U, u.data(), 1, dx.data())) ||
            !this->_check(
                this->_add(x_pred.data(), dx.data(), N, 1, x_pred.data())))
            return PYRO_ERROR;
        if (!this->_finite(x_pred))
            return this->_report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
        return base_t::predict(x_pred, F, Q_d);
    }

    /// 线性修正 y=z-H*x；缺测规则同 EKF。 / Linear correction; same missing
    /// rows.
    template <typename Policy = typename base_t::accept_t>
    status_t correct(const measurement_t &z, const observation_t &H,
                     const measurement_noise_t &R,
                     const measurement_mask_t *mask = nullptr,
                     Policy policy                  = Policy{})
    {
        if (!this->is_initialized())
            return this->_report(PYRO_NOT_FOUND, kf_result_t::NOT_INITIALIZED);
        measurement_t y{};
        measurement_mask_t active{};
        for (std::size_t row = 0; row < Z; ++row)
        {
            active[row] = (!mask || (*mask)[row]) && kf_is_finite(z[row]);
            if (!active[row])
                continue;
            for (std::size_t col = 0; col < N; ++col)
                if (!kf_is_finite(H[row * N + col]))
                    return this->_report(PYRO_PARAM_ERROR,
                                         kf_result_t::INVALID_INPUT);
            float z_pred = 0.0f;
            if (!this->_check(this->_multiply(H.data() + row * N, 1, N,
                                              this->get_state().data(), 1,
                                              &z_pred)))
                return PYRO_ERROR;
            y[row] = z[row] - z_pred;
            if (!kf_is_finite(y[row]))
                return this->_report(PYRO_ERROR,
                                     kf_result_t::NON_FINITE_RESULT);
        }
        return base_t::correct(y, H, R, &active, policy);
    }
};

} // namespace pyro
#endif // PYRO_ALGO_LINEAR_KF_H
