/**
 * @file pyro_algo_kf.tpp
 * @brief 固定维度 KF 模板实现；由 pyro_algo_kf.h 包含。
 *        Fixed-dimension KF template implementation, included by
 * pyro_algo_kf.h.
 */
#ifndef __PYRO_ALGO_KF_TPP__
#define __PYRO_ALGO_KF_TPP__

#include <algorithm>
#include <cmath>

namespace pyro
{

template <std::size_t N, std::size_t Z>
template <std::size_t L>
bool kalman_filter_t<N, Z>::_finite(const std::array<float, L> &values)
{
    for (float value : values)
        if (!kf_is_finite(value))
            return false;
    return true;
}

template <std::size_t N, std::size_t Z>
bool kalman_filter_t<N, Z>::_covariance_valid(const float *values,
                                              std::size_t size)
{
    // 检查有限值、非负对角和近似对称；并非完整特征值/半正定检验。
    // Check finite entries, nonnegative diagonal and symmetry; not a full PSD
    // test.
    for (std::size_t row = 0; row < size; ++row)
    {
        if (!kf_is_finite(values[row * size + row]) ||
            values[row * size + row] < 0.0f)
            return false;
        for (std::size_t col = 0; col < size; ++col)
        {
            const float a = values[row * size + col];
            const float b = values[col * size + row];
            if (!kf_is_finite(a) || !kf_is_finite(b) ||
                std::fabs(a - b) >
                    1.0e-5f *
                        std::max(1.0f, std::max(std::fabs(a), std::fabs(b))))
                return false;
        }
    }
    return true;
}

template <std::size_t N, std::size_t Z>
arm_status kalman_filter_t<N, Z>::_multiply(const float *a, std::size_t rows,
                                            std::size_t inner, const float *b,
                                            std::size_t cols, float *out)
{
    arm_matrix_instance_f32 A_desc, B_desc, out_desc;
    // CMSIS 乘法/转置只读取输入；const_cast 仅适配其描述符接口。
    // CMSIS multiply/transpose only read inputs; const_cast adapts descriptors.
    arm_mat_init_f32(&A_desc, rows, inner, const_cast<float *>(a));
    arm_mat_init_f32(&B_desc, inner, cols, const_cast<float *>(b));
    arm_mat_init_f32(&out_desc, rows, cols, out);
    return arm_mat_mult_f32(&A_desc, &B_desc, &out_desc);
}

template <std::size_t N, std::size_t Z>
arm_status kalman_filter_t<N, Z>::_transpose(const float *a, std::size_t rows,
                                             std::size_t cols, float *out)
{
    arm_matrix_instance_f32 A_desc, out_desc;
    arm_mat_init_f32(&A_desc, rows, cols, const_cast<float *>(a));
    arm_mat_init_f32(&out_desc, cols, rows, out);
    return arm_mat_trans_f32(&A_desc, &out_desc);
}

template <std::size_t N, std::size_t Z>
arm_status kalman_filter_t<N, Z>::_add(const float *a, const float *b,
                                       std::size_t rows, std::size_t cols,
                                       float *out, bool subtract)
{
    arm_matrix_instance_f32 A_desc, B_desc, out_desc;
    arm_mat_init_f32(&A_desc, rows, cols, const_cast<float *>(a));
    arm_mat_init_f32(&B_desc, rows, cols, const_cast<float *>(b));
    arm_mat_init_f32(&out_desc, rows, cols, out);
    return subtract ? arm_mat_sub_f32(&A_desc, &B_desc, &out_desc)
                    : arm_mat_add_f32(&A_desc, &B_desc, &out_desc);
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::_report(status_t status, kf_result_t result,
                                        arm_status math_status)
{
    _diagnostics.status      = status;
    _diagnostics.result      = result;
    _diagnostics.math_status = math_status;
    return status;
}

template <std::size_t N, std::size_t Z>
bool kalman_filter_t<N, Z>::_check(arm_status status)
{
    if (status == ARM_MATH_SUCCESS)
        return true;
    _prepared = false;
    _report(PYRO_ERROR, kf_result_t::MATH_FAILURE, status);
    return false;
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::init(const state_t &x0, const covariance_t &P0,
                                     const options_t &options)
{
    if (_initialized)
        return PYRO_ALREADY_INIT;
    if (!_finite(x0) || !_covariance_valid(P0.data(), N) ||
        !_finite(options.min_variance) ||
        (options.covariance_update != kf_covariance_update_t::JOSEPH &&
         options.covariance_update != kf_covariance_update_t::SIMPLIFIED) ||
        (options.solver != kf_innovation_solver_t::CHOLESKY &&
         options.solver != kf_innovation_solver_t::INVERSE))
        return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    for (float value : options.min_variance)
        if (value < 0.0f)
            return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    _options     = options;
    _initialized = true;
    return reset(x0, P0);
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::reset(const state_t &x0, const covariance_t &P0)
{
    _prepared = false;
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, kf_result_t::NOT_INITIALIZED);
    if (!_finite(x0) || !_covariance_valid(P0.data(), N))
        return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    x_ = x0;
    P_ = P0;
    _finish_covariance(P_);
    K_.fill(0.0f);
    y_.fill(0.0f);
    S_.fill(0.0f);
    _active_count = 0;
    _diagnostics  = {};
    return PYRO_OK;
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::predict(const state_t &x_pred,
                                        const transition_t &F,
                                        const covariance_t &Q_d,
                                        const covariance_t *P_prior)
{
    _prepared    = false;
    _diagnostics = {};
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, kf_result_t::NOT_INITIALIZED);
    // 旧简化公式的 P 可能累积非对称误差，兼容模式不强制变换其对称性。
    // Legacy simplified P may accumulate asymmetry; preserve compatibility
    // mode.
    if (!_finite(x_pred) || !_finite(F) || !_covariance_valid(Q_d.data(), N) ||
        (P_prior && !_finite(*P_prior)))
        return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    const auto &P = P_prior ? *P_prior : P_;
    for (std::size_t row = 0; row < N; ++row)
        if (P[row * N + row] < 0.0f)
            return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    // P_pred = F·P·Fᵀ + Qd；x_pred 已由调用方的状态函数计算。
    // P_pred = F·P·Fᵀ + Qd; the caller's model already computed
    // x_pred.
    if (!_check(_transpose(F.data(), N, N, nn2_.data())) ||
        !_check(_multiply(F.data(), N, N, P.data(), N, nn1_.data())) ||
        !_check(_multiply(nn1_.data(), N, N, nn2_.data(), N, P_next_.data())) ||
        !_check(_add(P_next_.data(), Q_d.data(), N, N, P_next_.data())))
        return PYRO_ERROR;
    if (!_finite(P_next_))
        return _report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
    // 候选值检查完成后一起提交 x、P；此前失败均保留原结果。
    // Commit x/P together after validation; earlier failures retain prior
    // values.
    _finish_covariance(P_next_);
    x_next_ = x_pred;
    x_      = x_next_;
    P_      = P_next_;
    K_.fill(0.0f);
    return _report(PYRO_OK, kf_result_t::PREDICTED);
}

template <std::size_t N, std::size_t Z>
template <typename Policy>
status_t
kalman_filter_t<N, Z>::correct(const measurement_t &y, const observation_t &H,
                               const measurement_noise_t &R,
                               const measurement_mask_t *mask, Policy policy)
{
    const status_t prepared = _prepare(y, H, R, mask);
    if (prepared != PYRO_OK)
        return prepared;
    dx_.fill(0.0f);
    correction_t correction{y_, S_, _diagnostics, K_, dx_};
    const status_t decision = policy(correction);
    if (decision == PYRO_OK)
        return _apply(correction.K,
                      correction.use_dx ? &correction.dx : nullptr);
    _prepared = false;
    return decision == PYRO_WARNING
               ? _report(PYRO_WARNING, kf_result_t::REJECTED)
               : _report(decision, kf_result_t::POLICY_FAILURE);
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::_prepare(const measurement_t &y,
                                         const observation_t &H,
                                         const measurement_noise_t &R,
                                         const measurement_mask_t *mask)
{
    _prepared     = false;
    _diagnostics  = {};
    _active_count = 0;
    K_.fill(0.0f);
    y_.fill(0.0f);
    S_.fill(0.0f);
    if (!_initialized)
        return _report(PYRO_NOT_FOUND, kf_result_t::NOT_INITIALIZED);
    for (std::size_t row = 0; row < Z; ++row)
    {
        if (mask && !(*mask)[row])
            continue;
        if (!kf_is_finite(y[row]))
            return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
        for (std::size_t col = 0; col < N; ++col)
            if (!kf_is_finite(H[row * N + col]))
                return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
        _active[_active_count++] = row;
    }
    const std::size_t count        = _active_count;
    _diagnostics.measurement_count = count;
    if (count == 0)
        return _report(PYRO_WARNING, kf_result_t::NO_MEASUREMENT);
    // m=count；压紧有效 H 行及 R 子矩阵，保留有效观测之间的相关噪声。
    // m=count; pack active H rows and the R submatrix, preserving correlations.
    // 公共新息保持原 Z 维索引，内部新息仅含 m 个分量。
    // Public innovation uses original Z indices; packed innovation has m
    // entries.
    for (std::size_t row = 0; row < count; ++row)
    {
        const std::size_t index = _active[row];
        y_[index]               = y[index];
        y_active_[row]          = y_[index];
        if (!kf_is_finite(y_active_[row]))
            return _report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
        std::copy_n(H.data() + index * N, N, H_active_.data() + row * N);
        for (std::size_t col = 0; col < count; ++col)
            R_active_[row * count + col] = R[index * Z + _active[col]];
    }
    if (!_covariance_valid(R_active_.data(), count))
        return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    // S = H·P·Hᵀ + R（m×m）；公共 S 按原观测索引回填。
    // S = H·P·Hᵀ + R (m×m); scatter public S back to original indices.
    if (!_check(_transpose(H_active_.data(), count, N, Ht_active_.data())) ||
        !_check(
            _multiply(H_active_.data(), count, N, P_.data(), N, zn_.data())) ||
        !_check(_multiply(zn_.data(), count, N, Ht_active_.data(), count,
                          zz_.data())) ||
        !_check(
            _add(zz_.data(), R_active_.data(), count, count, S_active_.data())))
        return PYRO_ERROR;
    for (std::size_t row = 0; row < count; ++row)
        for (std::size_t col = 0; col < count; ++col)
        {
            const float value = S_active_[row * count + col];
            if (!kf_is_finite(value))
                return _report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
            S_[_active[row] * Z + _active[col]] = value;
        }
    if (_solve_innovation() != PYRO_OK)
        return _diagnostics.status;
    for (std::size_t row = 0; row < N; ++row)
        for (std::size_t col = 0; col < count; ++col)
            K_[row * Z + _active[col]] = K_active_[row * count + col];
    if (!_finite(K_) || !kf_is_finite(_diagnostics.nis))
        return _report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
    _prepared = true;
    return _report(PYRO_OK, kf_result_t::PREPARED);
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::_solve_innovation()
{
    const std::size_t count = _active_count;
    arm_matrix_instance_f32 S_desc, L_desc, B_desc, out_desc;
    if (_options.solver == kf_innovation_solver_t::INVERSE)
    {
        // 显式求逆兼容路径：K = P·Hᵀ·S⁻¹，NIS = yᵀ·S⁻¹·y。
        // Explicit-inverse compatibility path: K = P·Hᵀ·S⁻¹, NIS = yᵀ·S⁻¹·y.
        // CMSIS 求逆会修改源矩阵，因此使用副本。
        // CMSIS inversion modifies its source, so use a copy.
        L_ = S_active_;
        arm_mat_init_f32(&S_desc, count, count, L_.data());
        arm_mat_init_f32(&out_desc, count, count, S_inv_.data());
        if (!_check(arm_mat_inverse_f32(&S_desc, &out_desc)) ||
            !_check(_multiply(S_inv_.data(), count, count, y_active_.data(), 1,
                              y_solved_.data())) ||
            !_check(_multiply(y_active_.data(), 1, count, y_solved_.data(), 1,
                              &_diagnostics.nis)) ||
            !_check(_multiply(P_.data(), N, N, Ht_active_.data(), count,
                              PHt_.data())) ||
            !_check(_multiply(PHt_.data(), N, count, S_inv_.data(), count,
                              K_active_.data())))
            return PYRO_ERROR;
    }
    else
    {
        // S=L·Lᵀ；先求 L·v=y，再取 NIS=vᵀ·v，无需显式求逆。
        // S=L·Lᵀ; solve L·v=y, then NIS=vᵀ·v, without explicit inversion.
        // CMSIS Cholesky 仅写下三角，必须清空复用缓冲区的上三角。
        // Clear the upper triangle: CMSIS Cholesky writes only lower entries.
        L_.fill(0.0f);
        arm_mat_init_f32(&S_desc, count, count, S_active_.data());
        arm_mat_init_f32(&L_desc, count, count, L_.data());
        if (!_check(arm_mat_cholesky_f32(&S_desc, &L_desc)))
            return PYRO_ERROR;
        arm_mat_init_f32(&B_desc, count, 1, y_active_.data());
        arm_mat_init_f32(&out_desc, count, 1, y_solved_.data());
        if (!_check(arm_mat_solve_lower_triangular_f32(&L_desc, &B_desc,
                                                       &out_desc)) ||
            !_check(_multiply(y_solved_.data(), 1, count, y_solved_.data(), 1,
                              &_diagnostics.nis)) ||
            !_check(_multiply(P_.data(), N, N, Ht_active_.data(), count,
                              PHt_.data())) ||
            !_check(_transpose(PHt_.data(), N, count, zn_.data())))
            return PYRO_ERROR;
        // 依次解 L·A=(P·Hᵀ)ᵀ、Lᵀ·Kᵀ=A，得到 N×m 增益。
        // Solve L·A=(P·Hᵀ)ᵀ, then Lᵀ·Kᵀ=A, yielding the N×m gain.
        arm_mat_init_f32(&B_desc, count, N, zn_.data());
        arm_mat_init_f32(&out_desc, count, N, nz_.data());
        if (!_check(arm_mat_solve_lower_triangular_f32(&L_desc, &B_desc,
                                                       &out_desc)) ||
            !_check(_transpose(L_.data(), count, count, zz_.data())))
            return PYRO_ERROR;
        arm_mat_init_f32(&L_desc, count, count, zz_.data());
        arm_mat_init_f32(&B_desc, count, N, nz_.data());
        arm_mat_init_f32(&out_desc, count, N, zn_.data());
        if (!_check(arm_mat_solve_upper_triangular_f32(&L_desc, &B_desc,
                                                       &out_desc)) ||
            !_check(_transpose(zn_.data(), count, N, K_active_.data())))
            return PYRO_ERROR;
    }
    return PYRO_OK;
}

template <std::size_t N, std::size_t Z>
void kalman_filter_t<N, Z>::_finish_covariance(covariance_t &P) const
{
    // 可选 P=(P+Pᵀ)/2，再按状态施加方差下限；不等同于半正定投影。
    // Optionally average P/Pᵀ, then floor variances; this is not a PSD
    // projection.
    if (_options.symmetrize)
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t col = row + 1; col < N; ++col)
            {
                const float value =
                    0.5f * P[row * N + col] + 0.5f * P[col * N + row];
                P[row * N + col] = P[col * N + row] = value;
            }
    for (std::size_t row = 0; row < N; ++row)
        P[row * N + row] =
            std::max(P[row * N + row], _options.min_variance[row]);
}

template <std::size_t N, std::size_t Z>
status_t kalman_filter_t<N, Z>::_apply(const gain_t &K,
                                       const state_t *dx_override)
{
    if (!_prepared)
        return _report(PYRO_ERROR, kf_result_t::NOT_PREPARED);
    // 一次准备结果只能应用或拒绝一次。 / Consume each preparation only once.
    _prepared = false;
    if (!_finite(K) || (dx_override && !_finite(*dx_override)))
        return _report(PYRO_PARAM_ERROR, kf_result_t::INVALID_INPUT);
    const std::size_t count = _active_count;
    for (std::size_t row = 0; row < N; ++row)
        for (std::size_t col = 0; col < count; ++col)
            K_active_[row * count + col] = K[row * Z + _active[col]];
    // 默认增量为 K·y；外部可覆盖增量，但协方差仍由传入 K 更新。
    // Default delta is K·y; an override changes delta, while P still uses K.
    if (dx_override)
        dx_ = *dx_override;
    else if (!_check(_multiply(K_active_.data(), N, count, y_active_.data(), 1,
                               dx_.data())))
        return PYRO_ERROR;
    if (!_check(_add(x_.data(), dx_.data(), N, 1, x_next_.data())) ||
        !_check(_multiply(K_active_.data(), N, count, H_active_.data(), N,
                          nn1_.data())))
        return PYRO_ERROR;
    if (_options.covariance_update == kf_covariance_update_t::SIMPLIFIED)
    {
        // 简化公式 P_new=P-K·H·P。 / Simplified update: P_new=P-K·H·P.
        if (!_check(_multiply(nn1_.data(), N, N, P_.data(), N, nn2_.data())) ||
            !_check(_add(P_.data(), nn2_.data(), N, N, P_next_.data(), true)))
            return PYRO_ERROR;
    }
    else
    {
        // Joseph 公式：A=I-K·H；P_new=A·P·Aᵀ+K·R·Kᵀ。
        // Joseph update: A=I-K·H; P_new=A·P·Aᵀ+K·R·Kᵀ.
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t col = 0; col < N; ++col)
                nn1_[row * N + col] =
                    (row == col ? 1.0f : 0.0f) - nn1_[row * N + col];
        if (!_check(_multiply(nn1_.data(), N, N, P_.data(), N, nn2_.data())) ||
            !_check(_transpose(nn1_.data(), N, N, nn3_.data())) ||
            !_check(
                _multiply(nn2_.data(), N, N, nn3_.data(), N, P_next_.data())) ||
            !_check(_multiply(K_active_.data(), N, count, R_active_.data(),
                              count, nz_.data())) ||
            !_check(_transpose(K_active_.data(), N, count, zn_.data())) ||
            !_check(
                _multiply(nz_.data(), N, count, zn_.data(), N, nn1_.data())) ||
            !_check(_add(P_next_.data(), nn1_.data(), N, N, P_next_.data())))
            return PYRO_ERROR;
    }
    // 施加下限前检查有限性，避免 min/max 运算掩盖 NaN。
    // Inspect before flooring: min/max operations must not hide NaNs.
    if (!_finite(x_next_) || !_finite(P_next_))
        return _report(PYRO_ERROR, kf_result_t::NON_FINITE_RESULT);
    _finish_covariance(P_next_);
    x_ = x_next_;
    P_ = P_next_;
    K_ = K;
    return _report(PYRO_OK, kf_result_t::CORRECTED);
}

} // namespace pyro
#endif // __PYRO_ALGO_KF_TPP__
