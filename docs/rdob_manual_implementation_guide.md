# RDOB 稳定分支手写落地指南

本文用于在不依赖测试分支现成实现的情况下，将六通道输入等效 RDOB 手写接入稳定版底盘。目标是先完成只观测、只记录的数据链路，不接入扰动补偿；矩阵与向量乘法在 `_rdob_update()` 内用明确的两层循环完成，生成文件只提供离线常量表，腿长调度函数由你自己编写。

本文对应以下约束：

- 控制周期为 `1 ms`。
- 观测输入依次为 `T_w1, T_w2, T_p1, T_p2, F_L1, F_L2`。
- RDOB 只维护六维辅助状态 `z`，不重新维护十四维物理状态估计。
- 六路离散扰动极点均为 `0.992`，对应 RDOB 梯形离散带宽约 `1.278353 Hz`。
- 扰动限幅为执行器上限的 `0.1` 倍：轮矩 `±0.326664 N·m`、腿摆矩 `±6 N·m`、腿力 `±30 N`。
- 第一阶段严禁把估计值接入 `_balance_control()`。

## 1. 先固定数据契约

RDOB 使用七个广义位置和七个广义速度，不使用十四维交错状态数组直接做矩阵乘法。

| RDOB 下标 | 位置 `q` | 速度 `v` | `state_vec_t` 对应成员 |
|---:|---|---|---|
| 0 | `x` | `dot_x` | `x`, `dot_x` |
| 1 | `psi` | `dot_psi` | `psi`, `dot_psi` |
| 2 | `theta` | `dot_theta` | `theta`, `dot_theta` |
| 3 | `phi` | `dot_phi` | `phi`, `dot_phi` |
| 4 | `L` | `dot_L` | `L`, `dot_L` |
| 5 | `beta1` | `dot_beta1` | `beta1`, `dot_beta1` |
| 6 | `beta2` | `dot_beta2` | `beta2`, `dot_beta2` |

六个输入的顺序必须在上位机、生成头文件、底盘结构体、遥测协议中完全一致：

| RDOB 下标 | 输入 | 单位 |
|---:|---|---|
| 0 | `T_w1` | N·m |
| 1 | `T_w2` | N·m |
| 2 | `T_p1` | N·m |
| 3 | `T_p2` | N·m |
| 4 | `F_L1` | N |
| 5 | `F_L2` | N |

建议在固件侧增加编译期检查：

```cpp
static_assert(STATE_DIM == 14U, "RDOB expects the 14-state interleaved layout");
static_assert(GENERAL_STATE_DIM == 7U, "RDOB expects seven q/v coordinates");
static_assert(INPUT_DIM == 6U, "RDOB expects six generalized inputs");
```

任何一个顺序发生变化，都必须重新生成系数并同步修改映射，不能只调整循环下标。

## 2. RDOB 实际执行的公式

线性化模型写成

\[
M\ddot q+D\dot q+Kq=S(u+d).
\]

RDOB 的辅助状态和扰动估计为

\[
\hat d=z+\Gamma\,\delta v,
\]

其中 `delta` 表示相对于本区间配平点的量。连续形式为

\[
\dot z=-\Lambda z+B_q\delta q+B_v\delta v-\Lambda\delta u.
\]

当前离线模型使用 `D=0`，因此生成器得到

\[
\Gamma=\Lambda S^\dagger M,\qquad
B_q=\Lambda S^\dagger K,\qquad
B_v=-\Lambda\Gamma.
\]

固件不需要运行连续形式，也不需要保存 `dot_z`。生成器已经按梯形法得到离散系数。对于区间 `[k,k+1)`，必须使用区间起点冻结的系数和配平：

\[
z_{k+1,i}=p_i z_{k,i}
+\sum_j G^q_{ij}(\delta q_{k,j}+\delta q_{k+1|k,j})
+\sum_j G^v_{ij}(\delta v_{k,j}+\delta v_{k+1|k,j})
+G^u_i\delta u_{k,i}.
\]

这里的 `delta q[k+1|k]` 和 `delta v[k+1|k]` 仍然减去 `k` 时刻的旧配平，而不是新配平。完成旧区间后再重构：

\[
\hat d_{k+1}=z_{k+1}+\Gamma_k\delta v_{k+1|k}.
\]

这条“先用旧坐标完成区间、后切换到新坐标”的规则是整个实现最重要的时序约束。

## 3. 生成并筛选固件系数

在上位机仓库 `PYRo-wl-python` 中运行：

```powershell
python run_rdob.py --mode schedule --output-dir data/generated/rdob
python -m pytest tests/test_rdob.py tests/test_rdob_schedule.py
```

固件参数来源应使用 `data/generated/rdob/rdob_schedule.h`，不要使用只适用于单个配平点的 `rdob_coefficients.h`。当前生成头文件尾部仍带有 `Coefficients` 和 `evaluate()`；本方案不复制这部分，只提取下列常量内容到：

```text
Robot/infantry1/Chassis/rdob_coef.h
```

需要保留的内容只有腿长上下界、`kRates/kStepPoles/kStepEndpointWeight/kStepInputGain`、`kGammaCoefficients` 和 `kPositionGainCoefficients`。多项式求值与写入 ctx 的代码由 `_rdob_gain_calculate()` 自己实现。

生成后至少检查 `rdob_schedule_report.md` 中以下项目：

- `gamma` 与 `position_gain` 的 float32 最大相对误差没有超过生成预算。
- `max_matched_rate_relative_error <= 0.01`。
- 原始和拟合后的冻结闭环谱半径均小于 `1`。
- 参数范围与固件 `MIN_LEG_LENGTH/MAX_LEG_LENGTH` 一致，当前应为 `[0.18, 0.38] m`。
- 六个 `kStepPoles` 均约为 `0.992`。

离散极点和带宽的换算关系为

\[
\lambda=\frac{2(1-p)}{T_s(1+p)},\qquad f=\frac{\lambda}{2\pi}.
\]

当 `p=0.992`、`Ts=0.001 s` 时，`lambda≈8.032128 rad/s`，`f≈1.278353 Hz`。不要把 `1.278353 Hz` 直接当成 `rad/s` 写入生成器。

### 3.1 不引入系数结构体和 `evaluate()`

生成头文件只保留离线拟合得到的常量表、腿长范围以及六路离散常量。不要让生成器再输出 `Coefficients` 结构体，也不要在固件中调用通用 `evaluate()`。

由你自己实现一个名称明确的系数计算成员函数，例如：

```cpp
bool _rdob_gain_calculate(float left_length, float right_length);
```

该函数只负责检查腿长并把当前调度结果直接写入 `wl_chassis_data_ctx_t`：

- `rdob_gamma[6][7]`；
- `rdob_step_q_gain[6][5]`。

不让它执行 `z` 更新、扰动重构、限幅或重基准化。这样系数计算和观测器状态更新仍是两个清晰阶段，但没有中间返回对象。

只保存这两块调度矩阵的依据是：

- `Gamma` 是完整的 `6×7` 矩阵，重构和重基准化都要使用。
- `Bq` 对应 `x/psi` 的前两列是经过模型与全部训练/验证点确认的固定零，只保存 `theta, phi, L, beta1, beta2` 五列。
- 当前 `D=0`，所以离散速度增益满足 `step_v_gain[i][j] = velocity_scale[i] * Gamma[i][j]`，无需再保存一块 `6×7` 矩阵。
- `step_z_pole[6]`、`step_u_gain[6]`、`velocity_scale[6]` 都只由固定带宽和采样周期决定，作为 `rdob_coef.h` 中的 `constexpr` 常量，不进入 `ctx`。

推荐常量定义：

```cpp
constexpr float RDOB_STEP_VELOCITY_SCALE[INPUT_DIM] = {
    // -kStepEndpointWeight[i] * kRates[i]
};
```

`_rdob_gain_calculate()` 内部的三次多项式或 Chebyshev 计算由你手写。它必须先完成 NaN、腿长范围检查，再覆盖 `ctx` 中的旧矩阵，避免失败时留下只更新了一部分的矩阵。

## 4. 复用现有 ctx，只增加跨区间必需量

稳定分支当前已有成员可直接承担以下职责：

| 已有成员 | RDOB 中的用途 |
|---|---|
| `measured_state` | 当前 `q_now/v_now`，不再新增当前测量数组到持久 ctx |
| `target_state` | 当前 `dot_x/dot_psi/dot_L` 移动配平及 beta 目标 |
| `U0` | 当前主动输入配平 |
| `output` | 当前稳定分支未被使用；直接重命名或定义为上一采样区间的 `applied_input_delta` |
| `control` | 当前控制命令；不能同时拿来缓存上一拍输入 |
| `leg[]/wheel[]` | 当前腿长、气弹簧力及执行器最终输出 |

在 `wl_chassis_data_ctx_t` 中直接增加下列成员即可，不再套一层 RDOB 或 coef 结构体：

```cpp
control_vec_t disturbance; // 唯一对外发布的 RDOB 原始估计

float rdob_gamma[INPUT_DIM][GENERAL_STATE_DIM];
// 只存 theta, phi, L, beta1, beta2 五列。
float rdob_step_q_gain[INPUT_DIM][GENERAL_STATE_DIM - 2U];

float rdob_z[INPUT_DIM];
float rdob_interval_q_begin[GENERAL_STATE_DIM];
float rdob_interval_v_begin[GENERAL_STATE_DIM];
float rdob_interval_trim_q[GENERAL_STATE_DIM];
float rdob_interval_trim_v[GENERAL_STATE_DIM];
bool rdob_initialized;
```

同时把已有的

```cpp
control_vec_t output;
```

改名为语义更明确的：

```cpp
control_vec_t applied_input_delta;
```

如果暂时不想改名，也可以继续使用 `output`，但必须注释清楚其中保存的是“上一采样区间实际总输入减该区间总配平输入”，而不是原始控制命令。

实际新增量共 112 个 float，即约 448 B，再加一个 `bool` 和可能的对齐填充。其中真正的调度矩阵只有 72 个 float；没有 138-float 或 222-float 系数对象。

这些新增量都不能安全地由当前值替代：

- `rdob_gamma/rdob_step_q_gain` 必须保持为刚刚过去区间的冻结系数，直到该区间更新完成。
- `rdob_interval_q_begin/rdob_interval_v_begin` 是区间起点误差，当前测量不能替代。
- `rdob_interval_trim_q/rdob_interval_trim_v` 用于让区间终点仍减去旧配平，当前 `target_state` 不能替代。
- 复用后的 `applied_input_delta` 是上一周期限幅后的实际输入减当时配平，当前 `control` 不能替代。

不要新增或复用以下内容：

- 不新增 `Coefficients`、`coef`、`next_coef` 或 `evaluate()`。
- 不保存 `dot_z`、连续 `Bq/Bv` 或完整 `step_v_gain`。
- 不同时保存上一拍原始 `q/v` 和上一拍 `delta_q/delta_v`。
- 不增加完整十四维 `equilibrium_state`。
- 不复用 `K` 存放 RDOB 矩阵；本周期 `_balance_control()` 仍需要 `K`。
- 不复用 `control` 缓存实际输入；它会在下一周期被覆盖。
- 不同时保留 `dist`、`rdob_dist`、`estimate` 多个同义输出；只使用 `disturbance`。

## 5. 每周期构造测量、配平和总输入

当前测量 `q_now/v_now` 从交错的 `measured_state` 提取：

```cpp
for (uint8_t j = 0; j < GENERAL_STATE_DIM; ++j)
{
    q_now[j] = measured_state.data[2U * j];
    v_now[j] = measured_state.data[2U * j + 1U];
}
```

当前静态位置配平建议显式构造，不要直接把整个 `target_state` 当作模型配平：

```text
trim_q = [0, 0, 0, 0, (L1+L2)/2,
          target_state.beta1, target_state.beta2]
```

这里直接复用 `_gain_calculate()` 已经写好的两个 beta 配平值，不再重复求一次 `BETA_TRIM_POLY_COEF`。`L` 配平仍取当前两腿平均长度，不能使用可能带有运动指令的 `target_state.L`。

移动速度配平为：

```text
trim_v = [target_state.dot_x, target_state.dot_psi, 0, 0,
          target_state.dot_L, 0, 0]
```

这样匀速前进、匀速旋转和匀速变腿长不会被 RDOB 当成输入扰动。`psi` 的位置误差必须回绕到 `[-pi, pi]`。

RDOB 模型中的腿力是总广义腿力，而 LQR 的 `U0` 和电机输出是主动部分。因此总输入必须统一为：

```text
trim_u[F_L]    = U0[F_L] + gas_spring_force
applied_u[F_L] = actual_out_F_L + gas_spring_force
```

轮矩和腿摆矩不需要加气弹簧项。气弹簧必须在实际输入和配平输入两边同时加入；只加一边会制造固定偏置。

`actual_out_F_L/actual_out_T_p` 必须由关节电机最终限幅后的转矩反算，不能使用限幅前的 `_ctx.data.control`。

不为实际输入再新增一个六维容器。VMC 完成后直接计算：

```cpp
_ctx.data.applied_input_delta.data[i] =
    actual_total_input[i] - trim_total_input[i];
```

`actual_total_input` 可以逐通道使用局部标量计算，不需要形成持久数组。下一拍 `_rdob_update()` 只读取已经冻结好的 `applied_input_delta`；因此无需再保存 `applied_u`、`trim_u` 或第二个输入差值数组。

## 6. 初始化：令首次估计严格为零

进入平衡状态、系数越界、发生漏周期或主动复位时：

```text
rdob_initialized = false
rdob_z = 0
disturbance = 0
```

第一次收到合法测量时先求当前系数，然后按下式初始化：

\[
z_i=-\sum_j\Gamma_{ij}(v_j-v_{0,j}).
\]

于是第一次发布的

\[
\hat d=z+\Gamma(v-v_0)=0.
\]

随后缓存当前 `delta q/delta v` 和当前 `trim_q/trim_v`，将 `rdob_initialized` 置为 `true`。本周期控制及 VMC 完成后，再生成第一个完整区间的 `applied_input_delta`。

## 7. 用循环完成一次 RDOB 更新

不要调用隐藏整个状态更新的 `step_rdob()` 大函数。`_rdob_update()` 内部按照下面顺序直接写循环，便于核对每个量的时间戳。

### 7.1 在旧配平下构造区间终点误差

```cpp
for (uint8_t j = 0; j < GENERAL_STATE_DIM; ++j)
{
    q_end_old[j] = q_now[j] - _ctx.data.rdob_interval_trim_q[j];
    v_end_old[j] = v_now[j] - _ctx.data.rdob_interval_trim_v[j];
}
q_end_old[1] = loop_fp32_constrain(q_end_old[1], -PI, PI);
```

`rdob_interval_q_begin[1]` 在写入缓存时也必须已经回绕。

### 7.2 两层循环更新 `z`

```cpp
float z_next[INPUT_DIM] = {};

for (uint8_t i = 0; i < INPUT_DIM; ++i)
{
    float q_term = 0.0f;
    float gamma_v_sum = 0.0f;

    for (uint8_t j = 0; j < GENERAL_STATE_DIM; ++j)
    {
        gamma_v_sum += _ctx.data.rdob_gamma[i][j] *
            (_ctx.data.rdob_interval_v_begin[j] + v_end_old[j]);
    }

    for (uint8_t j = 2U; j < GENERAL_STATE_DIM; ++j)
    {
        q_term += _ctx.data.rdob_step_q_gain[i][j - 2U] *
            (_ctx.data.rdob_interval_q_begin[j] + q_end_old[j]);
    }

    const float v_term = RDOB_STEP_VELOCITY_SCALE[i] * gamma_v_sum;
    z_next[i] = RDOB_STEP_POLE[i] * _ctx.data.rdob_z[i] +
                q_term + v_term + RDOB_STEP_INPUT_GAIN[i] *
                _ctx.data.applied_input_delta.data[i];
}
```

第一层循环利用 `Bv=-Lambda*Gamma` 的固定结构，因此不保存 `step_v_gain`。第二层循环只遍历 `Bq` 的五个非零列。循环中不要使用当前周期刚算出的控制量；这里需要的是已经结束区间内实际保持的输入差值。

### 7.3 用旧 `Gamma` 重构并限幅物理估计

```cpp
for (uint8_t i = 0; i < INPUT_DIM; ++i)
{
    float estimate = z_next[i];
    for (uint8_t j = 0; j < GENERAL_STATE_DIM; ++j)
    {
        estimate += _ctx.data.rdob_gamma[i][j] * v_end_old[j];
    }
    _ctx.data.disturbance.data[i] = std::clamp(
        estimate, -RDOB_DISTURBANCE_LIMIT[i],
        RDOB_DISTURBANCE_LIMIT[i]);
}
```

限幅对象是物理扰动估计 `d_hat`，不是 `z`。

### 7.4 原地求新系数并同步重基准化 `z`

旧区间已经完成后，才允许调用：

```cpp
_rdob_gain_calculate(L1, L2);
```

该函数直接原地覆盖 `_ctx.data.rdob_gamma` 和 `_ctx.data.rdob_step_q_gain`，不返回或构造 coef 对象。新坐标下的辅助状态为

\[
z_{k+1}^{new}=\hat d_{k+1}^{limited}
-\Gamma_{k+1}(v_{k+1}-v_{0,k+1}).
\]

循环写法为：

```cpp
for (uint8_t i = 0; i < INPUT_DIM; ++i)
{
    float gamma_v_new = 0.0f;
    for (uint8_t j = 0; j < GENERAL_STATE_DIM; ++j)
    {
        gamma_v_new += _ctx.data.rdob_gamma[i][j] *
                       (v_now[j] - trim_v_now[j]);
    }
    _ctx.data.rdob_z[i] =
        _ctx.data.disturbance.data[i] - gamma_v_new;
}
```

最后将 `q_now-trim_q_now`、`v_now-trim_v_now`、当前 `trim_q/trim_v` 写入下一周期的 `rdob_interval_*` 缓存。这个重基准化同时处理 `Gamma` 变化和移动速度配平变化，不能只在腿长变化时执行。

如果 `_rdob_gain_calculate()` 因 NaN 或腿长越界返回 `false`，立即清零 `disturbance/rdob_z` 并设置 `rdob_initialized=false`，等待重新初始化，不要沿用边界外系数。

## 8. 实际输入必须在控制链末端捕获

正常平衡周期的顺序应固定为：

```text
更新反馈和目标
    ↓
_gain_calculate()          生成本周期 K、U0、beta trim
    ↓
_rdob_update()             用缓存输入闭合上一采样区间
    ↓
_balance_control()         当前阶段不读取 disturbance
    ↓
附加控制项/限幅
    ↓
_vmc_trans_v2j()           关节转矩限幅并反算 actual F_L/T_p
    ↓
_rdob_capture_applied_input()
    ↓
发送电机命令
```

`_rdob_capture_applied_input()` 只做六个标量赋值，不做观测器更新：

- `T_w`：使用最终轮端命令。
- `T_p`：使用关节限幅后反算的 `actual_out_T_p`。
- `F_L`：使用 `actual_out_F_L + gas_spring_force`。

逐通道用以上实际总输入减去本周期总配平，直接写入由原 `output` 复用而来的 `_ctx.data.applied_input_delta`。因此下一拍不需要额外保存 `applied_u`、`trim_u` 或 `u_delta` 数组。

若以后接入 RDOB 补偿，补偿也必须先经过全部执行器限幅，再由此函数捕获，否则观测器使用的输入与真实施加输入不一致。

## 9. 带宽、限幅和补偿开关

在没有 LESO 的稳定分支中，RDOB 参数应独立命名，不再保留 `OBSERVER_*` 或 `LESO_*` 名称：

```cpp
#define RDOB_EN 1
#define RDOB_COMPENSATION_EN 0

constexpr float RDOB_DISTURBANCE_LIMIT[INPUT_DIM] = {
    0.1f * MAX_T_W, 0.1f * MAX_T_W,
    0.1f * MAX_T_P, 0.1f * MAX_T_P,
    0.1f * MAX_F_L, 0.1f * MAX_F_L,
};
```

第一阶段建议完全不在 `_balance_control()` 中写补偿分支。这样可以从代码结构上保证 RDOB 不改变控制输出，而不仅依赖六个补偿增益恰好为零。

后续若要接入补偿，单独提交并采用：

```text
u_command[i] -= compensation_gain[i] * disturbance.data[i]
```

补偿增益、补偿滤波和观测器带宽是三个独立概念，不能乘入 `z` 更新公式。

## 10. 栈和实时计算约束

取消通用 coef 结构和 `evaluate()` 后，系数计算开销取决于你编写的 `_rdob_gain_calculate()`，需要重新以实际代码计数和测量。原生成器的 `1018` 次乘法、`904` 次加减、`1` 次除法只能作为旧实现上界，不能直接作为新函数结果。

按本文循环结构，观测器部分的源代码级运算量约为：

- 离散更新和扰动重构：132 次乘法、约 132 次加减。
- 新系数重基准化：42 次乘法、42 次加减。

这里利用了 `Bq` 前两列固定为零以及 `Bv=-Lambda*Gamma`；最终周期数仍以编译器输出和 DWT 实测为准。

实现时遵守以下规则：

- 不创建任何局部或持久 `Coefficients` 对象。
- 调度矩阵直接原地写入 `rdob_gamma/rdob_step_q_gain`。
- 小型 `q/v/trim/z_next` 临时数组可以放栈上，但必须检查实际高水位。
- `_gain_calculate()` 与 `_rdob_update()` 顺序执行，二者的执行时间相加，但局部栈峰值不直接相加。
- 调试阶段建议让真正执行 `_fsm_execute()` 的控制任务使用 512 words；验证后至少保留 384 words，并以 `uxTaskGetStackHighWaterMark()` 的实测结果为准。不要只调整命令转发线程的栈。

STM32H723 为 480 MHz 时，`1 ms` 等于 480000 个周期。用 DWT 分别记录：

- `_rdob_gain_calculate()` 周期数；
- 两层循环更新和重基准化周期数；
- 整个控制周期的最大周期数。

关注最大值而不是平均值。整个控制周期建议至少保留 30% 裕量，即最坏情况不超过约 `700 us`。

生成系数固定采用 `Ts=1 ms`，不能把某一拍测得的 `_dt` 直接乘进离散增益。若检测到漏周期或异常长周期，应复位 RDOB，而不是用一组 1 ms 系数跨越多个采样周期。

## 11. 建议的分阶段验证

### 阶段 A：离线一致性

1. 运行上位机 RDOB 和调度测试。
2. 用固定输入序列同时驱动 Python 参考实现和固件循环实现。
3. 对比每一步的 `rdob_z[6]` 和 `disturbance[6]`，先用固定腿长，再测试变化腿长。
4. 验证左右腿长交换时的通道交换关系。

### 阶段 B：固件只观测

记录以下量即可：

- `disturbance.data[0..5]`：最终需要观察的六路扰动。
- `rdob_z[0..5]`：只用于诊断重基准化问题。
- 当前 `L1/L2`、`trim_v`、`applied_input_delta`。
- `rdob_initialized`、更新周期最大 DWT ticks、任务栈高水位。

如果仍保留一版 LESO 对照固件，则只比较 RDOB 的 `disturbance.data[i]` 与 LESO 的原始 `dist.data[i]`。两者只要求符号、趋势和量级大致一致，不要求冲激响应逐点相同。达到限幅后不能再用幅值相等判断性能。

### 阶段 C：场景检查

按以下顺序增加工况：

1. 架空静止：估计有限、无 NaN、初始化无跳变。
2. 落地静止：观察静态偏置和噪声。
3. 匀速前进和匀速旋转：移动配平不应制造持续大扰动。
4. 缓慢变腿长：切换调度后 `disturbance` 不应跳变。
5. 触及输出限幅：确认捕获的是限幅后输入。
6. 人工推扰和台阶：检查六通道符号、趋势和饱和恢复。

第一阶段整车测试前后，可额外比较开启/关闭 `RDOB_EN` 时的控制输出日志；在补偿关闭的前提下，两者应一致，仅允许观测代码带来的时序差异。

## 12. 必须单独测试的边界条件

- 初始化时 `d_hat` 严格为零。
- 腿长变化但物理估计固定时，重基准化前后 `d_hat` 连续。
- `target_dot_x/target_dot_psi/target_dot_L` 改变时，重基准化保持 `d_hat` 连续。
- 限幅后的 `d_hat` 被写回 `z`，不存在输出限幅后的隐藏积分积累。
- `psi` 穿过 `±pi` 时不产生脉冲。
- 任一腿长越界或非有限时清零并重新初始化。
- 状态退出 BALANCE、进入 AIR/STEP/JUMP 或发生控制周期丢失时复位，不跨模式沿用旧区间。
- 气弹簧在 `trim_u` 与 `applied_u` 两侧使用同一个模型和符号。
- 关闭补偿时，RDOB 估计没有任何路径写入控制量。

## 13. 常见错误清单

- 先计算新腿长系数，再用它更新刚刚结束的旧区间。
- 区间两个端点分别减去新旧两个不同配平。
- 用限幅前的 `control` 代替实际施加输入。
- 使用主动腿力减总配平腿力，或只在一侧加入气弹簧。
- 把十四维交错状态直接当成七维 `q` 或 `v`。
- 忘记 `psi` 回绕。
- 初始化直接令 `z=0`，导致首次 `d_hat=Gamma*v_delta` 跳变。
- 系数变化后不重基准化 `z`。
- 只裁剪发布值，不把裁剪结果同步回 `z`。
- 在栈上同时保存旧、新两个完整系数对象。
- 使用单配平点 `rdob_coefficients.h` 覆盖整个腿长范围。
- 在数据验证阶段提前加入补偿、滤波或额外泄漏。

## 14. 完成标准

满足以下条件后，可认为“只观测 RDOB”已落地：

- 上位机生成与全部 RDOB 测试通过。
- 下位机完整编译通过，源码中没有重复的 RDOB 状态或连续域冗余矩阵。
- 初始化、调度切换、移动配平、限幅、越界复位测试通过。
- 1 ms 控制周期最坏耗时留有不少于 30% 裕量。
- 实际控制任务栈具有明确的高水位记录和安全余量。
- `RDOB_COMPENSATION_EN=0`，估计值只进入遥测。
- 实车日志中六路估计有限、符号合理、趋势可重复，且变腿长时不存在由系数切换造成的假跳变。

完成以上步骤后，再把补偿作为独立阶段设计和验证。不要在首次落地时同时改变观测器、控制律和执行器限幅链路。
