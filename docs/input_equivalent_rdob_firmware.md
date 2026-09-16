# 六阶输入等效 RDOB：下位机接入

本实现位于 `Robot/infantry1/Chassis`，仅在正常平衡状态启用。它估计
`T_w1, T_w2, T_p1, T_p2, F_L1, F_L2` 六路输入等效扰动；`z[6]` 是唯一的
观测器动态状态。

## 调度与误差坐标

`rdob_coef.h` 是上位机生成的浮点系数。它以实际 `L1,L2` 调度，`Gamma`
采用三次全张量直接拟合，`Bq` 采用总阶三次直接拟合，默认 `D=0`，所以
`Bv=-Lambda*Gamma` 已包含在 `step_velocity_gain` 中。六路 RDOB 的离散
扰动极点均配置为 `0.992`，与当前固件 LESO 的 `LESO_DISTURBANCE_POLE`
相同；按 RDOB 梯形离散反算为约 `1.278353 Hz`。

位置、速度顺序固定为：

```
x, psi, theta, phi, L_bar, beta1, beta2
```

位置误差相对当前 LQR 配平 `q0` 计算；速度误差相对移动配平速度计算。正常
行进、旋转和变腿长时，`dot_x`、`dot_psi`、`dot_L` 分别使用当前目标速度，
因此名义运动不会被当作扰动。`psi` 的差值按 `[-pi, pi]` 回绕。

## 区间更新与同步重基准化

`_rdob_update()` 在本周期控制计算之前，使用上一个区间缓存的系数、配平、
首末测量以及实际保持输入完成梯形离散更新。`_rdob_capture_applied_input()`
在 VMC 和关节/轮端限幅之后记录本周期实际输入，供下一次测量到来时使用。
腿力通道在实际主动输出和 LQR 主动配平两边都加入相同气弹簧力，因此 RDOB
处理的是一致的总广义输入。

区间 `[k,k+1)` 始终冻结使用 `k` 时刻的 `Gamma`、离散增益以及
`q0/v0/u0`，不会把新旧配平混进一次更新。得到端点估计后，先施加与 LESO
相同的物理限幅，再切换到 `k+1` 时刻的系数和移动配平：

```
d_hat_limited = clamp(z_next + Gamma_old * (v_next - v0_old))
z_rebased = d_hat_limited - Gamma_new * (v_next - v0_new)
```

因此目标 `dot_x`、`dot_psi`、`dot_L` 或腿长调度变化时，`d_hat` 不会因
辅助坐标改变而产生假跳变。限幅也进入内部 `z`，不会形成只裁输出导致的
隐藏积累。

## 启动、越界与补偿选择

进入平衡状态时，观测器以 `d_hat=0` 初始化，即
`z=-Gamma*delta_v`。腿长超出生成系数的有效范围时 RDOB 清零并等待重新
初始化，不沿用越界系数。

默认 `LESO_EN=1`、`RDOB_EN=1`：`dist` 是 LESO 的原始输入扰动估计，
`rdob_dist` 是 RDOB 的原始输入等效扰动估计，两者在同一控制周期更新，
可直接记录比较。默认 `RDOB_COMPENSATION_EN=0`，因此 RDOB 不改变控制；
当前 LESO 六路补偿增益也均为零，平衡控制误差直接使用 `measured_state`，
因此本配置中两种观测结果都只用于数据对比，不进入控制量。

`dist` 和 `rdob_dist` 共用 `OBSERVER_DISTURBANCE_LIMIT`。当前
`DIST_RATIO=0.1` 时，Tw 两路为 `±0.326664 Nm`，Tp 两路为 `±6 Nm`，
FL 两路为 `±30 N`。触及限幅后只能比较符号、变化趋势和脱离饱和的行为，
不能再用幅值相等判断观测器精度。

准备接入 RDOB 时，需要将 `RDOB_COMPENSATION_EN` 设为 `1`，并把
`RDOB_COMPENSATION_GAIN` 中选定通道从当前的 `0` 改为所需增益；六个都设
为 `1` 才表示全 Tw+Tp+FL 补偿。保持为 `0` 的通道仍可观测但不反馈。
本版本没有远离平衡态衰减。
