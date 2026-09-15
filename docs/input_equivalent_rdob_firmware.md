# 六阶输入等效 RDOB：下位机接入

本实现位于 `Robot/infantry1/Chassis`，仅在正常平衡状态启用。它估计
`T_w1, T_w2, T_p1, T_p2, F_L1, F_L2` 六路输入等效扰动；`z[6]` 是唯一的
观测器动态状态。

## 调度与误差坐标

`rdob_coef.h` 是上位机生成的浮点系数。它以实际 `L1,L2` 调度，`Gamma`
采用三次全张量直接拟合，`Bq` 采用总阶三次直接拟合，默认 `D=0`，所以
`Bv=-Lambda*Gamma` 已包含在 `step_velocity_gain` 中。

位置、速度顺序固定为：

```
x, psi, theta, phi, L_bar, beta1, beta2
```

位置误差相对上一区间的 LQR 配平 `q0` 计算；速度误差相对移动配平速度
计算。正常行进、旋转和变腿长时，`dot_x`、`dot_psi`、`dot_L` 分别使用当前
目标速度，因此名义运动不会被当作扰动。`psi` 的差值按 `[-pi, pi]` 回绕。

## 一个控制周期的时序

1. 到达新测量时，`_gain_calculate()` 生成当前 LQR 配平和增益。
2. `_rdob_update()` 用上一区间保存的系数、两端测量和实际总输入更新
   `z`，并得到当前 `d_hat`。
3. 用当前 `L1,L2` 系数按
   `z = d_hat - Gamma_current * delta_v_current` 重基准化；这不会使
   `d_hat` 在调度切换时跳变。
4. `_balance_control()` 保持原 LESO 控制链；RDOB 默认只写入
   `rdob_dist`，不改变 `u`。
5. `_vmc_trans_v2j()` 完成关节力矩限幅；随后
   `_rdob_capture_applied_input()` 缓存实际总输入，供下一次更新使用。

离散更新为：

```
z[k+1] = Az*z[k]
       + Cq*(delta_q[k] + delta_q[k+1])
       + Cv*(delta_v[k] + delta_v[k+1])
       + Cu*delta_u_applied[k]
```

其中 `delta_u_applied` 绝不使用未经限幅的 LQR 原始指令。腿力输入使用
`actual_out_F_L + gas_spring_force`；同样，LQR 的主动 `U0[F_L]` 也加回
气弹簧模型力后才构成 RDOB 的总配平输入。这保证观测器内的输入定义与
线性化模型一致。

## 启动、越界与补偿选择

进入平衡状态时，观测器以 `d_hat=0` 初始化，即
`z=-Gamma*delta_v`。腿长非有限或离开 `[0.18, 0.38] m` 时，系数求值拒绝
该样本，RDOB 清零并等待下一次有效初始化。

默认 `LESO_EN=1`、`RDOB_EN=1`：`dist` 是 LESO 的原始输入扰动估计，
`rdob_dist` 是 RDOB 的原始输入等效扰动估计，两者在同一控制周期更新，
可直接记录比较。默认 `RDOB_COMPENSATION_EN=0`，因此 RDOB 不改变控制；
LESO 仍维持原有预测与补偿链路。

`dist` 保留原 LESO 的 `DIST_RATIO` 限幅，而 `rdob_dist` 不限幅。比较估计
幅值时应优先选取 LESO 未触及限幅的区间；触及限幅时，二者只能比较符号、
变化趋势和收敛时间，不能将幅值差直接归因于观测器性能。

准备接入 RDOB 时，将 `RDOB_COMPENSATION_EN` 设为 `1`。此时
`RDOB_COMPENSATION_GAIN` 的六个 `1` 表示全 Tw+Tp+FL 补偿；将任一元素
改为 `0` 可继续观测该通道但不反馈补偿。本版本没有远离平衡态衰减。
