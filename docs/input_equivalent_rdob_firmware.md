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

位置误差相对当前 LQR 配平 `q0` 计算；速度误差相对移动配平速度计算。正常
行进、旋转和变腿长时，`dot_x`、`dot_psi`、`dot_L` 分别使用当前目标速度，
因此名义运动不会被当作扰动。`psi` 的差值按 `[-pi, pi]` 回绕。

## 原有下位机框架中的计算

`_gain_calculate()` 在原有控制参数调度后，直接将当前 `L1,L2` 代入
`rdob_coef.h` 的 Horner 多项式：

```
Gamma[6][7]  <- 三次全张量
B_q[6][0:2]  <- 0
B_q[6][2:7]  <- 二维总阶三次
```

`_rdob_update()` 沿用已有的 `delta_q0`、`delta_dot_q0`、`z`、`dot_z`
缓存。它在 VMC 限幅和发送电机命令后，与 LESO 在同一控制周期更新。
RDOB 默认只写入 `rdob_dist`，不改变 `u`。

连续方程和显式 Euler 递推为：

```
dot_z = -Lambda * (z + Gamma * delta_dot_q)
      + B_q * delta_q
      - Lambda * delta_u
z += dt * dot_z
d_hat = z + Gamma * delta_dot_q
```

其中 `delta_u` 使用 VMC、关节力矩和轮端限幅后的实际输出。腿力的实际
输出和 LQR 的 `U0[F_L]` 都是主动电机等效力；在两边加上同一当前气弹簧
模型力后相消，因此它们的差就是总广义腿力增量。

## 启动、越界与补偿选择

进入平衡状态时，观测器以 `d_hat=0` 初始化，即
`z=-Gamma*delta_v`。调度输入沿用既有 LQR 的腿长归一化与边界钳位。

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
