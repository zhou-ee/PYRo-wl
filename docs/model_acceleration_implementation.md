# 前向模型加速度的下位机实现

本文对应 `Robot/infantry1/Chassis/pyro_wl_chassis.cpp` 中的 `_calc_ddot_x_model()`。当前按用户要求**不使用扰动估计**，计算结果写入 `_ctx.data.odom.ddot_x_model`，单位为 m/s²。

## 1. 模型及标量公式

广义坐标与输入顺序沿用现有 LQR：

\[
q=[x,\psi,\theta,\phi,L_{\rm bar},\beta_1,\beta_2]^T,
\quad u=[T_{w1},T_{w2},T_{p1},T_{p2},F_{l1},F_{l2}]^T.
\]

φ 是机体横滚，β1/β2 是虚拟腿摆角；x 是轮心中点 C 沿水平航向的累计位移。

静态配平满足 `S0 u0 = G0`，现有线性化取 `D0=0`，因此

\[
M_0\delta\ddot q=-K_0\delta q+S_0\delta u,
\qquad K_0=\left.\frac{\partial(G-Su_0)}{\partial q}\right|_{q_0}.
\]

上位机求解 `M0.T r = e_x` 后，仅保留前向加速度行：

\[
k=-r^T K_{0,:,\mathcal I},\qquad b=r^T S_0,
\qquad \mathcal I=(2,3,5,6).
\]

这里矩阵列索引使用 0 基索引，只选择 θ、φ、β1、β2。当前实测腿长连续调度使平均腿长偏差为零，按用户要求直接删除该通道。x、ψ 的位置系数为零，速度项在当前 `D0=0` 线性化中为零。下位机不计算上述矩阵，只计算

\[
\boxed{
\begin{aligned}
\ddot x_{\rm model}={}&k_\theta\delta\theta+k_\phi\delta\phi
+k_{\beta1}\delta\beta_1+k_{\beta2}\delta\beta_2\\
&+b_{w1}\delta T_{w1}+b_{w2}\delta T_{w2}
+b_{p1}\delta T_{p1}+b_{p2}\delta T_{p2}
+b_{f1}\delta F_{l1}+b_{f2}\delta F_{l2}.
\end{aligned}}
\]

**当前没有 `+b*d_hat` 项**，函数不读取 `rdob.dist`，结果不受 RDOB 使能或扰动估计值影响。已经作用到执行器指令上的补偿属于输入本身，不能从最终指令中另行扣除。

## 2. 总阶三次调度

使用上位机 `data/generated/model_acceleration/model_acceleration_schedule.h` 中的 100 个 float32 系数，逐项同步到 `wl_config.h` 的 `MODEL_ACCEL_COEF[10][10]`。新增 constexpr 常量全部使用大写名称并放在 `wl_config.h`，`MODEL_ACCEL_STATE_DIM=4U` 定义有效状态项数量。

归一化为

\[
\xi=\operatorname{clamp}((L_1-L_c)L_s^{-1},-1,1),
\qquad
\eta=\operatorname{clamp}((L_2-L_c)L_s^{-1},-1,1).
\]

`MODEL_ACCEL_LENGTH_CENTER` 为 0.28 m 的 float32 表示，`MODEL_ACCEL_LENGTH_INV_SCALE` 为沿用 LQR/RDOB 范围 `[0.18,0.38] m` 计算并舍入后的半区间倒数。两个值与上位机导出表一致。

每个通道的列顺序为

```text
1, eta, eta², eta³, xi, xi*eta, xi*eta², xi², xi²*eta, xi³
```

仅含 `i+j<=3` 的项，区别于三次全张量的 16 项。对于一行系数 a[0..9]，固件使用

\[
\begin{aligned}
x_0&=((a_3\eta+a_2)\eta+a_1)\eta+a_0,\\
x_1&=(a_6\eta+a_5)\eta+a_4,\\
x_2&=a_8\eta+a_7,\\
c&=((a_9\xi+x_2)\xi+x_1)\xi+x_0.
\end{aligned}
\]

全部 10 行用一个 for 循环求值，写入已有的两个数组：

| 表行 | 下位机存储 | 系数 |
|---:|---|---|
| 0–3 | `Hxi[0..3]` | k_theta、k_phi、k_beta1、k_beta2 |
| 4–9 | `Bx[0..5]` | b_w1、b_w2、b_p1、b_p2、b_f1、b_f2 |

未新增 ctx 数据成员；`Hxi` 从 5 个 float 缩减到 4 个 float，局部的归一化量、Horner 中间值及 4 项增量只在本次调用使用。系数表从 440 字节缩减到 400 字节，两个归一化常量 8 字节；每拍少一行多项式求值和一项状态乘加。没有动态内存、矩阵求逆或局部函数。

归一化截断沿用已有调度策略。上位机拟合误差预算适用于上述腿长域；截断不等于对域外动力学的验证。

## 3. 状态增量与控制目标的区别

调用前 `_gain_calculate()` 已按当前实际 L1/L2 更新腿角配平及主动输入配平。静态模型的 θ0、φ0 为零：

\[
\delta\theta=\theta,\quad\delta\phi=\phi,
\quad\delta\beta_i=\beta_i-\beta_{i0}.
\]

βi0 复用 `_gain_calculate()` 刚写入的 `target_state.beta1/beta2`，来源仍是既有 `BETA_TRIM_POLY_COEF`。不能把 βi0 当作零，也不能使用“目标值减测量值”的控制误差符号。

模型的腿长工作点是**当前调度腿长**：

\[
L_{{\rm bar},0}=(L_1+L_2)/2.
\]

`measured_state.L` 本来就是该均值，因此当前连续调度实现中 `delta_L_bar=0`，上下位机均不提取、拟合、存储或计算腿长偏差项。`target_state.L` 是升降腿的控制目标，不是本模型的静态工作点。用它计算增量会把控制目标变化错误计入模型加速度。完整地面动力学、LQR 与 RDOB 仍保持原来的七坐标/十四状态定义；这次仅缩减模型前向加速度的有效状态项。

实际运行的四项数组为

```cpp
const float delta_q[MODEL_ACCEL_STATE_DIM] = {
    state.theta, state.phi,
    state.beta1 - _ctx.data.target_state.beta1,
    state.beta2 - _ctx.data.target_state.beta2,
};
```

配平量使用既有 LQR 多项式近似，保持上下位机的配平路径一致；不另建立一套模型加速度配平调度。

## 4. 主动腿力与气弹簧

`_ctx.data.output` 保存上一个完整平衡控制区间的最终执行器指令。腿摆矩和主动腿力已按最终关节限幅结果反算；它不是未限幅的 LQR `control`，也不是电机扭矩反馈测量。

`U0` 中的腿力是主动电机配平输入。虽然上位机动力学采用总腿力，但同一当前工作点的气弹簧力在差分中抵消：

\[
\begin{aligned}
F_{l,\rm total}&=F_{l,\rm active}+F_{\rm gas}(L_i),\\
F_{l0,\rm total}&=U_{0,F_{li}}+F_{\rm gas}(L_i),\\
\delta F_{li}&=F_{l,\rm total}-F_{l0,\rm total}
=F_{l,\rm active}-U_{0,F_{li}}.
\end{aligned}
\]

所以六路输入统一写成

```cpp
const float delta_u = _ctx.data.output.data[input] - _ctx.data.U0[input];
odom.ddot_x_model += _ctx.data.Bx[input] * delta_u;
```

不对 `output` 单独添加气弹簧力，也不减两次气弹簧力。作用于最终指令的虚拟墙、关节限幅、横滚积分及已执行的控制补偿会自然包含在 output 中。

## 5. 调用位置及适用阶段

`pyro_wl_balance_state.cpp` 中连续地面平衡周期的顺序为

```text
读取 IMU/电机反馈并计算 measured_state、ddot_x_imu
更新控制目标
_gain_calculate()        // 当前腿长的 K、主动 U0、beta0
_calc_ddot_x_model()     // 当前反馈 + 已完成区间的 output；不使用扰动
_rdob_update()           // 原有 RDOB 更新流程
_balance_control()
_vmc_trans_v2j()
_actual_output_update()  // 保存最终指令，供下一周期读取
发送执行器指令
```

模型计算不改变控制目标、U0、执行器输出或 RDOB 状态。当前仅在地面平衡执行阶段更新该量；退出平衡时置零，避免在空中、手动等阶段留下上一平衡阶段的结果。置零代表该阶段停用本模型，不代表这些阶段的真实加速度为零。后续融合需按工作阶段使用，并在模式切换后等到本阶段已有完整上一控制区间。

## 6. 自洽性与验证

θ=φ=0、βi=βi0、output=U0 时，四项状态增量及六项输入增量都为零，因此模型输出严格为零。非零主动支撑力必须先减去 U0，不能直接乘 Bx。

验证结果：

- 使用现有 `cmake-build-debug-stm32` 配置完成 ARM 编译和链接，目标为 INFANTRY1 底盘。
- 对下位机**实际方法体**进行 PC C++ 编译，使用固件现有系数表、状态/输入布局，与 Python float32 计算对照。
- 1809 组训练/独立验证腿长点的小扰动计算、441 组配平零输出、10 组单通道检查，共 2260 组通过。
- 导入的 100 个系数与上位机 float32 表逐项相同；PC 对照中 10 个求值系数及最终标量结果逐项相同。
- 删除腿长偏差通道后，剩余十行 float32 拟合系数与删除前逐项一致；连续调度的模型输出等价。
- 对照包含刻意改变控制 θ/φ/L 目标的情况，确认没有将控制目标当作静态工作点；方法体不含 RDOB 访问。

上述是代码和离线数值验证，没有进行实机融合测试。ARM 构建采用现有浮点优化选项，PC 的逐位一致性检查关闭 FMA 收缩；MCU 上允许正常的浮点末位差异。

系数拟合仍采用原来的总阶三次结果：最大峰值归一误差约 1.308%，k_phi 训练 RMS 约 0.531%，略超 0.5% 建议目标。这是系数拟合误差，不是加速度输出误差。对照计算表明三次全张量没有显著改善，因此本次保留 10 项总阶三次。

## 7. 重新生成与同步

上位机命令：

```powershell
python run_model_acceleration.py --output-dir data/generated/model_acceleration
```

修改物理参数或配平模型后重新生成，并同步生成头文件中的 `MODEL_ACCEL_COEF`、`MODEL_ACCEL_LENGTH_CENTER` 和 `MODEL_ACCEL_LENGTH_INV_SCALE` 到 `wl_config.h`。同步后重新执行上位机验证和固件编译；不要把比较用的全张量结果替换进当前 10 项求值代码。
