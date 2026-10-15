# 四状态前向 KF：KF_ok 实测逻辑与通用库实现

实现位于 `Robot/infantry1/Chassis/pyro_wl_forward_kf.cpp/.h`，使用 `lkf_t<4,2>`。
当前数学逻辑和参数对齐实测分支 KF_ok / 19ea989；详细差异和仿真结果见
[forward_kf_kf_ok_alignment.md](forward_kf_kf_ok_alignment.md)。

## 1. 状态、预测与观测

固定周期 T=0.001 s，状态为 `[x,v,a,b_a]`，分别为前向位置、速度、加速度和
映射后的 IMU 前向加速度慢变零偏。采用匀加速度传播：

```text
F = [1 T T²/2 0
     0 1 T    0
     0 0 1    0
     0 0 0    1]

x- = F*x
P- = F*P*Fᵀ + Q_d
```

只有两条观测：

```text
z = [a_imu, v_wheel]ᵀ
H = [0 0 1 1
     0 1 0 0]
```

IMU 观测 `a+b_a`，轮速观测 `v`。不将积分轮里程再用作位置观测，也不引入
额外轮速平均延迟。零加速度、零轮速都是合法量测。
`ddot_x_model` 继续由现有动力学模块计算，供独立诊断/其他逻辑使用，
不参与此 KF 的状态更新或滑移评分。KF 不重算 IMU 安装偏移映射，也不加入 RDOB 扰动。

## 2. 噪声、初始化与存储

每拍过程噪声保持 KF_ok 的离散增量模型：

```text
g_a = [T²/2, T, 1, 0]ᵀ
Q_d = 0.04 * g_a*g_aᵀ + diag(0,0,0,1e-7)
R_base = diag(0.25,0.0025)
P0 = diag(0.01,0.25,1.0,0.25)
min_variance = [1e-8,1e-6,1e-5,1e-8]
```

Q_d 已包含噪声作用映射，核心不再乘 dt。这里是每拍增量方差约定，
不是连续白噪声 jerk 的精确积分。修改周期时需同时检查 F/Q_d 和微分器。
R/P 的对角项是方差，不是标准差；位置单位 m，速度 m/s，加速度与零偏 m/s²。

首次 init 的状态为零，使用固定存储，重复 init 不改变运行状态。
reset 使用 `[x,v_wheel,a_imu,0]` 和 P0，失效轮速的初始速度取零，并清除
滑移评分、轮速微分历史与动态 R。进入 balance/spin 时由当前底盘适配重设位移原点。
初始化、更新和重置都不分配堆内存。

## 3. 连续滑移评分与动态 R

两轮在线且轮速有限时才启用平均轮速。采用 15 Hz 带限微分：

```text
omega_c = 2*pi*15
A_d = (2-omega_c*T)/(2+omega_c*T)
B_d = 2*omega_c/(2+omega_c*T)
a_wheel = A_d*a_wheel_previous + B_d*(v_wheel-v_wheel_previous)
```

使用本拍预测、尚未融合当前量测的状态与协方差：

```text
r_v = v_wheel-v-
nis_wheel = r_v²/(P-(1,1)+R_wheel_base)
a_imu_corrected = a_imu-b_a-
r_wi = a_wheel-a_imu_corrected
g_v = clamp((nis_wheel-4)/(16-4),0,1)
g_a = clamp((abs(r_wi)-0.30)/(0.60-0.30),0,1)
slip_target = 0.5*(g_v+g_a)
```

单条轮速 NIS 使用基础 Rwheel，不用本拍放大值，也不是核心的联合 NIS。
使用预测零偏，避免当前轮速校正反馈到本拍评分。不再使用模型加速度置信度。

目标高于旧评分时 tau=0.002 s，否则 tau=0.03 s：

```text
score += T/(tau+T)*(slip_target-score)
R_wheel = 0.0025*(1+score*(500-1))
```

评分连续且在 [0,1]。固定 1 ms 时，上升系数 1/3、恢复系数 1/31。
目标从 0 突变为 1 的第一拍评分为 1/3，R 倍率约 167.3333；目标归零后
评分逐拍乘 30/31。有效轮速即使在高评分下仍参与融合，有限倍率不能替代独立速度参考。

## 4. 本拍时序、零偏冻结与 Joseph 更新

```text
更新轮速微分历史
predict(F,Q_d)
根据预测 x/P 调整本拍 R_wheel
correct(z,H,R,mask,policy)
    计算 y、S、K、NIS
    policy 按轮速置信度冻结零偏 K 行
    用同一个调整后的 K 更新状态和 Joseph 协方差
```

轮速失效时 mask 为 `{true,false}`，只融合 IMU；有效时为 `{true,true}`。
重连首拍不把失联期间的轮速差当作单拍加速度。

当轮速无效，或 score >=0.2 时，Policy 清零 `K(3,0)`、`K(3,1)`，并返回 PYRO_OK。
这是暂停零偏学习，不是拒绝整次校正。IMU 单独只能观测 `a+b_a`，此时 bias
保持预测值，加速度仍可更新；Joseph 使用同一 K，使 bias 不确定度不被虚假压小。
轮速可信且评分小于 0.2 时自动恢复学习，无须另一个解冻标志。

```text
y = z-H*x-
S = H*P-*Hᵀ+R
K = P-*Hᵀ*S⁻¹
K = policy_adjusted(K)
x_new = x-+K*y
A = I-K*H
P_new = A*P-*Aᵀ+K*R*Kᵀ
```

求解使用 INVERSE，协方差使用 Joseph。核心拥有固定工作区，按选项对称化 P
并施加方差下限。CMSIS 失败由诊断保留原始状态码；校正失败保留本拍预测，可在下一拍恢复。
IMU/模型输入映射沿用既有有效性约定，此模块额外检查的是轮速有效性。

## 5. 底盘接入与诊断

调用接口为 `update(a_imu,v_wheel,wheel_valid)`，不再接收 a_model。
当前 balance/spin 在控制前更新 KF，将估计写入 `odom.X[0..3]`、`odom.slip_score`，
并替换 14 维控制状态中的 x/v；其他十二项反馈来源保持原设计。
原始轮里程和两路加速度仍可用于诊断，其他状态机、RDOB 和模型系数未随 KF_ok 整体合并。

`get_filter()` 提供新库的 `get_state()/get_covariance()/get_gain()/get_diagnostics()`；
`get_measurement_noise()(1,1)` 为本拍轮速方差。冻结时检查 K 的第 3 行、bias 及其方差。
INS 使用独立的 `ekf_t<6,3>`，前向估计不复用其状态或工作区。

x 没有绝对位置观测，只能累计相对位移。bias 是前向映射残差的慢变估计，
不保证等同于传感器硬件零偏；长期失联/滑移时冻结 bias 也不能提供独立速度可观测性。

## 6. 验证

KF_ok 的估计器、后端和参数在 `tests/forward_kf/reference` 中冻结，仅测试链接。
normal/stress 各 30,000 拍比较 33 个 float32；检查每个冻结样本的 K/bias/方差，
覆盖离线、NaN/Inf、滑移、恢复与重置。另有独立 double KF、静止零量测、零偏收敛、
2 ms/30 ms 评分响应、学习恢复、协方差正定/对称/下限和优化构建测试。

13 项 CTest 和 infantry1 ARM Debug 构建通过，INS 原有轨迹仍逐字节一致。
本轮没有重新刷机或实测前向闭环；具体数值与适用范围见 KF_ok 对齐记录。

```powershell
cmake --build build/forward-kf-tests --parallel 4
ctest --test-dir build/forward-kf-tests --output-on-failure -V
cmake --build cmake-build-debug-stm32 --parallel 4
```
