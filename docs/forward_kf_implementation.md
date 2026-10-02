# 四状态前向 KF 与下位机实现

实现位于 `Robot/infantry1/Chassis/pyro_wl_forward_kf.cpp/.h`，复用 `PYRo/Algorithm/KF/kalman_filter.cpp/.h` 和真实 CMSIS-DSP 矩阵函数。参数集中在 `wl_config.h` 的全大写 `FORWARD_KF_*` 常量中。

当前在地面 `state_balance_t` 和 `state_spin_t` 启用，覆盖调用 `_balance_control()` 的两条控制路径，使用已有 `ddot_x_imu`、无扰动的 `ddot_x_model` 和原始当拍轮速。只替换十四状态 LQR 的 `measured_state.x`、`measured_state.dot_x`。其他状态仍来自既有反馈路径。其余模式暂停前向 KF 更新；进入 balance 或 spin 时重置前向 KF，避免把暂停前的状态当作上一毫秒的数据。

## 1. 状态与离散模型

\[
X=[x,v,a,b_a]^T,\qquad T=0.001\ \mathrm s,
\]

其中 x 为轮心中点的累计前向位移，v 为真实前向速度，a 为真实前向加速度，b_a 为映射后的 IMU 前向加速度慢变偏差。采用

\[
X^-_k=FX_{k-1},\qquad
F=\begin{bmatrix}
1&T&\tfrac12T^2&0\\
0&1&T&0\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}.
\]

`uSize=0`，不使用 B 或显式控制输入。六路实际执行输入已由现有动力学模块形成模型加速度观测。前向 KF 不加入 RDOB 扰动，不重新计算 IMU 映射。

\[
z_k=\begin{bmatrix}\ddot x_{imu}\\\ddot x_{model}\\v_{wheel}\end{bmatrix},
\qquad
H=\begin{bmatrix}
0&0&1&1\\
0&0&1&0\\
0&1&0&0
\end{bmatrix}.
\]

三条量测分别为 `a+b_a`、`a`、`v`。直接维护完整 H，固定 `UseAutoAdjustment=0`。零加速度和零轮速都是合法量测；不会触发现有自动调整机制的“零代表无效”逻辑。

## 2. Q、R、P 与初始化

按 prompt 的每拍离散加速度扰动模型构造

\[
G_a=[\tfrac12T^2,T,1,0]^T,\qquad
Q=\sigma_a^2G_aG_a^T+\operatorname{diag}(0,0,0,\sigma_b^2),
\]

\[
Q=\begin{bmatrix}
\sigma_a^2T^4/4&\sigma_a^2T^3/2&\sigma_a^2T^2/2&0\\
\sigma_a^2T^3/2&\sigma_a^2T^2&\sigma_a^2T&0\\
\sigma_a^2T^2/2&\sigma_a^2T&\sigma_a^2&0\\
0&0&0&\sigma_b^2
\end{bmatrix}.
\]

这里 σa²、σb² 是**每个离散周期**的增量方差；此 Q 沿用给定 prompt 的噪声作用约定，不是连续白噪声 jerk 的精确积分 Q。修改周期时需同时重新整定 Q、F 和带限微分系数。

初始数值为

\[
\sigma_a^2=0.04,\quad \sigma_b^2=10^{-7},
\qquad
Q=\begin{bmatrix}
10^{-14}&2\times10^{-11}&2\times10^{-8}&0\\
2\times10^{-11}&4\times10^{-8}&4\times10^{-5}&0\\
2\times10^{-8}&4\times10^{-5}&0.04&0\\
0&0&0&10^{-7}
\end{bmatrix}.
\]

所有矩阵按 row-major 保存，R 储存方差，初始设置为

\[
R_0=\operatorname{diag}(0.25,1.0,0.0025),
\qquad P_0=\operatorname{diag}(0.01,0.25,1.0,0.25).
\]

R 的单位依次为 `(m/s²)²`、`(m/s²)²`、`(m/s)²`；P 的单位依次为 `m²`、`(m/s)²`、`(m/s²)²`、`(m/s²)²`。`StateMinVariance=[1e-8,1e-6,1e-5,1e-8]`，防止各状态方差过度收敛。

对应初始化入口：

```cpp
Kalman_Filter_Init(&_kf, 4, 0, 3);
_kf.UseAutoAdjustment = 0;
_kf.UseJosephForm = 1;
_kf.UserData = this;
_kf.User_Func2_f = _adjust_wheel_r;
```

`wl_forward_kf_t::init()` 直接写入上述 F/H/Q/R、最小方差，并通过 `reset()` 写入 P。沿用库的初始化分配，整个底盘生命周期仅分配一次；`init()` 重复调用不会再次分配，周期更新和模式重置没有动态分配。没有引入 Eigen。

初次初始化为 `[0,0,0,0]`。进入 balance 或 spin 时不能保证机器人静止，因此用

\[
X_0=[0,v_{wheel},\ddot x_{imu},0]^T
\]

初始化，并把轮速微分器的上一样本设为当前轮速，避免启动微分冲击。两轮编码器失效时初始 v 取零。每次进入 balance 或 spin 都重置 P、bias 和 slip score，同时沿用既有 `wheel_x=0`、`target_state.x=0` 的位移原点约定。

## 3. 每拍预测与打滑评分

标准预测：

\[
X^-=FX,\qquad P^-=FPF^T+Q.
\]

轮速使用 `odom.wheel_dot_x[0]`，即既有两轮平均切向速度的当拍值；不把积分后的 `wheel_x` 再作为独立位置量测，也不使用两拍平均轮速，避免额外延迟和重复观测。

带限微分的截止频率为 15 Hz：

\[
a_{wheel,k}=A_da_{wheel,k-1}+B_d(v_{wheel,k}-v_{wheel,k-1}),
\]

\[
A_d=\frac{2-\omega_cT}{2+\omega_cT},\qquad
B_d=\frac{2\omega_c}{2+\omega_cT},\qquad \omega_c=2\pi\times15.
\]

利用**本拍量测更新之前**的预测量计算

\[
r_v=v_{wheel}-v^-,\qquad
S_v=P^-_{vv}+R_{w0},\qquad
\gamma_v=r_v^2/S_v,
\]

\[
a_{imu,corr}=\ddot x_{imu}-b_a^-,\qquad
r_{wi}=a_{wheel}-a_{imu,corr},\qquad
r_{im}=a_{imu,corr}-\ddot x_{model}.
\]

采用预测 bias，避免本拍轮速融合后的 bias 反过来参与当前轮速门控。NIS 的分母始终使用正常轮速方差 Rw0，不用已经放大的 Rwheel，防止评分自我稀释。

定义 MCU 分段线性函数

\[
\ell(y;y_0,y_1)=\operatorname{clamp}\!\left(\frac{y-y_0}{y_1-y_0},0,1\right),
\]

\[
g_v=\ell(\gamma_v;4,16),\qquad
g_a=\ell(|r_{wi}|;1,4),\qquad
c_{ref}=1-\ell(|r_{im}|;1,4).
\]

加速度阈值单位为 m/s²。4/16 是初始 NIS 整定阈值，不宣称为最终统计判据。评分采用两条 wheel 异常证据的平均，再乘以参考一致性：

\[
s_{target}=c_{ref}\frac{g_v+g_a}{2},\qquad
s_k=s_{k-1}+\frac{T}{\tau_k+T}(s_{target}-s_{k-1}),
\]

\[
\tau_k=\begin{cases}
0.003\ \mathrm s,&s_{target}>s_{k-1},\\
0.03\ \mathrm s,&s_{target}\le s_{k-1}.
\end{cases}
\]

因此 `s∈[0,1]`，只出现一个 wheel 异常证据时目标值最多为 0.5。IMU 与模型差异超过 4 m/s² 时参考可信度降为零。上升采用 3 ms 时间常数，快速降低轮速权重，同时保留少量平滑；下降采用 30 ms 时间常数，避免打滑证据暂时消失时过快恢复轮速权重。分支只选择时间常数，评分仍为连续量；未使用 expf 或硬切换打滑状态。

对应常量为 `FORWARD_KF_SLIP_RISE_TAU=0.003f`、`FORWARD_KF_SLIP_FALL_TAU=0.03f`。固定 1 ms 更新时上升系数为 `1/4`，下降系数为 `1/31`。当评分从零开始且原始目标保持为 1，第一拍评分为 0.25，第 3 拍为 0.578125，第 5 拍为 0.762695。原始目标回到零后评分逐拍乘以 `30/31`，不是立即清零。这些数值描述评分滤波器响应，实际目标还由 NIS、带限轮速微分及参考一致性共同决定。

更新量测方差：

\[
R_{wheel}=R_{w0}[1+s(500-1)],\qquad R_{w0}=0.0025.
\]

上述目标为 1 的首拍会把 Rwheel 放大至 Rw0 的 125.75 倍，当拍融合前即生效。3 ms 上升仍会削弱单拍评分峰值，并不保证完全排除错误轮速对状态的影响；具体轮速增益还取决于预测协方差。

Rimu、Rmodel 固定，Rwheel 连续变化。即使 s 接近 1，轮速观测仍参与融合。长期滑移期间估计可能逐渐被轮速牵引，因此评分和有限倍率不能替代外部速度参考；参数需通过实机滑移数据整定。

### 当拍时序

`Kalman_Filter_Update()` 中 `User_Func2_f` 位于预测完成之后、增益计算之前。因此由 `_adjust_wheel_r()` 写入 `R_data[8]`，可以准确使用当前 Pminus 和预测速度，无需手动执行一遍预测，也无需使用上一拍 NIS。

每周期外部入口仍为：

```cpp
_kf.MeasuredVector[0] = a_imu;
_kf.MeasuredVector[1] = a_model;
_kf.MeasuredVector[2] = v_wheel; // 有效时，0.0f 也会参与融合
Kalman_Filter_Update(&_kf);     // 当拍回调先更新 R_data[8]，再融合
```

两轮 `is_online()` 同时为真才启用平均轮速观测；离线/CAN 超时或轮速非有限值时，直接把完整 H 的第三行置零，并把该量测置零。其余两条加速度量测继续使用。轮速恢复时重新播种微分器，避免把缺失期间的差值作为单拍加速度。本实现仅对轮速做这一有效性判断，IMU 和模型输入沿用既有反馈有效性约定。

## 4. 量测修正与数值稳定

\[
S=HP^-H^T+R,\qquad K=P^-H^TS^{-1},
\]

\[
X=X^-+K(z-HX^-).
\]

本实例采用 Joseph form：

\[
P=(I-KH)P^-(I-KH)^T+KRK^T.
\]

复用现有三个临时矩阵和两个临时向量，不新增周期内存分配。后处理以 `0.5*(Pij+Pji)` 对称化非对角元素，再施加最小对角方差。

标准 `Kalman_Filter_SetK()` 检查 CMSIS 矩阵逆的返回值。逆失败则置 `MeasurementUpdateFailed=1`，该拍保留 `Xminus/Pminus`，不使用旧 K；下拍重试并清除失败标记。`MatStatus` 保留失败原因以供诊断。不会通过清零状态掩盖求逆失败。

## 5. 底盘接入与诊断输出

反馈阶段的前向状态统一读取最近一次 KF 输出：

```cpp
state.x     = _ctx.data.odom.X[0];
state.dot_x = _ctx.data.odom.X[1];
```

`_update_feedback()` 不再用原始轮里程覆盖这两项。随后 balance 或 spin 在执行本拍 LQR 前调用 `_update_forward_kf()`，同时更新 `odom.X` 和 `measured_state.x/dot_x`。因此 LQR 使用当拍融合值；反馈阶段暂存上一拍结果不会引入一拍控制延迟。其余模式保留最近一次结果，恢复地面控制时按上述规则重置。

两条地面控制路径的共同顺序为

```text
反馈与 IMU 映射
→ _gain_calculate()：本拍腿长对应的 beta0 / U0
→ _calc_ddot_x_model()：已结束控制区间的实际输出，无扰动
→ _update_forward_kf()：预测、slip/R、融合
→ _rdob_update() / _calc_state_deviation()：仅 balance，沿用原有策略
→ _balance_control() → 执行器限幅与实际输出缓存
```

balance 和 spin 都在执行器转换、限幅后调用 `_actual_output_update()`，让下一拍模型读取已完成区间的最终指令。退出任一模式时将 `ddot_x_model` 置零，表示暂停该模型。

只有 x、v 写入 LQR 反馈，模型加速度不依赖这两项，因而不会出现当前模型计算依赖当前 KF 输出的代数环。balance 的 RDOB 随后读到融合后的 x/v，其状态和参数保持既有实现；spin 保持原来的 RDOB 调用策略。

当前模型系数来自零广义速度的地面配平线性化，未包含高速旋转的非线性速度耦合。spin 接入相同 KF 并不改变这一模型适用限制；Rmodel 仍需结合旋转工况的模型残差整定。

输出类型：

```cpp
struct ForwardStateEstimate
{
    float x;
    float v;
    float a;
    float imu_acc_bias;
    float slip_score;
};
```

`reset()/update()` 返回该值。为复用已有监测字段，底盘保存至 `odom.X[0..3]` 和 `odom.slip_score`；`odom.wheel_x`、`wheel_dot_x`、两路原始加速度仍保留作对照。`get_filter()` 的只读引用可检查 P、K、Rwheel、MatStatus 和 MeasurementUpdateFailed。

x 没有绝对位置观测，因此只能继承初始化原点并累计相对位移；位置偏差不可通过本 KF 自身完全消除。模型加速度的持续偏差会影响 a 和 IMU bias 的分离；ba 是前向映射残差的慢变估计，不保证等同于加速度计硬件零偏。

## 6. 共用 KF 库的修改与 IMU 兼容性

| 修改 | 对现有 IMU 姿态实例的影响 |
|---|---|
| `uSize==0` 不分配 ControlVector、不 memcpy 空 u_data | 保持无控制输入的数值行为，消除空指针调用 |
| 临时矩阵/向量按 max(xhatSize,zSize) 分配 | 6 状态、3 量测姿态实例的缓冲区容量不变；修复一般 zSize>xhatSize 时的越界 |
| 新增 UserData 回调上下文 | 前向 KF 指向自身；现有姿态回调不读取此字段 |
| 新增 UseJosephForm，初始化默认零 | 前向 KF 显式开启；QuaternionEKF 继续使用原来的 P 更新 |
| 新增 MeasurementUpdateFailed、标准逆失败回退 | 标准线性 KF 启用保护；QuaternionEKF 使用 SkipEq3 和自定义逆/修正，原有路径保持不变 |

前向 KF 与 `QEKF_INS.IMU_QuaternionEKF` 为独立实例，F/H/Q/R/P、状态、回调和缓冲区各自拥有，不复用任何 IMU 姿态状态。新数值稳定行为不是全局开关。

注意：姿态 EKF 的自定义回调仍含自己的 `Matrix_Inverse()`；这次对标准 SetK 的检查不会自动接管它的错误处理。没有在本任务中修改姿态估计公式。`KalmanFilter_t` 增加字段，所有依赖它的源码需要一起重新编译，不能混用旧二进制布局。

## 7. 实机整定顺序

1. 保持机器人静止，记录两路原始加速度、轮速及 bias。去掉均值后估算 Rimu/Rwheel；包含结构振动和实际映射残差，而非仅引用传感器手册噪声。零速时三条量测均应参与。
2. 在可信无滑移运动段比较模型加速度与参考加速度，选择 Rmodel。多项式系数拟合误差不能直接当作模型加速度量测方差；实际还包含参数误差、未建模耦合和输入误差。
3. 调 σa²：跟随加速明显滞后时提高；估计 a 抖动过强时降低，同时核对 R。σb² 远小于 σa²，bias 应慢变；不要用快速 bias 去吸收真实加速度。
4. 用启动重复性确定 P0。bias 初始不确定度要允许收敛；检查 StateMinVariance 不高到压倒 Q/R。进入 balance 或 spin 会重新学习 bias。
5. 记录正常加速段的 γv、rwi、rim。带限轮速微分有相位滞后，阈值须覆盖正常动态和机械振动，先确认正常运动不会持续放大 Rwheel。
6. 再用可控滑移数据调 NIS/加速度阈值、3 ms 上升/30 ms 下降时间常数和最大倍率。500 是初始值，允许在 100–1000 范围实验；分别检查短促打滑时的降权速度和证据消失后的恢复速度，以及静止轮被架空转动时的速度估计。
7. 同步查看 x/v、估计 a、bias、slip score、Rwheel、求逆状态和实际周期。当前 F/Q/微分均按 1 ms 配置，周期明显变化或漏拍应先修正调度或引入真实 T。

以上为初始值和整定方法，不代表已经完成实机整定。

## 8. 验证

ARM 固件：

```powershell
cmake --build cmake-build-debug-stm32 --parallel 4
```

本地真实 CMSIS-DSP 数值测试：

```powershell
cmake -S tests/forward_kf -B build/forward-kf-tests -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/forward-kf-tests --parallel 4
ctest --test-dir build/forward-kf-tests -V
```

检查已通过：20,000 拍静止合法零量测和协方差正定/对称性、15,000 拍静态 bias 收敛、30,000 拍运动轨迹与独立 double KF、当拍打滑降权与参考一致性门控、单拍轮速异常的 3 ms 上升及 30 ms 恢复尾部、编码器无效与恢复、真实 CMSIS 求逆失败回退、zSize>xhatSize 临时缓冲区、原 QuaternionEKF 的 10,000 拍静止回归。运动轨迹的 float32 状态与 double 参考最大差约 `3.34e-4`（不同状态的数值比较，非实机估计精度）。

`chassis_control_chain` 从固件提取真实反馈、模型、KF 适配、LQR 及 balance/spin 状态函数，连续执行各三拍，包含单拍轮速异常。验证反馈不被原始轮里程覆盖、两种模式进入时的 KF 重置、每拍先调度再计算模型并融合、RDOB/LQR 读到当拍 KF 输出、其他十二项状态保持原值，以及下一拍模型读取上一拍限幅后的输入。传感器、执行器转换和 RDOB 入口采用测试替身；测试验证软件接线与 LQR 指令，不验证硬件动态或 RDOB 数值算法。两项 CTest 均已通过。

另用 `tests/forward_kf/quaternion_ekf_trace.cpp` 对修改前后 KF 库输入相同的 20,000 拍三轴动态 IMU 序列，对比每拍四元数、gyro bias、姿态角、6 维状态和 6×6 协方差，共 52 个 float/拍，结果逐字节一致。该对照使用同一 PC 编译器和真实 CMSIS 矩阵实现；验证本次库改动未改变已有姿态路径的计算结果，不代表 ARM 与 PC 位级一致。

已完成 ARM 编译，未刷机或验证实机动态闭环。
