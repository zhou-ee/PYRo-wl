# 五连杆气弹簧虚功补偿计划

## 1. 目标与采用方法

目标是在安装轻气弹簧后，让相同腿长 L、摆角 beta 下的总等效 VMC 输出保持与现有控制器一致。

五连杆结构中，气弹簧一端安装在大腿，另一端安装在与轮子相连的连杆。两个安装点分别属于不同的运动构件，不能假设它们绕同一个固定中心转动，也不能把 r_A、r_B 或安装方向角当成常量。

本计划采用以下解算方法：

1. 在机身上建立固定坐标系 GS_BASE。
2. 对每个 SolidWorks 姿态，直接导出两个安装点的相对二维坐标差 `dx、dz`。
3. 用相对坐标差计算气弹簧中心距 s。
4. 用安装点相对运动雅可比和气弹簧方向向量做虚功投影。
5. 将气弹簧广义力从电机侧控制输出中扣除，使“电机输出 + 气弹簧输出”仍等于原控制器输出。

轻气弹簧当前按恒力 300 N 处理，不做线性拟合，不建立刚度模型，也不需要在 SolidWorks 中模拟力-长度曲线。

现有重力配平继续使用 U0 和 BETA_TRIM。气弹簧只作为独立的外部广义力补偿，不能把一套完整重力模型再次叠加到旧 U0 上。

## 2. 当前 VMC 控制结构

当前每条腿的 VMC 广义坐标为：

~~~text
L     = 虚拟腿长度
beta  = 虚拟腿摆角
~~~

对应的共轭广义力为：

~~~text
F_L = 腿长方向广义力
T_p = 腿摆角方向广义力矩
~~~

现有 VMC 到髋、膝关节力矩的转换保持不变：

~~~text
tau_hip  = (T_p - F_L * J_L) / 2
tau_knee = (T_p + F_L * J_L) / 2
~~~

其中 J_L 是当前五连杆的腿长雅可比。气弹簧补偿应在 F_L、T_p 层完成，然后继续使用现有关节力矩映射。

## 3. 几何解算：安装点相对坐标法

### 3.1 固定坐标系的选取

在总装配体中建立固定在机身上的坐标系 GS_BASE：

~~~text
O = 任意固定在机身上的原点
X = 机器人前进方向，或腿部侧视平面的水平基准方向
Z = 竖直向上
Y = 垂直于腿部侧视运动平面
~~~

O 不是膝关节、轮电机中心或气弹簧转动中心。它只是给所有姿态提供同一个坐标基准，可以直接使用装配体原点，也可以选机身上容易定位的固定孔中心。只要整个姿态扫描过程中 O-X-Z 不随腿部运动，原点放在哪里都不会改变 A-B 距离和最终虚功结果。

本计划只使用 X-Z 平面数据，不导出 Y。使用二维数据前必须确认两个安装点的横向错层不会造成明显的三维长度变化；如果存在明显横向运动，后续需把 Y 坐标一起加入模型。

### 3.2 安装点定义

~~~text
A = 大腿侧气弹簧安装销轴中心或球头球心
B = 与轮相连连杆侧气弹簧安装销轴中心或球头球心
~~~

测量的是安装点的实际运动中心，不是气弹簧外筒端面、活塞杆端面、安装座外边缘或零件参考面的中心。

两点在不同构件上，二者的相对位移随姿态变化。正式输入直接使用：

~~~text
A(L, beta) = (x_A(L, beta), z_A(L, beta))
B(L, beta) = (x_B(L, beta), z_B(L, beta))
dx(L, beta) = x_B - x_A
dz(L, beta) = z_B - z_A
~~~

不再使用以下量作为主输入：

~~~text
r_A、r_B、ang_A、ang_B、ang_rel
~~~

这些量即使可以从坐标反算，也不需要导出，更不能放进一个假设固定的 geometry.csv。
绝对坐标 `x_A、z_A、x_B、z_B` 也不再作为 CSV 主输入；SolidWorks 直接导出
同一姿态下的 `dx、dz` 即可。

### 3.3 气弹簧中心距

对每个姿态，直接读取安装点坐标差：

~~~text
dx = x_B - x_A
dz = z_B - z_A
~~~

气弹簧两端中心距为：

~~~text
s = sqrt(dx * dx + dz * dz)
~~~

SolidWorks 直接测得的 A-B 中心距记为 s_check，只用于检查相对坐标导入是否正确：

~~~text
s_error = s - s_check
~~~

正式解算使用 `dx、dz` 计算的 s，不手工修改 s、导数或力值。

### 3.4 安装点相对运动雅可比

定义两个安装点的相对位移：

~~~text
r_rel = p_B - p_A = [dx, dz]^T
~~~

相对运动雅可比为：

~~~text
J_rel = d(r_rel)/d(L,beta)

J_rel = [ d_dx_dL      d_dx_dbeta
          d_dz_dL      d_dz_dbeta ]
~~~

气弹簧 A 到 B 的单位方向向量为：

~~~text
e_s = [e_s_x, e_s_z]^T
    = [dx/s, dz/s]^T
~~~

因此长度对虚拟坐标的导数由方向投影得到：

~~~text
ds/dL    = e_s_x * d_dx_dL    + e_s_z * d_dz_dL
ds/dbeta = e_s_x * d_dx_dbeta + e_s_z * d_dz_dbeta
~~~

Python 对 `dx`、`dz` 分别沿 `L` 和 `beta` 做有限差分，再用 `e_s` 投影。这样直接使用安装点的相对运动，不需要假设安装点绕同一个固定中心转动。

在工作区间端点没有两侧数据时，程序使用单边差分，并通过 `is_L_boundary`、`is_beta_boundary` 标记边界点。为获得 `ds/dbeta`，每个 L 至少需要多个 beta 姿态；如果第一阶段只有 beta = 0，则只计算 `ds/dL`，`ds/dbeta` 和 `T_gas_beta` 保持 `NaN`，不把它们误设为零。

### 3.5 虚功映射和补偿符号

气弹簧轴向力做功满足：

~~~text
F_s * ds = F_gas_L * dL + T_gas_beta * dbeta
~~~

因此：

~~~text
F_gas_L    = F_s * ds/dL
T_gas_beta = F_s * ds/dbeta
~~~

导数的正负号自动决定气弹簧对该广义坐标的作用方向，不要预先人为加负号。

为了保持原控制器的等效输出，电机侧补偿为：

~~~text
F_L_motor = F_L_base - F_gas_L
T_p_motor = T_p_base - T_gas_beta
~~~

检查关系为：

~~~text
F_L_motor + F_gas_L = F_L_base
T_p_motor + T_gas_beta = T_p_base
~~~

例如原 F_L_base = 100 N，当前姿态计算得到 F_gas_L = 200 N：

~~~text
F_L_motor = 100 - 200 = -100 N
总等效作用 = -100 + 200 = 100 N
~~~

这就是“气弹簧承担 200 N，电机反向输出 100 N，最终仍等效为原来的 100 N”。

## 4. 恒力气弹簧模型

本阶段直接使用轻气弹簧的近似恒力：

~~~text
F_s = 300 N
~~~

F_s 不是根据 s 手算出来的，也不是由 SolidWorks 计算出来的。它来自气弹簧的额定力或实测确认值，当前统一在 Python 命令中填写 --force 300。

本阶段不做以下工作：

~~~text
不测多个长度下的力
不拟合 F_s(s)
不计算刚度 k
不进行线性力-长度模拟
~~~

如果以后实测发现气弹簧力随长度变化明显，应另立非恒力模型并重新验证；不在当前方案中混入线性拟合。

## 5. SolidWorks 与 Python 的分工

### 5.1 SolidWorks：只建立姿态并导出几何

SolidWorks 负责“构型和测量”，不负责求导、虚功映射或补偿力计算。

#### 5.1.1 建立 GS_BASE

在总装配体中建立坐标系 GS_BASE：

1. 原点 O 选机身上的固定点，推荐使用装配体原点或容易重复定位的机身安装孔中心。
2. X 轴沿机器人前进方向或腿部侧视平面的水平基准。
3. Z 轴竖直向上。
4. Y 轴垂直于侧视平面。本方案不导出 Y，但要确认横向运动可忽略。

坐标系必须属于机身固定件，不能建立在大腿、小腿、轮连杆或气弹簧零件上。

#### 5.1.2 确定 A、B 两个测量点

在装配体中分别选择：

~~~text
A：大腿上的气弹簧安装销轴中心/球头球心
B：轮连杆上的气弹簧安装销轴中心/球头球心
~~~

如果安装座没有现成 Reference Point，可在销轴圆柱面上建立轴线，再以轴线与安装平面的交点作为销轴中心。两点定义完成后，确认 A、B 随所属构件运动，而 GS_BASE 保持不动。

#### 5.1.3 建立姿态配置

用 Configurations、Design Table 或现有运动学驱动建立姿态。每一行必须同时对应明确的 L 和 beta：

~~~text
第一阶段：beta = 0，L = 0.18, 0.20, 0.22, ..., 0.38 m
第二阶段：每个 L 增加 beta = -0.30, -0.20, ..., 0.30 rad
~~~

如果 SolidWorks 只能驱动髋、膝或连杆角度，则记录驱动角度，并用现有五连杆运动学换算出 L、beta。驱动角度只是姿态索引，不能直接当作 beta。

beta 应沿用固件定义：

~~~text
beta = current_leg_rad - PI / 2 - body_pitch
~~~

建立几何表时先固定机身水平，即 body_pitch = 0。

#### 5.1.4 每个姿态的测量流程

对每个配置重复以下步骤：

1. 激活配置并确认机身没有随腿部运动。
2. 用 Evaluate -> Measure 读取 A、B 在 GS_BASE 下的坐标差，记录 `dx = x_B - x_A`、`dz = z_B - z_A`。
3. 直接测量 A-B 两个中心点的距离，记录 s_check。
4. 记录该配置的 side、L、beta。

只抄录相对坐标差和中心距，不在 SolidWorks 里手算 ds/dL、ds/dbeta、F_gas_L 或 T_gas_beta。

#### 5.1.5 导出 CSV

只需要一个文件：

~~~text
data/solidworks/poses.csv
~~~

推荐字段顺序：

~~~csv
side,L,beta,dx,dz,s_check
L,0.180,0.000,0.060,-0.110,0.125
L,0.200,0.000,0.063,-0.117,0.133
~~~

字段含义和单位：

~~~text
side     = 左右腿标识，取 L 或 R
L        = 虚拟腿长度，单位 m
beta     = 虚拟腿摆角，单位 rad
dx       = x_B - x_A，单位 m，方向为 A 指向 B
dz       = z_B - z_A，单位 m，方向为 A 指向 B
s_check  = SolidWorks 直接测得的 A-B 中心距，单位 m
~~~

SolidWorks 若显示 mm，导出到 CSV 前统一除以 1000。角度若以 degree 记录，先转换为 rad：

~~~text
rad = degree * PI / 180
~~~

本方案不再创建或填写 geometry.csv，也不记录 r_A、r_B、ang_A、ang_B。

#### 5.1.6 SolidWorks 导出前检查

- GS_BASE 固定在机身，坐标轴方向在所有配置一致；
- A 是大腿安装销轴中心，B 是轮连杆安装销轴中心；
- A、B 没有误取到端面或安装座边缘；
- 每个配置都有明确且唯一的 L、beta；
- 左右腿分别导出，不能未经检查直接镜像坐标；
- s_check 在气弹簧机械允许长度范围内；
- 只导出 X-Z 相对坐标差 `dx、dz`，不填不存在的 Y 数据；
- L、dx、dz 和 s_check 全部使用 m，beta 使用 rad。

### 5.2 Python：解算、求导、验证和生成补偿表

Python 读取 poses.csv 后完成以下工作：

~~~text
1. 读取每行直接导出的 dx、dz，并计算 s
2. 将同一 side 的姿态按 beta、L 组织成网格
3. 计算气弹簧方向 e_s = [dx/s, dz/s]
4. 对 dx、dz 做有限差分，得到 J_rel
5. 用 e_s^T * J_rel 得到 ds/dL、ds/dbeta
6. 使用 F_s = 300 N 做虚功映射
7. 输出 F_gas_L、T_gas_beta 和中间雅可比量
8. 检查 s 与 s_check 的误差、数据完整性和虚功残差
~~~

核心计算为：

~~~text
dx = 相对坐标的 X 分量 = x_B - x_A
dz = 相对坐标的 Z 分量 = z_B - z_A
s = sqrt(dx * dx + dz * dz)

F_s = 300
F_gas_L = F_s * ds_dL
T_gas_beta = F_s * ds_dbeta
~~~

输出字段：

~~~csv
side,L,beta,dx,dz,s,F_s,e_s_x,e_s_z,d_dx_dL,d_dz_dL,d_dx_dbeta,d_dz_dbeta,ds_dL,ds_dbeta,F_gas_L,T_gas_beta,s_check,is_L_boundary,is_beta_boundary
~~~

输出中的 `s` 是由 `dx、dz` 计算的结果；`s_check` 是 SolidWorks 的独立测量值。`e_s_x`、`e_s_z` 是气弹簧方向，`d_dx_dL`、`d_dz_dL`、`d_dx_dbeta`、`d_dz_dbeta` 是相对安装点雅可比，便于检查数值求导和符号。

Python 不重新建立重力模型，不修改 U0、BETA_TRIM 或 LQR 增益。补偿只在运行时按当前姿态查表或插值得到。

### 5.3 Python 离线操作方法

#### 5.3.1 准备输入文件

在 Python 工程下准备：

~~~text
E:\RM27\PYRo-wl-python\data\solidworks\poses.csv
~~~

把 SolidWorks 导出的唯一 CSV 复制为 poses.csv。不需要 geometry.csv，也不需要手工填写 F_s、导数或补偿力。

第一阶段只有 beta = 0 时，每个腿长一行即可；要计算 ds/dbeta，则每个 L 必须有多个 beta 姿态，并尽量形成完整的 L-beta 网格。

#### 5.3.2 运行命令

在 PowerShell 中执行：

~~~powershell
cd E:\RM27\PYRo-wl-python
$env:PYTHONPATH = "src"
python -m wheel_leg_lqr.gas_spring_compensation --poses data\solidworks\poses.csv --output data\generated\gas_spring_compensation.csv --header E:\RM27\PYRo-wl\Robot\infantry1\Chassis\gas_spring_compensation_table.h --force 300
~~~

参数含义：

~~~text
--poses       SolidWorks 姿态 CSV
--output      输出补偿表路径
--force       恒定气弹簧力，默认 300 N
--header      可选，输出供 STM32 包含的 C++ 查表头文件
~~~

#### 5.3.4 把 Python 表放进控制代码

STM32 不能在运行时打开 `gas_spring_compensation.csv`。正确的使用方式是把
Python 的离线结果转换成 C++ 常量数组并随固件编译：

~~~powershell
cd E:\RM27\PYRo-wl-python
$env:PYTHONPATH = "src"
python -m wheel_leg_lqr.gas_spring_compensation `
  --poses data\solidworks\poses.csv `
  --output data\generated\gas_spring_compensation.csv `
  --header E:\RM27\PYRo-wl\Robot\infantry1\Chassis\gas_spring_compensation_table.h `
  --force 300
~~~

生成的头文件包含左右腿各自的 `L[]`、`BETA[]`、`F_GAS_L[]` 和
`T_GAS_BETA[]`。数组按每个 `L` 下依次排列所有 `beta`，固件以
`index = i_L * beta_count + i_beta` 访问。`pyro_wl_chassis.cpp` 中的查表函数先
对 `L`、`beta` 分别限幅并找到相邻节点，再进行双线性插值。超出 SolidWorks
工作区间时使用最近边界值，不做外推。

补偿调用链为：

~~~text
_balance_control()
  原始 LQR 输出 out_F_L / out_T_p
        ↓ 查表当前 leg.current_leg_length、相对机身 beta
  gas_f_l / gas_t_p
        ↓
_vmc_trans_v2j()
  motor_f_l = out_F_L - gas_f_l + virtual_wall_force
  motor_t_p = out_T_p - gas_t_p
        ↓
  原有 tau_hip / tau_knee 映射和限幅
~~~

这里没有修改 `U0`、`BETA_TRIM` 或 LQR 增益。补偿只在平衡态的
`_balance_control()` 路径运行；手动、对齐、跨步和空中状态沿用原来的输出。
默认 `GAS_SPRING_COMPENSATION_ENABLE = false`，完成悬空状态符号检查后，把它改成
`true`。需要降低初始补偿强度时，可把 `GAS_SPRING_COMPENSATION_SCALE` 设为
`0.2f`、`0.5f` 等，确认后再逐步调到 `1.0f`。第一阶段只有 `beta = 0` 数据时，头文件中的 `T_GAS_BETA` 为零，只启用
`F_gas_L`；具有完整多 beta 网格后才启用摆角方向补偿。

如果已经执行过可编辑安装，也可以使用项目脚本：

~~~powershell
python -m pip install -e E:\RM27\PYRo-wl-python
wheelleg-gas-spring --poses data\solidworks\poses.csv --output data\generated\gas_spring_compensation.csv --force 300
~~~

#### 5.3.3 检查输出

重点检查：

1. s 与 s_check 的误差是否在允许范围内；
2. F_s 是否每行都是 300 N；
3. F_gas_L 的正负是否符合悬空状态下气弹簧的实际伸缩方向；
4. 第一阶段 beta = 0 时只使用 F_gas_L；
5. 有完整 beta 网格后，再检查 T_gas_beta 是否需要接入 T_p。

专项测试命令：

~~~powershell
cd E:\RM27\PYRo-wl-python
python -m pytest -p no:cacheprovider -q tests/test_gas_spring.py
~~~

### 5.4 当前 Python 代码实现状态

Python 已经改成基于安装点相对坐标和相对运动雅可比的接口：

~~~text
输入：dx、dz、L、beta
中间量：s、e_s_x、e_s_z、d_dx_dL、d_dz_dL、d_dx_dbeta、d_dz_dbeta
输出：ds_dL、ds_dbeta、F_gas_L、T_gas_beta
~~~

当前 Python 核心接口：

~~~python
spring_length_from_relative_coordinates(dx, dz)
    由 A 到 B 的相对坐标差计算 s

spring_direction_from_relative_coordinates(dx, dz)
    计算 A 到 B 的单位方向 e_s

project_gas_force(...)
    用 e_s 和相对安装点雅可比计算 F_gas_L、T_gas_beta

build_compensation_table(samples, ConstantGasSpringModel())
    组织 L-beta 网格并生成完整补偿结果

compensate_leg_length_command(F_L_base, F_gas_L)
    返回 F_L_motor = F_L_base - F_gas_L

compensate_leg_angle_command(T_p_base, T_gas_beta)
    返回 T_p_motor = T_p_base - T_gas_beta
~~~

`ConstantGasSpringModel()` 默认返回 300 N；输出中的 `is_L_boundary` 和 `is_beta_boundary` 用于标记有限差分边界，便于后续查表时单独处理边界点。

已同步更新：

~~~text
E:\RM27\PYRo-wl-python\src\wheel_leg_lqr\core\gas_spring.py
E:\RM27\PYRo-wl-python\src\wheel_leg_lqr\gas_spring_compensation.py
E:\RM27\PYRo-wl-python\tests\test_gas_spring.py
E:\RM27\PYRo-wl-python\README.md
~~~

测试已覆盖相对坐标算长、气弹簧方向、相对雅可比投影、恒力 300 N、虚功残差、CSV 读写和命令行入口。旧的绝对安装点坐标、固定半径、角度和 `geometry.csv` 输入不再作为正式数据格式。

## 6. 控制器接入步骤

### 6.1 保持现有重力配平

保留现有：

~~~text
U0
BETA_TRIM
LQR 增益调度
~~~

气弹簧补偿作为单独的 VMC 外力加入。不要把 F_gas_L 直接永久加进 U0，否则会在控制器其他路径中重复计算。

### 6.2 第一阶段只补偿腿长方向

在 beta = 0 数据确认后，先使用：

~~~text
F_L_motor = F_L_LQR - F_gas_L
T_p_motor = T_p_LQR
~~~

随后照常使用原有 F_L、T_p 到关节力矩的转换。

### 6.3 第二阶段按需补偿摆角方向

完成多 beta 姿态数据并确认 T_gas_beta 后，再使用：

~~~text
F_L_motor = F_L_LQR - F_gas_L
T_p_motor = T_p_LQR - T_gas_beta
~~~

补偿查表应按当前 side、L、beta 插值，并设置工作区间外的限幅和禁用逻辑。第一版只在正常平衡 LQR 状态启用，空中、起身、跨步和被动状态保持原逻辑。

计划修改的固件位置：

~~~text
Robot/infantry1/Chassis/wl_config.h
Robot/infantry1/Chassis/pyro_wl_chassis.h
Robot/infantry1/Chassis/pyro_wl_chassis.cpp
~~~

增加恒力参数 F_s = 300 N、补偿查表或插值、调试量、补偿开关、渐变启用和关节力矩限幅。

## 7. 实施顺序

### 第一步：记录无气弹簧基线

记录不同腿长下现有控制器的：

~~~text
F_L_U0(L)
T_p_U0(L)
BETA_TRIM(L)
~~~

这些数据只作为补偿前基线，不重新叠加重力模型。

### 第二步：SolidWorks 建模和测量

按第 5.1 节建立 GS_BASE，确定大腿侧 A 点和轮连杆侧 B 点，建立 L-beta 配置，并导出 poses.csv。

### 第三步：Python 离线解算

运行命令生成补偿表，检查 s_error、导数连续性、左右腿符号和虚功残差。先只验证 F_gas_L，之后再验证 T_gas_beta。

### 第四步：悬空状态符号验证

关闭 LQR 或在安全的悬空条件下缓慢改变腿长，确认气弹簧实际帮助伸长时：

~~~text
F_gas_L > 0
F_L_motor 的补偿方向为负
~~~

若实物方向与计算相反，优先检查 A/B 点顺序、L 正方向和 beta 定义，不要直接翻转所有结果的符号。

### 第五步：低风险接入控制器

先只开启 F_L 补偿，使用较小的启用比例或渐变时间，检查腿长误差、电机力矩、电流和温升。确认稳定后再扩大工作区间。

### 第六步：评估 T_p 补偿

计算工作区间内：

~~~text
max(abs(T_gas_beta))
~~~

若该值相对 MAX_T_P 很小，可先不接入 T_p；否则再启用摆角补偿，并重新做关节力矩和俯仰稳定性测试。

## 8. 验收测试

### 8.1 几何和数值测试

- `dx、dz` 计算的 s 与 SolidWorks 的 s_check 误差在容许范围内；
- 气弹簧方向向量满足 `e_s_x^2 + e_s_z^2 = 1`；
- 相对安装点雅可比和端点单边差分结果连续、无异常尖峰；
- `ds/dq = e_s^T * J_rel` 的投影关系正确；
- 所有 s、导数和补偿力在工作区间内为有限值；
- 左右腿分别验证，不能只验证一侧后直接复制符号；
- 虚功关系满足：

~~~text
F_s * delta_s = F_gas_L * delta_L + T_gas_beta * delta_beta
~~~

### 8.2 静态配平测试

在 0.18 m <= L <= 0.38 m 的工作区间验证：

~~~text
F_L_motor + F_gas_L = F_L_base
T_p_motor + T_gas_beta = T_p_base
~~~

### 8.3 VMC 和电机测试

- 检查 F_L、T_p 到髋/膝力矩映射连续；
- 检查补偿开关和查表边界没有输出跳变；
- 检查补偿后的关节力矩、电流和温升；
- 检查 F_L、T_p 及关节力矩限幅；
- 检查不会引入明显腿摆角或机身俯仰漂移。

### 8.4 实机测试顺序

1. 关闭补偿，复核原始基线。
2. 只开启 F_L 补偿，低速改变腿长。
3. 检查腿长误差、电机力矩和电流。
4. 扩大到完整腿长工作区间。
5. 完成多 beta 数据后，再决定是否开启 T_p 补偿。

## 9. 最终数据流

~~~text
SolidWorks
  GS_BASE + A/B 安装点
        |
        | poses.csv: side,L,beta,dx,dz,s_check
        v
Python
  e_s + J_rel -> ds/dL, ds/dbeta -> F_gas_L, T_gas_beta
        |
        v
VMC
  F_L_motor = F_L_base - F_gas_L
  T_p_motor = T_p_base - T_gas_beta
        |
        v
原有髋/膝雅可比和电机控制
~~~

本计划的 SolidWorks 工作只负责得到每个姿态的安装点相对坐标差和中心距检查值；Python 负责全部数值求导、恒力映射、验证和补偿表生成；现有 U0、BETA_TRIM 和 LQR 调度保持不变。
