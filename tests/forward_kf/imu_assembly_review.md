# 新旧 IMU 算法 ARM 汇编审阅

审阅日期：2026-10-06。

本文记录静态审阅；后续的 ARM 机器码模拟与数学数据流比较见
[imu_arm_math_review.md](imu_arm_math_review.md)。后续已发现 unsafe math 配置的
实际舍入差异，并完成严格浮点配置的有限样本逐位比较。

结论：新旧实现不具备逐指令一致性。正常输入的数学公式及所用 CMSIS-DSP
运算保持迁移设计，但不能据此断言 ARM 浮点输出逐位一致或执行时间相同。
现有主机回放逐字节一致的结论，仅适用于其实际编译配置和输入样本。

## 比较范围与配置

- 新实现：`pyro_ins_attitude.cpp`、实例化的 `kf_t<6,3,false>`、矩阵后端。
- 旧实现：`reference/quaternion_ekf_reference.cpp`、`kalman_filter.cpp`。
  这是冻结的迁移基线；其中平方根倒数的位重解释已使用 `memcpy`，并非任意旧固件二进制。
- 编译器：Arm GNU Toolchain 15.3.Rel1，GCC 15.3.1 20260627。
- 从当前 STM32 编译数据库提取同一组宏、包含目录及编译选项。
- 目标：`-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb`。
- 当前 Debug 的 `-O0`，以及额外的 `-O3`；两者均保留项目的
  `-funsafe-math-optimizations`，未额外禁用浮点融合。
- 使用未链接 ARM 对象的 `objdump -drC`；没有运行 ARM 目标或刷写设备。

## 观察结果

1. `-O0` 下，快速平方根倒数函数的新/旧静态指令数分别为 31/37。
   二者均为 4 次 `vmul.f32` 和 1 次 `vsub.f32`，这些浮点算术指令的顺序和
   操作数一致；完整指令序列不同，主要区别是局部变量的加载、存储和返回。
   魔数仍为 `0x5f375a86`。
2. 新实现通过固定存储对象、模板核心及矩阵视图调用后端，旧实现通过全局对象、
   动态分配缓冲区及函数指针回调运行。相应地址偏移、调用及分支无法逐条对应。
   新实现还包含有效观测索引维护、矩阵复制、诊断和失败时保留原状态的逻辑。
3. 新实现的未检查核心，在 `-O0/-O3` 汇编的直接调用中都未发现
   `is_finite` 或 `_covariance_valid`。IMU 应用层自身的输入检查仍存在。
   该项是调用检查，不是对所有指令进行语义证明。
4. 正常 IMU 配置使用 INVERSE / SIMPLIFIED：通过矩阵后端调用同一 CMSIS
   乘法、转置、求逆、加减内核；NIS 的公式仍为 `yᵀ S⁻¹ y`。
   新核心还生成了运行时可选的 Cholesky / Joseph 分支，它们不属于当前 IMU
   正常配置的执行路径。
5. `-O3` 下发现 `vfma.f32`、`vfms.f32` 等融合指令。新旧函数的拆分、内联及
   浮点表达式优化结果不同，不能使用 `-O0` 的局部观察证明优化版逐位一致。
6. 两个主机回放测试重新通过：normal、fading 各 20,000 个样本，每个样本
   52 个 float32，覆盖四元数、零偏、欧拉角、状态与协方差，逐字节一致。
   fading 包含变周期、渐消因子及加速度脉冲。这些测试使用主机配置中的
   `-ffp-contract=off -fno-strict-aliasing`，不等同于当前 ARM 编译配置。

## 对象大小，仅作为静态生成规模参考

下表是 GNU `size` 的 text 列，包含代码和只读数据等；未链接、未进行最终
垃圾回收，且包括可选分支和辅助函数。不能当作固件增量或执行周期。
两组均未重复统计共享 CMSIS 内核。

| 优化级别 | 新姿态对象 | 新矩阵后端 | 旧姿态对象 | 旧 KF 对象 |
| --- | ---: | ---: | ---: | ---: |
| O0 | 23,876 B | 3,306 B | 5,284 B | 5,052 B |
| O3 | 10,280 B | 2,564 B | 2,952 B | 3,120 B |

## 复现

在仓库根目录执行：

```powershell
python tests/forward_kf/audit_imu_assembly.py cmake-build-debug-stm32/compile_commands.json build/imu-assembly-audit
ctest --test-dir build/forward-kf-tests -R quaternion_trace --output-on-failure -V
```

脚本输出各 ARM 对象的完整反汇编、编译命令及 `audit.json`。
静态审阅已完成；ARM 输出逐位一致性与实际周期需要在同一目标上使用相同输入
进行运行对比，本次没有获得这两项运行证据。
