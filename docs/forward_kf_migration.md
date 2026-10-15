# infantry1 前向滤波器迁移记录

2026-10-16。前向估计器已迁移到 `lkf_t<4,3>`；本阶段未刷机、未重新调参。
INS 已由用户确认验证，本次保留其生产实现。

## 实现

- 状态仍为 `[x,v,a,imu_acc_bias]`，量测仍为 `[a_imu,a_model,v_wheel]`。
- 保留 F/H、每拍 Q_d、P0、基础 R、方差下限，以及 Joseph / INVERSE 配置。
- 每拍执行 `predict → _adjust_wheel_r → correct`。滑移评分使用本拍预测 x/P，
  单条轮速 NIS 的分母仍为 `P-(1,1)+R_wheel_base`；不使用联合 NIS 或当前放大的 R。
- 速度/加速度一致性判断、参考置信度、3 ms 上升 / 30 ms 恢复、500 倍 R 上限保持。
- 轮速无效时用 mask 排除轮速行，仍融合两条加速度；合法零速参与融合。
  重连后不产生微分冲击，reset 清除微分历史、滑移评分和自适应 R。
- 保留原 init 的零速初值语义，重复 init 不改变运行状态；所有状态/矩阵固定存储。
- 底盘接口 `init/reset/update` 和估计输出保持，底盘控制时序与 IMU 映射未改。
  诊断通过 `get_filter()` 的新库 getter 读取；R 通过 `get_measurement_noise()` 读取。

## 验证

修改前的前向估计器和旧 KF 后端冻结在 `tests/forward_kf/reference`，仅测试链接。
冻结前向类只重命名类、估计结构和 include guard，数学实现及参数不改。

13 项 CTest 全部通过，覆盖：

- 静止合法零量测 20,000 拍，检查状态、对称性、正定性和方差下限。
- 静态加速度零偏收敛 15,000 拍；正弦运动与独立 double KF 对照 30,000 拍，
  最大状态差约 0.00033379，原有误差判据保持。
- 单拍轮速脉冲、本拍连续降权、评分恢复尾部和 IMU/模型冲突下的参考置信度。
- NaN/Inf/离线轮速屏蔽、重连恢复、重置和重复 init。
- 求逆失败保留预测及下一拍恢复、Z>N 工作区。
- O3 + unsafe-math + finite-math 构建下的相同前向行为检查。
- balance/spin 的实际软件控制接线：当前模型输入、KF、RDOB/LQR 和其他状态保持。
- 原有 INS 功能测试和两组各 20,000 拍的姿态新旧逐字节回归仍通过。

另对前向 normal/stress 各 30,000 拍逐拍比较 42 个 float32：
`x/v/a/bias/slip_score`、16 个 P、12 个 K、9 个 R。
stress 包括运动、噪声、周期打滑脉冲、IMU/模型冲突、NaN/Inf/离线轮速、恢复及多次重置。

| 主机对照最大绝对差 | normal | stress |
| --- | ---: | ---: |
| x / m | 1.7138291e-11 | 2.8610229e-6 |
| v、a、bias、slip_score | 0 | 0 |
| P 元素 | 1.3284485e-11 | 1.8626451e-9 |
| K 元素 | 2.6775524e-9 | 5.5413693e-8 |
| R 元素 | 0 | 0 |

未强求整体逐位一致：新核心在预测阶段也进行方差下限/对称化；缺测从旧零 H 行改为
真正压缩有效观测；对称化的浮点分组也可能不同。这些小差异已由独立参考及回放量化。
回放阈值写在比较脚本中，不是机器人的物理精度指标或全输入误差上界。

ARM infantry1 Debug 构建通过；DTCM 使用 76,904 B / 128 KiB，Flash 217,712 B / 1 MiB。
没有做本次前向估计的板上闭环、周期或任务栈高水位测试。

## 清理

- `PYRo/Algorithm/KF/kalman_filter.cpp/.h` 从生产目录移至测试 reference，移除生产 CMake 引用。
- 删除无调用的 `pyro_kalman_filter.cpp/.h`，消除与通用 `kf_t` 的同名类冲突。
- 保留 `kf.cpp/.h`：`Robot/Infantry2` 仍明确引用并编译它们；其文档标明旧版适用范围。
- 保留旧 QEKF / 前向 / KF 后端的冻结参考以便回归；它们不属于固件链接内容。
- 更新旧 IMU 汇编/机器码检验脚本的后端路径，清理后仍可复现。

## 复现

```powershell
cmake --build build/forward-kf-tests --parallel 4
ctest --test-dir build/forward-kf-tests --output-on-failure -V
cmake --build cmake-build-debug-stm32 --parallel 4
```

逐拍比较脚本：`tests/forward_kf/compare_forward_trace.py`。
机器可读结果：`build/forward-kf-tests/forward-trace-{normal,stress}/comparison.json`。
