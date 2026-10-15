# 前向滤波器与 KF_ok 实测版本对齐

基准：本地分支 `KF_ok`，提交 `19ea989ca62feaecf5bddbaaf51ead674fa613a8`。
当前继续使用通用 KF/Matrix 库，不恢复旧回调或动态分配。

## 对比与修改

| 项目 | 修改前 | KF_ok 与当前实现 |
| --- | --- | --- |
| 模板维度 | lkf_t<4,3> | lkf_t<4,2> |
| 观测 | IMU、模型加速度、轮速 | IMU 加速度、轮速 |
| 滑移目标 | 模型参考置信度 × (g_v+g_a)/2 | (g_v+g_a)/2 |
| 加速度起评/满评分 | 1 / 4 m/s² | 0.30 / 0.60 m/s² |
| 上升/恢复时间常数 | 3 / 30 ms | 2 / 30 ms |
| 零偏保护 | 无 | 轮速无效或滑移评分 >=0.2 时冻结 K 的零偏行 |
| update 参数 | a_imu,a_model,v_wheel,valid | a_imu,v_wheel,valid |

F、每拍过程噪声、P0、最小方差、Rimu、基础 Rwheel、15 Hz 微分器、NIS 阈值和
最大 500 倍 R 均与 KF_ok 相同。保持 INVERSE、Joseph 及其增益缩放时序。
模型加速度计算保留在当前底盘中供诊断/其他控制逻辑使用，不进入前向 KF。
没有整体合并 KF_ok 的跳跃、过坎、LQR、RDOB、模型系数或其他状态机修改。

## 零偏冻结

旧 `_freeze_bias_update(KalmanFilter_t*)` 在 K 计算完成后、状态及 Joseph 更新前
清零第 3 行。新实现通过 correct 的 Policy 清零 `correction.K(3,0/1)`，
返回 PYRO_OK 后核心用调整后的 K 重新计算 dx，并更新 Joseph 协方差。

因此冻结期间 bias 状态保持预测值，bias 方差不因不可信观测而缩小；加速度等状态
仍正常融合 IMU。轮速可信且评分降至 0.2 以下时，自然恢复零偏学习。
失效轮速用 mask 排除，合法零轮速仍融合；重连避免微分冲击，reset 清除评分和历史。

## 验证

冻结的 `tests/forward_kf/reference/forward_kf_reference.cpp/.h` 来自上述 KF_ok 提交，
只改测试标识、类型名和 include；参数独立冻结在 `reference/forward_kf_config.h`，
旧 KF 后端与 KF_ok 相同（仅增加 test-only 注释），不使用当前参数冒充实测基线。

normal/stress 各 30,000 拍，每拍比较 33 个 float32：5 个估计输出、16 个 P、
8 个 K、4 个 R。stress 包含打滑、IMU 突变、NaN/Inf/离线轮速、恢复和多次重置。
冻结条件成立的每一拍额外断言：零偏 K 行为零、bias 不变、bias 方差不缩小。

| 最大绝对差 | normal | stress |
| --- | ---: | ---: |
| x / m | 7.5669959e-10 | 2.3841858e-7 |
| v、a、bias、slip_score | 0 | 0 |
| P 元素 | 4.9476512e-10 | 3.7252903e-9 |
| K 元素 | 6.5192580e-9 | 5.4016709e-8 |
| R 元素 | 0 | 0 |

13 项 CTest 通过，含独立 double 参考、静态零偏收敛、打滑与失联零偏冻结、
恢复后继续学习、评分尾部、协方差正定/对称/下限、优化构建及底盘控制接线。
独立 double 参考最大状态差约 0.000323657，原有判据未放宽。
INS 的两组各 20,000 拍逐字节回归仍通过。infantry1 ARM Debug 构建通过。

新核心预测阶段的对称化/方差下限及 mask 压缩会产生小量数值差异，故没有要求
整个轨迹逐位一致；本次是有限样本的软件回归，没有重新刷机或实测闭环。

```powershell
cmake --build build/forward-kf-tests --parallel 4
ctest --test-dir build/forward-kf-tests --output-on-failure -V
cmake --build cmake-build-debug-stm32 --parallel 4
```

逐拍结果位于 `build/forward-kf-tests/forward-trace-{normal,stress}/comparison.json`。
