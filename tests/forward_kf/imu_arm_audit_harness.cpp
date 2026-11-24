// ARM-only audit harness: old and new algorithms share the same CMSIS kernels.
#include "reference/quaternion_ekf_reference.h"
#include "pyro_ins_attitude.h"
#include <cstddef>
#include <cstring>

pyro::ins_attitude_t audit_current;
extern float IMU_QuaternionEKF_P[36];
extern "C"
{
float audit_input[7];
float audit_config[5] = {10.0f, 0.001f, 1000000.0f, 1.0f, 0.01f};
float audit_snapshot[73]; // Existing 52-field trace + NIS, gains, scale, convergence.
float audit_aux[12]; // Accel, Gyro, orientation, gyro norm, accel norm, dt.
unsigned int audit_flags[4];
unsigned int audit_gate_state[2]; // ErrorCount, ConvergeFlag.
float audit_threshold = 1.0e-8f;
alignas(8) unsigned char audit_heap[32768];
std::size_t audit_heap_used;

void *__wrap_malloc(std::size_t bytes)
{
    const auto size = (bytes + 7u) & ~std::size_t(7u);
    if (size > sizeof(audit_heap) - audit_heap_used)
        return nullptr;
    void *result = audit_heap + audit_heap_used;
    audit_heap_used += size;
    return result;
}

void audit_enable_fpu()
{
    volatile unsigned int &cpacr = *reinterpret_cast<volatile unsigned int *>(0xE000ED88u);
    cpacr |= 0x00F00000u;
    asm volatile("dsb\nisb" ::: "memory");
}

void audit_init()
{
    audit_heap_used = 0;
    std::memset(audit_heap, 0, sizeof(audit_heap));
    std::memset(&QEKF_INS, 0, sizeof(QEKF_INS));
    // The legacy observer overwrites its global initialization buffer each tick.
    // Restore it so independent scenarios start from the same covariance.
    for (std::size_t row = 0; row < 6; ++row)
        for (std::size_t col = 0; col < 6; ++col)
            IMU_QuaternionEKF_P[row * 6 + col] =
                row == col ? (row < 4 ? 100000.0f : 100.0f) : 0.1f;
    audit_current = pyro::ins_attitude_t{};
    IMU_QuaternionEKF_Init(audit_config[0], audit_config[1], audit_config[2],
                           audit_config[3], audit_config[4]);
    audit_current.init(audit_config[0], audit_config[1], audit_config[2],
                       audit_config[3], audit_config[4]);
}

void audit_seed_gate()
{
    QEKF_INS.ErrorCount = audit_current.ErrorCount = audit_gate_state[0];
    QEKF_INS.ConvergeFlag = audit_current.ConvergeFlag = audit_gate_state[1];
    QEKF_INS.ChiSquareTestThreshold = audit_current.ChiSquareTestThreshold = audit_threshold;
    // Seed identical prior policy history for synthetic branch probes.
    if (audit_gate_state[1])
        QEKF_INS.AdaptiveGainScale = audit_current.AdaptiveGainScale = 1.0f;
}

void audit_tick_old()
{
    IMU_QuaternionEKF_Update(audit_input[0], audit_input[1], audit_input[2],
                             audit_input[3], audit_input[4], audit_input[5], audit_input[6]);
    std::memcpy(audit_snapshot, QEKF_INS.q, 4 * sizeof(float));
    std::memcpy(audit_snapshot + 4, QEKF_INS.GyroBias, 3 * sizeof(float));
    audit_snapshot[7] = QEKF_INS.Roll;
    audit_snapshot[8] = QEKF_INS.Pitch;
    audit_snapshot[9] = QEKF_INS.Yaw;
    std::memcpy(audit_snapshot + 10, QEKF_INS.IMU_QuaternionEKF.xhat_data, 6 * sizeof(float));
    std::memcpy(audit_snapshot + 16, QEKF_INS.IMU_QuaternionEKF.P_data, 36 * sizeof(float));
    audit_snapshot[52] = QEKF_INS.ChiSquare_Data[0];
    std::memcpy(audit_snapshot + 53, QEKF_INS.IMU_QuaternionEKF.K_data, 18 * sizeof(float));
    audit_snapshot[71] = QEKF_INS.AdaptiveGainScale;
    audit_snapshot[72] = QEKF_INS.YawTotalAngle;
    audit_flags[0] = QEKF_INS.ConvergeFlag;
    audit_flags[1] = QEKF_INS.StableFlag;
    audit_flags[2] = static_cast<unsigned int>(QEKF_INS.ErrorCount);
    audit_flags[3] = QEKF_INS.IMU_QuaternionEKF.SkipEq5;
    std::memcpy(audit_aux, QEKF_INS.Accel, 3 * sizeof(float));
    std::memcpy(audit_aux + 3, QEKF_INS.Gyro, 3 * sizeof(float));
    std::memcpy(audit_aux + 6, QEKF_INS.OrientationCosine, 3 * sizeof(float));
    audit_aux[9] = QEKF_INS.gyro_norm;
    audit_aux[10] = QEKF_INS.accl_norm;
    audit_aux[11] = QEKF_INS.dt;
}

void audit_tick_new()
{
    const auto status = audit_current.update(audit_input[0], audit_input[1], audit_input[2],
                                            audit_input[3], audit_input[4], audit_input[5], audit_input[6]);
    std::memcpy(audit_snapshot, audit_current.q, 4 * sizeof(float));
    std::memcpy(audit_snapshot + 4, audit_current.GyroBias, 3 * sizeof(float));
    audit_snapshot[7] = audit_current.Roll;
    audit_snapshot[8] = audit_current.Pitch;
    audit_snapshot[9] = audit_current.Yaw;
    std::memcpy(audit_snapshot + 10, audit_current.ekf.get_state().data(), 6 * sizeof(float));
    std::memcpy(audit_snapshot + 16, audit_current.ekf.get_covariance().data(), 36 * sizeof(float));
    audit_snapshot[52] = audit_current.ChiSquare_Data[0];
    std::memcpy(audit_snapshot + 53, audit_current.ekf.get_gain().data(), 18 * sizeof(float));
    audit_snapshot[71] = audit_current.AdaptiveGainScale;
    audit_snapshot[72] = audit_current.YawTotalAngle;
    audit_flags[0] = audit_current.ConvergeFlag;
    audit_flags[1] = audit_current.StableFlag;
    audit_flags[2] = static_cast<unsigned int>(audit_current.ErrorCount);
    audit_flags[3] = status == pyro::PYRO_WARNING;
    std::memcpy(audit_aux, audit_current.Accel, 3 * sizeof(float));
    std::memcpy(audit_aux + 3, audit_current.Gyro, 3 * sizeof(float));
    std::memcpy(audit_aux + 6, audit_current.OrientationCosine, 3 * sizeof(float));
    audit_aux[9] = audit_current.gyro_norm;
    audit_aux[10] = audit_current.accl_norm;
    audit_aux[11] = audit_current.dt;
}
}
