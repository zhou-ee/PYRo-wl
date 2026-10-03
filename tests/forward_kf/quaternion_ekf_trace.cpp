// Replay identical mixed-axis IMU inputs against the original and updated
// library.
#ifdef QEKF_REFERENCE
#include "reference/quaternion_ekf_reference.h"
#else
#include "pyro_ins_attitude.h"
#endif
#include <cmath>
#include <cstdio>

int main(int argc, char **argv)
{
    if (argc != 3)
        return 1;
    FILE *trace = std::fopen(argv[1], "wb");
    if (trace == nullptr)
        return 1;
#ifdef QEKF_REFERENCE
    auto &attitude = QEKF_INS;
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 1000000.0f,
                           (argv[2][0] == 'f' ? 0.9996f : 1.0f), 0.01f);
#else
    pyro::ins_attitude_t attitude;
    attitude.init(10.0f, 0.001f, 1000000.0f,
                  (argv[2][0] == 'f' ? 0.9996f : 1.0f), 0.01f);
#endif
    for (int i = 0; i < 20000; ++i)
    {
        const float t         = i * 0.001f;
        const float theta     = 0.35f * std::sin(1.3f * t);
        const float phi       = 0.15f * std::sin(0.7f * t);
        const float theta_dot = 0.455f * std::cos(1.3f * t);
        const float phi_dot   = 0.105f * std::cos(0.7f * t);
        const float yaw_dot   = 0.2f * std::cos(0.3f * t);
        const float gx        = phi_dot - yaw_dot * std::sin(theta) + 0.002f;
        const float gy        = theta_dot * std::cos(phi) +
                         yaw_dot * std::cos(theta) * std::sin(phi) - 0.001f;
        const float gz = -theta_dot * std::sin(phi) +
                         yaw_dot * std::cos(theta) * std::cos(phi);
        const float ax = -9.81f * std::sin(theta) + 0.15f * std::sin(4.0f * t);
        const float ay = 9.81f * std::cos(theta) * std::sin(phi);
        const float az = 9.81f * std::cos(theta) * std::cos(phi);
        const bool varied   = argv[2][0] == 'f';
        const float dt      = varied ? (i % 2 ? 0.0011f : 0.0009f) : 0.001f;
        const float impulse = varied && i % 2000 < 30 ? 6.0f : 0.0f;
#ifdef QEKF_REFERENCE
        IMU_QuaternionEKF_Update(gx, gy, gz, ax + impulse, ay, az, dt);
#else
        attitude.update(gx, gy, gz, ax + impulse, ay, az, dt);
#endif
        for (float value : attitude.q)
            if (!std::isfinite(value))
                return 2;
        std::fwrite(attitude.q, sizeof(float), 4, trace);
        std::fwrite(attitude.GyroBias, sizeof(float), 3, trace);
        std::fwrite(&attitude.Roll, sizeof(float), 3, trace);
#ifdef QEKF_REFERENCE
        std::fwrite(attitude.IMU_QuaternionEKF.xhat_data, sizeof(float), 6,
                    trace);
        std::fwrite(attitude.IMU_QuaternionEKF.P_data, sizeof(float), 36,
                    trace);
#else
        std::fwrite(attitude.ekf.get_state().data(), sizeof(float), 6, trace);
        std::fwrite(attitude.ekf.get_covariance().data(), sizeof(float), 36,
                    trace);
#endif
    }
    std::fclose(trace);
    return 0;
}
