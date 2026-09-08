#ifndef PYRO_ROBOT_GIMBAL_CONFIG_H
#define PYRO_ROBOT_GIMBAL_CONFIG_H

namespace pyro 
{


//手动模式下------------------------------------------

//yaw轴速度环pid参数
constexpr float YAW_SPEED_PID_KP = 3.0f;
constexpr float YAW_SPEED_PID_KI = 0.0f;
constexpr float YAW_SPEED_PID_KD = 0.0f;

//yaw轴位置环pid参数
constexpr float YAW_POS_PID_KP = 2.5f;
constexpr float YAW_POS_PID_KI = 0.0f;
constexpr float YAW_POS_PID_KD = 0.0f;


//pitch轴位置环pid
constexpr float DM_POS_PITCH_KP = 1.5f;
constexpr float DM_POS_PITCH_KD = 0.0f;

//pitch轴速度环pid
constexpr float DM_SPD_PITCH_KP = 2.5f;
constexpr float DM_SPD_PITCH_KD = 0.0f;


// //pitch轴达妙mit控制阻抗系数
// constexpr float DM_MOT_PITCH_KP = 25.0f;
// constexpr float DM_MOT_PITCH_KD = 0.7f;

//pitch轴物理限幅参数
constexpr float PITCH_LIMIT_MAX = 2.0f;
constexpr float PITCH_LIMIT_MIN = 3.0f;


//复位角度设置
constexpr float PITCH_ALIGN_TARGET_RAD = 2.0f;
constexpr float YAW_ALIGN_TARGET_RAD   = 0.884f;

constexpr float PITCH_K_GRAVITY_COS = -0.8f; // 水平方向质心补偿
constexpr float PITCH_K_GRAVITY_SIN = 0.0f; // 垂直方向质心补偿




}

#endif
