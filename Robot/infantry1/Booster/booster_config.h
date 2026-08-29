#ifndef PYRO_ROBOT_BOOSTER_CONFIG_H
#define PYRO_ROBOT_BOOSTER_CONFIG_H

#include "pyro_core_def.h"

namespace pyro
{

//拨弹盘速度环pid参数
constexpr float TRIGGER_SPEED_PID_KP = 0.16f;
constexpr float TRIGGER_SPEED_PID_KD = 0.0f;

//拨弹盘位置环pid参数
constexpr float TRIGGER_POS_PID_KP = 2300.0f;
constexpr float TRIGGER_POS_PID_KD = 0.0002f;

//摩擦轮速度环pid参数
constexpr float FRIC_SPEED_PID_KP = 0.3f;
constexpr float FRIC_SPEED_PID_KD = 0.00002f;


// 弹丸初速度
constexpr float PROJECTILE_TARGET_MUZZLE_VELOCITY                 = 23.5f;

// 弹速调整系数
constexpr float FRIC_ADJUST_K                                     = 0.81f;
// 摩擦轮半径
constexpr float FRIC_RADIUS                                       = 0.03f;
//摩擦轮速度
constexpr float FRIC_TARGET_SPEED = PROJECTILE_TARGET_MUZZLE_VELOCITY / FRIC_RADIUS * FRIC_ADJUST_K;





// 发射速度 (发/秒)
constexpr float SHOOT_SPEED                                       = 10.0f;

// 拨弹盘速度
constexpr float TRIGGER_SPEED        = SHOOT_SPEED / 8 * 2 * PI * 36;


//云台差的东西
//1.热量控制
//2.弹速闭环
//3.自瞄控制
//4.ui绘制
//5.功率控制
}
#endif