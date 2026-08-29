#ifndef PYRO_SHARED_DATA_H
#define PYRO_SHARED_DATA_H

#include "stdint.h"

union GimbalToChassisComm 
{

    __attribute__((packed)) struct 
    {
        uint32_t mode      : 2;//0下力，1手动(新遥控器下废除)，2平衡

        int32_t vx        : 6;
        int32_t w         : 6;

        uint32_t delta_leg : 2;//0不变，1增大，2减小
        uint32_t step_mode : 1;
        uint32_t spining   : 1;
        
    } msg;

    uint8_t buffer[8];
};



union ChassisToGimbalComm
{
    __attribute__((packed)) struct 
    {
        //未定义
        
    } msg;

    uint8_t buffer[8];
};

struct GimbalBoosterShared 
{
    //event: 
    // 0 NONE
    // 1 摩擦轮转换
    // 2 单发
    // 3 连发开始
    // 4 连发结束
    //事件优先级： 1 > 2 > 3 > 4 >0
    uint32_t event : 3;
    uint32_t mode  : 1;
};






#endif