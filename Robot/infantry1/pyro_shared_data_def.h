#ifndef PYRO_SHARED_DATA_H
#define PYRO_SHARED_DATA_H

#include "stdint.h"

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