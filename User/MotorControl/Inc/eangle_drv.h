#ifndef __EANGLE_DRV_H__
#define __EANGLE_DRV_H__

#include "main.h"

typedef struct
{
    uint8_t    Dir;                        // 编码器方向    
    uint8_t    PolePairs;                  // 转子极对数
    int32_t    EncoderVal;                 // 编码器原始数据
    int32_t    EncoderValMax;              // 编码器最大原始值
    int32_t    EncoderValChange;           // 编码器值变化量
    uint16_t   CalibFlag;                  // 校准完成标志
    int32_t    CalibOffset;                // 转子零位偏差    
    float      Ts;
    float      ElectricalAnglePU;          // 编码器电角度标幺值
    float      ElectricalAngleSpdSet;      // 给定的电角速度        
    float      ElectricalAngleSetPU;       // 给定电角度标幺值
}E_ANGLE_STRUCT; 

void Electrical_Angle_Generator(E_ANGLE_STRUCT *p);
void Calculate_Encoder_Data(E_ANGLE_STRUCT *p);

#endif
