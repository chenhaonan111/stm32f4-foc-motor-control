#ifndef __EANGLE_DRV_H__
#define __EANGLE_DRV_H__

#include "main.h"
#include "pll_drv.h"
#include "filter_drv.h"

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

    ENC_PLL_STRUCT EncPll;                 // 编码器PLL（角度跟踪锁相环）实例
    float      SpeedEmaAlpha;              // PLL速度输出的一阶低通系数（后向欧拉EMA，α≈2π·fc·TS，fc≈32Hz）
    float      ElectricalAngleRawPU;       // 原始直读电角度标幺值（A/B对比用）
    float      ThetaMechRad;               // PLL输入机械角度（rad，调试观察用）
    float      EncSpeedElecRPM;            // PLL速度输出（电角度rpm，与速度环单位一致）
}E_ANGLE_STRUCT;

void Electrical_Angle_Generator(E_ANGLE_STRUCT *p);
void Calculate_Encoder_Data(E_ANGLE_STRUCT *p);
void Calculate_Encoder_Pll(E_ANGLE_STRUCT *p);

#endif
