#ifndef __MOTOR_IDENTIFY_H__
#define __MOTOR_IDENTIFY_H__

#include "main.h"

typedef enum{
    RESISTANCE_IDENTIFICATION = 0,                            // 相电阻识别
    INDUCTANCE_IDENTIFICATION,                                // 相电感识别
    ENCODER_ROTOR_ALIGN                                        // 编码器对齐
}IDENTIFY_STATE;

typedef struct
{    
    IDENTIFY_STATE    State;
    uint8_t           Flag;
    uint8_t           EndFlag;           // 辨识完成标志    
    uint16_t          Count;             // 计数
    uint16_t          WaitTim;           // 等待时间
    float             Rs;                // 相电阻
    float             Ls;                // 相电感
    float             Lq;                // q轴电感
    float             Ld;                // d轴电感    
    float             Flux;              // 磁链
    float             LsSum;             // 电感累计值
    float             CurMax;            // 最大识别电流
    float             CurSum;            // 电流累计值
    float             CurAverage[2];     // 电流平均值
    float             VoltageSet[2];     // 电压给定值
}IDENTIFY_STRUCT;

void Motor_Identify(void);

#endif
