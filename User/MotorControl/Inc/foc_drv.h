#ifndef __FOC_DRV_H__
#define __FOC_DRV_H__

#include "main.h"

typedef struct
{        
    float Iu;            // U相电流  
    float Iv;            // V相电流 
    float Iw;            // W相电流     
    float Ialpha;        // alpha轴电流 
    float Ibeta;           // beta轴电流 

    float SinVal;        // 正弦值
    float CosVal;        // 余弦值
    float Id;               // d轴电流 
    float Iq;               // q轴电流 

    float IdLPF;            // d轴电流滤波值
    float IqLPF;              // q轴电流滤波值
    float IdLPFFactor;      // d轴电流滤波系数
    float IqLPFFactor;      // q轴电流滤波系数    

    float Ud;            // d轴电压 
    float Uq;            // q轴电压     
    float Ualpha;        // alpha轴电压
    float Ubeta;         // beta轴电压        
    float Ubus;          // 母线电压    

    uint16_t   PwmCycle;      // PWM周期
    uint16_t   PwmLimit;      // 最大占空比
    uint16_t   DutyCycleA;    // A相占空比
    uint16_t   DutyCycleB;    // B相占空比
    uint16_t   DutyCycleC;    // C相占空比    
}FOC_STRUCT;

void Clark_Transform(FOC_STRUCT *p);
void Park_Transform(FOC_STRUCT *p); 
void IPark_Transform(FOC_STRUCT *p);
void Calculate_SVPWM(FOC_STRUCT *p);
#endif
