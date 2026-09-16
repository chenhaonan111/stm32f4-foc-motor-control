#ifndef __PLL_DRV_H__
#define __PLL_DRV_H__

#include "main.h"

typedef struct{
    float ErrPrev;          // (数据)上一拍误差 e[k-1]
    float WmPrev;           // (数据)上一拍角速度输出 ωm[k-1]
    float PiNum[2];         // (中间量)Tustin离散PI分子系数 [0]=Kp+Ki*T/2, [1]=-Kp+Ki*T/2
    float IntegCoeff;       // (中间量)梯形积分系数(T/2)

    float Error;            // (输入数据)角度误差输入(rad, 编码器: θm-OutThetaM, 已归一到[-π,π))
    float OutWm;            // (输出数据)机械角速度输出(rad/s)
    float OutThetaM;        // (输出数据)机械角度输出(rad), ×PolePairs得电角度
}PLL_GO_STRUCT;

typedef struct{
    PLL_GO_STRUCT go;       // (结构体)锁相环运算数据

    float T;                // (系统时钟)T运算离散周期
    
    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式(1=OutThetaM持续累计不归一化; 0=每拍归一化到[0,2π))
}ENC_PLL_STRUCT;

void ENC_PLL_Init(ENC_PLL_STRUCT *pll);
void ENC_PLL_Loop(ENC_PLL_STRUCT *pll);

#endif

