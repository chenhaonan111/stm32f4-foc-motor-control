#ifndef __FILTER_DRV_H__
#define __FILTER_DRV_H__

#include "main.h"

/*二阶巴特沃斯低通滤波器（Tustin双线性离散）*/

typedef struct
{
    float i[2];             // 历史输入值（i[0]为上一次，i[1]为上上次）
    float o[2];             // 历史输出值（o[0]为上一次，o[1]为上上次）

    float Input;            // 本次输入
    float Output;           // 本次输出

    float num[2];           // 差分方程分子系数
    float den[2];           // 差分方程分母系数
}ENC_LPF_GO_STRUCT;

typedef struct
{
    ENC_LPF_GO_STRUCT filter;   // 滤波运算数据

    float T;                // 离散周期（秒）
    float Wc;               // 截止角频率（rad/s）
}ENC_LPF_STRUCT;

void ENC_LPF_Init(ENC_LPF_STRUCT *lpf);
void ENC_LPF_Loop(ENC_LPF_STRUCT *lpf);

#endif
