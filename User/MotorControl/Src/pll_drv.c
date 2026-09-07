#include "pll_drv.h"
#include "math_drv.h"

void ENC_PLL_Init(ENC_PLL_STRUCT *pll){
    double temp0 = ((double)pll->T) * ((double)pll->Ki);
    pll->go.X_num[0] = (float)((2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.X_num[1] = (float)((-2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.Y_num = (float)(((double)pll->T) / 2.0);

    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.We_i = 0.0f;
    pll->go.Re_i = 0.0f;
    pll->go.OutWe = 0.0f;
    pll->go.OutRe = 0.0f;
    pll->go.Error = 0.0f;
}

void ENC_PLL_Loop(ENC_PLL_STRUCT *pll){
    // 1.计算PI控制器(并输出We)
    pll->go.OutWe += pll->go.X_num[0] * pll->go.Error +
                    pll->go.X_num[1] * pll->go.We_i;

    // 2.计算积分器(并输出Re)
    pll->go.OutRe += pll->go.Y_num * (pll->go.OutWe + pll->go.Re_i);
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutRe = Value_normalize(pll->go.OutRe);
    }

    // 3.更新历史输入和输出数值
    pll->go.We_i = pll->go.Error;
    pll->go.Re_i = pll->go.OutWe;
}
