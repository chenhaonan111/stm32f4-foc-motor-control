#include "pll_drv.h"
#include "math_drv.h"

void ENC_PLL_Init(ENC_PLL_STRUCT *pll){
    double temp0 = ((double)pll->T) * ((double)pll->Ki);
    pll->go.PiNum[0] = (float)((2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.PiNum[1] = (float)((-2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.IntegCoeff = (float)(((double)pll->T) / 2.0);

    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.ErrPrev = 0.0f;
    pll->go.WmPrev = 0.0f;
    pll->go.OutWm = 0.0f;
    pll->go.OutThetaM = 0.0f;
    pll->go.Error = 0.0f;
}

void ENC_PLL_Loop(ENC_PLL_STRUCT *pll){
    // 1.计算PI控制器(增量式, 输出OutWm)
    pll->go.OutWm += pll->go.PiNum[0] * pll->go.Error +
                    pll->go.PiNum[1] * pll->go.ErrPrev;

    // 2.计算积分器(梯形积分, 输出OutThetaM)
    pll->go.OutThetaM += pll->go.IntegCoeff * (pll->go.OutWm + pll->go.WmPrev);
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutThetaM = Value_normalize(pll->go.OutThetaM);
    }

    // 3.更新历史值(ErrPrev=e[k-1], WmPrev=ωm[k-1])
    pll->go.ErrPrev = pll->go.Error;
    pll->go.WmPrev = pll->go.OutWm;
}
