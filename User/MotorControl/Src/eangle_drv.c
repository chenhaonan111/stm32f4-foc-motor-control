#include "eangle_drv.h"
#include "math_drv.h"
#include "motor_publicdata.h"

/*电角度积分*/
void Electrical_Angle_Generator(E_ANGLE_STRUCT *p)
{
    
    p->ElectricalAngleSetPU += (p->Ts * p->ElectricalAngleSpdSet * 0.01666f);

    if(p->ElectricalAngleSetPU >= 1)
    {
        p->ElectricalAngleSetPU = p->ElectricalAngleSetPU - 1;
    }
    // 当电角度低于0时，加上1.0（通常发生在设定速度为负时）
    if(p->ElectricalAngleSetPU < 0)
    {
        p->ElectricalAngleSetPU = p->ElectricalAngleSetPU + 1;
    }
}

void Calculate_Encoder_Data(E_ANGLE_STRUCT *p)
{
    // ------------------------------------------------------------------------
    // 1. 编码器方向处理
    // ------------------------------------------------------------------------
    // 若编码器安装方向与电机定义方向相反，可通过 Dir 标志进行取反修正。
    // p->Dir == 1 表示需要反向，将原始计数值变换为 (最大值 - 原始值) 实现反向计数。
    if(p->Dir == 1)
    {
        p->EncoderVal = p->EncoderValMax - p->EncoderVal;   // 方向取反
    }
    
    // ------------------------------------------------------------------------
    // 2. 计算电角度
    // ------------------------------------------------------------------------
    // 计算步骤：
    //   (1) 原始编码器值减去校准零点偏移（EncoderVal - CalibOffset）
    //   (2) 乘以极对数（PolePairs），将机械角度转换为电角度（每对极对应360°电角度）
    //   (3) 对编码器最大值取模（% p->EncoderValMax），得到在一个编码器周期内的计数值
    int32_t ElectricalVal = ((p->EncoderVal - p->CalibOffset) * p->PolePairs) % p->EncoderValMax;
    
    // ------------------------------------------------------------------------
    // 3. 修正可能出现的负值（由于校准偏移或取模运算导致）
    // ------------------------------------------------------------------------
    // 例如，若 (EncoderVal - CalibOffset) 为负，取模结果可能仍为负，导致电角度负值。
    if(ElectricalVal < 0)                                                //处理校准可能带来的负值
    {
        ElectricalVal = ElectricalVal + p->EncoderValMax;                     //计算电角度
    }
    
    // ------------------------------------------------------------------------
    // 4. 计算电角度标幺值
    // ------------------------------------------------------------------------
    p->ElectricalAnglePU = (float)ElectricalVal / (float)p->EncoderValMax;

    // ------------------------------------------------------------------------
    // 5. 保存原始直读电角度（PLL开启接管后，此处仍是未滤波的原始值，供A/B对比）
    // ------------------------------------------------------------------------
    p->ElectricalAngleRawPU = p->ElectricalAnglePU;
}

/*编码器PLL：角度跟踪锁相环，同时估计机械角与机械角速度 低速时更稳定*/
/*必须在 Calculate_Encoder_Data 之后调用：依赖其已完成的 EncoderVal 方向处理*/
void Calculate_Encoder_Pll(E_ANGLE_STRUCT *p)
{
    /* 1. 构造机械角度输入（单位：rad，[0,2π)） */
    /* 注意：EncoderVal 已在 Calculate_Encoder_Data 中做过方向取反，此处不可再做，否则方向反掉 */
    float theta_m = Value_normalize((float)(p->EncoderVal - p->CalibOffset)
                                    / (4.0f * ENCODER_LINE) * TWO_PI);
    p->ThetaMechRad = theta_m;

    /* 2. 误差计算 */
    /* Error 归一到 [-π, π)，避免角度过零时的 2π 跳变冲击 PLL */
    /* 注意：Error/OutThetaM 是 EncPll.go 子结构的成员，访问路径必须带 .go */
    p->EncPll.go.Error = theta_m - p->EncPll.go.OutThetaM;
    Value_Correct(&p->EncPll.go.Error, p->EncPll.go.Error);

    /* 3. PLL 运算（20kHz 每控制周期执行，不要放进速度环分频里！） */
    ENC_PLL_Loop(&p->EncPll);

    /* 4. 速度输出：OutWm(机械rad/s) → 一阶EMA(≈32Hz) → 换算电角度 rpm */
    /* 速度分支过滤波压量化毛刺，角度分支不滤保持快速跟踪 */
    {
        float rpm_raw = p->EncPll.go.OutWm * (60.0f / TWO_PI) * p->PolePairs;
        p->EncSpeedElecRPM += p->SpeedEmaAlpha * (rpm_raw - p->EncSpeedElecRPM);
    }

#if USE_ENCODER_PLL
    /* 5. PLL 接管电角度：下游 Park 变换/测速差分/位置累计自动生效 */
    /* 与原算法数学等价：frac(OutThetaM*PP/(2π))；原始值保留在 ElectricalAngleRawPU */
    p->ElectricalAnglePU = Value_normalize(p->EncPll.go.OutThetaM * p->PolePairs) / TWO_PI;
#endif
}

