#include "eangle_drv.h"

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
}

