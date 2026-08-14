#include "foc_drv.h"

/**
 * 函数功能: Clark变换（恒幅值变换）
 * 输入参数: p - FOC_STRUCT结构体指针，包含三相电流Iu、Iv，输出Ialpha、Ibeta
 * 返回参数: 无（结果直接存入结构体成员p->Ialpha, p->Ibeta）
 * 说    明: 
 *         将三相静止坐标系(a,b,c)的电流变换到两相静止坐标系(α,β)。
 *         采用恒幅值变换，变换矩阵系数保证变换前后电流幅值不变。
 *         由于三相电流满足 Iu+Iv+Iw=0，可消去Iw，简化公式为：
 *           Iα = Iu
 *           Iβ = (Iu + 2Iv) / √3
 *         代码中系数：1/√3 ≈ 0.57735027，2/√3 ≈ 1.1547004。
 *         该变换是矢量控制的基础步骤，为后续Park变换提供正交分量。
 */
void Clark_Transform(FOC_STRUCT *p)
{
    p->Ialpha = p->Iu;                                     // α轴电流等于U相电流
    p->Ibeta  = (p->Iu * 0.57735027f) + (p->Iv * 1.1547004f); // β轴电流 = (Iu + 2Iv) / √3
}

/**
 * 函数功能: Park变换
 * 输入参数: p - FOC_STRUCT结构体指针，包含Ialpha、Ibeta、SinVal、CosVal，输出Id、Iq
 * 返回参数: 无（结果存入p->Id, p->Iq）
 * 说    明: 
 *         将两相静止坐标系(α,β)的电流变换到旋转坐标系(d,q)。
 *         变换公式：
 *           Id =  Iα * cosθ + Iβ * sinθ
 *           Iq = -Iα * sinθ + Iβ * cosθ
 *         其中θ为转子电角度，SinVal = sinθ，CosVal = cosθ。
 *         d轴电流Id与磁场方向对齐，q轴电流Iq与力矩方向对齐。
 *         经过Park变换后，交流量变为直流量，便于使用PI控制器实现无静差调节。
 *         该变换需要准确的转子位置信息（有感控制来自编码器，无感控制来自观测器）。
 */
void Park_Transform(FOC_STRUCT *p)
{
    p->Id = (p->Ialpha * p->CosVal) + (p->Ibeta * p->SinVal);   // d轴电流
    p->Iq = (-p->Ialpha * p->SinVal) + (p->Ibeta * p->CosVal);   // q轴电流
}

/**
 * 函数功能: 反Park变换
 * 输入参数: p - FOC_STRUCT结构体指针，包含Ud、Uq、SinVal、CosVal，输出Ualpha、Ubeta
 * 返回参数: 无（结果存入p->Ualpha, p->Ubeta）
 * 说    明: 
 *         将旋转坐标系(d,q)的电压指令变换到静止坐标系(α,β)。
 *         变换公式：
 *           Uα = Ud * cosθ - Uq * sinθ
 *           Uβ = Uq * cosθ + Ud * sinθ
 *         这是Park变换的逆过程，用于将PI控制器输出的Ud、Uq转换为SVPWM可处理的Uα、Uβ。
 *         变换后得到的Uα、Uβ仍然是正弦量，其幅值和相位决定了电机定子磁链的旋转。
 *         通常Ud用于励磁控制（Id闭环），Uq用于力矩控制（Iq闭环）。
 */
void IPark_Transform(FOC_STRUCT *p)
{
    p->Ualpha = p->Ud * p->CosVal - p->Uq * p->SinVal;          // α轴电压
    p->Ubeta  = p->Uq * p->CosVal + p->Ud * p->SinVal;          // β轴电压
}

/**
 * 函数功能: 空间矢量脉宽调制（SVPWM）
 * 输入参数: p - FOC_STRUCT结构体指针，包含Ualpha、Ubeta、Ubus、PwmCycle、PwmLimit
 * 返回参数: 无（结果存入p->DutyCycleA, p->DutyCycleB, p->DutyCycleC）
 * 说    明: 
 *         根据αβ轴电压指令Ualpha、Ubeta和直流母线电压Ubus，计算三相逆变器的占空比。
 *         算法步骤：
 *         1. 扇区判断：计算U1=Uβ, U2=(√3/2)Uα - 0.5Uβ, U3=-(√3/2)Uα - 0.5Uβ，
 *            通过U1,U2,U3的符号组合得到扇区编号N（1~6）。
 *         2. 计算基本矢量作用时间：定义X,Y,Z为相邻矢量作用时间（与PWM周期相关），
 *            根据扇区分配T1、T2（两个有效矢量的作用时间）。
 *         3. 过调制处理：若T1+T2超过PwmLimit，按比例缩小T1、T2，确保输出电压在线性区内。
 *         4. 生成三相比较值：Ta = (PwmCycle - T1 - T2)/4（零矢量起始点），
 *            Tb = Ta + T1/2，Tc = Tb + T2/2。
 *         5. 扇区映射：根据扇区将Ta,Tb,Tc分配到A、B、C三相占空比寄存器。
 *         采用中心对齐PWM模式，可有效降低谐波含量，提高直流母线电压利用率。
 *         相比SPWM，SVPWM的电压利用率提高约15%。
 *         注意：PwmCycle为PWM周期对应的定时器计数值（如1000），
 *               PwmLimit通常设为略小于PwmCycle的值（如PwmCycle*0.95），防止上下桥臂直通。
 */
void Calculate_SVPWM(FOC_STRUCT *p)
{
    float U1, U2, U3 = 0;
    float X, Y, Z = 0;
    float T1, T2, T1Temp, T2Temp = 0;
    uint8_t A, B, C, N = 0;
    uint16_t Ta, Tb, Tc = 0;

    // ========== 1. 计算扇区判别变量 ==========
    // 将 αβ 电压转换为三个等效的相电压值（基于 60° 坐标系）
    U1 = p->Ubeta;                                          // U1 = Uβ
    U2 = (0.866f * p->Ualpha) - (0.5f * p->Ubeta);         // U2 = √3/2 * Uα - 1/2 * Uβ
    U3 = (-0.866f * p->Ualpha) - (0.5f * p->Ubeta);        // U3 = -√3/2 * Uα - 1/2 * Uβ

    // 根据正负确定三位二进制码（A,B,C）
    if (U1 > 0) { A = 1; } else { A = 0; }
    if (U2 > 0) { B = 1; } else { B = 0; }
    if (U3 > 0) { C = 1; } else { C = 0; }
    N = 4 * C + 2 * B + A;                                  // 扇区编号 1~6

    // ========== 2. 计算基本矢量作用时间（未归一化） ==========
    // X, Y, Z 对应不同扇区下两个相邻矢量的作用时间（单位：PWM计数值）
    X = (1.732f * p->PwmCycle * p->Ubeta) / p->Ubus;       // X = √3 * Tpwm * Uβ / Ubus
    Y = (1.5f * p->Ualpha * p->PwmCycle + 0.866f * p->Ubeta * p->PwmCycle) / p->Ubus;
    Z = (-1.5f * p->Ualpha * p->PwmCycle + 0.866f * p->Ubeta * p->PwmCycle) / p->Ubus;

    // ========== 3. 根据扇区分配 T1, T2 ==========
    switch (N)
    {
        case 3: { T1 = -Z; T2 =  X; } break;
        case 1: { T1 =  Z; T2 =  Y; } break;
        case 5: { T1 =  X; T2 = -Y; } break;
        case 4: { T1 = -X; T2 =  Z; } break;
        case 6: { T1 = -Y; T2 = -Z; } break;
        case 2: { T1 =  Y; T2 = -X; } break;
        default: { T1 = 0; T2 = 0; } break;
    }

    // ========== 4. 过调制处理（限制总作用时间不超过 PWM 周期） ==========
    T1Temp = T1;
    T2Temp = T2;
    if (T1 + T2 > p->PwmLimit)                              // 超出最大允许占空比
    {
        T1 = p->PwmLimit * T1Temp / (T1Temp + T2Temp);      // 等比例缩小
        T2 = p->PwmLimit * T2Temp / (T1Temp + T2Temp);
    }

    // ========== 5. 计算三相比较值（中心对齐 PWM） ==========
    Ta = (p->PwmCycle - T1 - T2) * 0.25f;                  // 零矢量作用时间/2（起始点）
    Tb = Ta + T1 * 0.5f;                                   // 第一个有效矢量中点
    Tc = Tb + T2 * 0.5f;                                   // 第二个有效矢量中点

    // ========== 6. 根据扇区将 Ta, Tb, Tc 映射到三相占空比 ==========
    switch (N)
    {
        case 3:
            p->DutyCycleA = Ta;
            p->DutyCycleB = Tb;
            p->DutyCycleC = Tc;
            break;
        case 1:
            p->DutyCycleA = Tb;
            p->DutyCycleB = Ta;
            p->DutyCycleC = Tc;
            break;
        case 5:
            p->DutyCycleA = Tc;
            p->DutyCycleB = Ta;
            p->DutyCycleC = Tb;
            break;
        case 4:
            p->DutyCycleA = Tc;
            p->DutyCycleB = Tb;
            p->DutyCycleC = Ta;
            break;
        case 6:
            p->DutyCycleA = Tb;
            p->DutyCycleB = Tc;
            p->DutyCycleC = Ta;
            break;
        case 2:
            p->DutyCycleA = Ta;
            p->DutyCycleB = Tc;
            p->DutyCycleC = Tb;
            break;
        default:
            p->DutyCycleA = Ta;
            p->DutyCycleB = Tb;
            p->DutyCycleC = Tc;
            break;
    }
}
