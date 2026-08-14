#include "observer_drv.h"
#include "math.h"

void SMO_Calculate(SMO_STRUCT *p)
{
    // 1. 电流预测（基于电机模型的前向欧拉离散化）
    // 公式：I_fore(k+1) = I_fore(k) + Ts * [ -Rs/Ld * I_fore(k) + (U - E_foreLPF)/Ld ]
    p->IalphaFore += p->Ts * (-p->Rs / p->Ld * p->IalphaFore + (p->Ualpha - p->EalphaForeLPF) / p->Ld);
    p->IbetaFore  += p->Ts * (-p->Rs / p->Ld * p->IbetaFore  + (p->Ubeta  - p->EbetaForeLPF)  / p->Ld);

    // 2. 滑模切换函数（饱和函数，减小抖振）
    // 边界层设定为 ±1.0（可根据实际电流误差范围调整）
    // 当误差 > 1.0 时，输出 +Gain
    if ((p->IalphaFore - p->Ialpha) > 0.25f)
        p->EalphaFore = p->Gain;
    // 当误差 < -1.0 时，输出 -Gain
    else if ((p->IalphaFore - p->Ialpha) < -0.25f)
        p->EalphaFore = -p->Gain;
    // 在边界层内，输出 = Gain * 误差（线性区）
    else
        p->EalphaFore = p->Gain * (p->IalphaFore - p->Ialpha);

    // β轴同理
    if ((p->IbetaFore - p->Ibeta) > 0.25f)
        p->EbetaFore = p->Gain;
    else if ((p->IbetaFore - p->Ibeta) < -0.25f)
        p->EbetaFore = -p->Gain;
    else
        p->EbetaFore = p->Gain * (p->IbetaFore - p->Ibeta);

    // 3. 反电动势低通滤波（滤除高频切换噪声）
    // 一阶低通滤波：E_filt(k) = α * E_raw(k) + (1-α) * E_filt(k-1)
    p->EalphaForeLPF = p->EalphaFore * p->EabForeLPFFactor + p->EalphaForeLPF * (1 - p->EabForeLPFFactor);
    p->EbetaForeLPF  = p->EbetaFore  * p->EabForeLPFFactor + p->EbetaForeLPF  * (1 - p->EabForeLPFFactor);
}

void PLL_Calculate(PLL_STRUCT *p)
{
    // 1. 计算反电势幅值 |E| = sqrt(Ealpha^2 + Ebeta^2)
    float EMag = sqrtf(p->Ain * p->Ain + p->Bin * p->Bin);

    // 2. 标幺化鉴相器:err = Dir * (Ealpha*cos + Ebeta*sin) / |E|
    //    标幺化后 err = sin(dTheta) 在 [-1, 1] 范围内,是纯角度误差信号
    //    与反电势幅值和转速无关,全速域带宽恒定
    //    低速保护:EMag 太小(反电势被噪声淹没)时 PLL 暂停跟踪
    if (EMag > 0.05f)
    {
        p->ThetaErr = p->Dir * (p->CosVal * p->Ain + p->SinVal * p->Bin) / EMag;
    }
    else
    {
        p->ThetaErr = 0.0f;   // 反电势太小(低速),不更新,PLL 保持上拍角度
    }

    // 3. PI控制器(环路滤波器)
    //    标幺化下 Kp/Ki 是纯常数,与电机参数(psi_f)和转速无关
    //    Kp = 2*xi*wn = 707 (xi=0.707, wn=500rad/s)
    //    Ki = wn^2 * Ts = 12.5 (Ts=5e-5s, 离散化系数)
    p->PPart = p->Kp * p->ThetaErr;
    p->IPart += p->Ki * p->ThetaErr;
    // 积分限幅:防止 PLL 未锁住时 IPart 无限累加导致发散(安全网)
    // 限幅值按最高电角速度的 3 倍(4000rpm -> 1676rad/s -> 限5000rad/s)
    if (p->IPart > 5000.0f)  p->IPart = 5000.0f;
    if (p->IPart < -5000.0f) p->IPart = -5000.0f;
    p->WeFore = p->PPart + p->IPart;           // 观测电角速度(rad/s)

    // 4. 角速度低通滤波(平滑输出)
    p->WeForeLPF = p->WeFore * p->WeForeLPFFactor + p->WeForeLPF * (1 - p->WeForeLPFFactor);

    // 5. 积分得到观测电角度(压控振荡器)
    p->ThetaFore += p->WeFore * p->Ts;

    // 6. 角度归一化到 [0, 2pi)
    //    用 fmodf 一步取模,O(1)复杂度,避免 while 循环在大角度时卡死
    p->ThetaFore = fmodf(p->ThetaFore, 6.28318f);
    if (p->ThetaFore < 0.0f)   // fmodf 可能返回负值,补到 [0, 2pi)
    {
        p->ThetaFore += 6.28318f;
    }

    // 7. 角度格式转换
    p->ETheta = p->ThetaFore;
    p->EThetaPU = p->ETheta / 6.28318f;
}
