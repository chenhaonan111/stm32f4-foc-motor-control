#include "filter_drv.h"

/*二阶巴特沃斯低通滤波器系数初始化*/
/*连续域传函：H(s) = Wc^2 / (s^2 + sqrt(2)*Wc*s + Wc^2)，阻尼比0.707最大平坦响应*/
/*Tustin双线性变换 s = (2/T)*(1-z^-1)/(1+z^-1) 离散后：*/
/*  y[n] = num0*(x[n]+x[n-2]) + num1*x[n-1] - den0*y[n-1] - den1*y[n-2]*/
/*  其中 temp1 = 2*sqrt(2)*Wc*T，temp2 = (Wc*T)^2，den = temp2+temp1+4*/
/*直流增益恒为1（分子系数和 = 分母系数和），稳态无比例衰减*/
void ENC_LPF_Init(ENC_LPF_STRUCT *lpf)
{
    // 系数用double运算再转回float，提高小系数的离散化精度
    double temp1 = ((double)lpf->T) * ((double)lpf->Wc) * 2.8284271247461903;
    double temp2 = ((double)lpf->T) * ((double)lpf->T) *
                    ((double)lpf->Wc) * ((double)lpf->Wc);
    double den = temp2 + temp1 + 4.0;

    lpf->filter.num[0] = (float)(temp2 / den);
    lpf->filter.num[1] = (float)((2.0 * temp2) / den);
    lpf->filter.den[0] = (float)((2.0 * temp2 - 8.0) / den);
    lpf->filter.den[1] = (float)((temp2 - temp1 + 4.0) / den);

    // 历史状态清零
    lpf->filter.i[0] = 0.0f;
    lpf->filter.i[1] = 0.0f;
    lpf->filter.o[0] = 0.0f;
    lpf->filter.o[1] = 0.0f;
    lpf->filter.Input = 0.0f;
    lpf->filter.Output = 0.0f;
}

/*滤波器离散差分方程运算（每个控制周期调用一次）*/
void ENC_LPF_Loop(ENC_LPF_STRUCT *lpf)
{
    // 双二阶差分方程：分子系数对称，x[n]与x[n-2]共用num[0]
    lpf->filter.Output = lpf->filter.num[0] * (lpf->filter.Input + lpf->filter.i[1]) +
                        lpf->filter.num[1] * lpf->filter.i[0] -
                        lpf->filter.den[0] * lpf->filter.o[0] -
                        lpf->filter.den[1] * lpf->filter.o[1];

    // 更新历史输入输出
    lpf->filter.i[1] = lpf->filter.i[0];
    lpf->filter.i[0] = lpf->filter.Input;
    lpf->filter.o[1] = lpf->filter.o[0];
    lpf->filter.o[0] = lpf->filter.Output;
}
