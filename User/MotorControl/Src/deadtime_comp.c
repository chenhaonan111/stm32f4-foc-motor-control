#include "deadtime_comp.h"
#include "main.h"
#include "motor_publicdata.h"
/**
 * 函数功能: 死区补偿模块初始化
 * 输入参数:
 *   p          - 死区补偿结构体指针
 *   comp_ticks - 基础补偿量，单位：PWM计数tick
 *   zero_band  - 电流过零滞环阈值，单位：A
 * 返回参数: 无
 */
void Deadtime_Comp_Init(DEADTIME_COMP_STRUCT *p, float comp_ticks, float zero_band)
{
    p->Iu = 0.0f;
    p->Iv = 0.0f;
    p->Iw = 0.0f;

    p->CompTicks = comp_ticks;
    p->CompGain = DT_COMP_GAIN_DEFAULT;
    p->ZeroBand = zero_band;

    p->SignU = 0;
    p->SignV = 0;
    p->SignW = 0;
}

/**
 * 函数功能: 电流极性判断（带过零滞环）
 * 输入参数: i - 电流值，band - 过零阈值
 * 返回参数: +1（正）、-1（负）、0（过零区）
 */
static int Current_Sign(float i, float band)
{
    if (i > band)  return -1;
    if (i < -band) return 1;
    return 0;
}

/**
 * 函数功能: 计算死区补偿量
 * 输入参数:
 *   p  - 死区补偿结构体指针
 *   iu - U相电流
 *   iv - V相电流
 *   iw - W相电流
 * 返回参数: 无
 * 说    明:
 *   根据三相电流方向判断死区造成的电压误差极性，
 *   补偿量大小由 CompTicks * CompGain 决定。
 */
void Deadtime_Comp_Calculate(DEADTIME_COMP_STRUCT *p, float iu, float iv, float iw)
{
    p->Iu = iu;
    p->Iv = iv;
    p->Iw = iw;

    p->SignU = Current_Sign(iu, p->ZeroBand);
    p->SignV = Current_Sign(iv, p->ZeroBand);
    p->SignW = Current_Sign(iw, p->ZeroBand);
}

/**
 * 函数功能: 应用死区补偿并限幅
 * 输入参数:
 *   p         - 死区补偿结构体指针
 *   duty_u    - U相占空比比较值指针
 *   duty_v    - V相占空比比较值指针
 *   duty_w    - W相占空比比较值指针
 *   pwm_cycle - PWM周期对应的计数最大值
 * 返回参数: 无
 * 说    明:
 *   根据电流极性，在占空比上叠加补偿量，并限制在 [0, pwm_cycle] 范围内。
 *   注意：本项目PWM为中心对齐模式，50%占空比对应 pwm_cycle/2。
 */
void Deadtime_Comp_Apply(DEADTIME_COMP_STRUCT *p, uint16_t *duty_u, uint16_t *duty_v, uint16_t *duty_w, uint16_t pwm_cycle)
{
    float comp_u, comp_v, comp_w;
    float temp_u, temp_v, temp_w;

    comp_u = (float)p->SignU * p->CompTicks * p->CompGain;
    comp_v = (float)p->SignV * p->CompTicks * p->CompGain;
    comp_w = (float)p->SignW * p->CompTicks * p->CompGain;

    temp_u = (float)(*duty_u) + comp_u;
    temp_v = (float)(*duty_v) + comp_v;
    temp_w = (float)(*duty_w) + comp_w;

    // 限幅处理，防止溢出
    if (temp_u > (float)pwm_cycle) temp_u = (float)pwm_cycle;
    if (temp_u < 0.0f)             temp_u = 0.0f;

    if (temp_v > (float)pwm_cycle) temp_v = (float)pwm_cycle;
    if (temp_v < 0.0f)             temp_v = 0.0f;

    if (temp_w > (float)pwm_cycle) temp_w = (float)pwm_cycle;
    if (temp_w < 0.0f)             temp_w = 0.0f;

    *duty_u = (uint16_t)temp_u;
    *duty_v = (uint16_t)temp_v;
    *duty_w = (uint16_t)temp_w;
}
