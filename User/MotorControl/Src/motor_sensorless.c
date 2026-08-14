#include "motor_sensorless.h"
#include "motor_publicdata.h"
#include "eangle_drv.h"
#include "math_drv.h"
#include "math.h"

void Strong_Drag_Current_Open_Loop(void);
void Strong_Drag_Current_Close_Loop(void);
void Strong_Drag_Smo_Speed_Current_Loop(void);
void Strong_Drag_To_Observer_Cal(STRONG_DRAG_TO_OBSERVER *p);

void Sensorless_Control(void)
{
    // 根据运行模式选择对应的控制策略
    switch(MC.Motor.RunMode)
    {
        // --------------------------------------------------------------------
        // 模式1: 电流开环强拖（STRONG_DRAG_CURRENT_OPEN）
        // --------------------------------------------------------------------
        // 说明: 最简单的开环强制拖动，不依赖电流反馈，直接给定固定电压（Ud通常为0），
        //       通过开环电角度发生器驱动电机旋转。用于电机启动初期或低速试探。
        //       此模式下无法带载，仅用于让电机转动起来。
        case STRONG_DRAG_CURRENT_OPEN:
        {
            Strong_Drag_Current_Open_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式2: 电流闭环强拖（STRONG_DRAG_CURRENT_CLOSE）
        // --------------------------------------------------------------------
        // 说明: 在开环强拖基础上，加入电流闭环控制（Id环给定固定励磁电流，Iq环给定0或固定值），
        //       能够限制电流、提供一定转矩，但角度仍是开环生成。适用于需要一定力矩的低速强拖。
        case STRONG_DRAG_CURRENT_CLOSE:
        {
            Strong_Drag_Current_Close_Loop();
        }
        break;

        // --------------------------------------------------------------------
        // 模式3: 强拖 + 滑模观测器（SMO）速度电流闭环（STRONG_DRAG_SMO_SPEED_CURRENT_LOOP）
        // --------------------------------------------------------------------
        // 说明: 从开环强拖向无传感器闭环过渡的阶段。使用滑模观测器估算反电动势，
        //       通过锁相环提取角度和速度，并实现速度环和电流环闭环。
        //       内部包含开环到闭环的平滑切换逻辑，适用于电机有一定转速后切入闭环。
        case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
        {
            Strong_Drag_Smo_Speed_Current_Loop();
        }
        break;
    }
}

void Strong_Drag_Current_Open_Loop(void)
{
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);                // 计算当前周期应输出的速度（rpm）
    
    // 将T型加减速输出的速度作为电角速度设定值（单位：rpm）
    MC.EAngle.ElectricalAngleSpdSet = MC.TAccDec.SpeedOut;
    // 电角度发生器：根据设定的电角速度，通过积分生成连续变化的电角度（标幺值0~1）
    Electrical_Angle_Generator(&MC.EAngle);
    
    Calculate_Sin_Cos(MC.EAngle.ElectricalAngleSetPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    
    MC.Foc.Ud = 0.0f;
    MC.Foc.Uq = 1.0f;
    
    IPark_Transform(&MC.Foc);
    
    MC.Foc.Ubus = MC.Sample.BusReal;
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

float temp = 0.4;//调试时候用，如果在函数内改立马被覆盖

void Strong_Drag_Current_Close_Loop(void)
{
    // 强拖时通常需要建立一定的磁场，给定一个固定的Id参考值（单位：安培）
    MC.IdPid.Ref = 0.0f;
    MC.IqPid.Ref = temp;

    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);           // 计算当前周期应输出的速度（rpm）

    // 将T型加减速输出的速度作为电角速度设定值（单位：rpm）
    MC.EAngle.ElectricalAngleSpdSet = MC.TAccDec.SpeedOut;
    // 电角度发生器：根据设定的电角速度积分生成电角度（开环方式）
    Electrical_Angle_Generator(&MC.EAngle);

    Calculate_Sin_Cos(MC.EAngle.ElectricalAngleSetPU, &MC.Foc.SinVal, &MC.Foc.CosVal);

    MC.Foc.Iu = MC.Sample.IuReal;
    MC.Foc.Iv = MC.Sample.IvReal;

    Clark_Transform(&MC.Foc);

    // 使用开环生成的当前电角度进行变换，得到实际反馈的Id、Iq
    Park_Transform(&MC.Foc);

    MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor);
    MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor);

    MC.IqPid.Fbk = MC.Foc.IqLPF;             // Iq环反馈（转矩电流）
    MC.IdPid.Fbk = MC.Foc.IdLPF;             // Id环反馈（励磁电流）

    PID_Control(&MC.IqPid);                  // Iq闭环调节，输出Uq
    PID_Control(&MC.IdPid);                  // Id闭环调节，输出Ud

    MC.Foc.Uq = MC.IqPid.Out;                // q轴电压指令
    MC.Foc.Ud = MC.IdPid.Out;                // d轴电压指令

    IPark_Transform(&MC.Foc);

    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

void Strong_Drag_Smo_Speed_Current_Loop(void)
{
    // ============================================================================
    // 1. 动态更新滑模观测器增益（基于母线电压）
    // ============================================================================
    // INV_SQRT3 = 1/√3 ≈ 0.57735，用于将电压限制在SVPWM线性调制区（最大不失真电压为母线电压/√3）
    // 观测器增益与母线电压成正比，以适应不同电压下的反电动势变化
    MC.SMO.Gain = MC.Sample.BusReal * INV_SQRT3;

    // ============================================================================
    // 2. 获取目标速度进行T型加减速处理
    // ============================================================================
    // 目标速度赋值给加减速结构体，并调用T型斜坡发生器
    MC.TAccDec.TargetSpeed = MC.Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(&MC.TAccDec);

    // ============================================================================
    // 3. 开环电角度发生器（用于强拖阶段及初始角度）
    // ============================================================================
    // 将T型加减速输出的速度（rpm）作为电角速度设定值
    MC.EAngle.ElectricalAngleSpdSet = MC.TAccDec.SpeedOut;
    // 积分生成开环电角度（标幺值0~1）
    Electrical_Angle_Generator(&MC.EAngle);

    // ============================================================================
    // 4. 滑模观测器（SMO）计算：观测反电动势（Ealpha, Ebeta）
    // ============================================================================
    // 设置观测器参数：定子电阻、d轴电感、当前电流和电压（静止坐标系）
    MC.SMO.Rs = MC.Identify.Rs;
    MC.SMO.Ld = MC.Identify.Ld;
    MC.SMO.Ialpha = MC.Foc.Ialpha;
    MC.SMO.Ibeta  = MC.Foc.Ibeta;
    MC.SMO.Ualpha = MC.Foc.Ualpha;
    MC.SMO.Ubeta  = MC.Foc.Ubeta;
    // 执行滑模观测器运算，输出估算的反电动势（Ealpha, Ebeta）及滤波值
    SMO_Calculate(&MC.SMO);

    // ============================================================================
    // 5. 锁相环（PLL）处理：从反电动势提取电角度和角速度
    // ============================================================================
    MC.SPLL.Dir = MC.TAccDec.SpeedOutDir;            // 速度方向（用于PLL方向判断）
    MC.SPLL.Ain = MC.SMO.EalphaForeLPF;              // 滤波后的Alpha轴反电动势
    MC.SPLL.Bin = MC.SMO.EbetaForeLPF;               // 滤波后的Beta轴反电动势
    PLL_Calculate(&MC.SPLL);                         // PLL计算，输出电角度（EThetaPU）和角速度
    // 根据PLL输出的电角度计算正余弦值，用于后续Park变换
    Calculate_Sin_Cos(MC.SPLL.EThetaPU, &MC.SPLL.SinVal, &MC.SPLL.CosVal);

    // ============================================================================
    // 6. 开闭环切换逻辑计算（强拖 -> 观测器闭环）
    // ============================================================================
    // 记录开环给定速度（来自T型加减速）、闭环观测速度（来自PLL）、开环角度、闭环角度、运动状态
    MC.StrongDragToObs.OpenSpeed = MC.TAccDec.SpeedOut;
    MC.StrongDragToObs.CloseSpeed = MC.SPLL.WeForeLPF / TWO_PI * 60.0f;   // 电角速度(rad/s) -> rpm
    MC.StrongDragToObs.ThetaRef = MC.EAngle.ElectricalAngleSetPU;         // 开环参考角度
    MC.StrongDragToObs.ThetaObs = MC.SPLL.EThetaPU;                       // 闭环观测角度
    MC.StrongDragToObs.MotionState = MC.TAccDec.MotionState;              // 当前加减速状态
    // 核心切换函数：根据速度差、角度差等条件判断应处于开环还是闭环，并处理平滑过渡
    Strong_Drag_To_Observer_Cal(&MC.StrongDragToObs);
    
    MC.Speed.MechanicalSpeed = MC.StrongDragToObs.CloseSpeed / POLEPAIRS;   //用于屏幕显示转速

    // ============================================================================
    // 7. 切换异常处理和恢复机制
    // ============================================================================
    if(MC.StrongDragToObs.ErrFlag != 0)           // 发生切换错误（例如观测器失锁）
    {
        if(MC.StrongDragToObs.ErrTimes == 0)      // 第一次进入错误状态
        {
            // 保存当前目标速度，然后将目标速度和所有累积量清零（紧急停车）
            MC.StrongDragToObs.LastTargetSpeed = MC.TAccDec.TargetSpeed;
            MC.TAccDec.TargetSpeed = 0;
            MC.TAccDec.SpeedOut = 0;
            MC.TAccDec.SpeedTargetIncrement = 0;
            MC.TAccDec.SpeedChangeIncrement = 0;
            // 强制切换到开环模式（强拖）
            MC.StrongDragToObs.GeneralMode = OPEN_LOOP;
        }
        // 错误计数递增
        MC.StrongDragToObs.ErrTimes++;
        // 如果错误持续超过5000个控制周期（时间取决于Ts），则尝试恢复
        if(MC.StrongDragToObs.ErrTimes >= 5000)
        {
            MC.StrongDragToObs.ErrTimes = 0;          // 清零错误计数
            MC.StrongDragToObs.ErrFlag = 0;           // 清除错误标志
            MC.TAccDec.TargetSpeed = MC.StrongDragToObs.LastTargetSpeed; // 恢复原目标速度
        }
    }
    else
    {
        // 无错误时，若当前为闭环模式，则将电角度发生器输出替换为观测器输出的角度（闭环角度）
        if(MC.StrongDragToObs.GeneralMode == CLOSE_LOOP)
        {
            MC.EAngle.ElectricalAngleSetPU = MC.SPLL.EThetaPU;
        }
    }

    // ============================================================================
    // 8. 速度环PID控制（按分频因子执行）
    // ============================================================================
    MC.Speed.SpeedCalculateCnt++;
    if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)   // 每SPEED_DIVISION_FACTOR次执行一次
    {
        MC.Speed.SpeedCalculateCnt = 0;
        // 速度环目标值：T型加减速输出的速度（rpm）
        MC.SpdPid.Ref = MC.TAccDec.SpeedOut;
        // 速度环反馈值：PLL观测的电角速度转换为机械速度（rpm）
        MC.SpdPid.Fbk = MC.SPLL.WeForeLPF / TWO_PI * 60.0f;
        // 仅在闭环模式下执行速度环PID（开环时速度环不调节，直接使用强拖电流）
        if(MC.StrongDragToObs.GeneralMode == CLOSE_LOOP)
        {
            PID_Control(&MC.SpdPid);   // 速度环调节，输出作为Iq参考
            // 若刚刚从开环切换到闭环，需要将速度环的积分和输出初始化为开环时的电流值，
            // 避免切换时电流突变。
            if(MC.StrongDragToObs.LastGeneralMode == OPEN_LOOP)
            {
                MC.SpdPid.Integrate = MC.StrongDragToObs.OpenCurr;
                MC.SpdPid.Out = MC.StrongDragToObs.OpenCurr;
            }
        }
    }
    // 保存本次模式，供下一次判断切换边缘使用
    MC.StrongDragToObs.LastGeneralMode = MC.StrongDragToObs.GeneralMode;

    // ============================================================================
    // 9. 电流采样及Clark变换
    // ============================================================================
    MC.Foc.Iu = MC.Sample.IuReal;
    MC.Foc.Iv = MC.Sample.IvReal;
    Clark_Transform(&MC.Foc);            // Iu,Iv -> Ialpha,Ibeta

    // ============================================================================
    // 10. 选择Park变换使用的电角度（开环角度或闭环观测角度）
    // ============================================================================
    if(MC.StrongDragToObs.GeneralMode == OPEN_LOOP)
    {
        // 开环模式：使用电角度发生器产生的角度
        Calculate_Sin_Cos(MC.EAngle.ElectricalAngleSetPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    }
    else
    {
        // 闭环模式：使用PLL观测器输出的电角度
        Calculate_Sin_Cos(MC.SPLL.EThetaPU, &MC.Foc.SinVal, &MC.Foc.CosVal);
    }

    // Park变换：Ialpha,Ibeta -> Id,Iq（使用上面计算的正余弦）
    Park_Transform(&MC.Foc);

    // ============================================================================
    // 11. Id/Iq低通滤波
    // ============================================================================
    MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor);
    MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor);

    // ============================================================================
    // 12. 设置电流环的参考值（根据开环/闭环模式不同）
    // ============================================================================
    if(MC.StrongDragToObs.GeneralMode == OPEN_LOOP)
    {
        // 开环强拖模式：速度环不工作，Iq参考值为固定的强拖电流（OpenCurr），Id参考值为0
        // 如果目标速度绝对值小于10 rpm，则停止输出电流（关闭强拖）
        if(fabsf(MC.SpdPid.Ref) < 10)
        {
            MC.SpdPid.Integrate = 0;    // 清零速度环积分
            MC.SpdPid.Out = 0;
            MC.IqPid.Ref = 0;           // Iq参考为0，电机无力矩
            MC.SMO.EalphaFore = 0;
            MC.SMO.EbetaFore = 0;
            MC.SPLL.WeFore = 0;
        }
        else
        {
            MC.IqPid.Ref = MC.StrongDragToObs.OpenCurr;   // 固定强拖电流
            MC.IdPid.Ref = 0;                             // Id参考为0（不励磁）
        }
    }
    else
    {
        // 闭环模式：Iq参考来自速度环输出，Id参考为0
        MC.IqPid.Ref = MC.SpdPid.Out;
        MC.IdPid.Ref = 0;
    }

    // 设置电流环反馈值（滤波后的Id/Iq）
    MC.IqPid.Fbk = MC.Foc.IqLPF;
    MC.IdPid.Fbk = MC.Foc.IdLPF;

    // 执行电流环PID调节
    PID_Control(&MC.IqPid);            // 输出Uq
    PID_Control(&MC.IdPid);            // 输出Ud

    // 更新电压指令
    MC.Foc.Uq = MC.IqPid.Out;
    MC.Foc.Ud = MC.IdPid.Out;

    // 反Park变换：Ud,Uq -> Ualpha,Ubeta（使用与Park相同的电角度正余弦）
    IPark_Transform(&MC.Foc);

    // ------------------------------------------------------------------------
    // 13. 更新母线电压（直流母线电压采样值）
    // ------------------------------------------------------------------------
    // 母线电压用于SVPWM的占空比计算，实现电压补偿和过调制处理。
    MC.Foc.Ubus = MC.Sample.BusReal;

    // ------------------------------------------------------------------------
    // 14. SVPWM调制：生成三相逆变器的PWM占空比
    // ------------------------------------------------------------------------
    // 空间矢量脉宽调制，根据 Ualpha, Ubeta 和母线电压 Ubus 计算三相占空比并写入PWM比较寄存器
    Calculate_SVPWM(&MC.Foc);
}

void Strong_Drag_To_Observer_Cal(STRONG_DRAG_TO_OBSERVER *p)
{
    // ============================================================================
    // 1. 计算角度误差（ThetaErr），并处理角度过零点跳变（标幺值范围0~1）
    // ============================================================================
    p->ThetaErr = p->ThetaRef - p->ThetaObs;          // 开环参考角度 - 闭环观测角度
    // 如果误差 > 0.5，说明实际误差应为负方向（因为角度是循环的），减去1.0
    p->ThetaErr = p->ThetaErr > 0.5f ? p->ThetaErr - 1.0f : p->ThetaErr;
    // 如果误差 < -0.5，说明实际误差应为正方向，加上1.0
    p->ThetaErr = p->ThetaErr < -0.5f ? p->ThetaErr + 1.0f : p->ThetaErr;
    // 最终 ThetaErr 范围在 [-0.5, 0.5] 之间，表示角度差的标幺值（±180°电角度）

    // ============================================================================
    // 2. 计算速度误差，并进行一阶低通滤波（平滑速度误差）
    // ============================================================================
    p->SpeedErr = p->OpenSpeed - p->CloseSpeed;       // 开环给定速度 - 闭环观测速度（rpm）
    // 一阶低通滤波：Y(n) = 0.1 * X(n) + 0.9 * Y(n-1)，滤波系数0.1
    p->SpeedErrFlt += 0.1f * (p->SpeedErr - p->SpeedErrFlt);

    // ============================================================================
    // 3. 取开环速度和闭环速度的绝对值（用于比较大小，忽略方向）
    // ============================================================================
    p->EleOpenSpeedAbs = fabsf(p->OpenSpeed);
    p->EleCloseSpeedAbs = fabsf(p->CloseSpeed);

    // ============================================================================
    // 4. 更新角度误差检测计数器（CheckCnt）
    //    当角度误差绝对值 ≤ 0.05（即18°电角度以内）时，计数器递增；
    //    否则计数器递减。该计数器用于判断角度是否已收敛。
    // ============================================================================
    p->CheckCnt = fabsf(p->ThetaErr) <= 0.05f ? p->CheckCnt + 1 : p->CheckCnt - 1;

    // ============================================================================
    // 5. 如果当前处于闭环模式，累加闭环运行时间计数器（带饱和限制）
    // ============================================================================
    if(p->GeneralMode == CLOSE_LOOP)
    {
        p->CloseRunTime++;
        p->CloseRunTime = Sat(p->CloseRunTime, 0, 40000);   // 限制在0~40000之间
    }

    // ============================================================================
    // 6. 动态调整开环强拖电流（OpenCurr）的大小
    //    根据运动状态、速度区间等，增加或减小电流，以优化切换时机和平滑性。
    // ============================================================================
    // 电流每次变化的步长（基于最大开环电流的0.03%）
    float curr_change_unit = p->OpenCurrMax * 0.0003f;

    // 情况1：非减速状态 && 开环速度绝对值 ≥ 切换速度阈值（如4200rpm）&& 角度未完全收敛（CheckCnt≤30）
    //        此时逐渐减小开环电流，为闭环切入做准备（降低强拖力矩，让观测器接管）
    if(p->MotionState != DECELERATE && p->EleOpenSpeedAbs >= p->OpenToCloseSwitchSpeed && p->CheckCnt <= 30)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) - curr_change_unit), 0.45f, p->OpenCurrMax);
    }
    // 情况2：非减速状态 && 开环速度绝对值 < 切换速度阈值（即速度还较低）
    //        此时逐渐增大开环电流，确保有足够力矩加速到切换速度
    else if(p->MotionState != DECELERATE && p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) + 0.01f), 0.4f, p->OpenCurrMax);
    }

    // 情况3：减速状态 && 开环速度绝对值在 [CloseMinSpeed, OpenToCloseSwitchSpeed+5] 区间内
    //        此时略微增加开环电流（小步长），防止减速时过早失步
    if(p->MotionState == DECELERATE && p->EleOpenSpeedAbs < p->OpenToCloseSwitchSpeed + 5 && p->EleOpenSpeedAbs > p->CloseMinSpeed)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) + 0.001f), 0.4f, p->OpenCurrMax);
    }
    // 情况4：减速状态 && 开环速度绝对值低于最低闭环速度阈值（如2000rpm）
    //        此时逐渐减小开环电流，避免低速下电流过大导致抖动
    else if(p->MotionState == DECELERATE && p->EleOpenSpeedAbs < p->CloseMinSpeed)
    {
        p->OpenCurr = Sat((fabsf(p->OpenCurr) - curr_change_unit), 0.35f, p->OpenCurrMax);
    }

    // ============================================================================
    // 7. 根据开环速度的方向，确定开环电流的符号（正反转方向）
    // ============================================================================
    if(p->OpenSpeed < 0)
    {
        p->OpenCurr = -fabsf(p->OpenCurr);   // 负速度方向，电流取负
    }

    // ============================================================================
    // 8. 切换条件判断与错误检测
    // ============================================================================

    // 条件A：当前处于闭环模式 && 闭环运行时间超过20000个周期 && 闭环速度绝对值 ≤ 最低闭环速度
    //        说明在闭环状态下速度过低，可能失步或观测器失效，触发错误标志。
    if(p->GeneralMode == CLOSE_LOOP && p->CloseRunTime > 20000 && (p->EleCloseSpeedAbs <= p->CloseMinSpeed))
    {
        p->ErrFlag = 1;           // 置位错误标志
        p->CloseRunTime = 0;      // 清零闭环运行计时
        p->ErrCnt++;              // 错误计数累加
    }
    // 条件B：角度检测计数器 > 40（即角度误差持续小于0.05超过40个周期）&& 开环速度 ≥ 切换速度阈值
    //        满足条件则切换到闭环模式（观测器接管）
    else if(p->CheckCnt > 40 && p->EleOpenSpeedAbs >= p->OpenToCloseSwitchSpeed)
    {
        p->ErrFlag = 0;           // 清除错误标志
        p->GeneralMode = CLOSE_LOOP;  // 切换到闭环
    }
    // 条件C：开环速度绝对值 < 闭环切换到开环的速度阈值（如3000rpm）
    //        此时强制切回开环模式（速度过低，观测器可能不可靠）
    else if(p->EleOpenSpeedAbs < p->CloseToOpenSwitchSpeed)
    {
        p->CloseRunTime = 0;      // 清零闭环运行时间
        p->GeneralMode = OPEN_LOOP;   // 切换到开环
    }

    // ============================================================================
    // 9. 限制 CheckCnt 的范围（0~41）
    // ============================================================================
    if(p->CheckCnt > 40)
    {
        p->CheckCnt = 41;         // 上限41，使得 >40 条件成立
    }
    else if(p->CheckCnt < 0)
    {
        p->CheckCnt = 0;          // 下限0
    }
}
