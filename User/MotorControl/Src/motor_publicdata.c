#include "motor_publicdata.h"

MOTORCONTROL_STRUCT MC;

void Motor_Struct_Init(void)
{
    // ============================================================================
    // 1. 电机基本运行状态与模式
    // ============================================================================
    // RunState: 电机运行状态机状态，ADC_CALIB表示初始状态为ADC校准（采样偏移校准）
    MC.Motor.RunState = ADC_CALIB;
    // ErrorCode: 故障代码，初始无错误
    MC.Motor.ErrorCode = NONE_ERR;
    // RunMode: 强拖+滑模观测器速度电流闭环(无感)
    MC.Motor.RunMode = STRONG_DRAG_SMO_SPEED_CURRENT_LOOP;
    
    // ============================================================================
    // 2. 采样参数（电流、电压）
    // ============================================================================
    // CurrentDir: 电流采样方向修正（1表示硬件采样方向与定义相反时取反）
    MC.Sample.CurrentDir = -1;
    // CurrentFactor: 相电流计算系数，将ADC原始值转换为实际安培数
    //                由采样电阻、运放放大倍数、ADC参考电压及分辨率决定
    MC.Sample.CurrentFactor = PHASE_CURRENT_FACTOR;
    // BusFactor: 母线电压计算系数，将ADC原始值转换为实际伏特数
    //            由分压电阻分压比和ADC分辨率决定
    MC.Sample.BusFactor = VBUS_FACTOR;
    
    // ============================================================================
    // 3. 编码器与电角度参数
    // ============================================================================
    // PolePairs: 电机极对数（永磁体磁极对数 = 磁钢数量/2）
    MC.EAngle.PolePairs = POLEPAIRS;
    // EncoderValMax: 编码器单圈脉冲数（增量式编码器线数 × 4倍频后的值，或绝对式编码器最大码值）
    MC.EAngle.EncoderValMax = PUL_MAX;
    // Ts: 控制周期（秒），用于角度积分和观测器时间基准
    MC.EAngle.Ts = TS;
    
    // ============================================================================
    // 4. FOC（磁场定向控制）参数
    // ============================================================================
    // IdLPFFactor / IqLPFFactor: d/q轴电流低通滤波系数（一阶滤波α值，0~1）
    //                             0.1表示滤波效果较强，响应稍慢
    MC.Foc.IdLPFFactor = 0.1f;
    MC.Foc.IqLPFFactor = 0.1f;
    // PwmCycle: PWM周期（通常为定时器的自动重装载值），用于计算占空比
    MC.Foc.PwmCycle = PWM_CYCLE;
    // PwmLimit: PWM输出限幅值，防止占空比过大导致上下桥臂直通或过调制
    MC.Foc.PwmLimit = PWM_LIMLT;

    // ============================================================================
    // 5. 位置环参数
    // ============================================================================
    // ElectricalValMax: 电角度单圈脉冲最大值（与编码器单圈最大值相同）
    MC.Position.ElectricalValMax = PUL_MAX;

    // ============================================================================
    // 6. T型加减速参数（速度斜坡）
    // ============================================================================
    MC.TAccDec.PolePairs = POLEPAIRS;   // 极对数，用于机械速度与电速度转换
    MC.TAccDec.Ts = TS;                 // 控制周期
    MC.TAccDec.AccSpeed = ACCELERATION; // 加速度设定值（单位：rpm/s 或 转/秒2）

    // ============================================================================
    // 7. 速度环参数
    // ============================================================================
    MC.Speed.PolePairs = POLEPAIRS;
    // SpeedDivsionFactor: 速度环分频因子（速度环执行周期 = 电流环周期 × 该因子）
    MC.Speed.SpeedDivsionFactor = SPEED_DIVISION_FACTOR;
    // ElectricalValMax: 电角度单圈最大脉冲数
    MC.Speed.ElectricalValMax = PUL_MAX;
    // ElectricalSpeedLPFFactor: 速度反馈的低通滤波系数（一阶滤波）
    MC.Speed.ElectricalSpeedLPFFactor = 0.02f;
    // ElectricalSpeedFactor: 速度计算系数，将电角度差转换为实际转速（rpm）
    //                        公式： (1 / (Ts * SpeedDivsionFactor)) * 60
    MC.Speed.ElectricalSpeedFactor = (1.0f / (TS * SPEED_DIVISION_FACTOR)) * 60.0f;
    
    // ============================================================================
    // 8. 电机参数识别相关（电阻电感识别）
    // ============================================================================
    // CurMax: 识别过程中允许的最大电流（防止过流损坏电机）
    MC.Identify.CurMax = 0.6f;
    
    // ============================================================================
    // 9. 滑模观测器（SMO）参数（用于无传感器中高速段）
    // ============================================================================
    // Gain: 滑模增益，越大观测收敛越快但可能引起抖振
    MC.SMO.Gain = 14.0f;
    MC.SMO.Ts = TS;                         // 观测器运行周期
    // EabForeLPFFactor: 反电动势（Ealpha, Ebeta）低通滤波系数
    MC.SMO.EabForeLPFFactor = 0.1f;
    
    // ============================================================================
    // 10. SMO锁相环(SPLL)参数(标幺化PLL,全速域带宽恒定)
    //    标幺化后 Kp/Ki 与电机参数(psi_f)和转速无关
    //    设计目标:xi=0.707(临界阻尼), wn=500rad/s(约80Hz,低于最高电频率267Hz的1/3)
    //    Kp = 2*xi*wn = 707      (连续域比例增益,单位 1/s)
    //    Ki = wn^2 * Ts = 12.5   (离散化积分增益,已乘Ts)
    //    重要:Ki 是离散化后的值(已乘Ts=5e-5s),不是连续域的 wn^2=250000
    // ============================================================================
    MC.SPLL.Kp = 707.0f;
    MC.SPLL.Ki = 12.5f;
    MC.SPLL.Ts = TS;                      // PLL运行周期
    MC.SPLL.Dir = -1;                     // 方向(已实测:角度方向需要取反)
    MC.SPLL.WeForeLPFFactor = 0.1f;       // 角速度滤波系数
    MC.SPLL.ThetaFore = 0.0f;             // 角度积分初值
    MC.SPLL.IPart = 0.0f;                 // 积分项初值

    // ============================================================================
    // 11. 强拖（强制拖动）到观测器切换参数
    //     用于启动时从开环强拖平滑切换至闭环观测器
    // ============================================================================
    MC.StrongDragToObs.GeneralMode = OPEN_LOOP;              // 当前模式：开环强拖
    MC.StrongDragToObs.LastGeneralMode = OPEN_LOOP;          // 上一次模式
    MC.StrongDragToObs.CloseRunTime = 0;                     // 闭环运行计时
    MC.StrongDragToObs.OpenCurr = 0.4f;                     // 开环强拖电流（安培）
    MC.StrongDragToObs.OpenCurrLast = 0;                     // 上次开环电流
    MC.StrongDragToObs.OpenCurrMax = 3.5f;                   // 开环最大电流限制
    MC.StrongDragToObs.CheckCnt = 0;                         // 切换检测计数器
    MC.StrongDragToObs.ThetaRef = 0;                         // 参考角度（开环给定）
    MC.StrongDragToObs.ThetaObs = 0;                         // 观测器角度
    MC.StrongDragToObs.ThetaErr = 0;                         // 角度误差
    MC.StrongDragToObs.OpenSpeed = 0;                        // 开环目标速度
    MC.StrongDragToObs.CloseSpeed = 0;                       // 闭环反馈速度
    MC.StrongDragToObs.SpeedErr = 0;                         // 速度误差
    MC.StrongDragToObs.SpeedErrFlt = 0;                      // 滤波后速度误差
    MC.StrongDragToObs.ErrFlag = 0;                          // 错误标志
    MC.StrongDragToObs.ErrCnt = 0;                           // 错误计数
    MC.StrongDragToObs.ErrTimes = 0;                         // 错误次数
    MC.StrongDragToObs.OpenToCloseSwitchSpeed = 2400.0f;     // 开环->闭环切换速度阈值（rpm）
    MC.StrongDragToObs.CloseToOpenSwitchSpeed = 1700.0f;     // 闭环->开环切换速度阈值（rpm）
    MC.StrongDragToObs.CloseMinSpeed = 1200.0f;              // 允许闭环的最低速度
    MC.StrongDragToObs.ObsMag = 0;                           // 观测器幅值
    
    // ============================================================================
    // 14. 电流环PID参数（Iq、Id环）
    // ============================================================================
    MC.IqPid.Kp = 1.9f;                     // 比例系数
    MC.IqPid.Ki = 0.087f;                   // 积分系数
    MC.IqPid.OutMax = 10;                   // 输出电压上限（伏特）
    MC.IqPid.OutMin = -10;                  // 输出电压下限（伏特）

    MC.IdPid.Kp = 1.9f;
    MC.IdPid.Ki = 0.087f;
    MC.IdPid.OutMax = 10;
    MC.IdPid.OutMin = -10;
    
    // ============================================================================
    // 15. 速度环PID参数（带分段限制）
    // ============================================================================
    MC.SpdPid.Kp = 0.001f;                  // 默认比例系数
    MC.SpdPid.KpMax = 0.005f;               // 比例系数最大值（用于变速调参）
    MC.SpdPid.KpMin = 0.001f;               // 比例系数最小值
    MC.SpdPid.Ki = 0.000002f;               // 积分系数
    MC.SpdPid.OutMax = 6.75;                   // 输出上限（对应Iq电流参考值，安培）
    MC.SpdPid.OutMin = -6.75;                  // 输出下限

    // ============================================================================
    // 16. 位置环PID参数（用于位置伺服控制）
    // ============================================================================
    MC.PosPid.Kp = 0.5f;                    // 比例系数
    MC.PosPid.Ki = 0;                       // 积分系数（纯比例控制）
    MC.PosPid.Kd = 0;                       // 微分系数（未使用）
    MC.PosPid.OutMax = 8000;               // 输出上限（对应速度参考值，rpm）
    MC.PosPid.OutMin = -8000;              // 输出下限
}                                                  
