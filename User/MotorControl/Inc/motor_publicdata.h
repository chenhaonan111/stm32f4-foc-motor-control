#ifndef __MOTOR_PUBLICDATA_H__
#define __MOTOR_PUBLICDATA_H__

#include "main.h"
#include "sample_drv.h"
#include "speed_drv.h"
#include "foc_drv.h"
#include "eangle_drv.h"
#include "pid_drv.h"
#include "motor_identify.h"
#include "position_drv.h"
#include "observer_drv.h"

#define LOW_RESITOR        1.0f     //母线电压检测下端电阻(KΩ)
#define HIGH_RESITOR       24.0f    //母线电压检测上端电阻(KΩ)
#define SAMPLING_RESITOR   0.02f    //相电流采样电阻(Ω)
#define MAGNIFICATION      6.0f     //采样放大倍数
#define ADC_RESOLUTION     4096.0f  //ADC分辨率
#define ADC_VREF           3.3f     //ADC基准电压（V）
#define PWM_LIMLT          7700     //限制最大占空比
#define PWM_CYCLE          8400     //PWM周期占空比
#define TS                 0.00005f //FOC执行间隔（S）

#define VBUS_FACTOR            ((ADC_VREF / ADC_RESOLUTION) / (LOW_RESITOR / (LOW_RESITOR + HIGH_RESITOR))) // 母线电压计算系数
#define PHASE_CURRENT_FACTOR   ((ADC_VREF / ADC_RESOLUTION) / MAGNIFICATION / SAMPLING_RESITOR)             // 相电流计算系数

#define POLEPAIRS          4               //默认极对数
#define ACCELERATION       900            //默认加速度(rpm/s)
#define MAX_MEC_SPEED      4000            //机械转速最快3000(rpm/s)

//按键给定速度转向
#define SPEED_SET_DIR      1

#define INV_SQRT3          0.57735f //1/√3
#define OVER_CURRENT       12.0f    //最大相电流值(A)  超过这个值报错停机
#define BUS_VOLTAGE_MIN    13.0f    //最小供电电压(V)  小于这个值报错停机
#define BUS_VOLTAGE_MAX    59.0f    //最大供电电压(V)  大于这个值报错停机

#define DT_COMP_TICKS      15.0f    //死区补偿

#define ENCODER_LINE 1000            //编码器线数
#define PUL_MAX (4*ENCODER_LINE -1)  //单圈脉冲最大值
#define PUL_MAX_HALF (PUL_MAX / 2)

#define SPEED_DIVISION_FACTOR  2     //速度环分频系数
#define POS_DIVISION_FACTOR    4     //位置环分频系数

#define HALF_PI  1.5707963f
#define ONE_PI   3.1415926f
#define TWO_PI   6.2831853f

/****************有感运行模式*******************/
#define ENCODER_CALIB                       0X00  // 编码器校准
#define CURRENT_OPEN_LOOP                   0X01  // 电流开环
#define CURRENT_CLOSE_LOOP                  0X02  // 电流闭环
#define SPEED_CURRENT_LOOP                  0X03  // 速度闭环
#define POS_SPEED_CURRENT_LOOP              0X04  // 位置闭环

/****************无感运行模式*******************/
#define STRONG_DRAG_CURRENT_OPEN            0X05  // 电流开环强拖
#define STRONG_DRAG_CURRENT_CLOSE           0X06  // 电流闭环强拖
#define STRONG_DRAG_SMO_SPEED_CURRENT_LOOP  0X07  // 强拖切滑膜速度电流闭环

/*电机运行状态*/
typedef enum{
    ADC_CALIB = 0,              // ADC校准
    MOTOR_STOP,                 // 停机
    MOTOR_ERROR,                // 故障报错
    MOTOR_IDENTIFY,             // 参数辨识
    MOTOR_SENSORUSE,            // 有感控制
    MOTOR_SENSORLESS            // 无感控制
} MOTOR_RUN_STATE;

/*错误码*/
typedef enum{
    NONE_ERR = 0,               //无错误
    ADC_CALIB_ERR,              //ADC校准错误
    ENCODER_ERR,                //编码器错误
    POWER_VOLT_ERR,             //欠压或过压
    OVER_CURRENT_ERR,           //过流
    TEMPERATURE_ERR             //温度过高
}MOTOR_ERROR_CODE;

typedef struct
{    
    MOTOR_RUN_STATE     RunState;          // 运行状态
    MOTOR_ERROR_CODE    ErrorCode;
    uint8_t             RunMode;           // 运行模式
}MOTOR_STRUCT;

typedef struct
{
    SAMPLE_STRUCT                     Sample;
    MOTOR_STRUCT                      Motor;
    TSHAPEDACCDEC_STRUCT              TAccDec;
    FOC_STRUCT                        Foc;
    SPEED_STRUCT                      Speed;
    E_ANGLE_STRUCT                    EAngle;
    PID_STRUCT                        IdPid;
    PID_STRUCT                        IqPid;
    PID_STRUCT                        SpdPid;
    PID_STRUCT                        PosPid;
    IDENTIFY_STRUCT                   Identify;
    POSITION_STRUCT                   Position;
    SMO_STRUCT                        SMO;
    PLL_STRUCT                        SPLL;
    STRONG_DRAG_TO_OBSERVER           StrongDragToObs;
} MOTORCONTROL_STRUCT;

extern MOTORCONTROL_STRUCT MC;

void Motor_Struct_Init(void);
#endif

