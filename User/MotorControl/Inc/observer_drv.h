#ifndef __OBSERVER_DRV_H__
#define __OBSERVER_DRV_H__

#include "main.h"
#include "speed_drv.h"

typedef struct
{      
    float Ts;                   //调用周期    
    float Rs;                   //相电阻
    float Ld;                   //相电感
    float Gain;                 //滑膜观测器增益

    float Ialpha;               //α轴实际电流
    float Ibeta;                //β轴实际电流        

    float IalphaFore;           //α轴预测电流
    float IbetaFore;            //β轴预测电流

    float Ualpha;               //α轴实际电压
    float Ubeta;                //β轴实际电压    

    float EalphaFore;           //α轴预测反势
    float EalphaForeLPF;        //α轴预测反势滤波值    

    float EbetaFore;            //β轴预测反势
    float EbetaForeLPF;         //β轴预测反势滤波值

    float EabForeLPFFactor;     //αβ轴预测反势滤波系数

    float EMag;                 //反电动势幅值
}SMO_STRUCT;

typedef struct
{    
    int8_t  Dir;                  //反电动势计算方向 
    float   Ts;                   //调用周期        
    
    float   Ain;                  //输入A
    float   Bin;                  //输入B
    
    float   ThetaErr;             //观测角度误差        
    float   ThetaFore;            //观测角度 单位：弧度
    float   ThetaCompensate;      //补偿后的观测角度 单位：弧度
    float   ETheta;               //补偿后的观测角度 0-3999
    float   EThetaPU;
    
    float   SinVal;               //正弦值
    float   CosVal;               //余弦值
    
    float   Kp;                   //锁相环KP
    float   Ki;                   //锁相环KI
    float   PPart;                //积分项
    float   IPart;                //积分项
            
    float   WeFore;               //观测电角速度（rad/s）    
    float   WeForeLPF;            //观测电角速度滤波值    
    float   WeForeLPFFactor;      //观测电角速度滤波系数    
}PLL_STRUCT;

typedef enum {
    OPEN_LOOP = 0,                                  //开环
    CLOSE_LOOP                                      //闭环
}GENERAL_MODE;

typedef struct{
    GENERAL_MODE GeneralMode;
    GENERAL_MODE LastGeneralMode;
    MOTION_STATE MotionState;
    uint16_t     CloseRunTime;
    float        OpenCurr;
    float        OpenCurrLast;
    float        OpenCurrMax;
    int16_t      CheckCnt;
    float        ThetaRef;
    float        ThetaObs;
    float        ThetaErr;
    float        OpenSpeed;
    float        CloseSpeed;
    float        EleOpenSpeedAbs;
    float        EleCloseSpeedAbs;
    float        SpeedErr;
    float        SpeedErrFlt;
    uint8_t      ErrFlag;
    uint16_t     ErrCnt;
    uint16_t     ErrTimes;
    float        LastTargetSpeed;
    float        OpenToCloseSwitchSpeed;
    float        CloseToOpenSwitchSpeed;
    float        CloseMinSpeed;
    float        CurrChangeRate;
    float        ObsMag;
}STRONG_DRAG_TO_OBSERVER;

static inline float Sat(float value, float min, float max){
    if(value >= max){
        return max;
    }
    if(value <= min){
        return min;
    }
    return value;
}

void SMO_Calculate(SMO_STRUCT *p);
void PLL_Calculate(PLL_STRUCT *p);

#endif
