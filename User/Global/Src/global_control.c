#include "global_control.h"
#include "tim.h"
#include "led_task.h"
#include "key_task.h"
#include "usart_task.h"
#include "adc.h"
#include "motor_publicdata.h"
#include "motor_system.h"
#include "key_drv.h"

extern volatile uint16_t LedTaskTim;
extern volatile uint16_t KeyTaskTim;
extern volatile uint16_t UsartTaskTim;

void Target_Set(void);

void Global_Init(void)
{
    HAL_Delay(100);
    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);     //LED交替闪烁
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

    Motor_System_Init();                                    //电机系统初始化
    
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)MC.Sample.AdcBuff, 2);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4200);        //初始占空比0(安全)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4200);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4200);
    
    MC.Foc.DutyCycleA = PWM_CYCLE / 2;   // 4200，零矢量=三相50%=无驱动
    MC.Foc.DutyCycleB = PWM_CYCLE / 2;
    MC.Foc.DutyCycleC = PWM_CYCLE / 2;
    
    /* 启动三相 PWM：主通道(上桥) + 互补通道(下桥) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);   // 下桥必须单独启动！
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    
    HAL_ADCEx_InjectedStart_IT(&hadc1);      // (TIM1_TRGO 硬件触发 + JEOS 中断)硬件触发ADC注入采样更稳定
    HAL_TIM_Base_Start_IT(&htim2);          //使能定时器中断
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);         //开启编码器计数
    
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);     //使能SD
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim2.Instance)
    {
        LedTaskTim++;
        KeyTaskTim++;
        UsartTaskTim++;
        
        Led_Task();
        Key_Task();
        Usart_Task();
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    MC.Sample.IuRaw = ADC1->JDR1;              //获取相电流
    MC.Sample.IvRaw = ADC1->JDR2;              //获取相电流
    MC.Sample.BusRaw = MC.Sample.AdcBuff[0];   //获取母线电压
    MC.Sample.TemperRaw = MC.Sample.AdcBuff[1];//获取开发板温度
    MC.EAngle.EncoderVal = TIM3->CNT; //获取编码器值
    
    Target_Set();                              //电位器给定目标值
    Motor_System_Run();                        //电机系统运行
    
//    /* ============ 软件死区补偿（只在标定完成后、真正驱动时补）============ */
//    if(MC.Sample.CalibEndFlag == 1)
//    {
//        if(MC.Sample.IuReal >= 0) MC.Foc.DutyCycleA += DT_COMP_TICKS; else MC.Foc.DutyCycleA -= DT_COMP_TICKS;
//        if(MC.Sample.IvReal >= 0) MC.Foc.DutyCycleB += DT_COMP_TICKS; else MC.Foc.DutyCycleB -= DT_COMP_TICKS;
//        if(MC.Sample.IwReal >= 0) MC.Foc.DutyCycleC += DT_COMP_TICKS; else MC.Foc.DutyCycleC -= DT_COMP_TICKS;
//    }
//    /* ================================================================ */
    
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,MC.Foc.DutyCycleA);     //更新PWM比较值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,MC.Foc.DutyCycleB);     //更新PWM比较值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,MC.Foc.DutyCycleC);     //更新PWM比较值
}

void Target_Set(void)
{
    switch(MC.Motor.RunMode)
    {
        case STRONG_DRAG_CURRENT_OPEN:
        {
            if(KeyNum == 1)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet += SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet >= MAX_MEC_SPEED)
                {
                    MC.Speed.MechanicalSpeedSet = SPEED_SET_DIR * MAX_MEC_SPEED;
                }
            }
            if(KeyNum == 3)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet -= SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet <= 0.0f)
                {
                    MC.Speed.MechanicalSpeedSet = 0.0f;
                }
            }
        }
        break;
        
        case STRONG_DRAG_CURRENT_CLOSE:
        {
            if(KeyNum == 1)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet += SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet >= MAX_MEC_SPEED)
                {
                    MC.Speed.MechanicalSpeedSet = SPEED_SET_DIR * MAX_MEC_SPEED;
                }
            }
            if(KeyNum == 3)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet -= SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet <= 0.0f)
                {
                    MC.Speed.MechanicalSpeedSet = 0.0f;
                }
            }
        }
        break;
        
        case SPEED_CURRENT_LOOP:
        {
            if(KeyNum == 1)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet += SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet >= MAX_MEC_SPEED)
                {
                    MC.Speed.MechanicalSpeedSet = SPEED_SET_DIR * MAX_MEC_SPEED;
                }
            }
            if(KeyNum == 3)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet -= SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet <= 0.0f)
                {
                    MC.Speed.MechanicalSpeedSet = 0.0f;
                }
            }
        }
        break;
        
        case POS_SPEED_CURRENT_LOOP:
        {
            if(KeyNum == 1)
            {
                KeyNum = 0;
                MC.Position.MechanicalPosSet += 300;
                if(MC.Position.MechanicalPosSet > PUL_MAX)
                {
                    MC.Position.MechanicalPosSet = PUL_MAX;
                }
            }
            if(KeyNum == 3)
            {
                KeyNum = 0;
                MC.Position.MechanicalPosSet -= 300;
                if(MC.Position.MechanicalPosSet < -PUL_MAX)
                {
                    MC.Position.MechanicalPosSet = -PUL_MAX;
                }
            }
        }
        break;
        
        case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
        {
            if(KeyNum == 1)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet += SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet >= MAX_MEC_SPEED)
                {
                    MC.Speed.MechanicalSpeedSet = SPEED_SET_DIR * MAX_MEC_SPEED;
                }
            }
            if(KeyNum == 3)
            {
                KeyNum = 0;
                MC.Speed.MechanicalSpeedSet -= SPEED_SET_DIR * 200.0f;
                if(SPEED_SET_DIR * MC.Speed.MechanicalSpeedSet <= 0.0f)
                {
                    MC.Speed.MechanicalSpeedSet = 0.0f;
                }
            }
        }
        break;
    }
}
