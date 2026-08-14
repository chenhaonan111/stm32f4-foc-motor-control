#ifndef __KEY_DRV_H__
#define __KEY_DRV_H__

#include "main.h"

#define KEY0      HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)
#define KEY1      HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)
#define KEY2      HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)

typedef struct button
{
    uint8_t  level;       /* 当前电平：0按下，1释放 */
    uint8_t  status;      /* 状态机：0空闲，1确认按下，2等待松开 */
    uint16_t scan_cnt;    /* 按压时长计数：按低每拍+1，长按判定用（每拍=10ms） */
    uint8_t  confirm_cnt; /* 消抖确认计数：连续N拍采样一致才允许状态切换 */
}button_t;

extern volatile uint8_t KeyNum;

void Key_Scan(void);

#endif
