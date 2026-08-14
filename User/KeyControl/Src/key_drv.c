#include "key_drv.h"

#define KEY_CONFIRM_CNT   3   /* 按下消抖：连续3拍(30ms)为低才算有效按下 */
#define KEY_RELEASE_CNT   2   /* 释放消抖：连续2拍(20ms)为高才算确认释放 */
#define KEY_LONG_SCAN_CNT 50  /* 长按阈值：累计按住50拍(500ms)判定长按 */

volatile uint8_t KeyNum = 0;

button_t button[3] = {0};

void ReadKey(void)
{
    button[0].level = KEY0;
    button[1].level = KEY1;
    button[2].level = KEY2;
}


void Key_Process(void)
{
    uint8_t index = 0;

    for (index = 0; index < 3; index++)
    {
        switch (button[index].status)
        {
            /* 状态0：等待按下（含按下消抖确认） */
            case 0:
                if (button[index].level == 0)              /* 采到低电平 */
                {
                    button[index].confirm_cnt++;           /* 连续按低计数 */
                    if (button[index].confirm_cnt >= KEY_CONFIRM_CNT)
                    {
                        /* 连续3拍(30ms)稳定为低，确认是有效按下，而非抖动毛刺 */
                        button[index].confirm_cnt = 0;
                        button[index].scan_cnt = 0;        /* 清零按压时长计数 */
                        button[index].status = 1;
                    }
                }
                else
                {
                    button[index].confirm_cnt = 0;         /* 采到高，抖动/干扰，立即清零重新累计 */
                }
                break;

            /* 状态1：确认按下，累计时长并做长短按判定（含释放消抖） */
            case 1:
                if (button[index].level == 0)              /* 仍按住 */
                {
                    button[index].confirm_cnt = 0;         /* 按住则清零释放确认计数 */
                    button[index].scan_cnt++;              /* 按压时长+1拍 */
                    if (button[index].scan_cnt >= KEY_LONG_SCAN_CNT)
                    {
                        /* 累计按住50拍(500ms)，判定长按 */
                        button[index].confirm_cnt = 0;
                        if (index == 0) KeyNum = 2;        /* 按键1长按 */
                        if (index == 1) KeyNum = 4;        /* 按键2长按 */
                        if (index == 2) KeyNum = 6;
                        button[index].status = 2;
                    }
                }
                else                                       /* 采到高：可能是释放，也可能是抖动 */
                {
                    button[index].confirm_cnt++;           /* 释放确认计数 */
                    if (button[index].confirm_cnt >= KEY_RELEASE_CNT)
                    {
                        /* 连续2拍(20ms)为高，确认释放；scan_cnt未到50，判定短按 */
                        if (index == 0) KeyNum = 1;        /* 按键1短按 */
                        if (index == 1) KeyNum = 3;        /* 按键2短按 */
                        if (index == 2) KeyNum = 5;
                        button[index].status = 0;
                    }
                }
                break;

            /* 状态2：等待松开（长按已上报，防重复触发；含释放消抖） */
            case 2:
                if (button[index].level == 1)
                {
                    button[index].confirm_cnt++;
                    if (button[index].confirm_cnt >= KEY_RELEASE_CNT)
                    {
                        button[index].confirm_cnt = 0;
                        button[index].status = 0;          /* 确认已完全松开，回空闲 */
                    }
                }
                else
                {
                    button[index].confirm_cnt = 0;
                }
                break;
        }
    }
}

void Key_Scan(void)
{
    ReadKey();      // 读取当前按键电平
    Key_Process();  // 执行按键状态机处理
}
