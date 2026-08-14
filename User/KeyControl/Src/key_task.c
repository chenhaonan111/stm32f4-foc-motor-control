#include "key_task.h"
#include "key_drv.h"

volatile uint16_t KeyTaskId = 10;
volatile uint16_t KeyTaskTim = 0;

void Key_Task(void)
{
    switch (KeyTaskId)
    {
        case 10:    // 延时等待状态
        {
            if (KeyTaskTim >= 100)        // 计时达到 100 个周期（10ms）
            {
                KeyTaskTim = 0;           // 计时器清零
                KeyTaskId = 20;           // 切换到扫描状态
            }
        }
        break;

        case 20:    // 执行按键扫描状态
        {
            Key_Scan();                   // 调用按键扫描函数（读取电平并更新键值）
            KeyTaskId = 10;               // 返回延时等待状态
        }
        break;

        default:
            break;
    }
}

