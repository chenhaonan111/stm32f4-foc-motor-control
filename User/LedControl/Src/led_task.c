#include "led_task.h"

volatile uint16_t LedTaskId = 10;
volatile uint16_t LedTaskTim = 0;

void Led_Task(void)
{
    switch(LedTaskId)
    {
        case 10:
        {
            if(LedTaskTim >= 5000)
            {
                LedTaskTim = 0;
                LedTaskId = 20;
            }
        }
        break;
        
        case 20:
        {
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);//交替闪烁
            LedTaskId = 10;
        }
        break;
        
        default:
            break;
    }
}
