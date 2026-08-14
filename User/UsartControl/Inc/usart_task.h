#ifndef __USART_TASK_H__
#define __USART_TASK_H__

#include "main.h"
/*最多9通道（上限）*/
#define CH_COUNT 4               //定义要发送的浮点数个数

typedef struct
{                                                   
    float      fdata[CH_COUNT];         //要发送的浮点数
    uint8_t    tail[4];                 //VOFA的JUSTFLOAT格式的帧尾
}TXDATA;

void Usart_Task(void);
#endif

