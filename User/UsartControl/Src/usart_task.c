#include "usart_task.h"
#include "dma.h"
#include "usart.h"
#include "key_drv.h"
#include "motor_publicdata.h"
TXDATA TxData;
volatile uint16_t UsartTaskId = 5;
volatile uint16_t UsartTaskTim = 0;


void Usart_Task(void)
{
    switch(UsartTaskId)
    {
        case 5: 
        {
            TxData.tail[0] = 0x00;
            TxData.tail[1] = 0x00;
            TxData.tail[2] = 0x80;
            TxData.tail[3] = 0x7f;
            UsartTaskId = 10;
        }
        break;        
        
        case 10:
        {
            if(UsartTaskTim >= 1)        //0.2ms
            {
                UsartTaskTim = 0;
                UsartTaskId = 20;
            }
        }
        break;
        
        case 20:
        {
            if (__HAL_DMA_GET_COUNTER(&hdma_usart1_tx) == 0)
            {
                /* 速度环整定观测（VOFA+ 4通道）：
                   CH1 速度给定(电rpm)  CH2 速度反馈(电rpm)——看阻尼/振荡直接用它
                   CH3 Iq给定(A)        CH4 U相实测电流(A) */
                TxData.fdata[0] = MC.SpdPid.Ref;
                TxData.fdata[1] = MC.SpdPid.Fbk;
                TxData.fdata[2] = MC.IqPid.Ref;
                TxData.fdata[3] = MC.Sample.IuReal;

                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxData, sizeof(TxData));//发送结构体(不用对float做处理了)
            }
            UsartTaskId = 10;
        }
        break;
        
    default:
      break;            
    }
}
