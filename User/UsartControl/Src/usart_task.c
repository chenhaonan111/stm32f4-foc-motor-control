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
                    TxData.fdata[0] = MC.EAngle.ElectricalAnglePU;         // ch1 编码器电角度(真实位置)
                    TxData.fdata[1] = MC.SPLL.EThetaPU;                    // ch2 SPLL观测电角度(看是否锁定跟踪)
                    TxData.fdata[2] = MC.SPLL.ThetaErr;                    // ch3 SPLL鉴相误差
                    TxData.fdata[3] = (float)MC.StrongDragToObs.GeneralMode; // ch4 开闭环状态(0=开环强拖,1=观测器闭环)
                
                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxData, sizeof(TxData));//发送结构体(不用对float做处理了)
            }
            UsartTaskId = 10;
        }
        break;
        
    default:
      break;            
    }
}
