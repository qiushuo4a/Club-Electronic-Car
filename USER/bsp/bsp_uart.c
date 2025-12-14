#include "bsp_uart.h"
#include "usart.h"

uint8_t USART1_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART1_RX_STA = 0; //接收状态标记
uint8_t aRxBuffer1[RXBUFFERSIZE];		  //HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler; //UART句柄
SBUS_CH_Struct SBUS_CH;

//将sbus信号转化为通道值
uint8_t update_sbus(uint8_t *buf)
{
//    int i;
    if (buf[23] == 0)
    {
        SBUS_CH.ConnectState = 1;
        SBUS_CH.CH1 =  ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
        SBUS_CH.CH2 =  ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
        SBUS_CH.CH3 =  ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
        SBUS_CH.CH4 =  ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
        SBUS_CH.CH5 =  ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
        SBUS_CH.CH6 =  ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
        SBUS_CH.CH7 =  ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
        SBUS_CH.CH8 =  ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
        SBUS_CH.CH9 =  ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
        SBUS_CH.CH10 = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
        SBUS_CH.CH11 = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
        SBUS_CH.CH12 = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
        SBUS_CH.CH13 = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
        SBUS_CH.CH14 = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
        SBUS_CH.CH15 = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
        SBUS_CH.CH16 = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;
        return 1;
    }
    else 
    {
        SBUS_CH.ConnectState = 0;
        return 0;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i;

while (huart->Instance == USART3) //如果是串口3
	{
		USART1_RX_BUF[USART1_RX_STA] = aRxBuffer1[0];
		if (USART1_RX_STA == 0 && USART1_RX_BUF[USART1_RX_STA] != 0x0F) break; //帧头不对，丢掉
		USART1_RX_STA++;
		if (USART1_RX_STA > USART_REC_LEN) USART1_RX_STA = 0;  ///接收数据错误,重新开始接收

		if (USART1_RX_BUF[0] == 0x0F && USART1_RX_BUF[24] == 0x00 && USART1_RX_STA == 25)	//接受完一帧数据
		{
			update_sbus(USART1_RX_BUF);
			for (i = 0; i<25; i++)
			{
				USART1_RX_BUF[i] = 0;
			}
			USART1_RX_STA = 0;
			

		}
		HAL_UART_Receive_DMA(&huart3, (uint8_t *)aRxBuffer1, RXBUFFERSIZE);
		break;
	}

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart -> Instance == USART3)
    {
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)aRxBuffer1, RXBUFFERSIZE);
    } 
}

