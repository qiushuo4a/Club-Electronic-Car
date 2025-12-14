#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include "stm32f1xx_hal.h"


#define USART_REC_LEN  			100  	//定义最大接收字节数 200
#define RXBUFFERSIZE   			1 		//缓存大小

extern uint8_t USART1_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
extern uint8_t aRxBuffer1[RXBUFFERSIZE];		  //HAL库使用的串口接收缓冲
extern UART_HandleTypeDef UART1_Handler; //UART句柄

typedef struct
{
	uint16_t CH1;//通道1数值
	uint16_t CH2;//通道2数值
	uint16_t CH3;//通道3数值
	uint16_t CH4;//通道4数值
	uint16_t CH5;//通道5数值
	uint16_t CH6;//通道6数值
    uint16_t CH7;//通道7数值
    uint16_t CH8;//通道8数值
    uint16_t CH9;//通道9数值
    uint16_t CH10;//通道10数值
    uint16_t CH11;//通道11数值
    uint16_t CH12;//通道12数值
    uint16_t CH13;//通道13数值
    uint16_t CH14;//通道14数值
    uint16_t CH15;//通道15数值
    uint16_t CH16;//通道16数值
	uint8_t ConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CH_Struct;

extern SBUS_CH_Struct SBUS_CH;

uint8_t update_sbus(uint8_t *buf);

#endif

