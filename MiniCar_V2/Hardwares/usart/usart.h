#ifndef __USART_H
#define __USART_H			  	 
#include "sys.h"
#include "stdio.h"

#define UART_REC_LEN 200 // 表示串口消息缓冲区大小

extern u8 UART0_RX_BUF[UART_REC_LEN];//串口接收缓冲区,最大UART_REC_LEN个字节.
extern u16 UART0_RX_STA;//接收到的有效字节数目
extern u8 uart0RXTime;

extern u8 UART1_RX_BUF[UART_REC_LEN];//串口接收缓冲区,最大UART_REC_LEN个字节.
extern u16 UART1_RX_STA;//接收到的有效字节数目
extern u8 uart1RXTime;

void Uart0_Init(uint32_t bound);
void Uart1_Init(uint32_t bound);
#endif  
