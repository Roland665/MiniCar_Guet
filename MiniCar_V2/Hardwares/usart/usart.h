#ifndef __USART_H
#define __USART_H			  	 
#include "sys.h"
#include "stdio.h"

#define UART_REC_LEN 200 // 表示串口消息缓冲区大小
#define UART3_REC_LEN 200 // 表示串口消息缓冲区大小
#define UART5_REC_LEN 10 // 表示串口消息缓冲区大小

extern u8 UART0_RX_BUF[UART_REC_LEN];//串口接收缓冲区,最大UART_REC_LEN个字节.
extern u16 UART0_RX_STA;//接收到的有效字节数目
extern u8 uart0RXTime;

//for zigbee
extern u8 UART3_RX_BUF[UART3_REC_LEN];//串口接收缓冲区,最大UART_REC_LEN个字节.
extern u16 UART3_RX_STA;//接收到的有效字节数目
extern u8 uart3RXTime;

//for mpu6050
extern u8 UART5_RX_BUF[UART5_REC_LEN];//串口接收缓冲区,最大UART_REC_LEN个字节.
extern u16 UART5_RX_STA;//接收到的有效字节数目
extern u8 uart5RXTime;

void Uart0_Init(uint32_t baud);
void Uart1_Init(uint32_t baud);
void Uart2_Init(uint32_t baud);
void Uart3_Init(uint32_t baud);
void Uart5_Init(uint32_t baud);
void Uart6_Init(uint32_t baud);
#endif  
