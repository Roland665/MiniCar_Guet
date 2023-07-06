#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"

#define USART_REC_LEN 200

extern u8 USART1_RX_BUF[USART_REC_LEN];//串口接收缓冲区,最大USART_REC_LEN个字节.
extern u16 USART1_RX_STA;//接收到的有效字节数目
extern u8 usart1RXTime;
extern u8 USART2_RX_BUF[USART_REC_LEN];//串口接收缓冲区,最大USART_REC_LEN个字节.
extern u16 USART2_RX_STA;//接收到的有效字节数目
extern u8 usart2RXTime;

void USART1_Init(u32 bound);
void USART2_Init(u32 bound);
#endif


