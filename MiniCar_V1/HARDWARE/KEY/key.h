#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"	
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

//�궨�尴��1��GPIO��
#define KEY1_GPIO 		GPIOB
#define KEY1_Pin 		GPIO_Pin_12
#define KEY1_GPIOClock	RCC_APB2Periph_GPIOB

#define Key1  GPIO_ReadInputDataBit(KEY1_GPIO,KEY1_Pin)//��ȡ����1

void Key_Init(void);//IO��ʼ��
void Key1_Init(void);
#endif
