#ifndef __LED_H 
#define __LED_H 
#include "sys.h" 
#include "delay.h"
#include "timer.h"

#define LED1_GPIO GPIOB
#define LED1_Pin GPIO_Pin_9
#define LED1_Clock RCC_APB2Periph_GPIOB

#define TestLED_GPIO GPIOC
#define TestLED_Pin GPIO_Pin_13
#define TestLED_Clock RCC_APB2Periph_GPIOC

#define LED1 PBout(9)// 用于提示终端正在寻找中控
#define TestLED PCout(13)// 调试直观观察程序进入主while循环


void LED_Init(void);//LED GPIO初始化
void LED1_Init(void);
void TestLED_Init(void);
void LED_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,u16 xms);//流水灯
void LED_Close(u8 number);
void LED_Open(u8 number);
void LED_Light_Plus(u8 number);
void LED_Light_Minus(u8 number);
void LED_Light_Set(u8 number, u8 value);
void LED_Mode(u8 number, u8 mode);
#endif 
