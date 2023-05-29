#ifndef __BEEP_H 
#define __BEEP_H 
#include "sys.h"

#define BEEP_GPIO GPIOA
#define BEEP_Pin GPIO_Pin_0
#define BEEP_Clock RCC_APB2Periph_GPIOA

#define BEEP PAout(0)// 用于提示终端正在寻找中控

void BEEP_Init(void);//LED GPIO初始化
#endif 
