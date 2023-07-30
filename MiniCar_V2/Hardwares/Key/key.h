#ifndef __KEY_H
#define __KEY_H			  	 
#include "sys.h"

#define KEY1_GPIO 		    GPIOA_BASE
#define KEY1_PIN 			GPIO_PIN_2
#define KEY1_GPIOPERIPH 	SYSCTL_PERIPH_GPIOA
#define KEY2_GPIO 		    GPIOF_BASE
#define KEY2_PIN 			GPIO_PIN_2
#define KEY2_GPIOPERIPH 	SYSCTL_PERIPH_GPIOF

void Key_ALL_Init(void);
void Key1_Init(void);
void Key2_Init(void);
u8 Key_Scan(void);
#endif  
