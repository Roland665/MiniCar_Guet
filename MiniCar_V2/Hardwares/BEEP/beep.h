#ifndef __BEEP_H 
#define __BEEP_H 
#include "sys.h" 

#define BEEP_GPIO 			GPIOF_BASE
#define BEEP_PIN 			GPIO_PIN_4
#define BEEP_GPIOPERIPH 	SYSCTL_PERIPH_GPIOF

#define BEEP_ENABLE GPIOPinWrite(BEEP_GPIO, BEEP_PIN, 0)
#define BEEP_DISABLE GPIOPinWrite(BEEP_GPIO, BEEP_PIN, BEEP_PIN)

void BEEP_Init(void);//BEEP GPIO初始化
#endif 
