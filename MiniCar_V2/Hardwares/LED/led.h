#ifndef __LED_H 
#define __LED_H 
#include "sys.h" 

#define LED0_RGB_R_GPIO 		GPIOA_BASE
#define LED0_RGB_R_PIN 			GPIO_PIN_3
#define LED0_RGB_R_GPIOPERIPH 	SYSCTL_PERIPH_GPIOA

#define LED0_RGB_B_GPIO 		GPIOA_BASE
#define LED0_RGB_B_PIN 			GPIO_PIN_4
#define LED0_RGB_B_GPIOPERIPH 	SYSCTL_PERIPH_GPIOA

#define LED0_RGB_G_GPIO 		GPIOA_BASE
#define LED0_RGB_G_PIN 			GPIO_PIN_5
#define LED0_RGB_G_GPIOPERIPH 	SYSCTL_PERIPH_GPIOA

#define LED0_RGB_R_DISABLE GPIOPinWrite(LED0_RGB_R_GPIO, LED0_RGB_R_PIN, LED0_RGB_R_PIN)
#define LED0_RGB_G_DISABLE GPIOPinWrite(LED0_RGB_G_GPIO, LED0_RGB_G_PIN, LED0_RGB_G_PIN)
#define LED0_RGB_B_DISABLE GPIOPinWrite(LED0_RGB_B_GPIO, LED0_RGB_B_PIN, LED0_RGB_B_PIN)

#define LED0_RGB_R_ENABLE GPIOPinWrite(LED0_RGB_R_GPIO, LED0_RGB_R_PIN, 0)
#define LED0_RGB_G_ENABLE GPIOPinWrite(LED0_RGB_G_GPIO, LED0_RGB_G_PIN, 0)
#define LED0_RGB_B_ENABLE GPIOPinWrite(LED0_RGB_B_GPIO, LED0_RGB_B_PIN, 0)

void LED_Init(void);//LED GPIO初始化
void LED0_RGB_Init(void);
#endif 
