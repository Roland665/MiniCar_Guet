#ifndef __HC_SR04_H
#define __HC_SR04_H			  	 
#include "sys.h"

#define HC_SR04_ECHO_GPIO 		GPIOF_BASE
#define HC_SR04_ECHO_PIN 		GPIO_PIN_4
#define HC_SR04_ECHO_GPIOPERIPH SYSCTL_PERIPH_GPIOF

void HC_SR04_Init(void);
void HC_SR04_Start(void);
void HC_SR04_Stop(void);
#endif 
