#ifndef __HC_SR04_H
#define __HC_SR04_H			  	 
#include "sys.h"

#define HC_SR04_TRIG_GPIO 		GPIOB_BASE
#define HC_SR04_TRIG_PIN 		GPIO_PIN_4
#define HC_SR04_TRIG_GPIOPERIPH SYSCTL_PERIPH_GPIOB

#define HC_SR04_ECHO_GPIO 		GPIOF_BASE
#define HC_SR04_ECHO_PIN 		GPIO_PIN_4
#define HC_SR04_ECHO_GPIOPERIPH SYSCTL_PERIPH_GPIOF


#define HC_SR04_TRIG_ENABLE GPIOPinWrite(HC_SR04_TRIG_GPIO, HC_SR04_TRIG_PIN, HC_SR04_TRIG_PIN)
#define HC_SR04_TRIG_UNABLE GPIOPinWrite(HC_SR04_TRIG_GPIO, HC_SR04_TRIG_PIN, 0)

void HC_SR04_Init(void);
void HC_SR04_Start(void);
void HC_SR04_Stop(void);
#endif 