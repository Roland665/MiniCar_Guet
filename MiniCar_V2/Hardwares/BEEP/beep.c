#include "beep.h"   

//初始化蜂鸣器IO口
void BEEP_Init(void){
	// Enable the GPIO port
    MAP_SysCtlPeripheralEnable(BEEP_GPIOPERIPH);
	// Enable the GPIO pins 
    MAP_GPIOPinTypeGPIOOutput(BEEP_GPIO, BEEP_PIN);
	
	BEEP_DISABLE;
}


