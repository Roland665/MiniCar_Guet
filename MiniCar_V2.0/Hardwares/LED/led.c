#include "LED/led.h"   

//所有独立IO口控制的LED总的初始函数
void LED_Init(void){
	LED0_RGB_Init();
}

void LED0_RGB_Init(void){
	// Enable the GPIO port
    MAP_SysCtlPeripheralEnable(LED0_RGB_R_GPIOPERIPH);
    MAP_SysCtlPeripheralEnable(LED0_RGB_G_GPIOPERIPH);
    MAP_SysCtlPeripheralEnable(LED0_RGB_B_GPIOPERIPH);
	// Enable the GPIO pins 
    MAP_GPIOPinTypeGPIOOutput(LED0_RGB_R_GPIO, LED0_RGB_R_PIN);
    MAP_GPIOPinTypeGPIOOutput(LED0_RGB_G_GPIO, LED0_RGB_G_PIN);
    MAP_GPIOPinTypeGPIOOutput(LED0_RGB_B_GPIO, LED0_RGB_B_PIN);
}


