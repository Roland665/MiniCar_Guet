#include "LED/led.h"   

//���ж���IO�ڿ��Ƶ�LED�ܵĳ�ʼ����
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


