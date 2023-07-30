#include "key.h"
#include "FreeRTOS.h"
#include "task.h"

#define delay_ms vTaskDelay 
void Key_ALL_Init(void){
	Key1_Init();
	Key2_Init();
}

void Key1_Init(void){
	SysCtlPeripheralEnable(KEY1_GPIOPERIPH);
	GPIODirModeSet(KEY1_GPIO, KEY1_PIN, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(KEY1_GPIO,KEY1_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//上拉
}

void Key2_Init(void){
	SysCtlPeripheralEnable(KEY2_GPIOPERIPH);
	GPIODirModeSet(KEY2_GPIO, KEY2_PIN, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(KEY2_GPIO,KEY2_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//上拉
}

u8 Key_Scan(void){
	if(GPIOPinRead(KEY1_GPIO, KEY1_PIN) == 0){
		delay_ms(10);
		while(GPIOPinRead(KEY1_GPIO, KEY1_PIN) == 0);
		delay_ms(10);
		return 1;
	}
	if(GPIOPinRead(KEY2_GPIO, KEY2_PIN) == 0){
		delay_ms(10);
		while(GPIOPinRead(KEY2_GPIO, KEY2_PIN) == 0)
		delay_ms(10);
		return 2;
	}
	return 0;
}
