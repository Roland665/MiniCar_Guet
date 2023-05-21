#include "key.h"

void Key_Init(){
	Key1_Init();
}

//按键初始化函数
void Key1_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(KEY1_GPIOClock,ENABLE);//使能KEY1时钟

	GPIO_InitStructure.GPIO_Pin  = KEY1_Pin;//设置GPIO_Pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成下拉输入
 	GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);//初始化KEY_GPIO

}



/*定时器消抖版*/
/*
u8 Key1CD = 0;//按键消抖
u8 Key_Scan()
{	 
	if(Key1 == 0){//如果已经满足了20ms的消抖cd
		if(Key1CD == 20){
			Key1CD = 0;
			return 1;
		}
	}
 	return 0;// 无按键按下
}
*/
