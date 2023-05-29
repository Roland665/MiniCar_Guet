#include "beep.h"   

void BEEP_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(BEEP_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = BEEP_Pin;				//LED1�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��
	GPIO_ResetBits(BEEP_GPIO,BEEP_Pin);					//LED1�����
}
