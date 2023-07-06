#include "hc_sr04.h"
#include "timer.h"
void HC_SR04_Init(void){

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(HC_SR04_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = HC_SR04_Pin;				//LED1�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(HC_SR04_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��
	GPIO_ResetBits(HC_SR04_GPIO,HC_SR04_Pin);					//LED1�����

    TIM1_Init(10, 72 - 1);//����Ƶ��Ϊ72M/72 = 1Mhz(����ʱ�侫��Ϊ1us),10usһ�θ����ж�
}
