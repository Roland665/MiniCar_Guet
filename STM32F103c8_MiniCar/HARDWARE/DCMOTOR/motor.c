#include "motor.h"
#include "timer.h"

void MTS_Init(void){
	//��50ms�����£�ռ�ձȲ��˵���30
	TIM3_PWM_Init(100-1,7200-1);//72M/7200=10Khz�ļ���Ƶ��,��װ��ֵ100������PWMƵ��Ϊ 10000hz/500=20hz.(����Ϊ0.05s)
}

void MTL_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MTLEN_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = MTLEN_Pin;				//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(MTLEN_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��

	RCC_APB2PeriphClockCmd(MTLDIR_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = MTLDIR_Pin;				//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(MTLDIR_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��
	//MTLEN = 0;//ֹͣ��ת
}

void MTR_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MTREN_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = MTREN_Pin;				//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(MTREN_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��

	RCC_APB2PeriphClockCmd(MTRDIR_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = MTRDIR_Pin;				//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(MTRDIR_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��
	//MTREN = 0;//ֹͣ��ת
}


/**

  * @brief    ��תһ��
  * @param    
  * @retval    
  */

void Turn_Letf(){
	
}
