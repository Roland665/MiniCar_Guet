#include "key.h"

void Key_Init(){
	Key1_Init();
}

//������ʼ������
void Key1_Init(void) //IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(KEY1_GPIOClock,ENABLE);//ʹ��KEY1ʱ��

	GPIO_InitStructure.GPIO_Pin  = KEY1_Pin;//����GPIO_Pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);//��ʼ��KEY_GPIO

}



/*��ʱ��������*/
/*
u8 Key1CD = 0;//��������
u8 Key_Scan()
{	 
	if(Key1 == 0){//����Ѿ�������20ms������cd
		if(Key1CD == 20){
			Key1CD = 0;
			return 1;
		}
	}
 	return 0;// �ް�������
}
*/
