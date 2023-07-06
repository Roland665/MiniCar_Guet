#include "led.h"   


//���ж���IO�ڿ��Ƶ�LED�ܵĳ�ʼ����
void LED_Init(void){
	LED1_Init();
	TestLED_Init();
}

void LED1_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(LED1_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = LED1_Pin;				//LED1�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(LED1_GPIO, &GPIO_InitStructure);			//�����趨������ʼ��
	GPIO_SetBits(LED1_GPIO,LED1_Pin);					//LED1�����
}


void TestLED_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(TestLED_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = TestLED_Pin;	    		//TestLED�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(TestLED_GPIO, &GPIO_InitStructure);	  		//�����趨������ʼ��
	GPIO_SetBits(TestLED_GPIO,TestLED_Pin); 					//TestLED ����� 
}

/**
  * @brief    ��һ��led����˸�𵽲��Գ����������
  * @param    GPIO_TypeDef* GPIOx��uint16_t GPIO_Pin: led����io��
  * @param    led�����л�������ʱ�䣨ms��
  * @retval    
  */
void LED_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,u16 xms){
    GPIO_ResetBits(GPIOx,GPIO_Pin);
    delay_ms(xms);
    GPIO_SetBits(GPIOx,GPIO_Pin);
    delay_ms(xms);
}

