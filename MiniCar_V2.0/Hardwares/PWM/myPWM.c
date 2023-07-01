#include "PWM/myPWM.h"
#include "HC_SR04/hc_sr04.h"

/**
  * @brief    PWMģ��0ͨ��2��ʼ��
  * @param    period    :pwm����
  * @param	  width     :pwm������
  * @retval   void
  */
void M0PWM2_Init(u16 period, u16 width){
	//����PWMʱ�ӣ�����USEPWMDIV��Ƶ����
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	
	//ʹ��ʱ��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//ʹ��PWMģ��1ʱ��																	
	SysCtlPeripheralEnable(HC_SR04_TRIG_GPIOPERIPH);	//ʹ��GPIOʱ��	
	
	//ʹ�����Ÿ���PWM����
	GPIOPinTypePWM(HC_SR04_TRIG_GPIO,HC_SR04_TRIG_PIN);

	//PWM�źŷ���
	GPIOPinConfigure(GPIO_PB4_M0PWM2);					//PB4->PWMģ��0�ź�2																						

	//����PWM������
	//ģ��0->������1->�¼�������ͬ��
	PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	
	
	//����PWM����
	/*
	N = (1 / f) * SysClk��
	����N�Ǻ�����������PWM����(��ϵͳʱ��Ϊ��λ),���Ϊ65535, Ҳ���Կ�����������װ��ֵ
	fΪ����Ƶ�ʣ�
	SysClkΪϵͳʱ��Ƶ�ʡ�
	*/
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1, period);			//pwm����													

	//����PWMռ�ձ�
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width);		//���峤��Ϊ10us(Ҳ����������Ĺ�ʽ)0.00001*80000000 = 800

	//ʹ��PWMģ��0���
	PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT,true);
	
}
