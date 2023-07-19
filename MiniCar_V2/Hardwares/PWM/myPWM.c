#include "PWM/myPWM.h"
#include "HC_SR04/hc_sr04.h"

/**
  * @brief    PWMģ��0ͨ��0��ʼ��
  * @param    PWM_CLKDIV    :pwmʱ�ӷ�Ƶ��(��ϵͳʱ�ӷ�Ƶ)
  * @param    period    :pwm����(��λ��1 ϵͳʱ������*��Ƶϵ��)
  * @param	  width     :pwm������
  * @retval   void
  */
void M0PWM0_Init(u32 PWM_CLKDIV, u16 period){
	//����PWMʱ�ӣ�����USEPWM_CLKDIV��Ƶ����
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//ʹ��ʱ��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//ʹ��PWMģ��1ʱ��																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//ʹ��GPIOʱ��	
	
	//ʹ�����Ÿ���PWM����
	GPIOPinTypePWM(GPIOB_BASE,GPIO_PIN_6);

	//PWM�źŷ���
	GPIOPinConfigure(GPIO_PB6_M0PWM0);					//PB4->PWMģ��0�ź�2																						

	//����PWM������
	//ģ��0->������1->�¼�������ͬ��
	PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	
    
	//����PWM����
	/*
	N = (1 / f) * SysClk��
	����N�Ǻ�����������PWM����(��ϵͳʱ��Ϊ��λ),���Ϊ65535, Ҳ���Կ�����������װ��ֵ
	fΪ����Ƶ�ʣ�
	SysClkΪϵͳʱ��Ƶ�ʡ�
	*/
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0, period);			//pwm����													

	//ʧ��PWMģ��0���,��֤���ſ��ֵ͵�ƽ
	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,false);

    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
}

/**
  * @brief    PWMģ��0ͨ��1��ʼ��
  * @param    PWM_CLKDIV    :pwmʱ�ӷ�Ƶ��(��ϵͳʱ�ӷ�Ƶ)
  * @param    period    :pwm����(��λ��1 ϵͳʱ������*��Ƶϵ��)
  * @param	  width     :pwm������
  * @retval   void
  */
void M0PWM1_Init(u32 PWM_CLKDIV, u16 period){
	//����PWMʱ�ӣ�����USEPWM_CLKDIV��Ƶ����
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//ʹ��ʱ��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//ʹ��PWMģ��1ʱ��																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//ʹ��GPIOʱ��	
	
	//ʹ�����Ÿ���PWM����
	GPIOPinTypePWM(GPIOB_BASE,GPIO_PIN_7);

	//PWM�źŷ���
	GPIOPinConfigure(GPIO_PB7_M0PWM1);					//PB4->PWMģ��0�ź�2																						

	//����PWM������
	//ģ��0->������1->�¼�������������
	PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	

	//����PWM����
	/*
	N = (1 / f) * SysClk��
	����N�Ǻ�����������PWM����(��ϵͳʱ��Ϊ��λ),���Ϊ65535, Ҳ���Կ�����������װ��ֵ
	fΪ����Ƶ�ʣ�
	SysClkΪϵͳʱ��Ƶ�ʡ�
	*/
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0, period);			//pwm����

	//ʧ��PWMģ��0���,��֤���ſ��ֵ͵�ƽ
	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,false);

    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
}


/**
  * @brief    PWMģ��0ͨ��2��ʼ��
  * @param    PWM_CLKDIV    :pwmʱ�ӷ�Ƶ��(��ϵͳʱ�ӷ�Ƶ)
  * @param    period    :pwm����(��λ��1 ϵͳʱ������*��Ƶϵ��)
  * @param	  width     :pwm������
  * @retval   void
  */
void M0PWM2_Init(u32 PWM_CLKDIV, u16 period, u16 width){
	//����PWMʱ�ӣ����÷�Ƶ����
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//ʹ��ʱ��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//ʹ��PWMģ��0ʱ��																	
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
