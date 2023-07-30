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
	//ʹ�ܷ�����0
    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
	//ʹ��PWMͨ��0���
	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);

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
	//ʹ�ܷ�����0
    PWMGenEnable(PWM0_BASE,PWM_GEN_0);

	//ʹ��PWMͨ��1
	PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
											
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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	    //ʹ��GPIOʱ��	
	
	//ʹ�����Ÿ���PWM����
	GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_4);

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
	PWMGenEnable(PWM0_BASE,PWM_GEN_1);

}


/**
  * @brief    PWMģ��1ͨ��4��ʼ��
  * @param    PWM_CLKDIV    :pwmʱ�ӷ�Ƶϵ��(��ϵͳʱ�ӷ�Ƶ)
  * @param    period        :pwm����(��λ��1 ϵͳʱ������*��Ƶϵ��)
  * @param	  width         :pwm������
  * @retval   void
  */
#include "inc/hw_gpio.h"
void M1PWM4_Init(u32 PWM_CLKDIV, u16 period, u16 width){
	//����PWMʱ�ӣ����÷�Ƶ����
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//ʹ��ʱ��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);			//ʹ��PWMģ��ʱ��																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	    //ʹ��GPIOʱ��	
	
	//����PF0
	HWREG(GPIOF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIOF_BASE + GPIO_O_CR)	= GPIO_PIN_0;
	//ʹ�����Ÿ���PWM����
	GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_0);

	//PWM�źŷ���
	GPIOPinConfigure(GPIO_PF0_M1PWM4);																						

	//����PWM������
	//ģ��1->������2->�¼�������ͬ��
	PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	
    
	//����PWM����
	/*
	N = (1 / f) * SysClk��
	����N�Ǻ�����������PWM����(��ϵͳʱ��Ϊ��λ),���Ϊ65535, Ҳ���Կ�����������װ��ֵ
	fΪ����Ƶ�ʣ�
	SysClkΪϵͳʱ��Ƶ�ʡ�
	*/
	PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2, period);			//pwm����													

	//����PWMռ�ձ�
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, width);		//���峤��Ϊ10us(Ҳ����������Ĺ�ʽ)0.00001*80000000 = 800

	//ʹ��PWM���
	PWMGenEnable(PWM1_BASE,PWM_GEN_2);
	PWMOutputState(PWM1_BASE,PWM_OUT_4_BIT,true);

}
