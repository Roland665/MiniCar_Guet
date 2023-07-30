#include "motor.h"
#include "PWM/myPWM.h"
#include "CAP/myCap.h"
//MOTOR 初始化
u16 motor_PWMPeriod;
void Motor_Init(void){
    Motor_Dir_IO_Init();//方向控制IO初始化
	motor_PWMPeriod = 5000;
    M0PWM0_Init(SYSCTL_PWMDIV_64, motor_PWMPeriod);//左电机控速PWM初始化，直接用系统时钟32分频，周期为1000，PWM频率为1.25Khz
	//失能PWM通道0输出,保证引脚开局低电平
	// PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,false);	TB6612模块不需要做此操作
	M0PWM1_Init(SYSCTL_PWMDIV_64, motor_PWMPeriod);//右电机控速PWM初始化，直接用系统时钟32分频，周期为1000，PWM频率为1.25Khz
	//失能PWM通道1输出,保证引脚开局低电平
	// PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,false);TB6612模块不需要做此操作
}

void Motor_Dir_IO_Init(void){
	// Enable the GPIO port
    MAP_SysCtlPeripheralEnable(MOTOR_L_IN1_GPIOPERIPH);
    MAP_SysCtlPeripheralEnable(MOTOR_L_IN2_GPIOPERIPH);
    MAP_SysCtlPeripheralEnable(MOTOR_R_IN1_GPIOPERIPH);
    MAP_SysCtlPeripheralEnable(MOTOR_R_IN2_GPIOPERIPH);
	// Enable the GPIO pins 
    MAP_GPIOPinTypeGPIOOutput(MOTOR_L_IN2_GPIO, MOTOR_L_IN2_PIN);
    MAP_GPIOPinTypeGPIOOutput(MOTOR_L_IN1_GPIO, MOTOR_L_IN1_PIN);
    MAP_GPIOPinTypeGPIOOutput(MOTOR_R_IN1_GPIO, MOTOR_R_IN1_PIN);
    MAP_GPIOPinTypeGPIOOutput(MOTOR_R_IN2_GPIO, MOTOR_R_IN2_PIN);
    // 初始默认停车
    Motor_Stop();
}

void HallEncoder_Cap_Init(){
    T1CCPBOTH_Init();
}

/**
  * @brief    指定速度直行
  * @param    pwmVal: 左右轮PWM占空比
  * @retval   void
  */
void Motor_Straight(u8 pwmVal){
    //直行
    Motor_SetSpeed(pwmVal, pwmVal);
}

/**
  * @brief    指定速度倒车
  * @param    pwmVal: 左右轮PWM占空比
  * @retval   void
  */
void Motor_Back(u8 pwmVal){
    //后退
    Motor_SetSpeed(-pwmVal, -pwmVal);
}

void MotorL_Stop(void){
	//AIN1=0,AIN2=0
    GPIOPinWrite(MOTOR_L_IN1_GPIO, MOTOR_L_IN1_PIN, 0);
	GPIOPinWrite(MOTOR_L_IN2_GPIO, MOTOR_L_IN2_PIN, 0);
}
void MotorR_Stop(void){
	//BIN1=0,BIN2=0
	GPIOPinWrite(MOTOR_R_IN1_GPIO, MOTOR_R_IN1_PIN, 0);
	GPIOPinWrite(MOTOR_R_IN2_GPIO, MOTOR_R_IN2_PIN, 0);
}
void Motor_Stop(void){
    //停车
	MotorL_Stop();
	MotorR_Stop();
}


/**
  * @brief    控制左右轮旋转速度
  * @param    lPwmVal: 左轮PWM脉宽，负数表示反转
  * @param    rPwmVal: 右轮PWM脉宽，负数表示反转
  * @retval   void
  */
void Motor_SetSpeed(int16_t lPwmVal, int16_t rPwmVal){
    //left
	if(lPwmVal == 0){
        MotorL_Stop();
//    	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,false);
    }
    else if(lPwmVal > 0){
        MOTOR_L_DIR_Straight;
	    PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, motor_PWMPeriod*0.01*lPwmVal);
    }
    else{
        MOTOR_L_DIR_Back;
	    PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);
        lPwmVal = -lPwmVal;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, motor_PWMPeriod*0.01*lPwmVal);
    }

    //right
    if(rPwmVal == 0){
        MotorR_Stop();
//    	PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,false);
    }
    else if(rPwmVal > 0){
        MOTOR_R_DIR_Straight;
	    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, motor_PWMPeriod*0.01*rPwmVal);
    }
    else{
        MOTOR_R_DIR_Back;
	    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
        rPwmVal = -rPwmVal;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, motor_PWMPeriod*0.01*rPwmVal);
    }
}

