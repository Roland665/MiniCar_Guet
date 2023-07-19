#include "motor.h"
#include "PWM/myPWM.h"

//MOTOR 初始化
u16 motor_PWMPeriod;
void Motor_Init(void){
    Motor_Dir_IO_Init();//方向控制IO初始化
    //电机FG信号捕获初始化
	motor_PWMPeriod = 5000;
    M0PWM0_Init(SYSCTL_PWMDIV_64, motor_PWMPeriod);//左电机控速PWM初始化，直接用系统时钟32分频，周期为1000，PWM频率为1.25Khz 
	M0PWM1_Init(SYSCTL_PWMDIV_64, motor_PWMPeriod);//右电机控速PWM初始化，直接用系统时钟32分频，周期为1000，PWM频率为1.25Khz
}

void Motor_Dir_IO_Init(void){
	// Enable the GPIO port
    MAP_SysCtlPeripheralEnable(MOTOR_L_DIR_GPIOPERIPH);
    MAP_SysCtlPeripheralEnable(MOTOR_R_DIR_GPIOPERIPH);
	// Enable the GPIO pins 
    MAP_GPIOPinTypeGPIOOutput(MOTOR_L_DIR_GPIO, MOTOR_L_DIR_PIN);
    MAP_GPIOPinTypeGPIOOutput(MOTOR_R_DIR_GPIO, MOTOR_R_DIR_PIN);
    // 初始默认直行
    MOTOR_L_DIR_Straight;
    MOTOR_R_DIR_Straight;
}

void Motor_Straight(void){
    //直行
    MOTOR_L_DIR_Straight;
    MOTOR_R_DIR_Straight;
}

void Motor_Back(void){
    //后退
    MOTOR_L_DIR_Back;
    MOTOR_R_DIR_Back;
}


/**
  * @brief    控制左右轮旋转速度
  * @param    lPwmVal: 左轮PWM占空比，负数表示反转
  * @param    rPwmVal: 右轮PWM占空比，负数表示反转
  * @retval   void
  */
void Motor_SetSpeed(int16_t lPwmVal, int16_t rPwmVal){
    //left
	if(lPwmVal == 0){
        MOTOR_L_DIR_Straight;
    	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,false);
    }
    else if(lPwmVal > 0){
	    PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);
        MOTOR_L_DIR_Straight;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, lPwmVal);
    }
    else{
	    PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);
        lPwmVal = -lPwmVal;
        MOTOR_L_DIR_Back;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, motor_PWMPeriod - lPwmVal);
    }

    //right
    if(rPwmVal == 0){
    	PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,false);
    }
    else if(rPwmVal > 0){
	    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
        MOTOR_R_DIR_Straight;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, rPwmVal);
    }
    else{
	    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
        rPwmVal = -rPwmVal;
        MOTOR_R_DIR_Back;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, motor_PWMPeriod - rPwmVal);
    }
}


