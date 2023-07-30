#ifndef __MOTOR_H 
#define __MOTOR_H 
#include "sys.h" 

#define MOTOR_L_IN1_GPIO 		GPIOD_BASE
#define MOTOR_L_IN1_PIN 		GPIO_PIN_3
#define MOTOR_L_IN1_GPIOPERIPH 	SYSCTL_PERIPH_GPIOD
#define MOTOR_L_IN2_GPIO 		GPIOD_BASE
#define MOTOR_L_IN2_PIN 		GPIO_PIN_2
#define MOTOR_L_IN2_GPIOPERIPH 	SYSCTL_PERIPH_GPIOD

#define MOTOR_R_IN1_GPIO 		GPIOE_BASE
#define MOTOR_R_IN1_PIN 		GPIO_PIN_3
#define MOTOR_R_IN1_GPIOPERIPH 	SYSCTL_PERIPH_GPIOE
#define MOTOR_R_IN2_GPIO 		GPIOE_BASE
#define MOTOR_R_IN2_PIN 		GPIO_PIN_2
#define MOTOR_R_IN2_GPIOPERIPH 	SYSCTL_PERIPH_GPIOE

#define MOTOR_L_DIR_Straight    {GPIOPinWrite(MOTOR_L_IN1_GPIO, MOTOR_L_IN1_PIN, 0);GPIOPinWrite(MOTOR_L_IN2_GPIO, MOTOR_L_IN2_PIN, MOTOR_L_IN2_PIN);}  //AIN1=0,AIN2=1
#define MOTOR_L_DIR_Back	    {GPIOPinWrite(MOTOR_L_IN1_GPIO, MOTOR_L_IN1_PIN, MOTOR_L_IN1_PIN);GPIOPinWrite(MOTOR_L_IN2_GPIO, MOTOR_L_IN2_PIN, 0);}  //AIN1=1,AIN2=0


#define MOTOR_R_DIR_Straight    {GPIOPinWrite(MOTOR_R_IN1_GPIO, MOTOR_R_IN1_PIN, 0);GPIOPinWrite(MOTOR_R_IN2_GPIO, MOTOR_R_IN2_PIN, MOTOR_R_IN2_PIN);}  //BIN1=0,BIN2=1
#define MOTOR_R_DIR_Back        {GPIOPinWrite(MOTOR_R_IN1_GPIO, MOTOR_R_IN1_PIN, MOTOR_R_IN1_PIN);GPIOPinWrite(MOTOR_R_IN2_GPIO, MOTOR_R_IN2_PIN, 0);}  //BIN1=1,BIN2=0


#define CODED_DISC_ONE_ROUND_PULSE  11                                      //编码器转一圈输出脉冲数（低电平脉冲）
#define REDUCTION_RATIO             30           //减速比
#define TYRE_CIRCUMFERENCE          3.1415926*67.5                          //车轮周长(单位毫米)
#define ONEPULSE_FOR_DISTANCE       TYRE_CIRCUMFERENCE/REDUCTION_RATIO/CODED_DISC_ONE_ROUND_PULSE //每一个脉冲代表着车轮走过的距离(单位毫米)
extern u16 motor_PWMPeriod;

void Motor_Init(void);// MOTOR 初始化
void Motor_Dir_IO_Init(void);//MOTOR_L GPIO初始化
void Motor_Straight(u8 pwmVal);
void Motor_Back(u8 pwmVal);
void Motor_Stop(void);
void Motor_SetSpeed(int16_t lPwmVal, int16_t rPwmVal);
void HallEncoder_Cap_Init(void);
#endif
