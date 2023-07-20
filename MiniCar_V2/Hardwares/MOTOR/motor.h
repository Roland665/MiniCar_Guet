#ifndef __MOTOR_H 
#define __MOTOR_H 
#include "sys.h" 

#define MOTOR_L_DIR_GPIO 		GPIOC_BASE
#define MOTOR_L_DIR_PIN 			GPIO_PIN_6
#define MOTOR_L_DIR_GPIOPERIPH 	SYSCTL_PERIPH_GPIOC
#define MOTOR_R_DIR_GPIO 		GPIOC_BASE
#define MOTOR_R_DIR_PIN 			GPIO_PIN_7
#define MOTOR_R_DIR_GPIOPERIPH 	SYSCTL_PERIPH_GPIOC

#define MOTOR_L_DIR_Back        GPIOPinWrite(MOTOR_L_DIR_GPIO, MOTOR_L_DIR_PIN, MOTOR_L_DIR_PIN)
#define MOTOR_L_DIR_Straight    GPIOPinWrite(MOTOR_L_DIR_GPIO, MOTOR_L_DIR_PIN, 0)
#define MOTOR_R_DIR_Back        GPIOPinWrite(MOTOR_R_DIR_GPIO, MOTOR_R_DIR_PIN, MOTOR_R_DIR_PIN)
#define MOTOR_R_DIR_Straight    GPIOPinWrite(MOTOR_R_DIR_GPIO, MOTOR_R_DIR_PIN, 0)

#define CODED_DISC_ONE_ROUND_PULSE 11 //编码器转一圈输出脉冲数（低电平脉冲）
#define TYRE_ONE_ROUND_PULSE CODED_DISC_ONE_ROUND_PULSE*30 //编码器转一圈输出脉冲数（低电平脉冲）
#define TYRE_DIAMETER 67.5//车轮直径(单位毫米)
#define ONEPULSE_FOR_DISTANCE 3.14*67.5/TYRE_ONE_ROUND_PULSE //每一个脉冲代表着车轮走过的距离
extern u16 motor_PWMPeriod;

void Motor_Init(void);// MOTOR 初始化
void Motor_Dir_IO_Init(void);//MOTOR_L GPIO初始化
void Motor_Straight(void);
void Motor_Back(u8 pwmVal);
void Motor_Stop(void);
void Motor_SetSpeed(int16_t lPwmVal, int16_t rPwmVal);
#endif 
