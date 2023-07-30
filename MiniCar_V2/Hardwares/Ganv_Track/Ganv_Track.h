#ifndef __GANV_TRACK_H
#define __GANV_TRACK_H			  	 
#include "sys.h"

#define GANV_ADDR 0x4C//0100 1100

#define GANV_ERR_GPIO 		    GPIOC_BASE
#define GANV_ERR_PIN 			GPIO_PIN_4
#define GANV_ERR_GPIOPERIPH 	SYSCTL_PERIPH_GPIOC

void Ganv_Track_Init(void);
void Ganv_Err_IO_Init(void);
u8 Ganv_Get_Err_State(void);
u8 Ganv_Get_DD(void);
int8_t Ganv_Calc_DD_Err(u8 trackState);
#endif  
