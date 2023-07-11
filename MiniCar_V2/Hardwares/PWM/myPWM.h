#ifndef __MYPWM_H
#define __MYPWM_H			  	 
#include "sys.h"
void M0PWM0_Init(u32 PWM_CLKDIV, u16 period, u16 width);
void M0PWM1_Init(u32 PWM_CLKDIV, u16 period, u16 width);
void M0PWM2_Init(u32 PWM_CLKDIV, u16 period, u16 width);
#endif  
