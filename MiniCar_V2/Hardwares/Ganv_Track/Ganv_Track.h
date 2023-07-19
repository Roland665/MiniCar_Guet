#ifndef __GANV_TRACK_H
#define __GANV_TRACK_H			  	 
#include "sys.h"

#define GANV_ADDR 0x4C//0100 1100

void Ganv_Track_Init(void);
u8 Ganv_Get_DD(void);
int8_t Ganv_Calc_DD_Err(u8 trackState);
#endif  
