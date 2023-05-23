#ifndef __IIC_HARD_H
#define __IIC_HARD_H			  	 
#include "sys.h"

void IIC_Init(void);
void IIC_Start(void); 
void IIC_Send_Address_Write(u8 Address); 
void IIC_Send_Byte(u8 Byte);
void IIC_Stop(void);

#endif  
