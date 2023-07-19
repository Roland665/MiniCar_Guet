#ifndef __IIC_HARD_H
#define __IIC_HARD_H			  	 
#include "sys.h"
void IIC1_Init(void);
void IIC2_Init(void);
void IIC3_Init(void);
uint8_t IIC_Register_Read_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address);
void IIC_Register_Write_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address, uint8_t data);
void IIC_Register_Write_len_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address, u8 len, uint8_t *data);
void IIC_Write_One_Byte(u32 ui32Base, uint16_t Device_address, u8 data);
u8 IIC_Read_One_Byte(u32 ui32Base, uint16_t Device_address);
#endif  
