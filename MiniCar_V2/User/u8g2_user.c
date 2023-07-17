#include "u8g2_user.h"
#include "IIC/IIC_Hard.h"
#include "FreeRTOS.h"
#include "task.h"

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buf_idx;
	uint8_t *data;
	// u8 i;
	switch(msg){
		case U8X8_MSG_BYTE_INIT:
			//Init the IIC GPIO
			IIC1_Init();
			break;
		
		case U8X8_MSG_BYTE_START_TRANSFER:
			//clear the index
			buf_idx = 0;
			break;
		
		case U8X8_MSG_BYTE_SEND:
			data = (uint8_t *)arg_ptr;
			while( arg_int > 0 ){
				buffer[buf_idx++] = *data;
				data++;
				arg_int--;
			}
			break;
			
		case U8X8_MSG_BYTE_END_TRANSFER:
			// IIC_Start();
			// IIC_Send_Address_Write(0x78);//write your oled address in here
			// for(i = 0; i < buf_idx; i++){
			// 	IIC_Send_Byte(buffer[i]);
			// }
			// IIC_Stop();
		
            IIC_Write_len_Byte(I2C1_BASE, OLED_ADDR, buffer[0], buf_idx-1, &buffer[1]);


			break;
			
		case U8X8_MSG_BYTE_SET_DC:
			/* ignored for i2c */
			break;
		
		default:
			return 0;
	  }
	  return 1;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg){
		
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
	    break;

	case U8X8_MSG_DELAY_MILLI:
		// vTaskDelay(arg_int);//this maybe can abandon directly
	    break;
		
	case U8X8_MSG_GPIO_I2C_CLOCK:		
        break;							
		
    case U8X8_MSG_GPIO_I2C_DATA:			
        break;
		
	default:	
		return 0;
	}
	return 1; // command processed successfully.
}
