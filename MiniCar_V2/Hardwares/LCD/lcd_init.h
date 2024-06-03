#ifndef __LCD_INIT_H
#define __LCD_INIT_H
#include "sys.h" 

#define USE_HORIZONTAL 1  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#define delay_ms(n); 		SysCtlDelay(n*(SysCtlClockGet()/3000));

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 128
#endif

#define SCK_GPIO 			GPIOB_BASE
#define SCK_PIN 			GPIO_PIN_0
#define SCK_GPIOPERIPH 		SYSCTL_PERIPH_GPIOB

		
#define SDA_GPIO 			GPIOB_BASE
#define SDA_PIN 			GPIO_PIN_1
#define SDA_GPIOPERIPH 		SYSCTL_PERIPH_GPIOB

		
#define DC_GPIO 			GPIOB_BASE
#define DC_PIN 				GPIO_PIN_2
#define DC_GPIOPERIPH 		SYSCTL_PERIPH_GPIOB

		
#define RES_GPIO 			GPIOB_BASE
#define RES_PIN 			GPIO_PIN_3
#define RES_GPIOPERIPH 		SYSCTL_PERIPH_GPIOB

		
#define CS_GPIO 			GPIOC_BASE
#define CS_PIN 				GPIO_PIN_2
#define CS_GPIOPERIPH 		SYSCTL_PERIPH_GPIOC

		
#define LED_GPIO 			GPIOC_BASE
#define LED_PIN 			GPIO_PIN_3
#define LED_GPIOPERIPH 		SYSCTL_PERIPH_GPIOC

#define LCD_SCLK_Clr()		GPIOPinWrite(SCK_GPIO, SCK_PIN, 0)							
#define LCD_SCLK_Set()		GPIOPinWrite(SCK_GPIO, SCK_PIN, SCK_PIN)

#define LCD_MOSI_Clr()      GPIOPinWrite(SDA_GPIO, SDA_PIN, 0)
#define LCD_MOSI_Set()      GPIOPinWrite(SDA_GPIO, SDA_PIN, SDA_PIN)

#define LCD_RES_Clr()		GPIOPinWrite(RES_GPIO, RES_PIN, 0)
#define LCD_RES_Set()       GPIOPinWrite(RES_GPIO, RES_PIN, RES_PIN)

#define LCD_DC_Clr()  		GPIOPinWrite(DC_GPIO, DC_PIN, 0)
#define LCD_DC_Set()        GPIOPinWrite(DC_GPIO, DC_PIN, DC_PIN)
 		     
#define LCD_CS_Clr()  		GPIOPinWrite(CS_GPIO, CS_PIN, 0)
#define LCD_CS_Set()        GPIOPinWrite(CS_GPIO, CS_PIN, CS_PIN)

#define LCD_BLK_Clr() 		GPIOPinWrite(LED_GPIO, LED_PIN, 0)
#define LCD_BLK_Set()       GPIOPinWrite(LED_GPIO, LED_PIN, LED_PIN)

void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化

#endif

