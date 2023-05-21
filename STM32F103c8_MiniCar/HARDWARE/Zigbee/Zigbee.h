#ifndef __ZIGBEE_H
#define __ZIGBEE_H	
#include "sys.h"
#include "usart.h"	
#include "delay.h"
#include "led.h"

#define E18  1
#define E180 2

#define model E18 //这里选择一下使用的模块时E180还是E18

/*Begin of extern*/
extern u8 SelfLongAddr[8];//Zigbee设备长地址
extern u8 SelfShortAddr[2];//Zigbee设备短地址 协调器初始化为0xFF 0xFF，终端初始化为0
extern u8 PANID[2];
extern u8 SetTypeFlag;
extern u8 GetStateFlag;
extern u8 ZigbeeChannel;
extern u8 EnterModeFlag;//Zigbee模块模式切换标志位，进入配置模式置0，透传模式置1
/*End of extern*/

void Zigbee_Change_Mode(u8 modeNum);
void Zigbee_Set_Send_Target(void);
void Zigbee_Open_Net(void);
void Zigbee_Get_State(void);
void Zigbee_Restore_Factory_Setting(void);
void Zigbee_Restart(void);
void Zigbee_Set_Type_To_Active_Terminal(void);
void Zigbee_Analyse_Command_Data(void);
void Zigbee_Close_Net(void);
void Zigbee_Update_OnlineFlag(void);
#endif
