#ifndef __ZIGBEE_H
#define __ZIGBEE_H	
#include "sys.h"
#include "usart.h"	
#include "delay.h"
#include "led.h"

#define E18  1
#define E180 2

#define model E18 //����ѡ��һ��ʹ�õ�ģ��ʱE180����E18

/*Begin of extern*/
extern u8 SelfLongAddr[8];//Zigbee�豸����ַ
extern u8 SelfShortAddr[2];//Zigbee�豸�̵�ַ Э������ʼ��Ϊ0xFF 0xFF���ն˳�ʼ��Ϊ0
extern u8 PANID[2];
extern u8 SetTypeFlag;
extern u8 GetStateFlag;
extern u8 ZigbeeChannel;
extern u8 EnterModeFlag;//Zigbeeģ��ģʽ�л���־λ����������ģʽ��0��͸��ģʽ��1
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
