#include "Zigbee.h"


/*Begin of 全局变量*/
u8 SelfLongAddr[8] = {0};//Zigbee设备长地址
u8 SelfShortAddr[2] = {0x00,0x00};//Zigbee设备短地址 协调器初始化为0xFF 0xFF，终端初始化为0
u8 EnterModeFlag = 2;//Zigbee模块模式切换标志位，进入配置模式置0，透传模式置1
u8 PANID[2] = {0x00,0x00};//Zigbee局网PANID,初始化为0x00 0x00
u8 NetFlag = 0;//打开网络标志位，打开成功置1
u8 GetStateFlag = 0;//读取模块状态标志位，读取成功置1
u8 ReadySetTargetFlag = 1;//成功设置透传目标标志位，设置成功置1(与协调器的同名变量作用有一定出入)
u8 SetSendTargetFlag = 0;//设置透传目标标志位，分两步，先设置目标短地址，再设置目标端口，短地址设置成功后置1，端口设置成功后置0
u8 SetTypeFlag = 0;//成功设置设备类型为终端标志位，设置成功置1
u8 ZigbeeChannel = 0;//Zigbee模块信道，配网后才能读取
/*End of 全局变量*/


/**
  * @brief		切换Zigbee模式
  * @param		ModeNum -> 0为进入HEX指令模式，1为进入透传模式
  * @retval		void
  */

void Zigbee_Change_Mode(u8 modeNum){
	u8 EnterMode0[] = {0x2B, 0x2B, 0x2B};//Zigbee透传模式切换HEX指令模式 0为HEX指令模式
	u8 EnterMode1[] = {0x55, 0x07, 0x00, 0x11, 0x00, 0x03, 0x00, 0x01, 0x13};//Zigbee HEX指令模式切换透传模式 1为数据透传模式
	u8 i;
	if(modeNum == 0){
		while(EnterModeFlag != 0){
			delay_ms(100);
			for(i = 0; i < 3;i++){
				USART_SendData(USART2, EnterMode0[i]);         //向串口1发送数据
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			}
		}
	}
	else if(modeNum == 1){
		//进入透传模式
		while(EnterModeFlag != 1){
			delay_ms(50);
			for(i = 0; i < 9;i++){
				USART_SendData(USART2, EnterMode1[i]);         //向串口1发送数据
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			}
		}
	}
}

/**
  * @brief		设置目标短地址为0x00 0x00(即协调器)和目标端口为01
  * @param		void
  * @retval		void
  */

void Zigbee_Set_Send_Target(){
	u8 i;
	u8 SetDirection[] = {0x55, 0x08, 0x00, 0x11, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x55, 0x07, 0x00, 0x11, 0x00, 0x02, 0x00, 0x01, 0x12};
	ReadySetTargetFlag = 0;
	while(SetSendTargetFlag != 1){
		delay_ms(10);
		for(i = 0; i < 10;i++){
			USART_SendData(USART1, SetDirection[i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
	}
	while(SetSendTargetFlag != 0){
		delay_ms(10);
		for(i = 10; i < 19; i++){
			USART_SendData(USART1, SetDirection[i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
	}
	delay_ms(50);
	ReadySetTargetFlag = 1;
}



/**
  * @brief		打开Zigbee模块的网络（终端：加入一个指定PANID的现有网络）
  * @param		void
  * @retval	    void
  */

void Zigbee_Open_Net(void){
	u8 i;
	u8 NetStart[] = {0x55, 0x04, 0x00, 0x02, 0x00, 0x02,};
	NetFlag = 0;
	while(NetFlag != 1){
		for(i = 0; i < 6;i++){
			USART_SendData(USART1, NetStart[i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
		delay_ms(100);
	}
}


/**
  * @brief		打开Zigbee模块的网络（终端：加入一个指定PANID的现有网络）
  * @param		void
  * @retval	    void
  */

void Zigbee_Close_Net(void){
	u8 i;
	u8 NetClose[] = {0x55, 0x03, 0x00, 0x03, 0x03};
	NetFlag = 0;
	delay_ms(10);
	for(i = 0; i < 5;i++){
		USART_SendData(USART1, NetClose[i]);         //向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
	delay_ms(50);
	for(i = 0; i < 5;i++){
		USART_SendData(USART1, NetClose[i]);         //向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

/**
  * @brief		查询模组当前状态（读取参数）
  * @param		void
  * @retval	    1->成功获取状态
  */
void Zigbee_Get_State(void){
	u8 i;
	u8 GetState[] = {0x55, 0x03, 0x00, 0x00, 0x00};//查询Zigbee模组当前状态
	GetStateFlag = 0;
	while(GetStateFlag != 1){
		delay_ms(10);
		for(i = 0; i < 5;i++){
			USART_SendData(USART1, GetState[i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
	}
}

/**
  * @brief		向协调器发送自己的长短地址
  * @param		void
  * @retval	    1->发送成功且协调器已应答
  */

void Zigbee_Update_OnlineFlag(void){
	
//	AckFlag = 0;
//	WaitTime = 0;
//	while(AckFlag != 1){
//		Send_Custom_Data(0x00,3,SelfShortAddr);
//		delay_ms(200);//考虑数据接收延迟,避免频繁发送导致中控数据拥堵
//		if(WaitTime >= 10){//10s没收到应答，直接退出，表示为没有入网
//			OnlineFlag = 0;
//			return ;
//		}
//	}
//	OnlineFlag = 1;
}

/**
  * @brief		模组恢复出厂设置
  * @param		void
  * @retval		void
  */

void Zigbee_Restore_Factory_Setting(void){
	u8 i;
	u8 checkCode = 0;
	u8 RestoreFactorySetting[] = {0x55 ,0x07 ,0x00 ,0x04 ,0x02 ,0x00 ,0x00 ,0x00 ,0x00};//恢复出厂指令
	RestoreFactorySetting[5] = PANID[0];
	RestoreFactorySetting[6] = PANID[1];	
	RestoreFactorySetting[7] = ZigbeeChannel;	
	//计算上面命令的BBC校验码
	for(i = 0; i < RestoreFactorySetting[1] - 1; i++){
		checkCode = checkCode^RestoreFactorySetting[2+i];
	}
    RestoreFactorySetting[8] = checkCode;//校验码赋值
	delay_ms(10);
	for(i = 0; i < 9;i++){
		USART_SendData(USART1, RestoreFactorySetting[i]);         //向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
	delay_ms(500);
	//发两次保证执行成功
	
}

/**
  * @brief		模组复位(恢复出厂后打开网络前必须重启)
  * @param		void
  * @retval		void
  */

void Zigbee_Restart(void){
	u8 i;
	u8 Restart[] = {0x55 ,0x07 ,0x00 ,0x04 ,0x00 ,0xFF ,0xFF ,0x00 ,0x04 };//恢复出厂指令
	delay_ms(10);
	for(i = 0; i < 9;i++){
		USART_SendData(USART1, Restart[i]);         //向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
	delay_ms(500);
	for(i = 0; i < 9;i++){
		USART_SendData(USART1, Restart[i]);         //向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
	//发两次保证执行成功
	
}


/**
  * @brief		设置模组类型为终端
  * @param		void
  * @retval		void
  */

void Zigbee_Set_Type_To_Active_Terminal(void){
	u8 i;
	u8 Set_Type_To_Terminal[] = {0x55 ,0x04 ,0x00 ,0x05 ,0x02 ,0x07};//设置模组类型为终端指令
	SetTypeFlag = 0;
	while(SetTypeFlag != 1){
		delay_ms(10);
		for(i = 0; i < 6;i++){
			USART_SendData(USART1, Set_Type_To_Terminal[i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
	}
}

/**
  * @brief		分析Zigbee的反馈命令
  * @param		void
  * @retval		void
  */


void Zigbee_Analyse_Command_Data(){
	if (model == E18){
		if(USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x00){//55 2A 00 00 00
			if(USART1_RX_BUF[4] == 0x00){//已配网获取一波设备信息
				SelfLongAddr[0] = USART1_RX_BUF[6];
				SelfLongAddr[1] = USART1_RX_BUF[7];
				SelfLongAddr[2] = USART1_RX_BUF[8];
				SelfLongAddr[3] = USART1_RX_BUF[9];
				SelfLongAddr[4] = USART1_RX_BUF[10];
				SelfLongAddr[5] = USART1_RX_BUF[11];
				SelfLongAddr[6] = USART1_RX_BUF[12];
				SelfLongAddr[7] = USART1_RX_BUF[13];
				ZigbeeChannel = USART1_RX_BUF[14];
				PANID[0] = USART1_RX_BUF[15];
				PANID[1] = USART1_RX_BUF[16];
				SelfShortAddr[0] = USART1_RX_BUF[17];
				SelfShortAddr[1] = USART1_RX_BUF[18];
				GetStateFlag = 1;
//				ZigbeeOnlineFlag = 1;//已入网
			}
			else if(USART1_RX_BUF[4] == 0xFF){//未组网
				GetStateFlag = 1;
//				ZigbeeOnlineFlag = 0;//未入网
			}
		}
		else if(USART1_RX_BUF[1] == 0x04 && USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x02 && USART1_RX_BUF[4] == 0x00 && USART1_RX_BUF[5] == 0x02){//55 04 00 02 00 02
			NetFlag = 1;//判断网络已打开
		}
		else if(USART1_RX_BUF[1] == 0x04 && USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x05 && USART1_RX_BUF[4] == 0x00 && USART1_RX_BUF[5] == 0x05){//55 04 00 05 00 05 
			SetTypeFlag = 1;//判断成功设置终端类型
		}
		else if(ReadySetTargetFlag == 1 && USART1_RX_BUF[1] == 0x04 && USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x11 && USART1_RX_BUF[4] == 0x00 && USART1_RX_BUF[5] == 0x11){//55 04 00 11 00 11 
			EnterModeFlag = 1;
		}
		else if(ReadySetTargetFlag == 0 && USART1_RX_BUF[1] == 0x04 && USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x11 && USART1_RX_BUF[4] == 0x00 && USART1_RX_BUF[5] == 0x11){//55 04 00 11 00 11 
			if(SetSendTargetFlag == 0) SetSendTargetFlag = 1;
			else SetSendTargetFlag = 0;
		}
		else if(USART1_RX_BUF[1] == 0x03 && USART1_RX_BUF[2] == 0xFF && USART1_RX_BUF[3] == 0xFE && USART1_RX_BUF[4] == 0x01){//55 03 FF FE 01
			if(EnterModeFlag != 0) EnterModeFlag = 0;
		}
	}
	else if(model == E180){
		
	}
}
