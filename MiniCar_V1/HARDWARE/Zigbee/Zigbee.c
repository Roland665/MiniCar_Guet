#include "Zigbee.h"


/*Begin of ȫ�ֱ���*/
u8 SelfLongAddr[8] = {0};//Zigbee�豸����ַ
u8 SelfShortAddr[2] = {0x00,0x00};//Zigbee�豸�̵�ַ Э������ʼ��Ϊ0xFF 0xFF���ն˳�ʼ��Ϊ0
u8 EnterModeFlag = 2;//Zigbeeģ��ģʽ�л���־λ����������ģʽ��0��͸��ģʽ��1
u8 PANID[2] = {0x00,0x00};//Zigbee����PANID,��ʼ��Ϊ0x00 0x00
u8 NetFlag = 0;//�������־λ���򿪳ɹ���1
u8 GetStateFlag = 0;//��ȡģ��״̬��־λ����ȡ�ɹ���1
u8 ReadySetTargetFlag = 1;//�ɹ�����͸��Ŀ���־λ�����óɹ���1(��Э������ͬ������������һ������)
u8 SetSendTargetFlag = 0;//����͸��Ŀ���־λ����������������Ŀ��̵�ַ��������Ŀ��˿ڣ��̵�ַ���óɹ�����1���˿����óɹ�����0
u8 SetTypeFlag = 0;//�ɹ������豸����Ϊ�ն˱�־λ�����óɹ���1
u8 ZigbeeChannel = 0;//Zigbeeģ���ŵ�����������ܶ�ȡ
/*End of ȫ�ֱ���*/


/**
  * @brief		�л�Zigbeeģʽ
  * @param		ModeNum -> 0Ϊ����HEXָ��ģʽ��1Ϊ����͸��ģʽ
  * @retval		void
  */

void Zigbee_Change_Mode(u8 modeNum){
	u8 EnterMode0[] = {0x2B, 0x2B, 0x2B};//Zigbee͸��ģʽ�л�HEXָ��ģʽ 0ΪHEXָ��ģʽ
	u8 EnterMode1[] = {0x55, 0x07, 0x00, 0x11, 0x00, 0x03, 0x00, 0x01, 0x13};//Zigbee HEXָ��ģʽ�л�͸��ģʽ 1Ϊ����͸��ģʽ
	u8 i;
	if(modeNum == 0){
		while(EnterModeFlag != 0){
			delay_ms(100);
			for(i = 0; i < 3;i++){
				USART_SendData(USART2, EnterMode0[i]);         //�򴮿�1��������
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
		}
	}
	else if(modeNum == 1){
		//����͸��ģʽ
		while(EnterModeFlag != 1){
			delay_ms(50);
			for(i = 0; i < 9;i++){
				USART_SendData(USART2, EnterMode1[i]);         //�򴮿�1��������
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
		}
	}
}

/**
  * @brief		����Ŀ��̵�ַΪ0x00 0x00(��Э����)��Ŀ��˿�Ϊ01
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
			USART_SendData(USART1, SetDirection[i]);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
	}
	while(SetSendTargetFlag != 0){
		delay_ms(10);
		for(i = 10; i < 19; i++){
			USART_SendData(USART1, SetDirection[i]);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
	}
	delay_ms(50);
	ReadySetTargetFlag = 1;
}



/**
  * @brief		��Zigbeeģ������磨�նˣ�����һ��ָ��PANID���������磩
  * @param		void
  * @retval	    void
  */

void Zigbee_Open_Net(void){
	u8 i;
	u8 NetStart[] = {0x55, 0x04, 0x00, 0x02, 0x00, 0x02,};
	NetFlag = 0;
	while(NetFlag != 1){
		for(i = 0; i < 6;i++){
			USART_SendData(USART1, NetStart[i]);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
		delay_ms(100);
	}
}


/**
  * @brief		��Zigbeeģ������磨�նˣ�����һ��ָ��PANID���������磩
  * @param		void
  * @retval	    void
  */

void Zigbee_Close_Net(void){
	u8 i;
	u8 NetClose[] = {0x55, 0x03, 0x00, 0x03, 0x03};
	NetFlag = 0;
	delay_ms(10);
	for(i = 0; i < 5;i++){
		USART_SendData(USART1, NetClose[i]);         //�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
	delay_ms(50);
	for(i = 0; i < 5;i++){
		USART_SendData(USART1, NetClose[i]);         //�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
}

/**
  * @brief		��ѯģ�鵱ǰ״̬����ȡ������
  * @param		void
  * @retval	    1->�ɹ���ȡ״̬
  */
void Zigbee_Get_State(void){
	u8 i;
	u8 GetState[] = {0x55, 0x03, 0x00, 0x00, 0x00};//��ѯZigbeeģ�鵱ǰ״̬
	GetStateFlag = 0;
	while(GetStateFlag != 1){
		delay_ms(10);
		for(i = 0; i < 5;i++){
			USART_SendData(USART1, GetState[i]);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
	}
}

/**
  * @brief		��Э���������Լ��ĳ��̵�ַ
  * @param		void
  * @retval	    1->���ͳɹ���Э������Ӧ��
  */

void Zigbee_Update_OnlineFlag(void){
	
//	AckFlag = 0;
//	WaitTime = 0;
//	while(AckFlag != 1){
//		Send_Custom_Data(0x00,3,SelfShortAddr);
//		delay_ms(200);//�������ݽ����ӳ�,����Ƶ�����͵����п�����ӵ��
//		if(WaitTime >= 10){//10sû�յ�Ӧ��ֱ���˳�����ʾΪû������
//			OnlineFlag = 0;
//			return ;
//		}
//	}
//	OnlineFlag = 1;
}

/**
  * @brief		ģ��ָ���������
  * @param		void
  * @retval		void
  */

void Zigbee_Restore_Factory_Setting(void){
	u8 i;
	u8 checkCode = 0;
	u8 RestoreFactorySetting[] = {0x55 ,0x07 ,0x00 ,0x04 ,0x02 ,0x00 ,0x00 ,0x00 ,0x00};//�ָ�����ָ��
	RestoreFactorySetting[5] = PANID[0];
	RestoreFactorySetting[6] = PANID[1];	
	RestoreFactorySetting[7] = ZigbeeChannel;	
	//�������������BBCУ����
	for(i = 0; i < RestoreFactorySetting[1] - 1; i++){
		checkCode = checkCode^RestoreFactorySetting[2+i];
	}
    RestoreFactorySetting[8] = checkCode;//У���븳ֵ
	delay_ms(10);
	for(i = 0; i < 9;i++){
		USART_SendData(USART1, RestoreFactorySetting[i]);         //�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
	delay_ms(500);
	//�����α�ִ֤�гɹ�
	
}

/**
  * @brief		ģ�鸴λ(�ָ������������ǰ��������)
  * @param		void
  * @retval		void
  */

void Zigbee_Restart(void){
	u8 i;
	u8 Restart[] = {0x55 ,0x07 ,0x00 ,0x04 ,0x00 ,0xFF ,0xFF ,0x00 ,0x04 };//�ָ�����ָ��
	delay_ms(10);
	for(i = 0; i < 9;i++){
		USART_SendData(USART1, Restart[i]);         //�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
	delay_ms(500);
	for(i = 0; i < 9;i++){
		USART_SendData(USART1, Restart[i]);         //�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
	//�����α�ִ֤�гɹ�
	
}


/**
  * @brief		����ģ������Ϊ�ն�
  * @param		void
  * @retval		void
  */

void Zigbee_Set_Type_To_Active_Terminal(void){
	u8 i;
	u8 Set_Type_To_Terminal[] = {0x55 ,0x04 ,0x00 ,0x05 ,0x02 ,0x07};//����ģ������Ϊ�ն�ָ��
	SetTypeFlag = 0;
	while(SetTypeFlag != 1){
		delay_ms(10);
		for(i = 0; i < 6;i++){
			USART_SendData(USART1, Set_Type_To_Terminal[i]);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
	}
}

/**
  * @brief		����Zigbee�ķ�������
  * @param		void
  * @retval		void
  */


void Zigbee_Analyse_Command_Data(){
	if (model == E18){
		if(USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x00){//55 2A 00 00 00
			if(USART1_RX_BUF[4] == 0x00){//��������ȡһ���豸��Ϣ
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
//				ZigbeeOnlineFlag = 1;//������
			}
			else if(USART1_RX_BUF[4] == 0xFF){//δ����
				GetStateFlag = 1;
//				ZigbeeOnlineFlag = 0;//δ����
			}
		}
		else if(USART1_RX_BUF[1] == 0x04 && USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x02 && USART1_RX_BUF[4] == 0x00 && USART1_RX_BUF[5] == 0x02){//55 04 00 02 00 02
			NetFlag = 1;//�ж������Ѵ�
		}
		else if(USART1_RX_BUF[1] == 0x04 && USART1_RX_BUF[2] == 0x00 && USART1_RX_BUF[3] == 0x05 && USART1_RX_BUF[4] == 0x00 && USART1_RX_BUF[5] == 0x05){//55 04 00 05 00 05 
			SetTypeFlag = 1;//�жϳɹ������ն�����
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
