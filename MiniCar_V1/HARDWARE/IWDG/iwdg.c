#include "iwdg.h"

//��ʼ���������Ź�
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
//��Ƶ����=4*2^prer.�����ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/40 (ms).
/**

  * @brief    ��ʼ���������Ź�    
  * @brief    ʱ�����(���):Tout=((4*2^prer)*rlr)/40 (ms)
  * @param    prer:��Ƶ��(0~7),��Ƶ����=4*2^prer(���ֵ��256)
  * @param    rlr :��װ�ؼĴ���ֵ(0~2047)
  * @retval   void
  */

void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	
	IWDG_SetPrescaler(prer);  //����IWDGԤ��Ƶֵ:����IWDGԤ��ƵֵΪ64
	
	IWDG_SetReload(rlr);  //����IWDG��װ��ֵ
	
	IWDG_ReloadCounter();  //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	
	IWDG_Enable();  //ʹ��IWDG
}


//ι�������Ź�
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}


