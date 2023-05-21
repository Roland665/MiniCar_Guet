#include "tea.h"
/**

  * @brief    tea�����㷨
  * @param    v:Ҫ���ܵ�����,����Ϊ8�ֽ�
  * @param    k:�����õ�key,����Ϊ16�ֽ�
  * @retval   void
  */
static void tea_encrypt(u32 *v,u32 *k){      
    u32 y = v[0],z = v[1],sum = 0,i;
    u32 delta = 0x9e3779b9;
    u32 a = k[0],b = k[1],c = k[2],d = k[3];        
    for (i = 0;i < 32; i++){
        sum += delta;
        y += ((z << 4) + a) ^ (z + sum) ^ ((z >> 5) + b);          
        z += ((y << 4) + c) ^ (y + sum) ^ ((y >> 5) + d);      
    }      
        v[0] = y;      
        v[1] = z;  
}

/**

  * @brief   tea�����㷨
  * @param    v:Ҫ���ܵ�����,����Ϊ8�ֽ� 
  * @param    k:�����õ�key,����Ϊ16�ֽ�
  * @retval   void
  */
static void tea_decrypt(u32 *v,u32 *k){      
    u32 y = v[0],z = v[1],sum = 0xC6EF3720,i;       
    u32 delta = 0x9e3779b9;                  
    u32 a = k[0],b = k[1],c = k[2],d = k[3];      
    for (i = 0;i < 32;i++){                                   
        z -= ((y << 4) + c) ^ (y + sum) ^ ((y >> 5) + d);          
        y -= ((z << 4) + a) ^ (z + sum) ^ ((z >> 5) + b);          
        sum -= delta;                           
        }      
        v[0] = y;
        v[1] = z;
}

/**

  * @brief   �����㷨
  * @param   src:Դ����,��ռ�ռ����Ϊ8�ֽڵı���.������ɺ�����Ҳ�������  
  * @param   size_src:Դ���ݴ�С,��λ�ֽ�
  * @param   key:��Կ,16�ֽ�
  * @retval  ���ĵ��ֽ���
  */
 
uint16_t encrypt(uint8_t *src,uint16_t size_src,uint8_t *key){      
    uint8_t a = 0;
    uint16_t i = 0;
    uint16_t num = 0;
    //�����Ĳ���Ϊ8�ֽڵı���      
    a = size_src % 8;
    if (a != 0){          
        for (i = 0;i < 8 - a;i++){
            src[size_src++] = 0;          
            }      
        }
    //����      
    num = size_src / 8;      
    for (i = 0;i < num;i++){
        tea_encrypt((u32 *)(src + i * 8),(u32 *)key);      
    }      
    return size_src;  
}

/**

  * @brief   �����㷨
  * @param   src:Դ����,��ռ�ռ����Ϊ8�ֽڵı���.������ɺ�����Ҳ�������
  * @param   size_src:Դ���ݴ�С,��λ�ֽ�
  * @param   key:��Կ,16�ֽ�
  * @retval  ���ĵ��ֽ���,���ʧ��,����0
  */

uint16_t decrypt(uint8_t *src,uint16_t size_src,uint8_t *key){      
    uint16_t i = 0;
    uint16_t num = 0;
    //�жϳ����Ƿ�Ϊ8�ı���
    if (size_src % 8 != 0){
        return 0;
    }
    //����      
    num = size_src / 8;      
    for (i = 0;i < num;i++){          
        tea_decrypt((u32 *)(src + i * 8),(u32 *)key);      
    }      
    return size_src;  
}
