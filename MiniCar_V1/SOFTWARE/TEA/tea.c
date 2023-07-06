#include "tea.h"
/**

  * @brief    tea加密算法
  * @param    v:要加密的数据,长度为8字节
  * @param    k:加密用的key,长度为16字节
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

  * @brief   tea解密算法
  * @param    v:要解密的数据,长度为8字节 
  * @param    k:解密用的key,长度为16字节
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

  * @brief   加密算法
  * @param   src:源数据,所占空间必须为8字节的倍数.加密完成后密文也存放在这  
  * @param   size_src:源数据大小,单位字节
  * @param   key:密钥,16字节
  * @retval  密文的字节数
  */
 
uint16_t encrypt(uint8_t *src,uint16_t size_src,uint8_t *key){      
    uint8_t a = 0;
    uint16_t i = 0;
    uint16_t num = 0;
    //将明文补足为8字节的倍数      
    a = size_src % 8;
    if (a != 0){          
        for (i = 0;i < 8 - a;i++){
            src[size_src++] = 0;          
            }      
        }
    //加密      
    num = size_src / 8;      
    for (i = 0;i < num;i++){
        tea_encrypt((u32 *)(src + i * 8),(u32 *)key);      
    }      
    return size_src;  
}

/**

  * @brief   解密算法
  * @param   src:源数据,所占空间必须为8字节的倍数.解密完成后明文也存放在这
  * @param   size_src:源数据大小,单位字节
  * @param   key:密钥,16字节
  * @retval  明文的字节数,如果失败,返回0
  */

uint16_t decrypt(uint8_t *src,uint16_t size_src,uint8_t *key){      
    uint16_t i = 0;
    uint16_t num = 0;
    //判断长度是否为8的倍数
    if (size_src % 8 != 0){
        return 0;
    }
    //解密      
    num = size_src / 8;      
    for (i = 0;i < num;i++){          
        tea_decrypt((u32 *)(src + i * 8),(u32 *)key);      
    }      
    return size_src;  
}
