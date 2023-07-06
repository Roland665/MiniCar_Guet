#include "Filters.h"

/**
 * @brief    一阶低通滤波算法
 * @param    newValue: 新采样的数据
 * @param    oldValue: 上一个滤波输出值
 * @param    alpha   : 滤波系数
 * @retval   本次滤波输出值
 */
float Filter(float newValue, float oldValue, float alpha)
{
    newValue = (alpha * newValue + (1 - alpha) * oldValue);
    return newValue;
}
