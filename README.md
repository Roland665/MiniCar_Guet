# 无刷电机驱动轮

## 无刷电机转动原理

与步进电机极其相似，都是用电磁力勾引永磁铁动(转子)起来，从而产生旋转

三相无刷电机就是有A、B、C三个电磁铁相

### Si x-step Commutation

![image-20230425134503919](F:\TyporaMarks\校电赛.assets\image-20230425134503919.png)

## 无刷电机转子位置

其实判断转子位置的方法挺多，用传感器也行，不用传感器也行。先说用传感器的，传感器一般用霍尔传感器（Hall Sensor）。

### 霍尔传感器

霍尔传感器通过霍尔效应（Hall Effect），能检测出磁场强度的变化。根据高中物理所学的左手定则（用来判断带电导体在磁场中的受力方向），在霍尔传感器所在的回路中，磁场使带电粒子的运动发生偏转，带电粒子“撞到”霍尔传感器的两边，产生电位差。这时就可以用电压计接到霍尔传感器的两边，检测出这种电压变化，从而检测出磁场强度的变化。

假设传感器是每隔120°安装的。假设转子N极划过霍尔传感器的感应区域时，霍尔传感器的输出电压为高（一般5V）。反之为低。

在三相无刷电机中，根据HA，HB，HC的电平，可以知道转子所处位置的角度。

### 电气角度和机械角度的关系

机械角度就是电动机转子实际转过的角度。

电气角度和机械角度的关系与转子的极对数有关。

因为实际上线圈生成的磁场要吸引的是转子的磁极。所以对于电机的转动控制来说，我们只关心电气角度就好。

电气角度 = 极对数 x 机械角度

# 电路设计

## 基础部分

### 电源供电

- 防止浪涌

# PID

[ PID算法原理 一图看懂PID的三个参数_pid控制器的三个参数_我是唐的博客-CSDN博客](https://blog.csdn.net/qq_41673920/article/details/84860697)

上面的连接讲的十分详细

## 简介

PID，就是“比例（proportional）、积分（integral）、微分（derivative）”

P，I，D是三种不同的调节作用，既可以单独使用（P，I，D），也可以两个两个用（PI，PD），也可以三个一起用（PID）。

PID控制器的三个最基本的参数：**kP**, **kI**, **kD**

**kP**

P 就是让偏差（目标减去当前）与调节装置的“调节力度”，建立一个一次函数的关系，就可以实现最基本的“比例”控制了~
kP越大，调节作用越激进，kP调小会让调节作用更保守。

**kD**

D 实际是一个控制作用，让被控制的物理量的“变化速度”趋于0，即类似于“阻尼”的作用。

**kI**

I 是设置一个积分量。只要偏差存在，就不断地对偏差进行积分（累加），并反应在调节力度上。

所以，I 的作用就是，减小静态情况下的误差，让受控物理量尽可能接近目标值。

I在使用时还有个问题：需要设定积分限制。防止在刚开始加热时，就把积分量积得太大，难以控制。

PID有**位置式算法**和**增量式算法**

## PID算法

PID算法的离散公式有两种形式：位置式和增量式。

**位置式PID算法的离散公式**是：

$$u(k)=K_p e(k)+K_i \sum_{i=0}^k e(i)+K_d \frac{e(k)-e(k-1)}{T}$$

其中，$u(k)$是控制输出量，$e(k)$是误差，$K_p$是比例系数，$K_i$是积分系数，$K_d$是微分系数，$T$是采样周期。

**增量式PID算法的离散公式**是：

$$\Delta u(k)=u(k)-u(k-1)=K_p [e(k)-e(k-1)]+K_i e(k)+K_d [e(k)-2e(k-1)+e(k-2)]$$

其中，$\Delta u(k)$是控制量的增量。

## PID代码实现

```c
typedef struct{
	float kp;           //比率系数
	float ki;           //积分系数
	float kd;           //微分系数
    float intergral;    //积分值(实际就是每次err的累加)
    float err_old;      //上一次计算的差值
    float err_new;      //本次的新差值
}PID;

/**
 * @brief    经典PID算法(位置式)
 * @param    pid: PID对象，里面应当存储了计算必要的数据
 * @param    err_new:最新的差值
 * @retval   
 */
float PID_Classic(PID* pid, float err_new){
    float result;
    pid->err_new = err_new;
    pid->intergral += pid->err_new;
    result = pid->kp*pid->err_new + pid->ki*pid->intergral + pid->kd*(pid->err_old - pid->err_new);
    pid->err_old = pid->err_new;
    return result;
}

int main(){
    rReplaceV = PID_Classic(rVelocityPID, rTargetV-rVelocity);
}
/*
上面 rReplaceV 就是直接拿来用的速度，rTargetV 是目标速度，rVelocity是当前速度
*/
```

## PID调参

PID的三个系数没有固定的值，需要根据具体的系统和控制目标进行调节。一般来说，可以采用经验法或试错法来确定合适的参数。经验法是根据PID控制器的作用原理和各个参数对系统性能的影响，先给出一个大致的范围，然后再进行微调。试错法是根据系统的响应曲线，不断地调整参数，直到达到满意的效果。¹²

对于智能小车的速度闭环控制，可以参考以下步骤来调节PID参数：

- 首先去掉积分项和微分项，只保留比例项，即令Ki=0，Kd=0。然后逐渐增加Kp的值，直到系统出现持续振荡或超调过大。
- 然后减小Kp的值，使系统稳定下来，并观察是否有静态误差。如果有静态误差，就增加Ki的值，使积分项消除误差。但是Ki不能太大，否则会导致积分饱和或过渡过程过长。
- 最后增加Kd的值，使微分项抑制误差变化和噪声干扰，并提高系统的响应速度和稳定性。但是Kd也不能太大，否则会放大噪声或引起高频振荡。

具体的参数值需要根据实际情况进行调整，可以参考一些经验公式或在线工具来辅助选择。例如：

- Kp = 1.3
- Ki = 0.15
- Kd = 0.03

# 滤波

一个简单的公式：

```math
Y(n)=αX(n) + (1-α)Y(n-1)
α=滤波系数；
X(n)=本次采样值；
Y(n-1)=上次滤波输出值；
Y(n)=本次滤波输出值。 
```

上式也叫**一阶低通滤波**

# V1.0

此版本已经是废案了，任务逻辑不恰

## FreeRTOS

使用FreeRTOS作为基础框架，减少程序逻辑构思时间

**所有对象**都是静态创建

### 计数信号量

使用两个计数信号量，来记录每秒光电测速传感器获取的脉冲

### 消息队列

使用一个消息队列来存储待修正的车速，消息固定2个位，第一个表示左车轮车速，第二个表示右车轮车速

使用一个消息队列来存储控制命令，串口接收到就存入

## 直流电机

### GPIO

TIM3提供的两路pwm波来使能电机，

直接短接 IN2与VCC 和 IN3与VCC 左电机反转，右电机正转，小车只能前进无法后退

### 电机调速步骤

1. 调速的前提是测速，先获取准确的数据，必要时采用一定的滤波
2. 让程序固定在一个速度值，让电机向着这个值跑，超速了就减速，低速了就加速，调节的是一个delay，每隔多久让电机看一下和目标值的差别，一般也会进行一个近似比较，误差在x之内认为相等

## 测速传感器

## Zigbee

windows上位机调好了直接用

## OLED

使用stm32上的硬件IIC驱动

## IO占用

PA0						：蜂鸣器
PA1						：金属探测
PA2(USART2_TX)：Zigbee TX(P14) 
PA3(USART2_RX)：Zigbee RX(P15)
PA4						：左轮传感器OUT口
PA5						：右轮传感器OUT口
PA6						：左电机EN1
PA7						：右电机EN2

PB10：1号红外（从左向右排序，其中4号不用）
PB11：2号红外
PB12：3号红外
PB13：5号红外
PB14：6号红外
PB15：7号红外

程序测试用LED PC13

IIC1_SCL PB6
IIC1_SDA PB7

## 中断优先级

串口1：    2   调试用
串口2：    2   Zigbee
外部1：    2   金属探测
定时器2：3   系统计时



# v1.1

## FreeRTOS

使用FreeRTOS作为基础框架，减少程序逻辑构思时间

**所有对象**都是静态创建

### 计数信号量

使用两个计数信号量，来记录每秒光电测速传感器获取的脉冲

### 消息队列

使用一个消息队列来存储待修正的车速，消息固定2个位，第一个表示左车轮车速，第二个表示右车轮车速

使用一个消息队列来存储控制命令，串口接收到就存入

使用一个消息队列来存储待发出的数据，主要是检测到的硬币数量

## 直流电机

### GPIO

TIM3提供的两路pwm波来使能电机，

直接短接 IN2与VCC 和 IN3与VCC 左电机反转，右电机正转，小车只能前进无法后退

### 电机调速步骤

- 任务一

  调速的前提是测速，先获取准确的数据，必要时采用一定的滤波

- 任务二
  1. 检测小车所处黑线的偏位
  2. 对不同的偏位有固定的目标速度
  3. 让电机向着目标速度跑，超速了就减速，低速了就加速，调节的是PWM

## 测速传感器

光电传感器每产生一个脉冲，程序通过计算脉冲前后时间间隔得出大致的瞬时速度，理论合理，但是有可能一个脉冲完了程序还没执行到测速任务，于是采用计数信号量，在光电传感器每产生一个及以上脉冲后，计算时除去计数信号量的累计值

## Zigbee

windows上位机调好了直接用透传模式，串口收发就可以

## OLED

使用stm32上的硬件IIC驱动，并基于U8G2库显示图像

## IO占用

PA0						：蜂鸣器
PA1						：金属探测
PA2(USART2_TX)：Zigbee RX(P14) 
PA3(USART2_RX)：Zigbee TX(P15)
PA4						：左轮传感器OUT口
PA5						：右轮传感器OUT口
PA6						：左电机EN1
PA7						：右电机EN2

PB10：1号红外（从左向右排序，其中4号不用）
PB11：2号红外
PB12：3号红外
PB13：5号红外
PB14：6号红外
PB15：7号红外

PC13：程序测试用LED 

PB6：IIC1_SCL 
PB7：IIC1_SDA

## 中断优先级

串口1：    2   调试用
串口2：    2   Zigbee
定时器2：3   系统计时

# 通信规范

采用HEX指令

指令中的数值信息都是16进制

| 指令                  | 内容                                                   |
| --------------------- | ------------------------------------------------------ |
| **C1 C2 C3 C4 00**    | 停车                                                   |
| C1 C2 C3 C4 01 xx yy  | 切换外部控制模式，左轮以xx、右轮以yy * 0.01m/s速度转动 |
| **C1 C2 C3 C4 02**    | 切换自巡航模式（上电默认外部控制模式），即比赛开始     |
| C1 C2 C3 C4 03 xx yy  | 一阶偏移时左右轮目标速度                               |
| C1 C2 C3 C4 04 xx yy  | 二阶偏移时左右轮目标速度                               |
| C1 C2 C3 C4 05 xx yy  | 三阶偏移时左右轮目标速度                               |
| C1 C2 C3 C4 06 xx     | 调速时PWM增幅为xx                                      |
| C1 C2 C3 C4 07        | 加速                                                   |
| C1 C2 C3 C4 08        | 减速                                                   |
| **C1 C2 C3 C4 09 xx** | 当前检测到的硬币数为xx                                 |
| C1 C2 C3 C4 0A        | 进内道                                                 |
| **C1 C2 C3 C4 FF**    | 应答                                                   |





# 一些怪异问题

- 因为直流电机的EN引脚不接线时好像内部有上拉，所以会自动旋转，程序内部初始化与EN相连的GPIO需要尽早初始化并置0，否则程序甚至会死机(进入一个死循环)
- 车轮车速在0.9 m/s ~ 1.5m/s
- 硬件IIC驱动U8G2时，经常程序卡死在等待EV6，尝试多种方式解决问题
  - 加超时判断，等太久EV6就直接跳过                ×   //一次无EV6后，以后一直都没EV6，程序不卡了，但是U8G2没用了
  - 取消FreeRTOS                                                 √   //可以，但是没有RTOS还玩个屁
  - 降低引脚IIC频率(原先是400khz)                       √  //改成100khz雀石可以，但是动图好慢qwq
  - 把 I2C_DutyCycle 改成 I2C_DutyCycle_16_9  √   //修改后，IIC时钟频率可以在200khz，400还是不行，动图快了一些a.a
  - 引脚从复用推挽改复用开漏                               ×   //不行，EV8一样无法变成succese
- 车速最高为0.64m/s,最低为0.28
- C99可以随处定义变量固然好，但是在操作系统的任务中，只能在while(1)之前写定义语句，如果在while(1)中写，那么会爆栈
- v1.1 小车，供电电压推荐大于11.8v