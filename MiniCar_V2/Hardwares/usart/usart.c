#include "usart.h"

//重定向printf函数
int fputc(int ch, FILE *f)
{
    UARTCharPut(UART0_BASE, ch);
    return (ch);
} 
int fgetc(FILE *f)
{
    int ch = UARTCharGet(UART0_BASE);
    return (ch);
}


u8 UART0_RX_BUF[UART_REC_LEN] = {0};  //串口接收缓冲区,最大UART_REC_LEN个字节.
u16 UART0_RX_STA = 0;//接收到的有效字节数目
u8 uart0RXTime = 0xFF;//串口消息间隔计时，初始化把时间拉满，表示没有收到新的消息

u8 UART3_RX_BUF[UART3_REC_LEN] = {0};  //串口接收缓冲区,最大UART_REC_LEN个字节.
u16 UART3_RX_STA = 0;//接收到的有效字节数目
u8 uart3RXTime = 0xFF;//串口消息间隔计时，初始化把时间拉满，表示没有收到新的消息

u8 UART5_RX_BUF[UART5_REC_LEN] = {0};  //串口接收缓冲区,最大UART_REC_LEN个字节.
u16 UART5_RX_STA = 0;//接收到的有效字节数目
u8 uart5RXTime = 0xFF;//串口消息间隔计时，初始化把时间拉满，表示没有收到新的消息

//初始化串口0和中断
void Uart0_Init(uint32_t baud){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART Peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, baud, 16000000);

    // Set usart int priorityset
    IntPrioritySet(INT_UART0,1);

    // Enable UART receive interrupt and receive timeout interrupt
    UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART0 interrupt
    MAP_IntEnable(INT_UART0);
}

//初始化串口1和中断
void Uart1_Init(uint32_t baud){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Enable UART Peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PC4_U1RX);
    MAP_GPIOPinConfigure(GPIO_PC5_U1TX);
    MAP_GPIOPinTypeUART(GPIOC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Use the internal 16MHz oscillator as the UART clock source.
    MAP_UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(1, baud, 16000000);//16Mhz

    // Set usart int priorityset
    MAP_IntPrioritySet(INT_UART1,1);

    // Enable UART receive interrupt and receive timeout interrupt
    MAP_UARTIntEnable(UART1_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART interrupt
    MAP_IntEnable(INT_UART1);
}

//初始化串口2和中断
void Uart2_Init(uint32_t baud){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Enable UART Peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PD6_U2RX);
    MAP_GPIOPinConfigure(GPIO_PD7_U2TX);
    MAP_GPIOPinTypeUART(GPIOD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Use the internal 16MHz oscillator as the UART clock source.
    MAP_UARTClockSourceSet(UART2_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(2, baud, 16000000);//16Mhz

    // Set usart int priorityset
    MAP_IntPrioritySet(INT_UART2,1);

    // Enable UART receive interrupt and receive timeout interrupt
    MAP_UARTIntEnable(UART2_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART interrupt
    MAP_IntEnable(INT_UART2);
}

//初始化串口3和中断
void Uart3_Init(uint32_t baud){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Enable UART Peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PC6_U3RX);
    MAP_GPIOPinConfigure(GPIO_PC7_U3TX);
    MAP_GPIOPinTypeUART(GPIOC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Initialize the UART for console I/O.
    UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet() , baud, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |  UART_CONFIG_PAR_NONE)); 

    // Set usart int priorityset
    MAP_IntPrioritySet(INT_UART3,1);

    // Enable UART receive interrupt and receive timeout interrupt
    MAP_UARTIntEnable(UART3_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART interrupt
    MAP_IntEnable(INT_UART3);
}

//初始化串口5和中断
void Uart5_Init(uint32_t baud){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable UART Peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PE4_U5RX);
    MAP_GPIOPinConfigure(GPIO_PE5_U5TX);
    MAP_GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Initialize the UART for console I/O.
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet() , baud, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |  UART_CONFIG_PAR_NONE)); 

    // Set usart int priorityset
    MAP_IntPrioritySet(INT_UART5,1);

    // Enable UART receive interrupt and receive timeout interrupt
    MAP_UARTIntEnable(UART5_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART interrupt
    MAP_IntEnable(INT_UART5);
}

//初始化串口6和中断
void Uart6_Init(uint32_t baud){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Enable UART Peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PD4_U6RX);
    MAP_GPIOPinConfigure(GPIO_PD5_U6TX);
    MAP_GPIOPinTypeUART(GPIOD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet() , baud, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |  UART_CONFIG_PAR_NONE)); 

    // Set usart int priorityset
    MAP_IntPrioritySet(INT_UART6,1);

    // Enable UART receive interrupt and receive timeout interrupt
    MAP_UARTIntEnable(UART6_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART interrupt
    MAP_IntEnable(INT_UART6);
}

