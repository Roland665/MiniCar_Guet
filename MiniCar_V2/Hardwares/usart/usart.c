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

u8 UART1_RX_BUF[UART_REC_LEN] = {0};  //串口接收缓冲区,最大UART_REC_LEN个字节.
u16 UART1_RX_STA = 0;//接收到的有效字节数目
u8 uart1RXTime = 0xFF;//串口消息间隔计时，初始化把时间拉满，表示没有收到新的消息

//初始化串口0和中断
void Uart0_Init(uint32_t bound){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, bound, 16000000);

    // Set usart int priorityset
    IntPrioritySet(INT_UART0,2);

    // Enable UART receive interrupt and receive timeout interrupt
    UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART0 interrupt
    MAP_IntEnable(INT_UART0);
}

//初始化串口1和中断
void Uart1_Init(uint32_t bound){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Enable UART0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PC4_U1RX);
    MAP_GPIOPinConfigure(GPIO_PC5_U1TX);
    MAP_GPIOPinTypeUART(GPIOC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Use the internal 16MHz oscillator as the UART clock source.
    MAP_UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(1, bound, 16000000);//16Mhz

    // Set usart int priorityset
    MAP_IntPrioritySet(INT_UART1,2);

    // Enable UART receive interrupt and receive timeout interrupt
    MAP_UARTIntEnable(UART1_BASE,UART_INT_RX | UART_INT_RT);

    // Enable UART0 interrupt
    MAP_IntEnable(INT_UART1);
}
