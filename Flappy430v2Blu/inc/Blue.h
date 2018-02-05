/* Flappy430-CopterM by HHY */


/* --COPYRIGHT--
 Flappy430-Copter by HHY
 * Copyright (c) 2014, HHY,Flappy430 Team, CHN
 * All rights reserved.
 * Can be used to study
 * shall not be used for any commercial purpose
 * thanks Alex，Holger Buss，gale，madwick and their codes
 * they give me much insperiation
   Function：This code IS created to control a  Four rotor aircraft 
             We use Texas Instruments' MSP430F5310 as MCU
             All circuits are integrated in a PCB board as large as 10*10 cm
             So it is a tiny  Four rotor aircraft， we call it "flappy430"
 */
/* 版权声明
 Flappy430-Copter by HHY
 flappy430@163.com
 * Copyright (c) 2014, HHY,Flappy430 Team， CHN
 * 我们保留一切权利
 * 此代码仅仅可以用于学习
 * 禁止用于任何商业目的
 */
/*
email：flappy430@163.com
website：http://shop112974196.taobao.com
*/

/************************Blue.C**************************************/ 
// 使用UCA0串口作为与蓝牙通信的串口。蓝牙模块使用HC-06. RXD--TXD--GND--VCC
// P4.6(TXD)--P4.7(RXD)--GND--VCC
// 主要使用PM部分的设置和定义。
// 使用串口中断部分接收数据，有自己的简单通信协议
//  也包含简单的AT指令对蓝牙模块进行设置。
// 
/************************Blue.C*************************************/ 
extern void ArmingRoutine_BLUE(void);          //判断5次以免误判 
extern void Blue_read (void);
extern void Port_Mapping_Blue(void);
extern void blue_hard_init(void);
extern void UCA0_UART_Init(void);
extern void Send_Char_SCIA0( char B);
extern void Send_Char_SCIA0_unsigned(unsigned  char B);
extern void Send_String_SCIA0(char *String);
extern void Send_String_SCIA0_unsigned (unsigned char *String);
extern char Get_Char_SCIA0(void);
extern unsigned char Get_Char_SCIA0_unsigned(void);
extern void SCIA0_Print_Float(float shu);
extern void HC_06_Init(void);

extern int RxRUD_BLUE,RxTHR_BLUE,RxELE_BLUE,RxAIL_BLUE;
extern char Rxch1_blue,Rxch2_blue,Rxch3_blue,Rxch4_blue;
extern char Blue_BUFF[10]; // 用来存储接收到的四个通道数据

extern  char SCIA_RX_BUF0; // 使用全局变量来存储接收到的数据，与UCA1区别开
// unsigned char还是char?

extern unsigned char flag_blue_done; // 完成接收一个数据包
extern int error_flag;               // 接收过程中的错误计数。错误累积到一定程度则自动降落

extern void UCA0_TST(void);  //测试UCA0的数据发送正确否。