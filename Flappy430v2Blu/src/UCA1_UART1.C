
/* Flappy430-Copter by HHY */


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
/***********************************************************************/
// Project Name : Flappy430-Copter
// File Name    : UCA1_UART1.c
// Author       : HHY,Flappy430 Team
// Create Date  : 2014.5.22
// Function     : UART发送与接收系列函数
// 发送中与接收的时候都使用了UCA1START的 BUSY位来判断是否需要等待总线
// 波特率115200,8bit，1stop，no 校验。
// UCA1--P4.4,5 = USCI_A1 TXD/RXD
//msp430f5310----  32k rom， 6k ram
/* + Copyright (c) 05.2014 HHY*/ 
/***********************************************************************/


#include <msp430f5310.h>
#include <math.h>

void UCA1_UART_Init(void);
void Send_Char_SCIA( char B);
void Send_Char_SCIA_unsigned(unsigned  char B);
void Send_String_SCIA(char *String);
void Send_String_SCIA_unsigned (unsigned char *String);
char Get_Char_SCIA(void);
unsigned char Get_Char_SCIA_unsigned(void);
void SCIA_Print_Float(float shu);

/************************UCA1_UART_Init************************************/
// 初始化uart1。8bit，1stop，无校验
// 选择ACLK作为时钟源。需要将ACLK设置为20M.波特率是与20M匹配的115200
// 需要先关闭串口功能再设置寄存器
// 允许RX中断并且设计返回数据功能
/***********************UCA1_UART_Init*************************************/

void UCA1_UART_Init(void)
{


  P4SEL |= BIT4 | BIT5;                     // P4.4,5 = USCI_A1 TXD/RXD
  
  //第一步 引脚初始化
  
  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**先关闭再去进行设置
  UCA1CTL1 |= UCSSEL_1;                     // ACLK作为时钟源，调整为20M.以便计算正确的波特率
 

  UCA1CTL0 &=~0XFF;                         //选择UART模式。
  //此寄存器也用于选择数据格式。8bit，1stop，0dd 校验。校验关闭
  
  
  UCA1BR0 = 173;                            // 20MHz 115200 (see User's Guide) 20000000/115200=173
  UCA1BR1 = 0;                        
  UCA1MCTL |= UCBRS_1 | UCBRF_0;           

  //UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt打开接收中断
  //比较关键的一句。打开中断  暂时没有设置用上位机发送数据回来改设置，暂时去掉吧
  UCA1CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**打开总开关
  // UCA1IE |= UCRXIE;必须是在UCA1CTL1 &= ~UCSWRST后面，不然是无效的
  UCA1IE |= UCRXIE;              // Enable USCI_A1 RX interrupt打开接收中断
  __bis_SR_register( GIE);       // interrupts enabled 打开总中断。
  
}


/*------------------------------------------*/
/*形式参数：char                            */
/*返回值:void				    */
/*函数描述:发送一个字符			    */
/*包括普通版本和unsigned 版本		    */
/*------------------------------------------*/
void Send_Char_SCIA( char B)
{
    while (!(UCA1IFG&UCTXIFG));  // USCI_A1 TX buffer ready?
    UCA1TXBUF = B;               // TX -> character
    while ( (UCA1STAT&0X01) == 1);  //状态检测，等待忙碌标识为空
}

void Send_Char_SCIA_unsigned(unsigned  char B)
{
    while (!(UCA1IFG&UCTXIFG));  // USCI_A1 TX buffer ready?
    UCA1TXBUF = B;               // TX -> character
    while ( (UCA1STAT&0X01) == 1);  //状态检测，等待忙碌标识为空
}

/*------------------------------------------*/
/*形式参数： char *String                   */
/*返回值:void				    */
/*函数描述:发送一个字符串		    */
/*包括普通版本和unsigned 版本		    */
/*------------------------------------------*/
void Send_String_SCIA(char *String)
{
	while(*String !='\0')
  	{
        Send_Char_SCIA(*String++);
  	}
}


void Send_String_SCIA_unsigned (unsigned char *String)
{
	while(*String !='\0')
  	{
        Send_Char_SCIA(*String++);
  	}
}

/*------------------------------------------*/
/*形式参数：void                            */
/*返回值:char				    */
/*函数描述:接收一个字符			    */
/*------------------------------------------*/
char Get_Char_SCIA(void)
{
     UCA1IE &= ~UCRXIE ;//关闭接收中断
     while ( (UCA1STAT&0X01) == 1); //状态检测，等待忙碌标识为空  //等待接收完毕
     UCA1IE |= UCRXIE;   //开接收中断
     return (UCA1RXBUF); // 返回接收到的数据
}


unsigned char Get_Char_SCIA_unsigned(void)
{
     UCA1IE &= ~UCRXIE ; //关闭接收中断 发送后面等到忙，接收前面等待
     while ( (UCA1STAT&0X01) == 1); //等待接收完毕
     UCA1IE |= UCRXIE;   //开接收中断
     return (UCA1RXBUF); // 返回接收到的数据
}


/*用串口以十进制字符形式发送浮点型数shu*/
/*用串口以十进制字符形式发送浮点型数shu*/
/*用于串口助手等用*/
/*由于最后的一位‘\0'难以用串口助手发出，谋求改成’e‘？*/

void SCIA_Print_Float(float shu)
{
	unsigned char sz[32],i=0;  //
	long dd,ff;
    if(shu>=0)
        sz[0]='+';
    else
    {                   //
        sz[0]='-';
        shu=fabs(shu);  //fabs函数
    }
	if(shu<1.0)        //
	{
		sz[1]='0';    //数字和ASCII码之间由于相差了0X30
		sz[2]='.';
		dd=(long)(shu*1000); //
		sz[3]=dd/100+0x30;
		sz[4]=dd%100/10+0x30;//价格加OX30是因为，传送的是ASCII码
		sz[5]=dd%10+0x30;//就是一直扩大再除而已，其实拿一个数，例如0.356
		sz[6]='\0';       //精确到小数点后面第3位。
		//sz[5]='E';       //
	}                    //加到第5位的话，就乘以100000即可！
	else				//12.678
	{
		dd=(long)shu;   //整数部分
		ff=dd;
    	for(i=1;;i++)  //不断的除以10，直到变成小数为止。其实13个的数组容量不仅小数有局限，整数部分也是有局限的
	    {
		 if(dd/10==0)
			   break;
			dd=dd/10;
		}
		sz[i+1]='.';
		dd=(long)(shu*1000)%1000;
		sz[i+2]=dd/100+0x30;
		sz[i+3]=dd%100/10+0x30;
		sz[i+4]=dd%10+0x30;
		sz[i+5]='\0';
		//sz[i+5]='E'; //换用‘e'作为标志位，算是自己的协议吧，中断里对于’e'的处理有没有问题呢？

		for(;;i--)
		{
			sz[i]=ff%10+0x30;
			if(ff/10==0)
			   break;
			ff=ff/10;    //这里既是将前面的整数部分还原至数组的前面了
		}
	}
	Send_String_SCIA_unsigned (sz);
}




