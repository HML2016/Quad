
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
// File Name    : TIMER.c
// Author       :  HHY,Flappy430 Team
// Create Date  : 2014.5.27
// Function     : TIEMR的初始化与设置
//                TIMER b 初始化为10M/4=2.5Mhz，用作遥控器PWM信号的捕获
//                TIMERA由于与PWM输出有冲突，暂时不用
//                如需使用TA,TC,TD等请参阅TI例程后仔细修改
//  msp430f5310----  32k rom， 6k ram
/* + Copyright (c) 05.2014 HHY*/ 
/**********************************************************************/ 

#include "msp430f5310.h"   
#include "math.h"         
#include "UCA1_UART1.H"
#include "UCS_XTAL2.h"
#include "Delay.h"
#include "Gpio_Init.h"
#include "PWM.h"
#include "Debug_Out.h"
#include "Timer.h"

void Init_timer_B (void);
void Init_timer_A (void);
unsigned char flag_500; //500hz触发信号 暂时没有用

/*********************************************************/
//初始化timerB为2.5MHZ，用作PWM捕获，采集遥控器信号
/*********************************************************/

void Init_timer_B (void)
{

  TB0CCR0 = 65534;//一直不停的计数就可
  TB0CTL = TBSSEL_2 | MC_1| ID_2 | TBCLR; // 10M/4=2.5MHZ
  _EINT();
  
}

// Timer B0 interrupt service routine
// 实际上并未允许中断，先写在这里
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR (void)
{
  //中断里面不做处理。次标志暂时也不用
}


/*********************************************************/
//初始化timerA为2.5Mhz 与输出PWM貌似有冲突，暂时不用
/*********************************************************/
void Init_timer_A (void)
{

  TA0CCR0 = 65534;//一直不停的计数

  TA0CTL = TASSEL_2 | MC_1| ID_2 | TACLR| TAIE; // 10M/4=2.5Mhz

  _EINT();
}

// 中断里面暂时没有做处理。且事实上没有初始化TA
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
  switch(__even_in_range(TA0IV,14))
  {
    case  0: break;                          // No interrupt
    case  2: break;                          // CCR1 not used
    case  4: break;                          // CCR2 not used
    case  6: break;                          // reserved
    case  8: break;                          // reserved
    case 10: break;                          // reserved
    case 12: break;                          // reserved
    case 14: //P1OUT ^= BIT1;                 
             break;
    default: break; 
  }
}
