
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
/*********************************************************************/
// Project Name : Flappy430-Copter
// File Name    : UCS_XTAL2.c
// Author       : HHY,Flappy430 Team
// Create Date  : 2014.5.22
//  2014 05 22   时钟频率设置以及各个时钟频率分配

// 时钟预分频 XTAL2--20M--1分-MCLK--CPU
//            XTAL2--20M--4分-5M--ACLK--UART等等
//            XTAL2--20M--2分-10M--smCLK--timer等

// 在main函数的初始化中可能有出入，以main函数为准

//msp430f5310----  32k rom， 6k ram
//NOTICE -----iar的分离源文件只需要添加即可，不需要添加路径。不，在c++那一项需要
/* + Copyright (c) 05.2014 HHY*/ 
/*******************************************************************/
#include"msp430f5310.h"


/******************MCLK_PinOut_Init*******************************/
//使用PM模块，定义P4.7为MCLK输出引脚。若有其他用途须注释掉
//需要而配合时钟设置使用，不是很熟悉
/******************MCLK_PinOut_Init*******************************/

void MCLK_PinOut_Init()//使用PM模块，定义P4.7为MCLK输出引脚。若有其他用途须注释掉
//若担心高频干扰，也应注释掉
{
  
  PMAPPWD = 0x02D52;                        // Enable Write-access to modify port mapping registers
  P4MAP7 = PM_MCLK;                        
  PMAPPWD = 0;                              // Disable Write-Access to modify port mapping registers
  //用的PM功能打开了P4，7。5系列的一个新功能
  
  P4DIR |= BIT7;                            // MCLK set out to pins
  P4SEL |= BIT7;                            // PxSEL=1就意味着选择其他功能。
  
  
}


/******************UCS_XTAL2_Init*******************************/
//首先设置时钟源为XTAL2，其次，输入三种时钟的分频系数--最多是5--32分频
//但是时钟源都选择XTLA2
/******************UCS_XTAL2_Init*******************************/

void UCS_XTAL2_Init(char mclkdiv,char smclkdiv,char aclkdiv)
//首先设置时钟源为XTAL2，其次，输入三种时钟的分频系数--最多是5--32分频
//但是时钟源都选择XTLA2
{
  P5SEL |= BIT2|BIT3;                       // Port select XT2

  UCSCTL6 &= ~XT2OFF;                       // Enable XT2 
  UCSCTL3 |= SELREF_2;                      // FLLref = REFO
                                            // Since LFXT1 is not used,
                                            // sourcing FLL with LFXT1 can cause
                                            // XT1OFFG flag to set
  //fll部分还不是很熟悉
  UCSCTL4 |= SELS_5 | SELM_5;               // SMCLK=MCLK=XT2
  UCSCTL4 |= SELA_5;                        // ACLK=XT2
  
  UCSCTL6 &= ~XT2DRIVE1;                    //理论上，20M应该要达到level2.
                                           
                                            // 1就够了、DRIVE1是8000u，也就是1000，前面是10.代表levle2
  // UCSCTL9 & =(0<<1);                     //xtl2的电源电压选择DVCC
  //分频设置
  UCSCTL5|=aclkdiv<<8;                 //ACLK分频，分频比例2^aclkdiv
  UCSCTL5|=smclkdiv<<4;                 //SMCLK分频，分频比例2^smclkdiv
  UCSCTL5|=mclkdiv;                 //SMCLK分频，分频比例2^smclkdiv

  // Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
  do
  {
    UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);
                                            // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
  
  
}