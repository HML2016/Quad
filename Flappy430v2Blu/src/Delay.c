
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


/******************Delay.c****************************/
//  各种延时函数。用作延时
//  主要是考虑了晶振频率的较为精确的延时用的多
/*****************Delay.c*****************************/


/******************Delay_no_accu.c****************************/
//  各种延时函数。用作延时
//  自己编写的跑循环加上系统的延时
//  很多时候效率偏低？仔细研究
//  #define MCU_SPEED  20  //20MHz 若更换时钟频率需要及时修改
/*****************Delay_no_accu*****************************/



#include "msp430f5310.h"   

/*****************以MCU为基准，较为精确的延时*******************/
#define MCU_SPEED           20  //20MHz 若更换时钟频率需要及时修改
#define delayUs(x)          __delay_cycles((x * MCU_SPEED))
/*****************以MCU为基准，较为精确的延时******************/

void DelayUs(unsigned int us); //较精确的微妙秒级延时，最多255us
void DelayMs(unsigned int ms); //较精确的毫秒级延时，最多255ms

void Delay_no_accu(void); 
void Delay_no_accu_1(unsigned char cnt); 
void Delay_no_accu_2(unsigned char cnt); 
void Delay_no_accu_3(void); 


void DelayUs(unsigned int us)//较精确的微妙秒级延时，最多255us
{
  volatile unsigned int i;
  for (i=0;i<us;i++)
    __delay_cycles(MCU_SPEED);
}

void DelayMs(unsigned int ms) //较精确的毫秒级延时，最多255ms
{
  volatile unsigned int i;
  for (i=0;i<ms;i++)
    __delay_cycles((MCU_SPEED * 1000));
}




/******************Delay_no_accu****************************/
//简单函数，循环256个周期
//但是，貌似执行起来效率有些问题，延时不是很精确。
/*****************Delay_no_accu*****************************/

void Delay_no_accu(void) 
{
  unsigned char i;
  for(i=0;i<255;i++);  
}

/******************Delay_no_accu_1****************************/
//简单函数，循环cnt
//但是，貌似执行起来效率有些问题，延时不是很精确。
/*****************Delay_no_accu_1*****************************/
void Delay_no_accu_1(unsigned char cnt) 
{
  unsigned char i;
  for(i=0;i<cnt;i++);  
}

/******************Delay_no_accu_2****************************/
//简单函数，循环cnt*256
//但是，貌似执行起来效率有些问题，延时不是很精确。
/*****************Delay_no_accu_2*****************************/
void Delay_no_accu_2(unsigned char cnt) //简单函数，循环256个周期
{
  unsigned char i,j;
  for(i=0;i<cnt;i++)
  {
    for(j=0;j<255;j++);
  }
}


/******************Delay_no_accu_3****************************/
//简单函数，循环256*256
//但是，貌似执行起来效率有些问题，延时不是很精确。
/*****************Delay_no_accu_3*****************************/
void Delay_no_accu_3(void)
{
  unsigned char i,j;
  for(i=0;i<255;i++)
  {
    for(j=0;j<255;j++);
  }
}
