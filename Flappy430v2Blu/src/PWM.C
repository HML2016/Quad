
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
/******************************************************************/
// Project Name : Flappy430-Copter
// File Name    : PWM_soft.c
// Author       : HHY,flappy430 Team
// Create Date  : 2014.5.27
// pwm输出与各个引脚以及TACCRx对应图
//     →   
//     1    2
//       X
//     3   4
//  PWM: PWM1-P1.5-TA4  PWM2--P1.2-TA1  PWM3--P1.3-TA2  PWM4--P1.4-TA3
//  PWM2    PWM1
//          x
//    PWM3     PWM4
/**************************************************************/ 
//  msp430f5310----  32k rom， 6k ram
/* + Copyright (c) 08.2014 HHY*/ 


/***********************************************************************/

#include "msp430f5310.h"   
#include "Delay.h"
#include "Gpio_Init.h"

#define  PWM_MAX 250       // 电机驱动限幅数值

unsigned char Motor1,Motor2,Motor3,Motor4; //用于最后输出moto数值
int MotorLimitValue(int v); //moto数值限幅

void PWM_Init_50(void);//初始化50hz的pwm波
void PWM_out_50(unsigned char ch, int period,unsigned char mode);
void PWM_Init_500(void);//初始化50hz的pwm波
void PWM_out_500(unsigned char ch, int period,unsigned char mode); //输出小四。占空比就可以很大
void PWM_out_500_brush(unsigned char ch, int period); 
void PWM_Pin_Tst(void);//测试P1.1--1.4

void MotorOut_Brush(unsigned char mode); //0是反向输出 1是正常输出
void MotorOut_Brushless(unsigned char mode); //dasizhou
void MotorOut_Brushless_down(unsigned char mode); //上锁的时候输出较小的转速


//   Motor drive limit
//   电机驱动限幅重新设施PWMmax为250
int  MotorLimitValue(int v)
{
 	if(v>PWM_MAX)	return PWM_MAX;
	if(v<=0)	return 0; //这个，其实硬件PWM是不可以等于0的
        
	return v;
}
//限幅还是一个比较经典的外置式限幅


/*************************************************************/
//测试P1.2--1.5用delay来制作一个简单的PWM波形测试引脚有没有焊上
/*************************************************************/
void PWM_Pin_Tst(void)//测试P1.2--1.5
{
   DelayMs(20);
   P1OUT|=BIT5+BIT2+BIT3+BIT4; //用delay来制作一个简单的PWM波形
   DelayMs(20);
   P1OUT&=~(BIT5+BIT2+BIT3+BIT4); //用delay来制作一个简单的PWM波形
   
}


/*******************PWM_Init_50***************************************************/
// 初始化PWM的ta0.1--ta0.4为输出50hz的方波
// 增减技术模式，outmode7
/*******************PWM_Init_50***************************************************/

void PWM_Init_50(void)//初始化50hz的pwm波
{

  
  TA0CTL|=MC_1;   // SMCLK, up-down mode

  TA0CCTL1 = OUTMOD_7; // CCR1 reset/set  
  TA0CCTL2 = OUTMOD_7; // CCR2 reset/set
  TA0CCTL3 = OUTMOD_7; // CCR3 reset/set
  TA0CCTL4 = OUTMOD_7; // CCR4 reset/set

  TA0CTL = TASSEL_2 | MC_1 | ID_2| TACLR;   //

  TA0CTL|=ID_3;

  TA0CCR0 = 25000;    // PWM Period/2 50HZ 
  
}

/*******************PWM_out_50***************************************************/
// 限幅。125-250先扩展为1250-2500然后到CCRX当中对应着5%--10%的占空比
// 前面是通道的名字，对应着ch1--ch4，然后后面是周期。也就是125-250的输入。
// 用作moto的主要输出函数。控制大四轴
// 两种初始化，50和500hz 同样，有两种初始化输出函数，一定要记得初始化
// 主输出函数pwmout，可以选择通道和period。但是有限幅
// 如果使用有刷电机，其方向与占空比都要调整
// mode选择输出是高电平触发mos还是低电平触发mos
/*******************PWM_out_50***************************************************/

void PWM_out_50(unsigned char ch, int period,unsigned char mode) //四个通道。占空比虽然无限制，但是要限制在5%--10%
{
  if(mode==0) //如果是低电平触发，则输出的pwm占空比取反
  {           // 其实这个是小四用的，大四用不上
    period=250-period;}
  
  

  if(period>250){period=250;}  //防止溢出
  if(period<125){period=125;}  //限制占空比到5%-10%
   switch(ch) 
  {
   
    case 1:
           TA0CCR1 = period*10;  //因为50hz太小，所以TACCRX需要乘以10来起到5%--10%的作用 
           break;   
    case 2:
           TA0CCR2 = period*10;  
           break;    // 
    case 3:
           TA0CCR3 = period*10;  
           break;    // 
    case 4:
           TA0CCR4 = period*10; 
           break;    //
    default: break;  
  }
  
}

/*******************PWM_Init_500***************************************************/
// 初始化PWM的ta0.1--ta0.4为输出500hz的方波
// 增减技术模式，outmode7，这个带检验
// TACCRX越大，则占空比越大。这个的设置与timerA本身貌似是分开的
// 具体的周期计算见前面的注释
/*******************PWM_Init_500***************************************************/

void PWM_Init_500(void)//初始化500hz的pwm波
{
  
  TA0CTL|=MC_1;   // SMCLK, up-down mode,记得周期要乘以2

  TA0CCTL1 = OUTMOD_7; // CCR1 reset/set
  TA0CCTL2 = OUTMOD_7; // CCR2 reset/set
  TA0CCTL3 = OUTMOD_7; // CCR3 reset/set
  TA0CCTL4 = OUTMOD_7; // CCR4 reset/set
  //TA0CTL|=ID_3;//四分频。一级分频。ID这个必须在后面，CLEAR 会清除分频
  TA0CTL = TASSEL_2 | MC_1 | ID_2| TACLR;   
   // 最后的设置必须要跟TACLR卸载一起才有效
  TA0CTL|=ID_3;
  // smclk，增减技术模式，清楚计数数值，貌似这个必须放在后面，不然就没有打开
  // CERA就像是总开关一样，必须放在后面

  TA0CCR0 = 2500;    // 50hz 与500，只有这个不同
  
}

/*******************PWM_out_500***************************************************/
// 限幅。125-250到CCRX当中对应着5%--10%的占空比，不用乘以10
// 前面是通道的名字，对应着ch1--ch4，然后后面是周期。也就是125-250的输入。
// 用作moto的主要输出函数。具体的moto的方位的话，到前面注释里面去看
// 小四的占空比可以达到满状态，微限幅
/*******************PWM_out_500**************************************************/

void PWM_out_500(unsigned char ch, int period,unsigned char mode) //四个通道。占空比无限制
{
   period*=10; //将0-250放大到0-2500；ok
   
  
   if(mode==0) //如果是低电平触发，则输出的pwm占空比取反
   {           // 小四用的，大四用不上
    period=2500-period;} 
  
  
  if(period>2400){period=2400;}  //防止溢出
  if(period<5){period=5;}  //小四的占空比可以达到满状态，微限幅
   switch(ch) 
  {
   
    case 1:
           TA0CCR1 = period;  
           break;   
    case 2:
           TA0CCR2 = period;  
           break;    // 
    case 3:
           TA0CCR3 = period;  
           break;    // 
    case 4:
           TA0CCR4 = period; 
           break;    //
    default: break;  
  }
  
}



void PWM_out_500_brush(unsigned char ch, int period) //四个通道。占空比几乎无限制
{
  
  if(period>2500){period=2500;}  //防止溢出
  if(period<15){period=15;}  
   switch(ch) 
  {
   
    case 1:
           TA0CCR1 = period;  
           break;   
    case 2:
           TA0CCR2 = period;  
           break;    // 
    case 3:
           TA0CCR3 = period;  
           break;    // 
    case 4:
           TA0CCR4 = period; 
           break;    //
    default: break;  
  }
  
}
/*******************MotorOut_Brushless***************************************************/
// 50hz输出大四周信号，5%-10%。注意各个moto的匹配
// 通过mode可以选择输出电平的高低方向
// 主输出函数pwmout，可以选择通道和period。但是有限幅
// 用于驱动大四轴，输出5%-10%的波。暂时没有检测，如需使用。仔细看懂，修改再用
/*******************MotorOut_Brushless***************************************************/

void MotorOut_Brushless(unsigned char mode) //0是反向输出 1是正常输出
{
  PWM_out_50(1,Motor1,1);  
  PWM_out_50(2,Motor3,1);   //各个motor的输出对应在开头有解释 
  PWM_out_50(3,Motor4,1);   
  PWM_out_50(4,Motor2,1);   
   
}


/*******************MotorOut_Brushless_dowm**********************************************/
// 上锁后只给与小小的转速
// 通过mode可以选择输出电平的高低方向
// 统一给moto输出5，恒定的较小的数值
/*******************MotorOut_Brushless***************************************************/

void MotorOut_Brushless_down(unsigned char mode) //0是反向输出 1是正常输出
{
  PWM_out_500(1,0,1);   
  PWM_out_500(2,0,1);   //各个motor的输出对应在开头有解释 
  PWM_out_500(3,0,1);   
  PWM_out_500(4,0,1);   
   
}





/*******************MotorOut_Brush***************************************************/
// 500hz输出小四周的四个pwm波。可选高低电平驱动mos管
// 通过mode可以选择输出电平的高低方向
// 主输出函数pwmout，可以选择通道和period。但是有限幅
// 注意mos管高电平驱动
/*******************MotorOut_Brush***************************************************/

void MotorOut_Brush(unsigned char mode) //0是反向输出 1是正常输出
{
  PWM_out_500(1,Motor1,mode); 
  PWM_out_500(2,Motor3,mode);   //各个motor的输出对应在开头有解释 
  PWM_out_500(3,Motor4,mode);    
  PWM_out_500(4,Motor2,mode);   
   
}