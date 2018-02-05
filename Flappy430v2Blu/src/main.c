
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
/*************************************************************************/

 // Project Name : Flappy430M
 // File Name    : FC.c
 // Author       : XIDIAN. HHY
 // Create Date  : 2014.5.19
 // Function     : 在较为简单的K的基础尝试加入了M的滤波方法，争取使得
 //滤波融合波形更稳定，以及输出控制更好--主要滤波算法采用德国M法
 //采用ADXL345与ITG3205的组合。数据范围-256-256.LSB与之前的K相比增大了两倍
 //在数据采集上直接使用读出的原始数据，之后的积分，计算注意限幅。直到最后的
 //PWM输出的时候，将占空比从128/1280扩大至256/2560
 //原始数据在K中采用了直接减去中立点自动校正。考虑增加判断是否水平的函数。
 //传感器的方向存在与计算标准相反从而需要取负数的情况，传感器方向如下。
// 加入磁力计mag3110作为方向传感器
//      ---------     
//     |●      |      ---------         -------
//     |  3205  |     |      ●|         | 3110|
//     |        |     |  345   |         |   ●|
//     ----------     ---------          -------
//    
/***************************上位机数据说明***********************************/
//  Debug[0]=MOTOR1   Debug[1]=MOTOR2  Debug[2]=MOTOR3  Debug[3]=MOTOR4
//  Debug[4]=RxRud 旋转   Debug[5]= RxThr油门   Debug[6]=RxEle俯仰   Debug[7]=RxAil横滚
//   Debug[8]=tmp_long
//   Debug[9]=tmp_long2
//   Debug[10]=mag_ang
//   Debug[11]=IntegralErrorRoll; //有个上位机方便多了
//   Debug[12]=IntegralErrorNick;
//   Debug[13]=Reading_IntegralGyroNick; 
//   Debug[14]=Reading_IntegralGyroRoll;
//   Debug[16]=IntegralErrorNick; 
//   Debug[15]=IntegralErrorRoll; 

/******************硬件联系，MOTOR方向等************************************/ 
//  接收机引脚对应关系： 
//                  P1.1--RC-YAW  P1.6--RC--THR P1.7--RC-ELE P2.0--RCAIL
// pwm输出与各个引脚以及TACCRx对应图
//     →   
//     1    2   旋转方向如图
//       X
//     3   4
//  PWM: PWM1-P1.5-TA4  PWM2--P1.2-TA1  PWM3--P1.3-TA2  PWM4--P1.4-TA3
//  PWM2    PWM1
//          x
//    PWM3     PWM4
/******************以下为具体设置，函数等*********************************/  
// 新增安卓手机控制模式。
// 可以从MODE处选择控制模式



//msp430f5310----  32k rom， 6k ram
/* + Copyright (c) 05.2014 HHY*/ 
 /* Xidian University*/
 //版本信息 14.0519  未调试
 //版本信息 14.0520 未调试 修正了一些标点，语句等小BUG.增加一些初始化函数
/***********************************************************************/
#include "msp430f5310.h"   
#include "math.h"         
#include "UCA1_UART1.H"
#include "UCS_XTAL2.h"
#include "Delay.h"
#include "Gpio_Init.h"
#include "PWM.h"
#include "Debug_Out.h"
#include "Timer.h"
#include "IIC_SENSOR.H"
#include "FC.H"
#include "MAG3110.H"
#include "Blue.H"


void Init_debug(int* debugdata); //chushihuashuju



void Init_debug(int* debugdata) //chushihuashuju
{ 
  unsigned char i;
 for(i=0;i<23;i++)
  {
   debugdata[i]=0;
 }
}

int main(void)
{
  
  WDTCTL = WDTPW | WDTHOLD;        // Stop watchdog timer
  UCS_XTAL2_Init(0,1,0);           //mclk--不分 SMCLK--2分。ACLK--不分，给uart
  UCA1_UART_Init();               //初始化串口
  Init_timer_B (); 
  DelayMs(500);                    //等待电源稳定。尤其是等待2.4g稳定
  DelayMs(500);                    //等待电源稳定。尤其是等待2.4g稳定
  DelayMs(500);   

  __bis_SR_register( GIE);       // interrupts enabled 打开总中断。

  /***************各个外设的初始化****************/ 
  RC_Pin_Init();
  // 换用硬件pwm
  PWM_Pin_Init_hard(); // 一步步仔细研究，不要漏掉
  PWM_Init_500();//初始化500hz的pwm波

  IIC_Pin_Init();
    //MAG_Init();
  Init_ITG3205();
  READ_ITG3205(); // 试试读取
  Init_ADXL345(); 
  read_ADXL345();
  
 // PWM_Pin_Init_soft();//使用硬件PWM的时候注释掉这一句
  Led_Init();
  Delay_no_accu();
  DelayUs(100);
  DelayMs(100); // 略作等待
  /**********************************************************************/
  blue_hard_init();//蓝牙串口初始化。加油
  DelayUs(600);
  HC_06_Init();    //波特率，名称，密码设置
 /**********************************************************************/

   MAG3110_Init(); //初始化MAG3110
   MAG_INIT();     //成最大值最小值的采集。
   magI=0;         // 积分赋给初始值
   // 点亮LED,表示在采集中
  // 采集50次
   //PWM_Pin_Tst(); //测试pin到底是不是好的
   Init_timer_B (); //替换掉ta，避免与pwm冲突。若需要500hz触发等，需要使用tc等
  // Init_timer_A (); //15  Qhzchufa
   Init_debug(Debug); //这就必须有，初始化开始的数值，不然的话，没有补上0的胡上位机不识别
   

   
   //IIC_Pin_DeInit(); //使用外部处理器，测试mag的时候可以使用
   //使用内部测试的时候切记要注释掉
   /*********************************************************/
   
   // 测试UCA0实际上对不对
   
    //while(1)
     //{
      // UCA0_TST();
      //__no_operation();
    //}
   
   
  while(1)         //蓝牙必须初始化两次才能保证可靠的工作
  {  
     DelayMs(100); // 略作等待
     HC_06_Init();    //波特率，名称，密码设置
     DelayMs(100); // 略作等待
     Led_Toggle(100);
     Led_Toggle(100);
     Led_Toggle(100);  // 蓝牙校准完成
     Led_On();
     Led_Off();        // 等待解锁命令
  //主要的流程，数据处理等等，都在loop函数中表达
  while(1)
  {
    
    
    
    
    
    
    
    __no_operation(); 
    _loop(0);  //主飞控函数
    // MAG_TST();
    //  MAG_CAL();
    //  mag3110_tst();
    __no_operation(); 
    //__no_operation(); 
    
    


  }
  }
}


  
  
  

    
    



