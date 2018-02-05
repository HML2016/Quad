
/* Flappy430-CopterM by HHY */


/* --COPYRIGHT--
 Flappy430-Copter by HHY
 * Copyright (c) 2014, HHY,Flappy430 Team, CHN
 * All rights reserved.
 * Can be used to study
 * shall not be used for any commercial purpose
 * thanks Alex��Holger Buss��gale��madwick and their codes
 * they give me much insperiation
   Function��This code IS created to control a  Four rotor aircraft 
             We use Texas Instruments' MSP430F5310 as MCU
             All circuits are integrated in a PCB board as large as 10*10 cm
             So it is a tiny  Four rotor aircraft�� we call it "flappy430"
 */
/* ��Ȩ����
 Flappy430-Copter by HHY
 flappy430@163.com
 * Copyright (c) 2014, HHY,Flappy430 Team�� CHN
 * ���Ǳ���һ��Ȩ��
 * �˴��������������ѧϰ
 * ��ֹ�����κ���ҵĿ��
 */
/*
email��flappy430@163.com
website��http://shop112974196.taobao.com
*/
/*************************************************************************/

 // Project Name : Flappy430M
 // File Name    : FC.c
 // Author       : XIDIAN. HHY
 // Create Date  : 2014.5.19
 // Function     : �ڽ�Ϊ�򵥵�K�Ļ������Լ�����M���˲���������ȡʹ��
 //�˲��ںϲ��θ��ȶ����Լ�������Ƹ���--��Ҫ�˲��㷨���õ¹�M��
 //����ADXL345��ITG3205����ϡ����ݷ�Χ-256-256.LSB��֮ǰ��K�������������
 //�����ݲɼ���ֱ��ʹ�ö�����ԭʼ���ݣ�֮��Ļ��֣�����ע���޷���ֱ������
 //PWM�����ʱ�򣬽�ռ�ձȴ�128/1280������256/2560
 //ԭʼ������K�в�����ֱ�Ӽ�ȥ�������Զ�У�������������ж��Ƿ�ˮƽ�ĺ�����
 //�������ķ������������׼�෴�Ӷ���Ҫȡ������������������������¡�
// ���������mag3110��Ϊ���򴫸���
//      ---------     
//     |��      |      ---------         -------
//     |  3205  |     |      ��|         | 3110|
//     |        |     |  345   |         |   ��|
//     ----------     ---------          -------
//    
/***************************��λ������˵��***********************************/
//  Debug[0]=MOTOR1   Debug[1]=MOTOR2  Debug[2]=MOTOR3  Debug[3]=MOTOR4
//  Debug[4]=RxRud ��ת   Debug[5]= RxThr����   Debug[6]=RxEle����   Debug[7]=RxAil���
//   Debug[8]=tmp_long
//   Debug[9]=tmp_long2
//   Debug[10]=mag_ang
//   Debug[11]=IntegralErrorRoll; //�и���λ���������
//   Debug[12]=IntegralErrorNick;
//   Debug[13]=Reading_IntegralGyroNick; 
//   Debug[14]=Reading_IntegralGyroRoll;
//   Debug[16]=IntegralErrorNick; 
//   Debug[15]=IntegralErrorRoll; 

/******************Ӳ����ϵ��MOTOR�����************************************/ 
//  ���ջ����Ŷ�Ӧ��ϵ�� 
//                  P1.1--RC-YAW  P1.6--RC--THR P1.7--RC-ELE P2.0--RCAIL
// pwm�������������Լ�TACCRx��Ӧͼ
//     ��   
//     1    2   ��ת������ͼ
//       X
//     3   4
//  PWM: PWM1-P1.5-TA4  PWM2--P1.2-TA1  PWM3--P1.3-TA2  PWM4--P1.4-TA3
//  PWM2    PWM1
//          x
//    PWM3     PWM4
/******************����Ϊ�������ã�������*********************************/  
// ������׿�ֻ�����ģʽ��
// ���Դ�MODE��ѡ�����ģʽ



//msp430f5310----  32k rom�� 6k ram
/* + Copyright (c) 05.2014 HHY*/ 
 /* Xidian University*/
 //�汾��Ϣ 14.0519  δ����
 //�汾��Ϣ 14.0520 δ���� ������һЩ��㣬����СBUG.����һЩ��ʼ������
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
  UCS_XTAL2_Init(0,1,0);           //mclk--���� SMCLK--2�֡�ACLK--���֣���uart
  UCA1_UART_Init();               //��ʼ������
  Init_timer_B (); 
  DelayMs(500);                    //�ȴ���Դ�ȶ��������ǵȴ�2.4g�ȶ�
  DelayMs(500);                    //�ȴ���Դ�ȶ��������ǵȴ�2.4g�ȶ�
  DelayMs(500);   

  __bis_SR_register( GIE);       // interrupts enabled �����жϡ�

  /***************��������ĳ�ʼ��****************/ 
  RC_Pin_Init();
  // ����Ӳ��pwm
  PWM_Pin_Init_hard(); // һ������ϸ�о�����Ҫ©��
  PWM_Init_500();//��ʼ��500hz��pwm��

  IIC_Pin_Init();
    //MAG_Init();
  Init_ITG3205();
  READ_ITG3205(); // ���Զ�ȡ
  Init_ADXL345(); 
  read_ADXL345();
  
 // PWM_Pin_Init_soft();//ʹ��Ӳ��PWM��ʱ��ע�͵���һ��
  Led_Init();
  Delay_no_accu();
  DelayUs(100);
  DelayMs(100); // �����ȴ�
  /**********************************************************************/
  blue_hard_init();//�������ڳ�ʼ��������
  DelayUs(600);
  HC_06_Init();    //�����ʣ����ƣ���������
 /**********************************************************************/

   MAG3110_Init(); //��ʼ��MAG3110
   MAG_INIT();     //�����ֵ��Сֵ�Ĳɼ���
   magI=0;         // ���ָ�����ʼֵ
   // ����LED,��ʾ�ڲɼ���
  // �ɼ�50��
   //PWM_Pin_Tst(); //����pin�����ǲ��Ǻõ�
   Init_timer_B (); //�滻��ta��������pwm��ͻ������Ҫ500hz�����ȣ���Ҫʹ��tc��
  // Init_timer_A (); //15  Qhzchufa
   Init_debug(Debug); //��ͱ����У���ʼ����ʼ����ֵ����Ȼ�Ļ���û�в���0�ĺ���λ����ʶ��
   

   
   //IIC_Pin_DeInit(); //ʹ���ⲿ������������mag��ʱ�����ʹ��
   //ʹ���ڲ����Ե�ʱ���м�Ҫע�͵�
   /*********************************************************/
   
   // ����UCA0ʵ���϶Բ���
   
    //while(1)
     //{
      // UCA0_TST();
      //__no_operation();
    //}
   
   
  while(1)         //���������ʼ�����β��ܱ�֤�ɿ��Ĺ���
  {  
     DelayMs(100); // �����ȴ�
     HC_06_Init();    //�����ʣ����ƣ���������
     DelayMs(100); // �����ȴ�
     Led_Toggle(100);
     Led_Toggle(100);
     Led_Toggle(100);  // ����У׼���
     Led_On();
     Led_Off();        // �ȴ���������
  //��Ҫ�����̣����ݴ���ȵȣ�����loop�����б��
  while(1)
  {
    
    
    
    
    
    
    
    __no_operation(); 
    _loop(0);  //���ɿغ���
    // MAG_TST();
    //  MAG_CAL();
    //  mag3110_tst();
    __no_operation(); 
    //__no_operation(); 
    
    


  }
  }
}


  
  
  

    
    



