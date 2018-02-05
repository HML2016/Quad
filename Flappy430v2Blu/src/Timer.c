
/* Flappy430-Copter by HHY */


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


/***********************************************************************/
// Project Name : Flappy430-Copter
// File Name    : TIMER.c
// Author       :  HHY,Flappy430 Team
// Create Date  : 2014.5.27
// Function     : TIEMR�ĳ�ʼ��������
//                TIMER b ��ʼ��Ϊ10M/4=2.5Mhz������ң����PWM�źŵĲ���
//                TIMERA������PWM����г�ͻ����ʱ����
//                ����ʹ��TA,TC,TD�������TI���̺���ϸ�޸�
//  msp430f5310----  32k rom�� 6k ram
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
unsigned char flag_500; //500hz�����ź� ��ʱû����

/*********************************************************/
//��ʼ��timerBΪ2.5MHZ������PWM���񣬲ɼ�ң�����ź�
/*********************************************************/

void Init_timer_B (void)
{

  TB0CCR0 = 65534;//һֱ��ͣ�ļ����Ϳ�
  TB0CTL = TBSSEL_2 | MC_1| ID_2 | TBCLR; // 10M/4=2.5MHZ
  _EINT();
  
}

// Timer B0 interrupt service routine
// ʵ���ϲ�δ�����жϣ���д������
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR (void)
{
  //�ж����治�������α�־��ʱҲ����
}


/*********************************************************/
//��ʼ��timerAΪ2.5Mhz �����PWMò���г�ͻ����ʱ����
/*********************************************************/
void Init_timer_A (void)
{

  TA0CCR0 = 65534;//һֱ��ͣ�ļ���

  TA0CTL = TASSEL_2 | MC_1| ID_2 | TACLR| TAIE; // 10M/4=2.5Mhz

  _EINT();
}

// �ж�������ʱû������������ʵ��û�г�ʼ��TA
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
