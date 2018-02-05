
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


/******************Delay.c****************************/
//  ������ʱ������������ʱ
//  ��Ҫ�ǿ����˾���Ƶ�ʵĽ�Ϊ��ȷ����ʱ�õĶ�
/*****************Delay.c*****************************/


/******************Delay_no_accu.c****************************/
//  ������ʱ������������ʱ
//  �Լ���д����ѭ������ϵͳ����ʱ
//  �ܶ�ʱ��Ч��ƫ�ͣ���ϸ�о�
//  #define MCU_SPEED  20  //20MHz ������ʱ��Ƶ����Ҫ��ʱ�޸�
/*****************Delay_no_accu*****************************/



#include "msp430f5310.h"   

/*****************��MCUΪ��׼����Ϊ��ȷ����ʱ*******************/
#define MCU_SPEED           20  //20MHz ������ʱ��Ƶ����Ҫ��ʱ�޸�
#define delayUs(x)          __delay_cycles((x * MCU_SPEED))
/*****************��MCUΪ��׼����Ϊ��ȷ����ʱ******************/

void DelayUs(unsigned int us); //�Ͼ�ȷ��΢���뼶��ʱ�����255us
void DelayMs(unsigned int ms); //�Ͼ�ȷ�ĺ��뼶��ʱ�����255ms

void Delay_no_accu(void); 
void Delay_no_accu_1(unsigned char cnt); 
void Delay_no_accu_2(unsigned char cnt); 
void Delay_no_accu_3(void); 


void DelayUs(unsigned int us)//�Ͼ�ȷ��΢���뼶��ʱ�����255us
{
  volatile unsigned int i;
  for (i=0;i<us;i++)
    __delay_cycles(MCU_SPEED);
}

void DelayMs(unsigned int ms) //�Ͼ�ȷ�ĺ��뼶��ʱ�����255ms
{
  volatile unsigned int i;
  for (i=0;i<ms;i++)
    __delay_cycles((MCU_SPEED * 1000));
}




/******************Delay_no_accu****************************/
//�򵥺�����ѭ��256������
//���ǣ�ò��ִ������Ч����Щ���⣬��ʱ���Ǻܾ�ȷ��
/*****************Delay_no_accu*****************************/

void Delay_no_accu(void) 
{
  unsigned char i;
  for(i=0;i<255;i++);  
}

/******************Delay_no_accu_1****************************/
//�򵥺�����ѭ��cnt
//���ǣ�ò��ִ������Ч����Щ���⣬��ʱ���Ǻܾ�ȷ��
/*****************Delay_no_accu_1*****************************/
void Delay_no_accu_1(unsigned char cnt) 
{
  unsigned char i;
  for(i=0;i<cnt;i++);  
}

/******************Delay_no_accu_2****************************/
//�򵥺�����ѭ��cnt*256
//���ǣ�ò��ִ������Ч����Щ���⣬��ʱ���Ǻܾ�ȷ��
/*****************Delay_no_accu_2*****************************/
void Delay_no_accu_2(unsigned char cnt) //�򵥺�����ѭ��256������
{
  unsigned char i,j;
  for(i=0;i<cnt;i++)
  {
    for(j=0;j<255;j++);
  }
}


/******************Delay_no_accu_3****************************/
//�򵥺�����ѭ��256*256
//���ǣ�ò��ִ������Ч����Щ���⣬��ʱ���Ǻܾ�ȷ��
/*****************Delay_no_accu_3*****************************/
void Delay_no_accu_3(void)
{
  unsigned char i,j;
  for(i=0;i<255;i++)
  {
    for(j=0;j<255;j++);
  }
}
