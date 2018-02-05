
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
/******************Debugout.c***********************************************/
//���ڷ������ݣ�����λ��ͨ��
//����ѡ��0-22ͨ������ʱ����0-17ͨ������ʾ����
/******************Debugout.c***********************************************/


#include <msp430f5310.h>         
#include "UCA1_UART1.h"
#include "Gpio_Init.h"
#include "PWM.h"
#include "Debug_Out.h"
#include "Timer.h"
#include "IIC_SENSOR.H"

void Debug_Out(int*datastring);
void Gyro_Random_Tst( char ch1, char ch2, int*datastring);

/******************Gyro_Random_Tst***********************************************/
//��������������������λ��
//�ü򵥵����������������ģ������
//ʵ��ʹ�õ�ʱ��ֻ���ڼ���������ݵ����ݴ���debug��������
//����ѡ���Ӧ��λ���ļ�ͨ����0--ch1 1--ch2.....
//����ѡ��0-22ͨ������ʱ����0-17ͨ������ʾ����
/******************Gyro_Random_Tst***********************************************/
void Gyro_Random_Tst( char ch1, char ch2, int*datastring)
{                    //�ü򵥵����������������ģ������
   unsigned char i;  
   int random_rol,random_pit;//�����ʹ�����������ݱ仯
   for(i=0;i<11;i++) //ʹ��10��������Ϊ�Ա�
   {
     random_rol=20*i;
     random_pit=50*i;//��Щ���ݿ����Լ��޸ģ�������ͬ����
     datastring[ch1]=random_rol;
     datastring[ch2]=random_pit;
     Debug_Out(datastring);     //���ݷ��͵���λ��
     
   }
   for(i=11;i<32;i++)         
   {
     random_rol=100-20*(i-10);
     random_pit=200-50*(i-10);
     datastring[ch1]=random_rol;
     datastring[ch2]=random_pit;
     Debug_Out(datastring);
     
   }
     
     
}


/******************Debug_Out*************************************/
// ����֮ǰ�õ������ݣ������ݵ������������λ��
// �������ϴ�Ч��ֵ�ü�⡣
// �����ͨ��Э��
// ͨ��Э�飺 SYNC + DATA + SUM
//    SYNC : 0x7F 0x7F 0xFE
//    DATA : ��12�ֽ�,ÿ�����ֽڱ�ʾһ���з���16λ���� �����ֽڲ�0
//    ע��Ҫ����0~��������
//    SUM  : ��1�ֽ�,SYNC��DATA�ۼ�ȡ���8λ
// sum1+=data1+data2
/*****************Debug_Out*************************************/
void Debug_Out(int*datastring)  //��Ҫ�����һ��Ҫ����Ϊsz[22],���õ�bit��ֵ0����Ȼ�����
{
  
  int data1,data2;
  int sum1=0; //�ۺϣ���Ϊ���ı�־λ
  int sum2;
  unsigned char chaifen[46]; 
  unsigned char i; 
  for(i=0;i<23;i++)
  {              
     data1=datastring[i];
     data2=datastring[i];

     data1&=0XFF00; //�����8bit��
     data1=data1>>8;//����8bit�ƶ�����8bit
     data1=(unsigned char)data1;
     chaifen[i*2]=data1;//Э�鵱���ȷ��͸�λ.�洢��ϵ��i--2i��2i+1,

     
      data2&=0X00ff; //�����8bit��
      data2=(unsigned char)data2;
      chaifen[i*2+1]=data2;//Э�鵱���ȷ��͸�λ

      sum1+=data1+data2; 
                    
  }
  
  sum2=0x7F+0x7F+0xFE+sum1;//Э���еĸ���λ�ĺ�
  sum2&=0x00ff;//ȥ����8λ
  sum2=(unsigned char )sum2;//ת����uchar

  Send_Char_SCIA_unsigned(0x7F);
  Send_Char_SCIA_unsigned(0x7F);
  Send_Char_SCIA_unsigned(0xFE);//SYNC
  
  for(i=0;i<46;i++)
  {
    
     Send_Char_SCIA_unsigned(chaifen[i]);//��0-45�������ݡ������Ҫ�ڿ�ʼ��0
                           
  }
  
  Send_Char_SCIA_unsigned (sum2);//У��

  /************************************************************************/ 
}

/******************************************************************************/
void* mymemcpy( void* dest, void* src, char count )
{
    char* d = (char*)dest;
    const char* s = (const char*)src;
    int n = (count + 7) / 8; // count > 0 assumed





    switch( count & 7 )
    {
    case 0: do { *d++ = *s++;
    case 7:        *d++ = *s++;
    case 6:        *d++ = *s++;
    case 5:        *d++ = *s++;
    case 4:        *d++ = *s++;
    case 3:        *d++ = *s++;
    case 2:        *d++ = *s++;
    case 1:        *d++ = *s++;
               } while (--n > 0);
    }



    return dest;
}