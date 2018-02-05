
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
/******************Debugout.c***********************************************/
//串口发送数据，与上位机通信
//可以选择0-22通道。暂时用了0-17通道来显示数据
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
//生产两个波形来测试上位机
//用简单的随机增减的数据来模拟陀螺
//实际使用的时候，只需在计算完后将陀螺等数据存入debug【】即可
//可以选择对应上位机的及通道。0--ch1 1--ch2.....
//可以选择0-22通道。暂时用了0-17通道来显示数据
/******************Gyro_Random_Tst***********************************************/
void Gyro_Random_Tst( char ch1, char ch2, int*datastring)
{                    //用简单的随机增减的数据来模拟陀螺
   unsigned char i;  
   int random_rol,random_pit;//随机的使得这两个数据变化
   for(i=0;i<11;i++) //使用10个数据作为对比
   {
     random_rol=20*i;
     random_pit=50*i;//这些数据可以自己修改，产生不同波形
     datastring[ch1]=random_rol;
     datastring[ch2]=random_pit;
     Debug_Out(datastring);     //数据发送到上位机
     
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
// 根据之前得到的数据，将陀螺等数据输出到上位机
// 计算量较大，效率值得检测。
// 具体的通信协议
// 通信协议： SYNC + DATA + SUM
//    SYNC : 0x7F 0x7F 0xFE
//    DATA : 共12字节,每两个字节表示一个有符号16位变量 其它字节补0
//    注意要补齐0~！！！！
//    SUM  : 共1字节,SYNC和DATA累加取最低8位
// sum1+=data1+data2
/*****************Debug_Out*************************************/
void Debug_Out(int*datastring)  //需要输出的一定要设置为sz[22],不用的bit赋值0，不然会出错
{
  
  int data1,data2;
  int sum1=0; //综合，作为最后的标志位
  int sum2;
  unsigned char chaifen[46]; 
  unsigned char i; 
  for(i=0;i<23;i++)
  {              
     data1=datastring[i];
     data2=datastring[i];

     data1&=0XFF00; //清除低8bit；
     data1=data1>>8;//将高8bit移动到低8bit
     data1=(unsigned char)data1;
     chaifen[i*2]=data1;//协议当中先发送高位.存储关系是i--2i，2i+1,

     
      data2&=0X00ff; //清除高8bit；
      data2=(unsigned char)data2;
      chaifen[i*2+1]=data2;//协议当中先发送高位

      sum1+=data1+data2; 
                    
  }
  
  sum2=0x7F+0x7F+0xFE+sum1;//协议中的各个位的和
  sum2&=0x00ff;//去掉高8位
  sum2=(unsigned char )sum2;//转换成uchar

  Send_Char_SCIA_unsigned(0x7F);
  Send_Char_SCIA_unsigned(0x7F);
  Send_Char_SCIA_unsigned(0xFE);//SYNC
  
  for(i=0;i<46;i++)
  {
    
     Send_Char_SCIA_unsigned(chaifen[i]);//从0-45发送数据。不足的要在开始补0
                           
  }
  
  Send_Char_SCIA_unsigned (sum2);//校验

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