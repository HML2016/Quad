/* --COPYRIGHT--
 Flappy430-Copter by HHY,XDU,CHN
 * Copyright (c) 2014, HHY,Flappy430 Team CHN
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
 Flappy430-Copter by HHY,XDU,CHN
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
// File Name    : GPIO_Init.c
// Author       : HHY,Flappy430 Team
// Create Date  : 2014.5.25
// Function     : ������Ҫ�õ��ĸ���ģ���GPIO��ʼ���Լ�ʱ��
//                ����RC�����ж������ж��ڲ������жϴ�������
//                ADC������ʱû�����á�PWM��Ӳ���������
//                ������Ӳ��PWM���������PWM��moto��c���Բ���

//  ���ջ����Ŷ�Ӧ��ϵ�� 
//                  P1.1--RC-YAW  P1.6--RC--THR P1.7--RC-ELE P2.0--RCAIL

//  msp430f5310----  32k rom�� 6k ram
/* + Copyright (c) 05.2014 HHY*/ 

/***********************************************************************/
#include "msp430f5310.h"   //������һ��Ҫ��������ͷ�ļ�
#include "Delay.h"
#include "UCA1_UART1.H"
#include "Gpio_Init.h"
#include "Debug_Out.h"
#include "FC.H"
 /****************RC_PPM***************************/
unsigned int RxChStart1,RxChStart2,RxChStart3,RxChStart4;
int  RxCh1,RxCh2,RxCh3,RxCh4; 
 //����Ϊȫ�ֱ���ʹ�ã���ʱ������ֵ 
       int  RxAil,RxEle,RxRud;  //���������βδ��ݣ�ֱ�ӷŵ�����

        int  RxThr,RxThrLow;
unsigned char InLock=1;	//model is in lock ģ�ʹ�������״̬
unsigned char ArmCnt=0;	//Count for arm/disarm
unsigned char debug_off;//������ʾ׼����ɣ�Ȼ��ر�degug
 /****************ȫ�ֱ�������ϸ����***************************/


                  
//#######################################

//Busy flag, when true the RxCh? is invalid
//����æ��־��Ϊ1��ʱ��˵��RxCh���ɶ�	 
unsigned char RxChBusy;	
void PpmReadSignal(void);
void PpmWaitSignal(void);
int  LimitPpmValue(int v);

#define PPM_MAX_1 125
#define PPM_MIN_1 -125
#define RxChBase  3750 //70�Ϳ��Խ�����

#define GYROBASECNT 200

//#######################################
//Signal flag, each bit indicate a ch is got
//�źű�־��ÿһλ����1��ͨ�����źű��յ�	 
unsigned char RxValid;	

// int  LimitPpmValue(int v);



void Led_Init(void);
void Looprate_tst(void);
void Led_On(void);//led on;
void Led_Off(void);//led off
void Led_Toggle(unsigned char cnt);//led shanshuo ��Χ0-510
void Led_Toggle_short(void);//led shanshuo �̶̵���һ��
void Led_Toggle_long(void);//led shanshuo �ϳ�����һ��
void Led_tst(void);  // ���ϱ仯��˸pinl������led
void PWM_Pin_Init_soft(void); //���PWM
void PWM_Pin_Init_hard(void); //Ӳ��PWM
void IIC_Pin_Init(void);
void SDA_In(void);// Set P6.1--sda-In
void SDA_Out(void);// Set P6.1--sda-out
void RC_Pin_Init(void);
void IIC_Pin_DeInit(void);

/*********************Led_Init******************************************/
//��ʼ��LED.����ֻ��һ��LED,��Ҫ����ȷ����ʱ����˸���������
//P6.2 Ϊled���͵�ƽ��
//�ǵ���ʹ��LEDǰ��Ҫ��ʼ����
//�����и��ֿ��أ���˸LED

void Led_Init(void)
{
  P6DIR |= BIT2; // Set P6.2 to output direction
  P6REN |= BIT2; //������� 
  P6OUT |= BIT2; // ��ʱ����ߣ�ledϨ��
  
}



void Led_On(void)//led on
{
 
  P6OUT &= ~BIT2; // ���di��led on
  
}

void Led_Off(void)//led off
{
 
  P6OUT |= BIT2; // ���gao��led off
  
}

/**************ֱ��ȡ��LED����������loop��ʱ��*************************/
//��Ҫ��LED�ĳ�ʼ�������ʹ��
void Looprate_tst(void)
{
  P6OUT |=BIT2; 
  DelayUs(5);     // ��ȡ�ӿ��ٶ�
  P6OUT &= ~BIT2; // ���di��led on
  
}


void Led_Toggle(unsigned char cnt)//led shanshuo ��Χ0-510
{                                 //��������˸��������˸��ʱ�����2
  unsigned char cnt1;            
  cnt1=cnt/2;
  Led_On(); // on
  DelayMs(cnt1);
  Led_Off(); // off
  DelayMs(cnt1);
  
}

void Led_Toggle_short(void)//led shanshuo �̶̵���һ��
{                         //��������˸��������˸��ʱ�����2
 
  P6OUT &= ~BIT2; // on
  DelayMs(50);
  P6OUT |= BIT2; // off
  DelayMs(50);
  
}

void Led_Toggle_long(void)//led shanshuo �ϳ�����һ��
{                         //��������˸��������˸��ʱ�����2
 
  P6OUT &= ~BIT2; // on
  DelayMs(250);
  P6OUT |= BIT2; // off
  DelayMs(250);
  
}

void Led_tst(void)  // ���ϱ仯��˸pinl������led
{                    // ��0-250ms����˸������
  unsigned char i;
  for(i=0;i<250;i++)
  {
    Led_Toggle(i);
  }
  
}

/*********************Led_Init**************************************************/

/*********************PWM_Init**************************************************/
// PWM������ų�ʼ������ΪӲ��PWM�����PWM.
// Ӳ��PWM.ֱ��ѡ��ڶ�����
// ����Ļ�����ͨGPIO.�����������Ӧ�ø������š�ok
// ����ֱ��ʹ��Ӳ��PWM���
/*********************PWM_Init**************************************************/

void PWM_Pin_Init_soft(void) //���PWM
{
  
  P1DIR |= BIT1+BIT2+BIT3+BIT4; // Set P1.1--P1.4
  P1REN |= BIT1+BIT2+BIT3+BIT4; //�������

  
}
/********************************************************************/
//   P1SEL &= ~BIT5; //��ֹ��rc��ͻ����ʱע�͵�������ʵ�õ�ʱ��ע�����
/********************************************************************/

void PWM_Pin_Init_hard(void) 
{                           
  
  P1DIR |= BIT5+BIT2+BIT3+BIT4; // Set P1.2--P1.5

  P1REN |= BIT5+BIT2+BIT3+BIT4;   
  P1SEL |= BIT5+BIT2+BIT3+BIT4; 
  

  
}
/*********************PWM_Init**************************************************/

// ʹ���ⲿ����������IIC���ߵ�ʱ����Ҫȡ����430�����IIC
void IIC_Pin_DeInit(void)
// Set P6.0--scl  Set P6.1--sda

{
  P6DIR &= ~BIT1; // Set P6.1����
  P6DIR &= ~BIT0; // Set P6.0����
}
//  ����һ�ԣ����͹�Ŷ




/*********************iic_Init**************************************************/
// ������Ҫ���ٶȺ��ȶ������Խ����λ��������
// ʹ�����IIC
/*********************iic_Init**************************************************/
void IIC_Pin_Init(void)
// Set P6.0--scl  Set P6.1--sda

{
  P6DIR |= BIT1; // Set P6.1--sda
  P6DIR |= BIT0; // Set P6.0--scl

}

void SDA_Out(void)// Set P6.1--sda-out
{
   P6DIR |= BIT1;
  
}
void SDA_In(void)// Set P6.1--sda-In
{
   P6DIR &= ~BIT1;
  
}

/*********************iic_Init**************************************************/



/*********************RC_Init**************************************************/
// ������Ҫ�ǿ���������׼ȷ�Ժͱ���
// RC���ų�ʼ����1 ����Ϊ���� 2 �жϴ� 3 �жϷ�����L-H �任ΪH-L 
// 4 �ж�����ʹ�ô��ڷ��������Ա�����жϵķ�����
// ��·�ж�ͬʱ�᲻�ᷢ������жϵ����⡣
/*********************RC_Init**************************************************/

void RC_Pin_Init(void)
{
  P1DIR &= ~(BIT1+BIT6+BIT7);  // SetP1.1,P1.6,P1.7
  P2DIR &= ~BIT0;              // SetP2.0Ϊ����
  P1IE|=BIT1+BIT6+BIT7;        //SetP1.1,P1.6,P1.7�����ж�P
  P2IE|=BIT0;                  // SetP2.0�����ж�
  
  P2IES &=~BIT0;               // LOW-HIGH������һ��ʼ��׽PWM������
  P1IES &=~(BIT1+BIT6+BIT7);   // LOW-HIGH������һ��ʼ��׽PWM������
  
  P2IFG &=~BIT0;               // ����жϱ�־
  P1IFG &=~(BIT1+BIT6+BIT7);   // ����жϱ�־
  DelayMs(100); //�ȴ�2.4gģ���ȶ�
  
  _EINT();                     // �����ж�
  
}



/*********************�жϲ������**************************************/ 
//  �ж���ŷ�������־λ�������жϴ�����
//  ͨ�����ε�timer������ͬ�������PWM����ʱ��
//  �Ӷ��õ�ң����������

// ����ԭ��
       // �˴�����������2.5Mhz���ٶȽ���������
       // �����ջ���PWM��ռ�ձ�5%--10%��50hz 
       // ������Ϊ�������������һ��5%��50hz �Ĳ���Ҳ���� 1/50*0.05=1ms
       // �����10%��50hz����Ҳ����1/50*0.1=2ms
       // �����ʱ�������������2.5M���ٶ������У�Ҳ����˵��������������
       // ��ȡ�����ʱ�򣬲�ֵ����2.5m���ٶȣ���1MS--2MS��ʱ�����������ĸ���
       // Ҳ���� 2500--5000
/*********************�жϲ������**************************************/ 
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void) 
{
  
  int t=0;       // �����������ͨ������ֵ�����в������ж������
         
  if(P1IFG&0x02) // ֱ���ж� P1.1���ж�
  {

         P1IE&=~(BIT1+BIT6+BIT7);  //p1�����й��жϹر�
         P1IFG&=~BIT1;// �ֶ������־λ
         if(P1IN & BIT1)//�����أ�����cnt
         {
           RxChStart1=TB0R; //+��¼������ʱ����������ֵ������A5
           P1IES|=BIT1;
         } // ����h-low���ͣ�׼����һ���ж�
         if((P1IN & BIT1)==0)
         {
           RxChBusy=1;
	   RxCh1=TB0R-RxChStart1;	//�������ֵ��ȥǰ��ĵ���ռ�ձȵ�ʱ�䡣
           if(RxCh1<0){RxCh1=RxCh1+65535;} //�޷��������Ļ�˵�������һ��,������һȦ
           if(RxCh1<2500){RxCh1=3750;} 
           if(RxCh1>5000){RxCh1=3750;} //��������޶ȣ�����һ���޷���ȡ�м�ֵ   
	   RxChBusy=0;	 
           P1IES&=~BIT1;
           /***************************************************/
           t=RxCh1;		  	//ֱ���ڸ����ж��м���õ�������ֵ   
	   t-=RxChBase;	  			
	   RxRud=LimitPpmValue(t/10);	 //P1.1--YAW ����10
           /***************************************************/
         } //����L-HIGH���ͣ�׼����һ���ж�
         /***********************************************************/
 
         P1IE|=BIT1+BIT6+BIT7;  //p1�����й��жϴ�
  }

         
 if(P1IFG&0x40) // �ж� P1.6���ж�
 {
   
          P1IE&=~(BIT1+BIT6+BIT7);  
          P1IFG&=~BIT6;
          if(P1IN&BIT6)
          {
            RxChStart2=TB0R; 
            P1IES|=BIT6;
          } 
          if((P1IN&BIT6)==0)
          {
            RxChBusy=1;
	    RxCh2=TB0R-RxChStart2;	
            if(RxCh2<0){RxCh2=RxCh2+65535;} 
            if(RxCh2<2500){RxCh2=3750;} 
            if(RxCh2>5000){RxCh2=3750;} 
	    RxChBusy=0;	
            P1IES&=~BIT6;
            /*****************������ֵ��ֱ���ж����洦��******************/
            t=RxCh2;
	    t-=RxChBase;
	    //RxEle=LimitPpmValue(t/4);
            t+=1750;
            RxThr=LimitPpmValue(t/20);      //P1.6--thr ����10 thr�������и�����
            if(RxThr<0){RxThr=0;}           //�����м���1750Ȼ�����20
            /*****************************************/
          } 

          P1IE|=BIT1+BIT6+BIT7; 
  }
  
  
  if(P1IFG&0x80) 
  {

          P1IE&=~(BIT1+BIT6+BIT7);  
          P1IFG&=~BIT7;
          if(P1IN &BIT7)
          {
            RxChStart3=TB0R; 
            P1IES|=BIT7;
          } 
          if((P1IN &BIT7)==0)
          {
            RxChBusy=1;
	    RxCh3=TB0R-RxChStart3;	
            if(RxCh3<0){RxCh3=RxCh3+65535;} 
            if(RxCh3<2500){RxCh3=3750;} 
            if(RxCh3>5000){RxCh3=3750;}
	    RxChBusy=0;	
            P1IES&=~BIT7;
            /***********************PIT��ֵ��ֱ�Ӽ������**********************/
            t=RxCh3;		
	    t-=RxChBase;	  
            RxEle=LimitPpmValue(t/10);     //P1.7--pit ����10
            /***********************PIT��ֵ��ֱ�Ӽ������**********************/
          } 
 
           P1IE|=BIT1+BIT6+BIT7;  

  }       
  //else DelayUs(1); // ��Ҫ�����������ȷʵ��Ҫ�Ļ���ע�͵���Ӱ���ٶ�
  
}

 


#pragma vector=PORT2_VECTOR
__interrupt void Port_2_ISR(void) 
{   int t; //ÿ��ͨ��һ�����Iλ
  
    if(P2IFG&0x01==1) // �ж� P2.0���ж�
    {

          P2IE&=~BIT0; // ��ʱ��ֹ�ж�
          P2IFG&=~BIT0;// �ֶ������־λ
          if((P2IN&BIT0)==1)
          {
            RxChStart4=TB0R; //+��¼������ʱ����������ֵ������A5
            P2IES|=BIT0;
          } // ����h-low���ͣ�׼����һ���ж�
          if((P2IN&BIT0)==0)
          {
            RxChBusy=1;
	    RxCh4=TB0R-RxChStart4;	//�������ֵ��ȥǰ��ĵ���ռ�ձȵ�ʱ�䡣
            if(RxCh4<0){RxCh4=RxCh4+65535;} //�޷��������Ļ�˵�������һ��
            if(RxCh4<2500){RxCh4=3750;} 
            if(RxCh4>5000){RxCh4=3750;} //��������޶ȣ�����һ���޷���ȡ�м�ֵ   
	    RxChBusy=0;	
            P2IES&=~BIT0;
            /************************rol��ֵ��ֱ���ж����洦��********************************/
            t=RxCh4;
	    t-=RxChBase;
            RxAil=LimitPpmValue(t/10);     //P2.0--rol����10 -125--125
            /**************************rol��ֵ��ֱ���ж����洦��******************************/
          } //����L-HIGH���ͣ�׼����һ���ж�

          P2IE|=BIT0; //���ж�
       
    }
      
}

 int  LimitPpmValue(int v)
{
 	if(v>PPM_MAX_1)	return PPM_MAX_1;	   // PPM��ֵ�޷�
	if(v<PPM_MIN_1)	return PPM_MIN_1;	
	return v;
} 

/****************************************************************/
//ֱ����ȫ�ֱ���������ֵ
//��������2.5Mhz���ٶ����У���������/�½��ص�ʱ��ֱ��ȡһ�μ�����
//�õ��Ĳ�ֵ���Ǳ���ң����ͨ������������ֵ
/****************************************************************/
void PpmReadSignal(void)  //���������ݴ��ݣ�ֱ��ȫ�ֱ���
{
  
       // YAW,ELE,ROL��Χ -125-125 ��Ӧҡ������
       // thr��Χ 0-125
       // Ϊ�˸��Ӿ�ȷ����ʡʱ�䣬���в��������жϵ��н��С�
       // ������ֻ��Ϊ��ֵ��
       // ʵ���ϸ�����ֵ���ж������Ѿ���ֵ
 	

 	//RxRud=RxRud;	 //P1.1--YAW
	//RxThr=RxThr;     //P1.6--thr ����10 thr�������и�����
        //RxEle=RxEle;     //P1.7--pit ����10
	//RxAil=RxAil;     //P2.0--rol����10 -125--125
        
  /**********rc����ֵ����debug���鵱�з��͵���λ��**********************/ 
       if(MODE==0)           // ����ģʽ�жϣ�ֻ��24gģʽ�Ż�ѹ���ջ��
       {
         Debug[4]=RxRud;//   Debug[4]=RxRud;
         Debug[5]= RxThr;
         Debug[6]=RxEle;
         Debug[7]=RxAil;
        }
        
        if(RxThr>45){debug_off=1;} //����40 ��ʱ��׼����ɣ��ر�debug
        if(RxThr<=45){debug_off=0;} 

        
        
}




//###############################################
//   Test arming/disarming
//   �ж��Ƿ���Ҫ����/����
//----Hold rud stick for a while, ARMING_TIME should mul main-loop cycle
//----���ַ���ҡ��һ����/����ARMING_TIME������ѭ�����ھ���ʱ��
// ������������ͣ�����40�����Ե�������΢����+yaw����
// ������������ͼ�yaw����
// û����һ��loop������ͷ��ʹ�ô˺�������������
//###############################################

#define ARMING_TIME		  5
void ArmingRoutine(void)          //�ж�5���������� 
{       
	if((RxRud<-STICKGATE || RxRud>STICKGATE)&&(RxThr<RXthrGATE)) ArmCnt++; //����+yaw�����ж�
	else  ArmCnt=0;
			
	//Hold rud stick for a while, the num should mul main-loop cycle
	if(ArmCnt>ARMING_TIME)		
	{ 	
		if(InLock)
		{
		 	if((RxRud>STICKGATE)&&(RxThr<RXthrGATE)) 
			{
			 	//��������ŵ�+yaw�����ң�����
				//��������ŵ�+yaw�����󣬼���
							
							
			 	InLock=0;	     
			}
		}
		if(InLock==0)
		{
                  if((RxRud<-STICKGATE)&&(RxThr<RXthrGATE)){InLock=1;}
		}
	}
}



