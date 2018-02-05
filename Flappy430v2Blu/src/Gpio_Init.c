/* --COPYRIGHT--
 Flappy430-Copter by HHY,XDU,CHN
 * Copyright (c) 2014, HHY,Flappy430 Team CHN
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
 Flappy430-Copter by HHY,XDU,CHN
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

/***********************************************************************/
// Project Name : Flappy430-Copter
// File Name    : GPIO_Init.c
// Author       : HHY,Flappy430 Team
// Create Date  : 2014.5.25
// Function     : 设置需要用到的各个模块的GPIO初始化以及时钟
//                设置RC引脚中断允许。中断内部更改中断触发条件
//                ADC引脚暂时没有设置。PWM有硬件软件两种
//                最后采用硬件PWM波。故软件PWM的moto。c可以不看

//  接收机引脚对应关系： 
//                  P1.1--RC-YAW  P1.6--RC--THR P1.7--RC-ELE P2.0--RCAIL

//  msp430f5310----  32k rom， 6k ram
/* + Copyright (c) 05.2014 HHY*/ 

/***********************************************************************/
#include "msp430f5310.h"   //主函数一定要包含各个头文件
#include "Delay.h"
#include "UCA1_UART1.H"
#include "Gpio_Init.h"
#include "Debug_Out.h"
#include "FC.H"
 /****************RC_PPM***************************/
unsigned int RxChStart1,RxChStart2,RxChStart3,RxChStart4;
int  RxCh1,RxCh2,RxCh3,RxCh4; 
 //均作为全局变量使用，及时传递数值 
       int  RxAil,RxEle,RxRud;  //由于有了形参传递，直接放到里面

        int  RxThr,RxThrLow;
unsigned char InLock=1;	//model is in lock 模型处于锁定状态
unsigned char ArmCnt=0;	//Count for arm/disarm
unsigned char debug_off;//用来表示准备起飞，然后关闭degug
 /****************全局变量，仔细设置***************************/


                  
//#######################################

//Busy flag, when true the RxCh? is invalid
//接收忙标志，为1的时候说明RxCh不可读	 
unsigned char RxChBusy;	
void PpmReadSignal(void);
void PpmWaitSignal(void);
int  LimitPpmValue(int v);

#define PPM_MAX_1 125
#define PPM_MIN_1 -125
#define RxChBase  3750 //70就可以解锁了

#define GYROBASECNT 200

//#######################################
//Signal flag, each bit indicate a ch is got
//信号标志，每一位代表1个通道的信号被收到	 
unsigned char RxValid;	

// int  LimitPpmValue(int v);



void Led_Init(void);
void Looprate_tst(void);
void Led_On(void);//led on;
void Led_Off(void);//led off
void Led_Toggle(unsigned char cnt);//led shanshuo 范围0-510
void Led_Toggle_short(void);//led shanshuo 短短的闪一下
void Led_Toggle_long(void);//led shanshuo 较长的闪一下
void Led_tst(void);  // 不断变化闪烁pinl，测试led
void PWM_Pin_Init_soft(void); //软件PWM
void PWM_Pin_Init_hard(void); //硬件PWM
void IIC_Pin_Init(void);
void SDA_In(void);// Set P6.1--sda-In
void SDA_Out(void);// Set P6.1--sda-out
void RC_Pin_Init(void);
void IIC_Pin_DeInit(void);

/*********************Led_Init******************************************/
//初始化LED.但是只有一个LED,需要更精确的延时，闪烁次数区别等
//P6.2 为led。低电平亮
//记得在使用LED前需要初始化。
//后面有各种开关，闪烁LED

void Led_Init(void)
{
  P6DIR |= BIT2; // Set P6.2 to output direction
  P6REN |= BIT2; //上拉输出 
  P6OUT |= BIT2; // 暂时输出高，led熄灭
  
}



void Led_On(void)//led on
{
 
  P6OUT &= ~BIT2; // 输出di，led on
  
}

void Led_Off(void)//led off
{
 
  P6OUT |= BIT2; // 输出gao，led off
  
}

/**************直接取反LED，用来测试loop的时间*************************/
//需要与LED的初始化等配合使用
void Looprate_tst(void)
{
  P6OUT |=BIT2; 
  DelayUs(5);     // 争取加快速度
  P6OUT &= ~BIT2; // 输出di，led on
  
}


void Led_Toggle(unsigned char cnt)//led shanshuo 范围0-510
{                                 //由于是闪烁，所以闪烁的时间除以2
  unsigned char cnt1;            
  cnt1=cnt/2;
  Led_On(); // on
  DelayMs(cnt1);
  Led_Off(); // off
  DelayMs(cnt1);
  
}

void Led_Toggle_short(void)//led shanshuo 短短的闪一下
{                         //由于是闪烁，所以闪烁的时间除以2
 
  P6OUT &= ~BIT2; // on
  DelayMs(50);
  P6OUT |= BIT2; // off
  DelayMs(50);
  
}

void Led_Toggle_long(void)//led shanshuo 较长的闪一下
{                         //由于是闪烁，所以闪烁的时间除以2
 
  P6OUT &= ~BIT2; // on
  DelayMs(250);
  P6OUT |= BIT2; // off
  DelayMs(250);
  
}

void Led_tst(void)  // 不断变化闪烁pinl，测试led
{                    // 从0-250ms的闪烁半周期
  unsigned char i;
  for(i=0;i<250;i++)
  {
    Led_Toggle(i);
  }
  
}

/*********************Led_Init**************************************************/

/*********************PWM_Init**************************************************/
// PWM输出引脚初始化，分为硬件PWM和软件PWM.
// 硬件PWM.直接选择第二功能
// 软件的话，普通GPIO.上拉输出。对应好各个引脚。ok
// 最终直接使用硬件PWM输出
/*********************PWM_Init**************************************************/

void PWM_Pin_Init_soft(void) //软件PWM
{
  
  P1DIR |= BIT1+BIT2+BIT3+BIT4; // Set P1.1--P1.4
  P1REN |= BIT1+BIT2+BIT3+BIT4; //上拉输出

  
}
/********************************************************************/
//   P1SEL &= ~BIT5; //防止与rc冲突，暂时注释掉。后面实用的时候注意清除
/********************************************************************/

void PWM_Pin_Init_hard(void) 
{                           
  
  P1DIR |= BIT5+BIT2+BIT3+BIT4; // Set P1.2--P1.5

  P1REN |= BIT5+BIT2+BIT3+BIT4;   
  P1SEL |= BIT5+BIT2+BIT3+BIT4; 
  

  
}
/*********************PWM_Init**************************************************/

// 使用外部处理器连接IIC总线的时候，需要取消掉430的软件IIC
void IIC_Pin_DeInit(void)
// Set P6.0--scl  Set P6.1--sda

{
  P6DIR &= ~BIT1; // Set P6.1输入
  P6DIR &= ~BIT0; // Set P6.0输入
}
//  可以一试，加油哈哦




/*********************iic_Init**************************************************/
// 测试主要是速度和稳定，可以结合上位机来测试
// 使用软件IIC
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
// 测试主要是看发送数据准确性和比例
// RC引脚初始化。1 设置为输入 2 中断打开 3 中断方向由L-H 变换为H-L 
// 4 中断里面使用串口发送数据以便表面中断的发生。
// 几路中断同时会不会发生错过中断等问题。
/*********************RC_Init**************************************************/

void RC_Pin_Init(void)
{
  P1DIR &= ~(BIT1+BIT6+BIT7);  // SetP1.1,P1.6,P1.7
  P2DIR &= ~BIT0;              // SetP2.0为输入
  P1IE|=BIT1+BIT6+BIT7;        //SetP1.1,P1.6,P1.7允许中断P
  P2IE|=BIT0;                  // SetP2.0允许中断
  
  P2IES &=~BIT0;               // LOW-HIGH触发。一开始捕捉PWM上升沿
  P1IES &=~(BIT1+BIT6+BIT7);   // LOW-HIGH触发。一开始捕捉PWM上升沿
  
  P2IFG &=~BIT0;               // 清楚中断标志
  P1IFG &=~(BIT1+BIT6+BIT7);   // 清楚中断标志
  DelayMs(100); //等待2.4g模块稳定
  
  _EINT();                     // 打开总中断
  
}



/*********************中断捕获程序**************************************/ 
//  中断里欧面清楚标志位，反向中断触发沿
//  通过两次的timer读数不同，计算出PWM波的时间
//  从而得到遥控器的数据

// 捕获原理：
       // 此处，计数器以2.5Mhz的速度进行数数。
       // 而接收机的PWM是占空比5%--10%，50hz 
       // 以油门为例。油门最低是一个5%的50hz 的波，也就是 1/50*0.05=1ms
       // 最高是10%的50hz波，也就是1/50*0.1=2ms
       // 在这段时间里，计数器是以2.5M的速度在运行，也就是说，计数器在两次
       // 读取捕获的时候，差值是以2.5m的速度，在1MS--2MS的时间里面数数的个数
       // 也就是 2500--5000
/*********************中断捕获程序**************************************/ 
// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void) 
{
  
  int t=0;       // 用来计算各个通道的数值。所有操作在中断中完成
         
  if(P1IFG&0x02) // 直接判断 P1.1有中断
  {

         P1IE&=~(BIT1+BIT6+BIT7);  //p1所有有关中断关闭
         P1IFG&=~BIT1;// 手动清除标志位
         if(P1IN & BIT1)//上升沿，处理cnt
         {
           RxChStart1=TB0R; //+记录下来此时计数器的数值。都是A5
           P1IES|=BIT1;
         } // 换成h-low类型，准备下一次中断
         if((P1IN & BIT1)==0)
         {
           RxChBusy=1;
	   RxCh1=TB0R-RxChStart1;	//后面的数值减去前面的等于占空比的时间。
           if(RxCh1<0){RxCh1=RxCh1+65535;} //限幅。这样的话说明溢出了一次,多数了一圈
           if(RxCh1<2500){RxCh1=3750;} 
           if(RxCh1>5000){RxCh1=3750;} //如果误差超出限度，先做一个限幅，取中间值   
	   RxChBusy=0;	 
           P1IES&=~BIT1;
           /***************************************************/
           t=RxCh1;		  	//直接在各个中断中计算得到最终数值   
	   t-=RxChBase;	  			
	   RxRud=LimitPpmValue(t/10);	 //P1.1--YAW 除以10
           /***************************************************/
         } //换成L-HIGH类型，准备下一次中断
         /***********************************************************/
 
         P1IE|=BIT1+BIT6+BIT7;  //p1所有有关中断打开
  }

         
 if(P1IFG&0x40) // 判断 P1.6有中断
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
            /*****************油门数值。直接中断里面处理******************/
            t=RxCh2;
	    t-=RxChBase;
	    //RxEle=LimitPpmValue(t/4);
            t+=1750;
            RxThr=LimitPpmValue(t/20);      //P1.6--thr 除以10 thr不可以有负数。
            if(RxThr<0){RxThr=0;}           //所以有加上1750然后除以20
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
            /***********************PIT数值，直接计算完成**********************/
            t=RxCh3;		
	    t-=RxChBase;	  
            RxEle=LimitPpmValue(t/10);     //P1.7--pit 除以10
            /***********************PIT数值，直接计算完成**********************/
          } 
 
           P1IE|=BIT1+BIT6+BIT7;  

  }       
  //else DelayUs(1); // 需要添加这个吗？如果确实需要的话？注释掉，影响速度
  
}

 


#pragma vector=PORT2_VECTOR
__interrupt void Port_2_ISR(void) 
{   int t; //每個通道一個標誌位
  
    if(P2IFG&0x01==1) // 判断 P2.0有中断
    {

          P2IE&=~BIT0; // 暂时禁止中断
          P2IFG&=~BIT0;// 手动清除标志位
          if((P2IN&BIT0)==1)
          {
            RxChStart4=TB0R; //+记录下来此时计数器的数值。都是A5
            P2IES|=BIT0;
          } // 换成h-low类型，准备下一次中断
          if((P2IN&BIT0)==0)
          {
            RxChBusy=1;
	    RxCh4=TB0R-RxChStart4;	//后面的数值减去前面的等于占空比的时间。
            if(RxCh4<0){RxCh4=RxCh4+65535;} //限幅。这样的话说明溢出了一次
            if(RxCh4<2500){RxCh4=3750;} 
            if(RxCh4>5000){RxCh4=3750;} //如果误差超出限度，先做一个限幅，取中间值   
	    RxChBusy=0;	
            P2IES&=~BIT0;
            /************************rol数值。直接中断里面处理********************************/
            t=RxCh4;
	    t-=RxChBase;
            RxAil=LimitPpmValue(t/10);     //P2.0--rol除以10 -125--125
            /**************************rol数值。直接中断里面处理******************************/
          } //换成L-HIGH类型，准备下一次中断

          P2IE|=BIT0; //打开中断
       
    }
      
}

 int  LimitPpmValue(int v)
{
 	if(v>PPM_MAX_1)	return PPM_MAX_1;	   // PPM数值限幅
	if(v<PPM_MIN_1)	return PPM_MIN_1;	
	return v;
} 

/****************************************************************/
//直接用全局变量传递数值
//计数器以2.5Mhz的速度运行，遇到上升/下降沿的时候分别读取一次计数器
//得到的差值就是本次遥控器通道传递来的数值
/****************************************************************/
void PpmReadSignal(void)  //不利于数据传递，直接全局变量
{
  
       // YAW,ELE,ROL范围 -125-125 对应摇杆两端
       // thr范围 0-125
       // 为了更加精确，节省时间，所有操作换到中断当中进行。
       // 本函数只作为赋值用
       // 实际上各个数值在中断里面已经赋值
 	

 	//RxRud=RxRud;	 //P1.1--YAW
	//RxThr=RxThr;     //P1.6--thr 除以10 thr不可以有负数。
        //RxEle=RxEle;     //P1.7--pit 除以10
	//RxAil=RxAil;     //P2.0--rol除以10 -125--125
        
  /**********rc的数值加入debug数组当中发送到上位机**********************/ 
       if(MODE==0)           // 增加模式判断，只有24g模式才会压入堆栈。
       {
         Debug[4]=RxRud;//   Debug[4]=RxRud;
         Debug[5]= RxThr;
         Debug[6]=RxEle;
         Debug[7]=RxAil;
        }
        
        if(RxThr>45){debug_off=1;} //大于40 的时候，准备起飞，关闭debug
        if(RxThr<=45){debug_off=0;} 

        
        
}




//###############################################
//   Test arming/disarming
//   判断是否需要锁定/解锁
//----Hold rud stick for a while, ARMING_TIME should mul main-loop cycle
//----保持方向摇杆一会后解/锁，ARMING_TIME乘以主循环周期就是时间
// 解锁：油门最低（低于40，可以调节油门微调）+yaw向右
// 上锁：油门最低加yaw向左
// 没进行一次loop计算的最开头，使用此函数检测上锁情况
//###############################################

#define ARMING_TIME		  5
void ArmingRoutine(void)          //判断5次以免误判 
{       
	if((RxRud<-STICKGATE || RxRud>STICKGATE)&&(RxThr<RXthrGATE)) ArmCnt++; //油门+yaw进行判断
	else  ArmCnt=0;
			
	//Hold rud stick for a while, the num should mul main-loop cycle
	if(ArmCnt>ARMING_TIME)		
	{ 	
		if(InLock)
		{
		 	if((RxRud>STICKGATE)&&(RxThr<RXthrGATE)) 
			{
			 	//如果是油门低+yaw轴向右，解锁
				//如果是油门低+yaw轴向左，加锁
							
							
			 	InLock=0;	     
			}
		}
		if(InLock==0)
		{
                  if((RxRud<-STICKGATE)&&(RxThr<RXthrGATE)){InLock=1;}
		}
	}
}



