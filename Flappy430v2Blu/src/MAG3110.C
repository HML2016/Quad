
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

/*************************mag3110.c****************************************/
// mag3110传感器的读取，使用，计算
// 因为比较复杂所以单独出来作为一个。c文件
// 设置，读取并计算。具体实施参考了龙丘的51程序
// 单签还不是很理想。
// 读取后MAG_CAL放到了FC中，每次读取，参与计算
/************************mag3110.c*****************************************/

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
#define MAG_CTRL_REG1 0x10
//nop指令个数定义

//端口位定义，可修改
#define SCL_H         P6OUT|=BIT0
#define SCL_L         P6OUT&=~BIT0
   
#define SDA_H         P6OUT|=BIT1
#define SDA_L         P6OUT&=~BIT1

#define SCL_read      P6IN&BIT0      
#define SDA_read      P6IN&BIT1      //430支持这种类型的定义
//内部数据定义
uchar IIC_ad_main; //器件从地址
uchar IIC_ad_sub;  //器件子地址
uchar *IIC_buf;    //发送|接收数据缓冲区
uchar IIC_num;     //发送|接收数据个数
#define ack 1      //主应答
#define no_ack 0   //从应答	  




int MAG3110_XOFF=0,MAG3110_YOFF=0;
int MAG3110_XMax=0,MAG3110_YMax=0,MAG3110_XMin=0,MAG3110_YMin=0;
int MAG3110_XData=0,MAG3110_YData=0;
int ang;
int ang0; //第一次初始化的时候的角度，电机还没有转动，必须是全局变量
int magI; // 磁偏角的积分。也必须是全局变量。 需要赋给初始值0.到main中
/*************************计算中用到的各个变量*****************************/ 


void nops(void) 
{
  //DelayUs(1); //有待优化
  //__no_operation();
  // __no_operation();
  
}




//**********************************************
//送起始位 SDA_H->0
unsigned char IIC_start_MAG(void){
	SDA_H;
	SCL_H;
	I2C_delay();
        P6DIR&=~BIT1; //换成输入模式准备读
	//if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
        __no_operation();
        P6DIR|=BIT1; //换成输出模式准备输出
	SDA_L;
	I2C_delay();
	//if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
         __no_operation();
        P6DIR|=BIT1; //换成输出模式准备输出
	SDA_L;
	I2C_delay();
	//return TRUE;
}
//************************************************
//送停止位 SDA_L->1
void IIC_stop_MAG(void){
	SCL_L;
	__no_operation();
	SDA_L;
	__no_operation();
	SCL_H;
	nops();
	SDA_H;
	nops();
	SCL_L;
}
//*************************************************
//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
void send_byte_MAG(uchar c){
	uchar i;
	for(i=0;i<8;i++){
		SCL_L;
		if((c<<i) & 0x80)SDA_H; //判断发送位
		else SDA_L;
		__no_operation();
		SCL_H;
		nops();
		SCL_L;
	}
	nops();
	SDA_H; //发送完8bit，释放总线准备接收应答位
	__no_operation();
	SCL_H;
	nops(); //sda上数据即是从应答位
	SCL_L; //不考虑从应答位|但要控制好时序
}
//**************************************************
//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//return: uchar型1字节
uchar read_byte_MAG(void){
	uchar i;
	uchar c;
	c=0;
	SCL_L;
	__no_operation();
	SDA_H; //置数据线为输入方式
        P6DIR&=~BIT1;//置数据线为输入方式
	for(i=0;i<8;i++){
		__no_operation();
               // P6DIR|=BIT1; //置数据线为输出方式
		SCL_L; //置时钟线为低，准备接收数据位
		nops();
		SCL_H; //置时钟线为高，使数据线上数据有效
		__no_operation();
                
               // P6DIR&=~BIT1;//置数据线为输入方式
                
		c<<=1;
		if(SDA_read)c+=1; //读数据位，将接收的数据存c
               
	}
        
	SCL_L;
        P6DIR|=BIT1; //置数据线为输出方式
        
	return c;
}

unsigned char LQMAG_readbyte_MAG(unsigned char address)
{
	uchar ret = 100;
	IIC_start_MAG();		//启动
	send_byte_MAG(MAG3110_IIC_ADDRESS);	//写入设备ID及写信号
	send_byte_MAG(address);	//X地址
	IIC_start_MAG();		//重新发送开始
	send_byte_MAG(MAG3110_IIC_ADDRESS+1);	//写入设备ID及读信
	ret = read_byte_MAG();	//读取一字节
	IIC_stop_MAG();

	return ret;
}

//写入
void LQMAG_writebyte_MAG(unsigned char address, unsigned char thedata)
{
	IIC_start_MAG();		//启动
	send_byte_MAG(MAG3110_IIC_ADDRESS);	//写入设备ID及写信号
	send_byte_MAG(address);	//X地址
	send_byte_MAG(thedata);	//写入设备ID及读信
	IIC_stop_MAG();
}
/*********************************************************\
* Put MAG3110Q into Active Mode
\*********************************************************/
void MAG3110_Active ()
{
  byte n;
  n = LQMAG_readbyte_MAG(CTRL_REG1);
  LQMAG_writebyte_MAG(CTRL_REG1,n&0XFC|ACTIVE_MASK);
}

/*********************************************************\
* Put MAG3110Q into Standby Mode
\*********************************************************/
void MAG3110_Standby (void)
{
  byte n;

  n = LQMAG_readbyte_MAG( CTRL_REG1);
  LQMAG_writebyte_MAG(CTRL_REG1, n&0xFC|STANDBY_MASK);
}

/*********************************************************\
* Initialize MAG3110Q
\*********************************************************/
void MAG3110_Init (void)
{  
  MAG3110_Standby();   
  LQMAG_writebyte_MAG(CTRL_REG1, DATA_RATE_5MS);    
  MAG3110_Active();
}


void MAG_TST(void)
{
  
  int i;
  //tword wx, wy; 
  //tword wz;
  //char buf[17];
  MAG3110_Init(); //初始化MAG3110
	  
	  i= LQMAG_readbyte_MAG(WHO_AM_I_REG);
	  if (i == MAG3110Q_ID)	//确认初始化是否成功
	  {
	    Send_String_SCIA("ID:MAG3110Q,OK!\n ");  	     
	  }  
	  else //初始化失败
	  {
	    Send_String_SCIA("ID not identified,FAILED!\n"); 
		//for(;;);//初始化失败,停止在这里 	    
	  }
}

  


/*********************************************************\
*正式处理3110数据
*判断太多，恐怕严重拖慢速度啊~！
* 先看看对不对，再进行优化吧
\*********************************************************/

 int MAG3110_DataProcess (int MAG3110_XData,int MAG3110_YData)
{

  int MAG3110_Ang;
  MAG3110_XData -= MAG3110_XOFF; //还自带了一个减去中立点的函数呢。算是比较的
  MAG3110_YData -= MAG3110_YOFF; //考虑周全吧。好滴
  MAG3110_Ang= atan2((double)MAG3110_YData,(double)MAG3110_XData) * (180 / 3.14159265) + 180; // angle in degrees
  return MAG3110_Ang;
}


/*************************************************************************/
void MAG3110_STD(void)
// 此函数需多次执行以保证旋转一圈中
{
// 能够采集到真实的最大值和最小值
	tword wx, wy; 
       // tword wz;
	static   unsigned char	First_Flag=0;
	wx.mbyte.hi = LQMAG_readbyte_MAG(OUT_X_MSB_REG); //读取X轴高字节
	wx.mbyte.lo = LQMAG_readbyte_MAG(OUT_X_LSB_REG); //读取X轴低字节
	wy.mbyte.hi = LQMAG_readbyte_MAG(OUT_Y_MSB_REG); //读取Y轴高字节
	wy.mbyte.lo = LQMAG_readbyte_MAG(OUT_Y_LSB_REG); //读取Y轴低字节
	//wz.mbyte.hi = Single_Read(MAG3110_IIC_ADDRESS,OUT_Z_MSB_REG); //读取Z轴高字节
	//wz.mbyte.lo = Single_Read(MAG3110_IIC_ADDRESS,OUT_Z_LSB_REG); //读取Z轴低字节
	//printf("X:%d  ",wx.mbyte.hi*256+wx.mbyte.lo);
	//printf("Y:%d  ",wy.mbyte.hi*256+wy.mbyte.lo);
	//printf("Z:%d  ",wz.mbyte.hi*256+wz.mbyte.lo);
	
	
	MAG3110_XData=wx.mbyte.hi*256+wx.mbyte.lo;
	MAG3110_YData=wy.mbyte.hi*256+wy.mbyte.lo;
	
	if (!First_Flag)
	{
	MAG3110_XMax = MAG3110_XData;
	MAG3110_XMin = MAG3110_XData;
	MAG3110_YMax = MAG3110_YData;
	MAG3110_YMin = MAG3110_YData;
	First_Flag = 1;
	}
	if (MAG3110_XData > MAG3110_XMax)
	{
	MAG3110_XMax =  MAG3110_XData;
	}
	else if (MAG3110_XData < MAG3110_XMin)
	{
	MAG3110_XMin =  MAG3110_XData;
	}
	if (MAG3110_YData > MAG3110_YMax)
	{
	MAG3110_YMax =  MAG3110_YData;
	}
	else if (MAG3110_YData < MAG3110_YMin)
	{
	MAG3110_YMin =  MAG3110_YData;
	}
	MAG3110_XOFF = (MAG3110_XMax + MAG3110_XMin) / 2;
	MAG3110_YOFF = (MAG3110_YMax + MAG3110_YMin) / 2;
	
        // printf("\r\nMAG3110_XMax：%d ",MAG3110_XMax);
	//printf("MAG3110_XMin：%d\r\n",MAG3110_XMin);
       // printf("MAG3110_XOFF：%d\r\n",MAG3110_XOFF);

       // printf("\r\nMAG3110_YMax：%d  ",MAG3110_YMax);
       //printf("MAG3110_YMin：%d\r\n ",MAG3110_YMin);
       //printf("AG3110_YOFF：%d\r\n",MAG3110_YOFF);

	ang=MAG3110_DataProcess(wx.mbyte.hi*256+wx.mbyte.lo,wy.mbyte.hi*256+wy.mbyte.lo);
        ang0=ang; // ang0一直到后面都作为初始的磁偏角来使用。好
}


void mag3110_tst(void)
{
  unsigned char iflag;
  

  MAG3110_Init(); //初始化MAG3110
  iflag=LQMAG_readbyte_MAG(WHO_AM_I_REG);
  
  if (iflag == MAG3110Q_ID)	//确认初始化是否成功
  {
    Send_String_SCIA("ID:MAG3110Q,OK!\n ");  	     
   }  
  else //初始化失败
  {
     Send_String_SCIA("ID not identified,FAILED!\n"); 
     //for(;;);//初始化失败,停止在这里 	    
	  }
 while(1)
  {	
     DelayMs(100);
     iflag=LQMAG_readbyte_MAG(STATUS_00_REG); 
     Send_Char_SCIA(iflag);
     Send_Char_SCIA(iflag);
     if(iflag&ZYXDR_MASK) //数据就绪
     { 	
	MAG3110_STD(); //读取MAG3110数据，标定中值，数据处理
	Send_String_SCIA("The data needs to be calibrated, turning a lap");
	Send_String_SCIA("\r\n\r\n");
        Send_String_SCIA("\r\nPoint to the south angle：%d°\r\n");
        SCIA_Print_Float(ang); //用float形式发送ang出去。好滴
      }
     else //数据未就绪
     {
	Send_String_SCIA("ID Failed!\n");
      }	
      }
}


/************************************************************************/
// 初始化mag。在main函数中，一次上电只执行一次
// 上电后，LED闪烁三次。提醒旋转以便校准中立点。
//  这一阶段完成了最大，最小值的采集之后，后面阶段
//  就可以直接进行计算了。
/************************************************************************/
void MAG_INIT(void)
{
   unsigned char i;
   Led_Toggle_long();
   Led_Toggle_long();
   Led_Toggle_long();  // 闪烁3下，提示校准中立点。
   for(i=0;i<100;i++)
   {
     MAG3110_STD();    // 多次执行，以便确定最大值。
   }                   // 这一阶段，使用串口助手观察数据
   Led_On();
   DelayMs(50);       // 点亮LED，表示mag的初始化已经完成好低
   Led_Toggle_long(); // 再闪烁两下，校准完毕
   Led_Toggle_long(); // 闪烁3下，提示校准中立点。
   Led_Off();         // 熄灭LED，表示等待下面的解锁才继续点亮
  
}


/************************************************************************/
// 切记在使用此函数之前，需要进行初始化
// 用于一次loop中，比较快速的计算出ang
// 并且要进行PID运算。
// 需要将第一次的初始ang另外设置一个变量来储存。
// 每次loop都要调用的东西，必须使用移位来进行优化。
// 是否需要一个两次采用取平均？
 /************************************************************************/


int MAG_CAL(void)  // 很多句子可以优化
{
       unsigned char mag_ready; //数据就绪的时候再读取
       tword wx, wy; 
       // tword wz;

       mag_ready=LQMAG_readbyte_MAG(STATUS_00_REG); 
 
       if(  mag_ready&ZYXDR_MASK) //数据就绪
       {
	wx.mbyte.hi = LQMAG_readbyte_MAG(OUT_X_MSB_REG); //读取X轴高字节
	wx.mbyte.lo = LQMAG_readbyte_MAG(OUT_X_LSB_REG); //读取X轴低字节
	wy.mbyte.hi = LQMAG_readbyte_MAG(OUT_Y_MSB_REG); //读取Y轴高字节
	wy.mbyte.lo = LQMAG_readbyte_MAG(OUT_Y_LSB_REG); //读取Y轴低字节
	//wz.mbyte.hi = Single_Read(MAG3110_IIC_ADDRESS,OUT_Z_MSB_REG); //读取Z轴高字节
	//wz.mbyte.lo = Single_Read(MAG3110_IIC_ADDRESS,OUT_Z_LSB_REG); //读取Z轴低字节
	//printf("X:%d  ",wx.mbyte.hi*256+wx.mbyte.lo);
	//printf("Y:%d  ",wy.mbyte.hi*256+wy.mbyte.lo);
	//printf("Z:%d  ",wz.mbyte.hi*256+wz.mbyte.lo);
	
	
	MAG3110_XData=wx.mbyte.hi<<8+wx.mbyte.lo;
	MAG3110_YData=wy.mbyte.hi<<8+wy.mbyte.lo;
	ang=MAG3110_DataProcess(MAG3110_XData,MAG3110_YData);
        magI+=ang;  // 使用积分，一直累积。加油 只有在完成一次新的读取的时候才加
      }
 
      if(magI>4000){magI=4000;}
      if(magI<-4000){magI=-4000;}
  
      Debug[10]=ang;
      // 其他的应用可以有printf，但是，作为loop中的不可以。因为要发送到上位机
      // LQMAG_writebyte_MAG(MAG_CTRL_REG1,0x12); 
      //SCIA_Print_Float(ang);  //之前有乱码。显示错乱应该是因为有这个。！！！
      return ang;  // 输出到上位机显示。注意测试时钟周期变慢了多少。
  
}

  
  
  
  
