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

/************************IIC_SENXOR.C**************************************/ 
// IIC硬件的设置，以及各个传感器的寄存器，使用方法
// 使用模拟IIC。
// IIC(模拟） SCL--P6.0  SDA--P6.1 
// mag的有关知识没有加入
// 切换IIC基础函数的输入输出，也就是SDA_IN,SDA_OUT对效果有很大帮助
// delay5ms(); 这个对于整体速度提升应该大大有害的。注释掉！
/************************IIC_SENXOR.C*************************************/ 


/**************************************************************/ 
#include "msp430f5310.h"   //主函数要包含各个头文件
#include "Delay.h"
#include "Gpio_Init.h"
#include "Debug_Out.h"
#include "Debug_Out.h"
#include "IIC_SENSOR.h"
#include "FC.H" //数据的交换与匹配
//定义ITG3205内部地址********************
#define WHO	0x00
#define	SMPL	0x15
#define DLPF	0x16
#define INT_C	0x17
#define INT_S	0x1A
#define	TMP_H	0x1B
#define	TMP_L	0x1C
#define	GX_H	0x1D
#define	GX_L	0x1E
#define	GY_H	0x1F
#define	GY_L	0x20
#define GZ_H	0x21
#define GZ_L	0x22
#define PWR_M	0x3E
//****************************
#define FALSE 0
#define TRUE  1
//****************************
#define	ADXL345_Addr    0xA6	
//#define	ADXL345_Addr    0x3A //前面的地址有冲突的话用这个地址
#define	ITG3205_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

unsigned char TX_DATA[4];  	 //显示据缓存区
unsigned char BUF[10];       //接收数据缓存区
char  test=0; 				 //IIC用到
int T_X,T_Y,T_Z,T_T,A_X,A_Y,A_Z; //X,Y,Z轴，温度 acc

//************************************
/*模拟IIC端口输出输入定义*/
#define SCL_H         P6OUT|=BIT0
#define SCL_L         P6OUT&=~BIT0
   
#define SDA_H         P6OUT|=BIT1
#define SDA_L         P6OUT&=~BIT1

#define SCL_read      P6IN&BIT0      
#define SDA_read      P6IN&BIT1      //430支持这种类型的定义


void I2C_delay(void);
void delay5ms(void);
void I2C_SendByte(unsigned char SendByte); //数据从高位到低位//
unsigned char I2C_WaitAck(void); 	
void I2C_NoAck(void);
void I2C_Ack(void);
void I2C_Stop(void); //对于IIC的整体运行速度有很大的作用
unsigned char I2C_Start(void);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
unsigned char I2C_RadeByte(void);  //数据从高位到低位//
unsigned char Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
void Init_ITG3205(void); 
void READ_ITG3205(void);
void  Init_ADXL345(void);
void read_ADXL345(void);
int Gyro_Acc_Filter_1(char flitercnt);//关键的数字滤波函数。
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
		
   // unsigned char i=30; //这里可以优化速，经测试最低到5还能写入
  //  while(i)            // 这个对于430应该要加速吧？或者换用delayus会好一点？
   // {                   // 对于IIC的运行速度起到关键性作用
    //  i--; 
   // }  
    //DelayUs(3);  //尝试使用这个函数 .貌似还真是1us是效果比较好的？
   //__no_operation();
}

void delay5ms(void)
{
   DelayMs(5);       // 直接使用已有的延时函数，更加精确
}


/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
unsigned char I2C_Start(void)                  //一步步仔细移植测试
{
	SDA_H;
	SCL_H;
	I2C_delay();
        P6DIR&=~BIT1; //换成输入模式准备读
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
        P6DIR|=BIT1; //换成输出模式准备输出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
        P6DIR|=BIT1; //换成输出模式准备输出
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void) //对于IIC的整体运行速度有很大的作用
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
unsigned char I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
        P6DIR&=~BIT1; //换成输入模式准备读
	if(SDA_read)
	{P6DIR|=BIT1; //输出模式
      SCL_L;
	  I2C_delay();
      return FALSE;
	}
        P6DIR|=BIT1; //输出模式
	SCL_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
{
    unsigned char  i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}  

/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    unsigned char i=8;
    unsigned char ReceiveByte=0;

    SDA_H;	
    P6DIR&=~BIT1; //换成输入模式，应该会读取数据更加准确。试试好滴
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    P6DIR|=BIT1; //换回输出模式。好
    SCL_L;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

unsigned char Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    DelayUs(5); //改成延时5us吧
    // delay5ms(); 这个对于整体速度提升应该大大有害的。注释掉！
    return TRUE; //这个为什么是delay这么久呢？到底？会不会太慢了？
}

//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   
    unsigned char REG_data;     	
    if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((unsigned char) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}	

// 记得还需要进行GPIO的初始化

//初始化ITG3205，根据需要请参考pdf进行修改************************

void Init_ITG3205(void) // 量程与滤波可以进一步优化的
{                       // 
   Single_Write(ITG3205_Addr,PWR_M, 0x80);   //
   
  //  Single_Write(ITG3205_Addr,SMPL, 0x07);    // 125hz
  // Single_Write(ITG3205_Addr,DLPF, 0x1E);    //±2000°1khz输出 5hz低通滤波器.
   
   Single_Write(ITG3205_Addr,INT_C, 0x00 );  //上面的那个是设置的关键。参考小M.1K速率，98hz滤波器
   
   Single_Write(ITG3205_Addr,DLPF, 0x1A);    //±2000°98HZ滤波器
   Single_Write(ITG3205_Addr,SMPL, 0x00);    //1khz刷新
   
   Single_Write(ITG3205_Addr,PWR_M, 0x00);   
}
	
//******读取ITG3205数据****************************************
void READ_ITG3205(void)
{
   BUF[0]=Single_Read(ITG3205_Addr,GX_L); 
   BUF[1]=Single_Read(ITG3205_Addr,GX_H);
   T_X=	(BUF[1]<<8)|BUF[0];
   //直接使用原始数数据
   //读取计算各个轴数据
   BUF[2]=Single_Read(ITG3205_Addr,GY_L);
   BUF[3]=Single_Read(ITG3205_Addr,GY_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
					   
   BUF[4]=Single_Read(ITG3205_Addr,GZ_L);
   BUF[5]=Single_Read(ITG3205_Addr,GZ_H);
   T_Z=	(BUF[5]<<8)|BUF[4];

}
 //********串口发送数据***************************************


//*********************加速度计*************************
//选用100hz太慢了吧
void  Init_ADXL345(void) 
{
   //Single_Write(ADXL345_Addr,0x31,0x0B);   //测量范围,正负16g，13位模式
   Single_Write(ADXL345_Addr,0x31,0x00);   //测量范围,正负2g，10bit模式
    //  Single_Write(ADXL345_Addr,0x2C,0x0e);   //速率设定为100hz 参考pdf13页
   Single_Write(ADXL345_Addr,0x2C,0x09);   //速率设定为50hz 参考pdf13页
   Single_Write(ADXL345_Addr,0x2D,0x08);   //选择电源模式   参考pdf24页 与M一样
   Single_Write(ADXL345_Addr,0x2E,0x80);   //使能 DATA_READY 中断
 
}



void read_ADXL345(void)
{
       
       BUF[0]=Single_Read(ADXL345_Addr,0x32);//OUT_X_L_A
       BUF[1]=Single_Read(ADXL345_Addr,0x33);//OUT_X_H_A

       BUF[2]=Single_Read(ADXL345_Addr,0x34);//OUT_Y_L_A
       BUF[3]=Single_Read(ADXL345_Addr,0x35);//OUT_Y_H_A

       BUF[4]=Single_Read(ADXL345_Addr,0x36);//OUT_Z_L_A
       BUF[5]=Single_Read(ADXL345_Addr,0x37);//OUT_Z_H_A

       A_X=(BUF[1]<<8)+BUF[0];  //合成数据  
       A_Y=(BUF[3]<<8)+BUF[2];  //合成数据
       A_Z=(BUF[5]<<8)+BUF[4];  //合成数据
       
}



//******软件滤波Gyro-Acc-Filter_1***************************************
//  基础读取滤波函数。平均值滤波法。可以选择采样几次
//  虽然是简单的平均值滤波，还是比较有作用的
//  最多可以采样20次取平均值
//**********************************************************************

int Gyro_Acc_Filter_1(char flitercnt) //可以自定义滤波几次1-5次 暂时取两次
{
  
    int Gyro_rol_value[20],Gyro_pit_value[20],Gyro_yaw_value[20],Acc_rol_value[20],Acc_pit_value[20],Acc_yaw_value[20];
    char cnt;
    cnt=flitercnt;
    for(cnt=0;cnt<flitercnt;cnt++)  //读取两个传感器的数据cnt次，并压入堆栈 0-cnt-1
    {                               // 在函数里面定义数组，最终输出改变全局变量
      READ_ITG3205();               // 此后这6个大型数组的空间会被释放
      read_ADXL345();
      Gyro_rol_value[cnt]=T_Y;//GyroRol
      Gyro_pit_value[cnt]=T_X;//GyroPit
      Gyro_yaw_value[cnt]=T_Z;//GyroYaw
      
     if(A_X>250){A_X=0;}  
     if(A_X<-250){A_X=0;} //限幅策略
     
     if(A_Y>250){A_Y=0;}
     if(A_Y<-250){A_Y=0;}
     
     if(A_Z>250){A_Z=0;}
     if(A_Z<-250){A_Z=0;}
      
      
      Acc_rol_value[cnt]=A_X; //GyroAccRol 在read当中赋值。
      Acc_pit_value[cnt]=A_Y;//GyroAccPit
      Acc_yaw_value[cnt]=A_Z;//GyroAccYaw 读取n次，压满堆栈
      
      
      
   }
  
   // 直接使用平均值滤波法。
   for(cnt=0;cnt<flitercnt;cnt++) //平均值滤波法。
     {
       T_Y+=Gyro_rol_value[cnt]; //int型
       T_X+=Gyro_pit_value[cnt]; //加5次
       T_Z+=Gyro_yaw_value[cnt]; 
       A_X+= Acc_rol_value[cnt];
       A_Y+= Acc_pit_value[cnt];
       A_Z+= Acc_yaw_value[cnt];
     }
     T_Y/=(flitercnt*8);    // 平均值滤波
     T_X/=(flitercnt*8);    
     T_Z/=(flitercnt*8);    
     
     A_Y/=(flitercnt);     //平均值滤波
     A_X/=(flitercnt);    
     A_Z/=(flitercnt);  
     
     GyroAccRol=A_X;       //赋值，为kalman滤波等等做准备
     GyroAccPit=A_Y;
     GyroAccYaw=A_Z;
     
     GyroRol=T_Y;
     GyroPit=T_X;
     GyroYaw=T_Z; //限幅
     if(GyroAccRol>250){GyroAccRol=0;}
     if(GyroAccRol<-250){GyroAccRol=0;}
     
     if(GyroAccPit>250){GyroAccPit=0;}
     if(GyroAccPit<-250){GyroAccPit=0;}
     
     if(GyroAccYaw>250){GyroAccYaw=0;}
     if(GyroAccYaw<-250){GyroAccYaw=0;}
     
     
     return T_Y;
     
     
}