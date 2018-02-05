
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
 //具体的方向，PID参数设置在程序开头的define中。
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
//   Debug[13]=Reading_IntegralGyroNick; //有个上位机方便多了
//   Debug[14]=Reading_IntegralGyroRoll;
//   Debug[16]=IntegralErrorNick; //有个上位机方便多了
//   Debug[15]=IntegralErrorRoll; //有个上位机方便多了

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
/*Blue*/ 
// 加入蓝牙模块HC06，可与手机连接进行控制。软件及PID参数有待改善。
//油门计算，thr_cal=RxTHR_BLUE; 
//这里由于是直接串口发，直接发0-125 无需再乘以
// 可以选择蓝牙控制或者是遥控器控制

// 看一下融合效果
//  AttitudeCorrectionNick>>1; //可以根据当前传感器以及IIC速度等
//  AttitudeCorrectionRoll>>1; // 进行修正。例如，/2等
//  还是原来的好吧。
//msp430f5310----  32k rom， 6k ram
/* + Copyright (c) 05.2014 HHY*/ 
 /* Xidian University*/
 //版本信息 14.0519  未调试
 //版本信息 14.0520 未调试 修正了一些标点，语句等小BUG.增加一些初始化函数
/***********************************************************************/
#include "msp430f5310.h"   //主函数要包含各个头文件
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



/******************pid参数及陀螺翻转等设置*****************************/
// PID参数采用最原始的。直接按照客观大小算kp，kp*0.15等于ki
#define Kp 0.12  // 初始推算：250/500/2 x字 然后再调节
#define Ki 0.0028  // I参数 初始推算250/8000/20.015 
#define Kyaw 0.11  // yaw轴系数 
#define KyawI 0.006 // YAW的积分系数

/*********************************************************/
#define KpMAG 0.01   // mag还说不是很稳定，先较少
#define KpMAGI 0.001
// mag初始推算：
//设。一共是0-360度。而motor的数值是250，所以kp=0.7.再调节

/**********************互补滤波***************************/
#define KpNEW 0.65  // 80%采信本次的数据
#define KpOLD 0.35



/*****************以上是磁偏角的设置**********************************/

/***********************传感器方向设置********************************/

#define Rol_fu 1     //gyro翻转信号，根据实际位置测定
#define Pit_fu 0
#define Yaw_fu 1
#define Rol_ACC_fu 0 //ACC翻转信号
#define Pit_Acc_fu 0  
#define Yaw_Acc_fu 0 

#define PI_PIT 127  //可以用于作为PID系数，暂时没有用到。
#define PI_ROL 127 
#define PI_YAW 127
/**********************上位机储存数据数组****************************/

int Debug[23];          //用来存储debug数据

/*****************  飞行控制参数等设置******************************/
// Global Vars define
// 全局变量定义
#define AxisMode    1  //Fly control mode 飞控模式//0--十字，1--X字
#define GYROBASECNT 250 //中立点采样次数设置
//--------------------------------------
unsigned char  SoftSet;	//Fly control software setting 飞控软件设置
//--------------------------------------
//##Arm 锁定
// unsigned char  InLock=1;	//model is in lock 模型处于锁定状态
// unsigned char  ArmCnt=0;	//已更换其他变量起到此作用
// 是在要用就换个名字好了
/*****************  飞行控制参数等设置*****************************/

unsigned char GyroBaseCnt=0;//Gyro base build count 陀螺仪中点建立计数器

//--------------------------------------
//##Integral angle 角度积分

#define I_MAX	  4000  
                      
int GyroRolI,GyroPitI,GyroYawI;
// 积分上限初步计算：
//14.375*180度=2587 所以用180度来作为积分限度是2587.实际测试用4000
int thr_cal_old=0,ail_cal_old=0,ele_cal_old=0,rud_cal_old=0,MAG_cal_old=0; 
// 全局变量，初始值0
// 因为感觉单纯采信本次的，有时有振荡。所以增加互补滤波。好滴
/*********************************************************************/

/************************各个计算用的变量定义*************************/

int GyroRol,GyroPit,GyroYaw;              //三轴陀螺仪数据，数字传感器输入
int GyroBaseRol,GyroBasePit,GyroBaseYaw;  //三轴陀螺仪基准。

int GyroAccRol,GyroAccPit,GyroAccYaw;     //加速度计的三轴数据
int GyroAccBaseRol,GyroAccBasePit,GyroAccBaseYaw; 
//这些需要与read配合，所以必须设置为全局变量

/*********************kalman滤波计算要素********************************/

#define ACC_AMPLIFY 8         //将加速度计的数据与陀螺数据匹配起来可以融合。参考数据手册
                    
#define TurnOver180Roll 4000  //积分上限，旋转180度
#define TurnOver180Nick 4000 
#define BALANCE_NUMBER 256L   //256次后进行长期数据融合
#define GyroAccFactor 8       //缩小陀螺仪的数据，与ACC融合
#define BALANCE 320           // 调节量的限幅。


#define GyroAccTrim   8       //缩小陀螺仪的数据
#define BALANCE_NUMBER_1 25   //测试水平

int  balance_number=0; //
int Reading_GyroYaw,Reading_GyroRoll,Reading_GyroNick;
int Mean_AccNick,Mean_AccRoll,Mean_AccTop;

int Reading_IntegralGyroRoll2,Reading_IntegralGyroRoll,Reading_IntegralGyroNick2,Reading_IntegralGyroNick;
int IntegralAccNick = 0, IntegralAccRoll = 0;
//用于姿态融合的各个变量。均设置为全局变量。以便后面方便调用与融合
int Reading_IntegralGyroNick = 0, Reading_IntegralGyroNick2 = 0;
int Reading_IntegralGyroRoll = 0, Reading_IntegralGyroRoll2 = 0;//带有2的是原始数据。不带的是融合后的数据
float  AttitudeCorrectionRoll = 0, AttitudeCorrectionNick = 0;  //长期融合的关键更正
//经常是0.xx。所以设置为float
int IntegralNick = 0,IntegralNick2 = 0;
int IntegralRoll = 0,IntegralRoll2 = 0;//积分与2 

float MeanIntegralNick; //后面做融合用
float MeanIntegralRoll; //这个是积分的积分

// unsigned char Motor1,Motor2,Motor3,Motor4; 在motor中已经有定义了

int ReadingNetNick=0; //积分中立点
int ReadingNetRoll=0; //正确的积分加正确的PID叙述

// 以上变量的赋值与调用基本思路：
// 一开始等于0.后面到mean——kalman中去更改
/**********************************************/
int thr_cal=0; //用于最后的PID输出计算
int ail_cal=0;
int ele_cal=0;
int rud_cal=0; 
//int GyroYawI=0; //一开始=0，后面进行计算，不用清零
//前面已经有定义了
/***************************各个FC.C中的函数声明******************************************/

/*********************函数名称******************************************/
void plane_tst(void);
int GyroCompe(int gyro,signed char pn);
void CaclAttitude(void);
void AxisMixer(void);
void AxisMixer_Loop_TST(void);
/*********************kalman滤波计算***********************************/
void Mean_kalman(void);
void MotorControl_kalman(void);
void Base_Tst(void);//检测是否解锁，中立点建立。
void Gyro_Acc_Read(void); //数据基础来源
void AxisMixer_Loop(void);
int labs(int shu);
void PID_Mixer (void); //输出到大四周。5%-10%占空比
void MotorControl_kalman_zhonglidiantst(void);

void PWM_Out_Da(void);   //输出大四周的5%-10%的波形
void PWM_Out_xiao(void); //输出到小四轴。0%--100%的占空比
void _loop(char mode);   // mode=0 小四 mode=1 大四 主循环 有待完善

/*************飞行状态指示。在loop开始后的初始化中很重要**************/

unsigned char plane_ready;  //是否水平标志，用于判断是否可以起飞

/***************************************************/


/***************************具体的函数******************************************/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


int labs(int shu)  //取绝对值函数
{
  if(shu>=0){return shu;}
  else
  return -shu;
}


/************************palne-tst*********************************************/
//   palne-tst 是否水平判断 
//   在解锁后使用，判断初始起飞状态是否水平，不水平则LED一直闪烁
//   因为本程序将解锁后的姿态认为是水平的姿态，并作为飞行标准。所以
//   解锁后放置为水平很重要。
//   判断方法：
//   如果超过15度倾角则锁定，直到水平并且解锁后才可以起飞。
//   实际中，为简便起见，并未计算atan，而是直接使用原始值
/************************palne-tst*********************************************/

void plane_tst(void) // 检测当前是否水平
{
  char i;
  int angle_rol,angle_pit; // 横滚角度与俯仰角度

  read_ADXL345();
  Gyro_Acc_Filter_1(10);
  Gyro_Acc_Read();// 必须调用这个，才有减去中立点和正负号操作，不然就错了
  //angle_rol=atan(labs(GyroAccRol)/labs(GyroAccPit))*57.3;
  //angle_pit=atan(labs(GyroAccPit)/labs(GyroAccRol))*57.3;
  
  /*******************直接根据经验判断**************************/
  angle_rol=labs(GyroAccBaseRol);
  angle_pit=labs(GyroAccBasePit);
  // 因为有中立点修正，所以后面的都是减去中立点的，必须用前面的中立点来计算
  if(angle_rol>25||angle_pit>25)// 有任意一个角度大于15度，则闪烁LED,锁定
  { 
    for(i=0;i<250;i++)  //闪烁250次提示没有放平
    {
      Led_Toggle(100);           
      plane_ready=0;  // 如果不平的话就上锁加水平标志复位
    }
  }
  else
  plane_ready=1;//水平标志位1.可以进行下一步动作。
  
}


/*******************************Base_Tst**************************************/
//  检测中立点是否建立。如果没有，建立中立点
//  为后面的数据计算做准备
//   解锁后调用，建立中立点基准。作为之后的飞行标准姿态
//   解锁后放平十分重要
/*******************************Base_Tst**************************************/


void Base_Tst(void)          //检测是否解锁，中立点建立。
{

  Gyro_Acc_Filter_1(4);      // 滤波4次。里面有读取snsor
  Gyro_Acc_Read();           // 减去中立点和正负号操作

  plane_tst();               // 检测当前是否水平。不水平则不予解锁
  //If no gyro base, calibrate gyro
  //如果还未建立基准，建立它
  //有没有建立基准、建议还是有一个检测倾斜角并且提醒
  GyroBaseCnt=GYROBASECNT;   // 设定中立点
  
  GyroBaseRol=GyroRol;
  GyroBasePit=GyroPit;
  GyroBaseYaw=GyroYaw;
  GyroAccBaseRol=GyroAccRol; // 提前赋值，避免第一次的误差。
  GyroAccBasePit=GyroAccPit; // 
  GyroAccBaseYaw=GyroAccYaw; // 
  for(GyroBaseCnt=GYROBASECNT;GyroBaseCnt>0;GyroBaseCnt--)
  {	
        Gyro_Acc_Filter_1(10);  //读取GYROBASECNT次并且取平均
        GyroBaseRol+=GyroRol;  //GyroBaseRol一开始是没有数值的啊、这样直接除以2不是有误差吗
        GyroBasePit+=GyroPit; 
	GyroBaseYaw+=GyroYaw;
	GyroAccBaseRol+=GyroAccRol;
	GyroAccBasePit+=GyroAccPit; 
	GyroAccBaseYaw+=GyroAccYaw;
	if(GyroBaseCnt==1)   
        {
          GyroBaseRol=GyroBaseRol/GYROBASECNT;
          GyroBasePit=GyroBasePit/GYROBASECNT;
          GyroBaseYaw=GyroBaseYaw/GYROBASECNT;
          
          GyroAccBaseRol=GyroAccBaseRol/GYROBASECNT;
          GyroAccBasePit=GyroAccBasePit/GYROBASECNT;
          GyroAccBaseYaw=GyroAccBaseYaw/GYROBASECNT;
          Led_Toggle(100);;   //Shine LED show gyro cali 闪烁LED表示在进行陀螺仪校准
        }
    
                
       
    }
    //加入测定运行250周期后，reading-intergral本身误差的函数，将其设置为中立点
    for(GyroBaseCnt=0;GyroBaseCnt<GYROBASECNT;GyroBaseCnt++)
    {
      MotorControl_kalman();
      ReadingNetNick+=Reading_IntegralGyroNick;
      ReadingNetRoll+=Reading_IntegralGyroRoll;
      if(GyroBaseCnt==GYROBASECNT)     //直接叠加200次取平均值
      {
        ReadingNetNick/=GYROBASECNT;   //在这里叠加取平均值即可
        ReadingNetRoll/=GYROBASECNT;   //后面到kalman当中剪去这个基础值
        
        
      }
    }
      
       
    GyroRolI=GyroPitI=GyroYawI=0;     //Reset I value 清空积分值  
    Led_On();                         //建立完毕
    DelayMs(800);                     //建立完毕,chang 亮
    Led_Off();                        //熄灭led。为后面的使用解锁亮灯做准备

}

/*******************************Gyro_Acc_Read**************************************/
//  根据之前的设置翻转陀螺仪和加速计的数据
//  为后面的读取做准备
//  各个数据的读取，方向设置都在本函数中完成
//  作为基础的数据来源函数。需要里面各个全局变量的配合
/*******************************Gyro_Acc_Read**************************************/

void Gyro_Acc_Read(void)      //输出带有正确符号的数据，且具有是否放平的判断
{
   //读取两次取平均
   Gyro_Acc_Filter_1(2);      //在使用这个之前必须有base—tst，建立中立点
  
  //Remove base part from gyro value
  //减去基础值
   GyroRol-=GyroBaseRol;
   GyroPit-=GyroBasePit;
   GyroYaw-=GyroBaseYaw; 
   //减去中立点。消除初始误差
   GyroAccRol-=GyroAccBaseRol; //加速度计的数值
   GyroAccPit-=GyroAccBasePit;
   GyroAccYaw-=GyroAccBaseYaw; 
   //Reverse gyro signals if necessary
    //根据设置反转各个陀螺仪信号
   if(Rol_fu)	  GyroRol=-GyroRol; 
   if(Pit_fu)	  GyroPit=-GyroPit; 
   if(Yaw_fu)	  GyroYaw=-GyroYaw; 
   //Reverse ACC signals if necessary
   //根据设置反转各个加速度计信号
   if(Rol_ACC_fu) GyroAccRol=-GyroAccRol; 
   if(Pit_Acc_fu) GyroAccPit=-GyroAccPit; 
   if(Yaw_Acc_fu) GyroAccYaw=-GyroAccYaw;  

}

/*******************************以下为卡尔曼滤波计算，姿态融合**************************************/
/*******************************以下为卡尔曼滤波计算，姿态融合**************************************/


/*******************************Mean_kalman**************************************/
// 为后面的kalman计算做最初的赋值
// 数值来自于之前小K的滤波后的采样。
// 缺乏一些基础变量的仔细研究与后面函数的配合
// int类型会不会溢出，以及运算速度跟不跟的上是最大的问题
// mag数据加入，各个数据在此进行初步的处理，方向，匹配。
/*******************************Mean_kalman**************************************/

void Mean_kalman(void)
{
    
    Gyro_Acc_Read(); //陀螺和加速度计数据
    MAG_CAL();       //MAG
    
    Reading_GyroYaw   = GyroYaw; //由于之前已经采集了
    Reading_GyroRoll  = GyroRol; //所以这里直接使用赋值就可以了
    Reading_GyroNick  = GyroPit; 
    
    Mean_AccNick  = ACC_AMPLIFY*GyroAccPit; //将加速度计数值匹配到GYRO
    Mean_AccRoll  = ACC_AMPLIFY*GyroAccRol; 
    Mean_AccTop   = ACC_AMPLIFY*GyroAccYaw;
    
    
    GyroYawI+= GyroYaw;  //yaw轴积分
    if(GyroYawI>4000){GyroYawI=4000;}
    if(GyroYawI<-4000){GyroYawI=-4000;} //限幅
    
    
    
    IntegralAccNick += ACC_AMPLIFY * GyroAccPit;//积分
    IntegralAccRoll += ACC_AMPLIFY * GyroAccRol;//后面的各个数据均在此第一次更新
    if(IntegralAccNick>4000){IntegralAccNick=4000;} //限幅
    if(IntegralAccNick<-4000){IntegralAccNick=-4000;}
    if(IntegralAccRoll>4000){IntegralAccRoll=4000;}
    if(IntegralAccRoll<-4000){IntegralAccRoll=-4000;}
    
    
    Reading_IntegralGyroRoll2 += Reading_GyroRoll;
    //带有2的是原始数据，没有2的是有长期融合修正的。
    //用长期融合得到的AttitudeCorrectionRoll来对积分数据进行修正。从而使得姿态稳定
    Reading_IntegralGyroRoll +=  Reading_GyroRoll - AttitudeCorrectionRoll;
    // 长期融合的关键更正
    
    if(Reading_IntegralGyroRoll > TurnOver180Roll)//限幅
    {
	Reading_IntegralGyroRoll  = TurnOver180Roll;
	Reading_IntegralGyroRoll2 = Reading_IntegralGyroRoll;
     }
     if(Reading_IntegralGyroRoll < -TurnOver180Roll)
     {
	Reading_IntegralGyroRoll = -TurnOver180Roll;
	Reading_IntegralGyroRoll2 = Reading_IntegralGyroRoll;
     }
     // 长期融合的关键更正
     // 上面是roll，下面是pit轴
     Reading_IntegralGyroNick2 += Reading_GyroNick;
     Reading_IntegralGyroNick  += Reading_GyroNick - AttitudeCorrectionNick;//长期融合更正
    
    if(Reading_IntegralGyroNick > TurnOver180Nick)
    {
        Reading_IntegralGyroNick = TurnOver180Nick; //限幅
	Reading_IntegralGyroNick2 = Reading_IntegralGyroNick;
     }
     if(Reading_IntegralGyroNick < -TurnOver180Nick)
     {
	Reading_IntegralGyroNick = -TurnOver180Nick;
	Reading_IntegralGyroNick2 = Reading_IntegralGyroNick;
     }
    //牵涉后面一些变量的反复调用
    IntegralNick  = Reading_IntegralGyroNick;
    IntegralRoll   = Reading_IntegralGyroRoll;
    IntegralNick2 = Reading_IntegralGyroNick2;
    IntegralRoll2  = Reading_IntegralGyroRoll2;
    //各个基础变量，姿态变量都在这里赋值   
    balance_number++;
    //一共进行了多少次数据采样/计算了？达到256次的时候进行长期融合/修正
}


/*******************************MotorControl_kalman**************************************/
// 姿态数据计算的核心函数
// 通过全局变量的方式，更改，修正之前得到的各个数据
// 采样--短期融合修正--长期融合修正--修正中立点
// 以上三种方法共同得到稳定的陀螺仪积分，也就是姿态数据。对于后面的PID调节十分重要
// 最后得到的数据输入到PID混合器中，对电机转速进行调节
/*******************************MotorControl_kalman**************************************/

void MotorControl_kalman(void)
{
  static int IntegralErrorNick = 0;   //多次采样之后的数据。static类型
  static int IntegralErrorRoll = 0;   //为节约存储，都设置成了局部变量
  static int CorrectionNick, CorrectionRoll; //长期融合更正数据，在mean中调用
  int tmp_long, tmp_long2;            //短期数据融合用
  
  Mean_kalman();                      //先采样数据，进行初步梳理

  MeanIntegralNick  += IntegralNick;  //mean=integral=reading—intergral
  MeanIntegralRoll  += IntegralRoll;  //intergral前面有赋值，后面貌似还有往返调用
  
  // 短期融合，用加速度计来补偿陀螺仪的数据
  //陀螺积分需要除以ParamSet.GyroAccFactor，而ACC的mean是已经乘以了ACC_AMPIFY的。
  //一乘一除，才能恰好吻合。 吻合后，才能用ACC来补偿gyro
 
  tmp_long   =  (int)(IntegralNick / GyroAccFactor - (int)Mean_AccNick);
  tmp_long  /= 16;                    // 调节量做衰减，避免调节过大
  //上面是pit，下面是roll。yaw轴暂时没有这样处理
  tmp_long2  = (int)(IntegralRoll   / GyroAccFactor - (int)Mean_AccRoll);
  tmp_long2 /= 16;
  
  /**************输出到上位机*************/
  Debug[8]=tmp_long; 
  Debug[9]=tmp_long2;
  /***************输出到上位机************/  
  if(tmp_long >  BALANCE)  tmp_long  = BALANCE; //限幅
  if(tmp_long < -BALANCE)  tmp_long  =-BALANCE;
  if(tmp_long2 > BALANCE)  tmp_long2 = BALANCE;
  if(tmp_long2 <-BALANCE)  tmp_long2 =-BALANCE;
  
  Reading_IntegralGyroNick -= tmp_long;        // 将误差修正到陀螺仪中。
  Reading_IntegralGyroRoll -= tmp_long2;       // 短期融合。
  
  /**************输出到上位机**************/
  Debug[13]=Reading_IntegralGyroNick; 
  Debug[14]=Reading_IntegralGyroRoll;
  /***************输出到上位机*************/  
  
  //短期融合是本周期内就可以作用与PID的。而长期的两种是下一个运算周期才生效
  //短期融合完成，最后这个数据作为正确的积分值，加入校K的PID算法中最终输出
  //长期融合每采样计算256次之后进行一次，数据下次计算的时候再使用
  //短期和长期共同作用，使得最终的姿态数据得以稳定
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 以下为长期融合
 // MeasurementCounter is incremented in the isr of analog.c--now every mean
  if(balance_number >= BALANCE_NUMBER) // averaging number has reached256次之后才进入
  {
   
     MeanIntegralNick /= BALANCE_NUMBER;  //之前有个mean=intergral在积分，增加。
     MeanIntegralRoll  /= BALANCE_NUMBER; 
     // MeanIntegralNick积分再叠加了一次。等于积分的积分
     //短期融合是gyro积分对应ACC瞬时值。而这里是积分的积分对应积分
     IntegralAccNick = (GyroAccFactor * IntegralAccNick) / BALANCE_NUMBER;
     IntegralAccRoll  =(GyroAccFactor * IntegralAccRoll ) / BALANCE_NUMBER;
     //	分两个轴进行计算
     // Nick ++++++++++++++++++++++++++++++++++++++++++++++++
     // Calculate deviation of the averaged gyro integral and the averaged acceleration integral
     IntegralErrorNick = (int)(MeanIntegralNick - (int)IntegralAccNick);
     //一个是陀螺的，一个是加速度计的。256周期后作差，补偿长期误差
    
     /**********************************************************/
     Debug[16]=IntegralErrorNick; //输出到上位机
     /**********************************************************/
     
     CorrectionNick = IntegralErrorNick / GyroAccTrim;
     AttitudeCorrectionNick = CorrectionNick / BALANCE_NUMBER_1;
     //经常是0.XX,所以需要设置为float
     //每次mean采样都需要调用这个数值，所以也不能太大。
     // Roll ++++++++++++++++++++++++++++++++++++++++++++++++
     // Calculate deviation of the averaged gyro integral and the averaged acceleration integral
     IntegralErrorRoll = (int)(MeanIntegralRoll - (int)IntegralAccRoll);
     
     /**********************************************************/
     Debug[15]=IntegralErrorRoll; 
     /**********************************************************/
     
     CorrectionRoll  = IntegralErrorRoll / GyroAccTrim;
     AttitudeCorrectionRoll  = CorrectionRoll  / BALANCE_NUMBER_1;
     
     //长期融合完成。到mean中去起作用，每次mean都调用
     
     AttitudeCorrectionNick *= 1; //可以根据当前传感器以及IIC速度等
     AttitudeCorrectionRoll *= 1; // 进行修正。例如，/2等
     
     //AttitudeCorrectionNick*=0.5; //可以根据当前传感器以及IIC速度等
     //AttitudeCorrectionRoll*=0.5; // 进行修正。例如，/2等
     
/**************************************中立点修正**************************************************/     
// 根据256次采样计算之后，处理后的数据与未处理的数据的总误差
// 算出一次计算中的误差
// 如果偏移太大则进行中立点的修正。
/**************************************中立点修正**************************************************/  
   // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // Gyro-Drift ermitteln 中立点修正
   // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // deviation of gyro nick integral (IntegralNick is corrected by averaged acc sensor)
    IntegralErrorNick  = IntegralNick2 - IntegralNick;
    //实际上是两个积分的比较。原始的与修正的
    //也就是若这次积分相差大，就需要修正中立点。
    Reading_IntegralGyroNick2 -= IntegralErrorNick;
    //reading2又要再减去一次，这样，这两个就只相差一次积分值。下次还可以用来判断
    // deviation of gyro nick integral (IntegralNick is corrected by averaged acc sensor)
    IntegralErrorRoll = IntegralRoll2 - IntegralRoll;
    Reading_IntegralGyroRoll2 -= IntegralErrorRoll;
    
    // 思路
    // 中立点修正还是在前面的大于BALANCE_NUMBER后。
    //所以误差累计了BALANCE_NUMBER次。而IntegralErrorNick>BALANCE_NUMBER*0.5的意思就是表示，
    //平均每次相差0.5.此时开始修正
    
    if(IntegralErrorNick>BALANCE_NUMBER*8)
    // 设置为2000。不太容易达到。太容易修正的话效果反而不好了
    { //最后可以进行正负的判断，然后加减1
       
        GyroBasePit+=1;
    }    
    if(IntegralErrorNick<-BALANCE_NUMBER*8)
    {
 
        GyroBasePit-=1; 
    }   
    if( IntegralErrorRoll<-BALANCE_NUMBER*8)
    {
                             
        GyroBaseRol+=1;      
    } 
  
    if(IntegralErrorRoll>BALANCE_NUMBER*8)
    {
       GyroBaseRol-=1;      
    } 
  //中立点修正完成，输出到上位机
  Debug[11]=IntegralErrorRoll; 
  Debug[12]=IntegralErrorNick;
  //中间值清零。最终使用的reading-integral 不清零，保证积分准确性
  IntegralAccNick = 0;  
  IntegralAccRoll = 0;  
  MeanIntegralNick = 0;
  MeanIntegralRoll = 0;
  balance_number = 0;  //采样次数清零。下一次采样了256次再进来。自我修正功能
  
  
   } //此括号对应长期融合以及中立点修正的结束
         

  GyroRolI= Reading_IntegralGyroRoll;
  GyroPitI= Reading_IntegralGyroNick;
  GyroYawI=GyroYawI;
  //使用比较简化的名称赋值，之后的PID融合器使用
 
  
}



/************************************************************************/
// PID_Mixer for 大小四 PID融合器，最后输出到电机
/************************************************************************/

void PID_Mixer (void) //PID控制器
{
 	int thr_cal,ail_cal,ele_cal,rud_cal,MAG_cal; 
        //最后输出调用的各个量，注意累加
        if(MODE==0) // 普通2.4G模式
        {
	thr_cal=RxThr*2;	
	ail_cal=RxAil/2;                  // 横滚。
	ele_cal=-RxEle/2;                 // 俯仰
        //小M算法中，pit，yaw需要取反
	//与姿态积分等融合。
        }
        if(MODE==1) // 蓝牙模式
        {
	thr_cal=RxTHR_BLUE;// 这里由于是直接串口发，直接发0-125 无需再乘以
	ail_cal=RxAIL_BLUE/2;                  // 横滚。
	ele_cal=-RxELE_BLUE/2;                 // 俯仰
        //小M算法中，pit，yaw需要取反
	//与姿态积分等融合。
        }
        
        // 下面这些量需要限幅否？因为之前的积分等等都有限幅。所以就不需要？
        //  要不还是严谨一点？
        
        /*------------------------需要互补滤波----------------------------------*/
        
        ail_cal+=Kp*GyroRol+Ki*GyroRolI;  //+=的目的是加上积分等
	ele_cal+=Kp*GyroPit+Ki*GyroPitI; 
        
        rud_cal=(int)(-RxRud/4+Kyaw*GyroYaw+KyawI*GyroYawI); 
        //Kyaw在这里单独列出来作为一个系数
        /**************加入地磁数据******************************/
        MAG_cal=(int)(KpMAG*(ang-ang0)+KpMAGI*magI);
        
        //思路：
        // mag是面对南方，由0-360度。也就是说，nag角度内部是有方向的
        // 然后，若ang-ang0》0，说明是顺时针，加大1,3
        // 若《0.则是逆时针，加大2.4
        
        // 以下为互补滤波
        
        ail_cal=(int)(KpNEW*ail_cal+KpOLD*ail_cal_old);  
	ele_cal=(int)(KpNEW*ele_cal+KpOLD*ele_cal_old);  
        rud_cal=(int)(KpNEW*rud_cal+KpOLD*rud_cal_old);  
        MAG_cal=(int)(KpNEW*MAG_cal+KpOLD*MAG_cal_old);
        
        // 保存本次的数值
        
        ail_cal_old=ail_cal;
        ele_cal_old=ele_cal; 
        rud_cal_old=rud_cal;
        MAG_cal_old=MAG_cal; 
        
/*------------------------需要互补滤波----------------------------------*/   
        
        
        if(AxisMode==0)//0--十字，1--X字
	{
	 	// + Mode 十字模式
		//       1  
		//     3 + 2
		//       4	
	 	Motor1=MotorLimitValue(thr_cal - ele_cal + rud_cal);
	 	Motor2=MotorLimitValue(thr_cal - ail_cal - rud_cal);
	 	Motor3=MotorLimitValue(thr_cal + ail_cal - rud_cal);
	 	Motor4=MotorLimitValue(thr_cal + ele_cal + rud_cal);
              
                  
                Debug[0]=Motor1;  
                Debug[1]=Motor2;
                Debug[2]=Motor3;
                Debug[3]=Motor4; //与tmplong等数据可能有互相覆盖
                
	}
	else
	{
	 	// X Mode X模式
		//     1   2
		//       X
		//     3   4
	 	Motor1=MotorLimitValue(thr_cal + ail_cal - ele_cal + rud_cal + MAG_cal);
	 	Motor2=MotorLimitValue(thr_cal - ail_cal - ele_cal - rud_cal - MAG_cal);
	 	Motor3=MotorLimitValue(thr_cal + ail_cal + ele_cal - rud_cal - MAG_cal);
	 	Motor4=MotorLimitValue(thr_cal - ail_cal + ele_cal + rud_cal + MAG_cal);
                
                Debug[0]=Motor1;  
                Debug[1]=Motor2;
                Debug[2]=Motor3;
                Debug[3]=Motor4; //与tmplong等数据可能有互相覆盖
                
                
        }
	//X字运动性能可能较好
}


/*******************PWM_Out_Da**************************************/
// 配合前面的计算使用，最终输出
/*******************PWM_Out_Da**************************************/

void PWM_Out_Da(void) //输出大四轴的5%-10%的波形
{
  PID_Mixer();
  MotorOut_Brushless(1); //大四轴
   
}

/*******************PWM_Out_xiao**************************************/
// 配合前面的计算使用，最终输出
/*******************PWM_Out_xiao**************************************/

void PWM_Out_xiao(void) //输出小四轴的0-100%占空比控制
{
  PID_Mixer();
  
  MotorOut_Brush(1); //500hz硬件pwm波

}




/************************************_loop*****************************************/
// 主飞行控制函数
// 放平--检测mag--解锁--检测水平--水平--采集中立点
// 准备起飞--采集姿态数据--卡尔曼处理--输出到PID控制器
// --控制电机
// 可选输出到大四周和小四周，大四周部分尚未完善
/************************************_loop*****************************************/

void _loop(char mode)  // mode=0 小四 mode=1 大四
{
  Gyro_Acc_Read();     //先读取一下
  PpmReadSignal();     
  //plane_tst()。改为解锁之后
  Led_On();
  DelayMs(200);
  DelayMs(200);
  Led_Off();
  
  while(1)
  {
    //__no_operation();  
    if(MODE==0)         // 选择遥控模式。0--普通2.4G 1-蓝牙模式
    {
      PpmReadSignal();
      ArmingRoutine();   //检测是否上锁了
    }
    if(MODE==1)
    {
      Blue_read();
      ArmingRoutine_BLUE(); 
    }
     //每个循环都可以检测以及上/解锁。
     if(InLock==0)      // 先看是否解锁，后看是否水平。
                        // 如果是第一次解锁，那么采集中立点
     {
        if(plane_ready==1)
     {
       Led_On();       //点亮LED表示已经解锁；
       //Gyro_Acc_Read(); 其实这个在mean中也有，可以不读取
       //Mean_kalman();//MotorControl_kalman里面是有mean的，不要重复
       MotorControl_kalman(); //各项数据滤波,debugout的数据压入堆栈
       if(mode==0)
       {
         PWM_Out_xiao(); //按照小四周--mos输出
       }
       if(mode==1)       //PID系数设置什么在这两个函数中
       {
         PWM_Out_Da();   //按照大四轴输出。
       }
       
       
     }
     
     // 开锁之后，首先关掉moto，然后再次检测中立点，查看是否水平。
     // 闪烁表示开始进行中立点校准。
     if(plane_ready==0)
     {
       MotorOut_Brushless_down(1); //上锁的时候输出较小的转速
       Led_Toggle(200);
       Led_Toggle(200);
       Led_Toggle(200);
       Led_Off();
       Base_Tst(); //解锁后要放置水平，初始值，立点在此时注入。
       
       
     }
      }
     
     if(InLock==1)
     {
        Led_Off();
       MotorOut_Brushless_down(1); //上锁的时候输出较小的转速
       
       plane_ready=0; //清楚水平标志，下次还要放置在水平位置。
       
       
     }                //如果已经上锁的话led熄灭
                      // ArmingRoutine(); //继续检测是否上锁了
                      //放到解锁的外部。解不解锁都输出。
     __no_operation();  
     if(debug_off==0) // 只有油门小鱼40，表示在地面测试的时候才发生debug
     {
        Debug_Out(Debug);//输出各个参数到上位机。
     }
     //__no_operation();  
 /******************************************************/   
    //测试loop的速率。用led闪烁表示，从示波器看
    //测得的周期=2*loopT
     
     Looprate_tst();  
       
       
     }
    
}



           