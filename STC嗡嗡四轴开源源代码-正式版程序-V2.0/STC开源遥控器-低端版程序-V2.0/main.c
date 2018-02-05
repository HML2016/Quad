/*------------------------------------------------------------
开源程序-广大四轴爱好者都可以进行修改与技术交流。
您如果在此基础上能调出更好的飞行效果，可与设计者联系，有丰厚实物奖励哦。
原创设计者：联系方式QQ：250278069  
STC迷你四轴技术交流群：421109612
淘宝网站：http://shop117773209.taobao.com/shop/view_shop.htm?spm=a1z0e.1.0.0.eqIhKz&mytmenu=mdianpu&utkn=g,meztsobuhe3to1427461542456&user_number_id=669230514&scm=1028.1.1.20001
制作完成时间：2016年8月8日-V2.0版
单片机型号：STC15W408AS-TSSOP20	内部晶振工作频率：24M
单片机晶振：11.0592M（在STC-ISP下载软件中选择）
备注：兼容低短版遥控器（黑色）PCB-V0.41版本的 不兼容V0.3的。
------------------------------------------------------------*/
#include <STC15W408AS.h>	//STC15W408AS 专用头文件
#include "STC15W4K-ADC.h"	//STC15W4K-ADC	硬件ADC模数转换
#include "EEPROM.h"			//STC-EEPROM内部存储
#include "LT8910.h"	    	//2.4G无线收发模块
/***********************************************************************
设置按键 IO口定义
**********************************************************************/
sbit	KAL   =P3^2; //左边按键 L
sbit	KAR   =P5^4; //右边按键 R
sbit	KAS   =P5^5; //独立按键 S
sbit	Feng  =P3^6; //有源蜂鸣器 
sbit	LED   =P3^7; //独立LED 蓝
/*------------------------------------------------
                  全局函数定义
------------------------------------------------*/
unsigned char TxBuf[20]; //设置发送长度，最高为32字节
/*------------------------------------------------
                  延时函数
------------------------------------------------*/
void Delay(unsigned int s)
{
	unsigned int i,j;
	for(i=0; i<s; i++)
		for(j=0; j<250; j++);
}
//--------------------------------------------------//
//  飞控所有设置存储数据变量复位 函数
//--------------------------------------------------//
void init_FKCanShu(void) 
{
	unsigned char data DD[10];
	DD[0]=128;//存储 复位初始值
	DD[1]=128;//存储 复位初始值
	DD[2]=128;//存储 复位初始值
	EEPROM_Xie(DD);//存储 手动校准参数值到EEPROM
	Feng=0;Delay(4000);Feng=1;
}

/*------------------------------------------------
 //所有I/O口全设为准双向，弱上拉模式           
------------------------------------------------*/
void GPIOINT(void)
{
	P0M0=0x00;P0M1=0x00;
	P1M0=0x00;P1M1=0x00;
	P2M0=0x00;P2M1=0x00;
	P3M0=0x00;P3M1=0x00;
	P4M0=0x00;P4M1=0x00;
	P5M0=0x00;P5M1=0x00;
	P6M0=0x00;P6M1=0x00;
	P7M0=0x00;P7M1=0x00;
}
/*------------------------------------------------
                  主函数
------------------------------------------------*/
void main(void)
{	
	unsigned char GGGXYZ[3];//手动校准修改变量
	unsigned int AD1;//ADC模数转换摇杆输出值0~1023
	unsigned char AD2,AD3,AD4,AD5;//ADC模数转换摇杆输出值0~255
	unsigned char HXz,HGz,FYz;//摇杆参数 中间值 0~255
	unsigned char AJin,AJout,AJkkk; //ADC按键值 变量
	char HXcha,HGcha,FYcha;//差数值 互补变量  -128~128
	unsigned int YMout;				//控制参数输出值 0~1000
	unsigned char HXout,HGout,FYout;//控制参数输出值 0~128~255
	unsigned char HangXiang,HengGun,FuYang;//微调变量
	unsigned char SZML;//四轴命令状态控制变量
	unsigned char SSLL;//通讯状态 变量
	unsigned char Xiao;//发送效验包 变量


	GPIOINT();		//单片机IO口初始化
	Delay(10);		//延时一会
	InitADC();		//单片机硬件ADC初始化
	Delay(100);		//延时一会
	LT8910_Init();	//无线2.4G模块初始化
	Delay(100);		//延时一会
	if(KAR==0)init_FKCanShu();//开机按下R键不放 进入微调参数变量初始值
	//------------------------------------------------------------------//
	//读出摇杆中间值 
	AD2=GetADCResult(1)/4; //航向
	AD3=GetADCResult(2)/4; //横滚
	AD4=GetADCResult(6)/4; //俯仰
	HXz=AD2;  HGz=AD3;  FYz=AD4; //读出当前摇杆中间值
	if(AD2>128)HXcha=128-AD2; else if(AD2<128)HXcha=128-AD2; else HXcha=0; //航向 互补值
	if(AD3>128)HGcha=128-AD3; else if(AD3<128)HGcha=128-AD3; else HGcha=0; //横滚 互补值
	if(AD4>128)FYcha=128-AD4; else if(AD4<128)FYcha=128-AD4; else FYcha=0; //俯仰 互补值
	//------------------------------------------------------------------//
	EEPROM_Du(GGGXYZ);//读出EEPROM到 手动校准参数值
	HengGun  =GGGXYZ[0];	//读出 横滚变量
	FuYang   =GGGXYZ[1];	//读出 俯仰变量
	HangXiang=GGGXYZ[2];	//读出 航向变量
	SSLL=0;			//失联变量 初始化
	SZML=0;			//命令变量 初始化
	Delay(10);      //延时 10
	while(1)
	{
	AD1=GetADCResult(0);   //油门  模数转换 0-1023 （10位精度）
	AD2=GetADCResult(1)/4; //航向  模数转换 0-1023 除以4 得到0-255
	AD3=GetADCResult(6)/4; //横滚  模数转换 0-1023 除以4 得到0-255
	AD4=GetADCResult(2)/4; //俯仰  模数转换 0-1023 除以4 得到0-255
	AD5=GetADCResult(7)/4; //按键  模数转换 0-1023 除以4 得到0-255
 
 	//油门参数 摇杆值									 //输出算法
	if(AD1>1000)YMout=1000;else YMout=AD1;				 //允许油门最大值 1000
	//航向参数 摇杆值									 //输出算法
	if(AD2>HXz+5)		HXout=HXz+HXcha+(AD2-(HXz+5))/2; //大于参考值 解算法
	else if(AD2<HXz-5)	HXout=HXz+HXcha-((HXz-5)-AD2)/2; //小于参考值 解算法
	else  HXout=128;									 //摇杆居中输出值
	//横滚参数 摇杆值									 //输出算法
	if(AD3>HGz+5)		HGout=HGz+HGcha+(AD3-(HGz+5))/2; //大于参考值 解算法
	else if(AD3<HGz-5)	HGout=HGz+HGcha-((HGz-5)-AD3)/2; //小于参考值 解算法
	else  HGout=128;									 //摇杆居中输出值
	//俯仰参数 摇杆值									 //输出算法							 
	if(AD4>FYz+5)		FYout=FYz+FYcha+(AD4-(FYz+5))/2; //大于参考值 解算法
	else if(AD4<FYz-5)	FYout=FYz+FYcha-((FYz-5)-AD4)/2; //小于参考值 解算法
	else  FYout=128;									 //摇杆居中输出值

	//ADC按键值 处理算法（只支持单键按下, 不支持多键同时按下）
	if(AD5>2){AJin=AD5;						//判断按键是否被按下 ADC>0
		 if(AJin>35&&AJin<43)   AJout=1;    //按键名 S1
	else if(AJin>70&&AJin<80)   AJout=2;	//按键名 S2
	else if(AJin>108&&AJin<117) AJout=3;	//按键名 S3
	else if(AJin>146&&AJin<156) AJout=4;	//按键名 S4
	else if(AJin>192&&AJin<202) AJout=5;	//按键名 S5
	else if(AJin>250&&AJin<256) AJout=6;	//按键名 S6
	}
	else 					    AJout=0;	//无按键
	if(AJout!=0){ //按键结果 判断处理
	if(AJkkk!=AJout&&AJout==1){HangXiang++;Feng=0;Delay(200);Feng=1;}//微调航向参数 +
	if(AJkkk!=AJout&&AJout==2){HangXiang--;Feng=0;Delay(200);Feng=1;}//微调航向参数	-
	if(AJkkk!=AJout&&AJout==3){FuYang++;   Feng=0;Delay(200);Feng=1;}//微调俯仰参数	+
	if(AJkkk!=AJout&&AJout==4){FuYang--;   Feng=0;Delay(200);Feng=1;}//微调俯仰参数	-
	if(AJkkk!=AJout&&AJout==6){HengGun++;  Feng=0;Delay(200);Feng=1;}//微调横滚参数	+
	if(AJkkk!=AJout&&AJout==5){HengGun--;  Feng=0;Delay(200);Feng=1;}//微调横滚参数	-
	}
	AJkkk=AJout;//按键变化参数
	
	
	//按键L 【上锁停飞】  发送命令值 （最终按键功能由飞控代码决定）
	if(KAL==0){SZML=1;LED=1;Feng=0;Delay(300);Feng=1;}//上锁停飞

	//按键S 【解锁飞行】 油门必须下拉最低点 方可解锁飞行
	if(YMout<5){if(KAS==0){SZML=5;LED=0;Feng=0;Delay(200);
	                 Feng=1;Delay(200);Feng=0;Delay(200);Feng=1;}}//解锁飞行

	if(KAR==0){  	  //按键R 执行将【微调参数保存】至 EEPROM
	GGGXYZ[0]=HengGun; 	//接收 横滚手动值 
	GGGXYZ[1]=FuYang; 	//接收 俯仰手动值
	GGGXYZ[2]=HangXiang;//接收 航向手动值
	EEPROM_Xie(GGGXYZ);		//存储 手动校准参数值到EEPROM
	Feng=0;Delay(500);Feng=1; //蜂鸣器响一声
	}

	Xiao=SSLL+SZML+(YMout/0xff)+(YMout%0xff)+HXout+HGout+FYout+HengGun+FuYang+HangXiang;//计算出效验包

	TxBuf[0]=SSLL++;		//发送 失联变量
	TxBuf[1]=SZML;			//发送 命令值 1=上锁  5=解锁
	TxBuf[2]=YMout/0xff;	//发送 油门参数 高2位 
	TxBuf[3]=YMout%0xff;	//发送 油门参数 低8位
	TxBuf[4]=HXout;			//发送 航向摇杆参数
	TxBuf[5]=HGout;			//发送 横滚摇杆参数
	TxBuf[6]=FYout;			//发送 俯仰摇杆参数
	TxBuf[7]=HengGun;		//发送 横滚微调变量
	TxBuf[8]=FuYang;  		//发送 俯仰微调变量
	TxBuf[9]=HangXiang;		//发送 航向微调变量
	TxBuf[10]=Xiao;			//发送 效验包

	fs_shuju(TxBuf,15);		//执行将数据包发送出去
	Delay(200);				//延时一会
	}
}