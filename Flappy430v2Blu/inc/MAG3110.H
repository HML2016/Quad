
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

#ifndef __MAG3110_H__
#define __MAG3110_H__

#define uchar unsigned char
#define uint unsigned int
#define byte unsigned char
#define word unsigned int

/***********************************************************************************************\
* Public macros
\***********************************************************************************************/  
#define MAG3110_IIC_ADDRESS       0x1C

/***********************************************************************************************
**
**  Variable type definition: BIT_FIELD
*/
typedef union
{
  byte Byte;
  struct {
    byte _0          :1;
    byte _1          :1;
    byte _2          :1;
    byte _3          :1;
    byte _4          :1;
    byte _5          :1;
    byte _6          :1;
    byte _7          :1;
  } Bit;
} BIT_FIELD;


/***********************************************************************************************
**
**  Variable type definition: tword
*/
typedef union
{
  word mword;
  struct
  {
    byte hi;
    byte lo;
  } mbyte;
} tword; 

/***********************************************************************************************
**
**  MAG3110Q Sensor Internal Registers
*/
enum
{
  MAG3110_STATUS_00 = 0,          // 0x00
  MAG3110_OUT_X_MSB,              // 0x01
  MAG3110_OUT_X_LSB,              // 0x02
  MAG3110_OUT_Y_MSB,              // 0x03
  MAG3110_OUT_Y_LSB,              // 0x04
  MAG3110_OUT_Z_MSB,              // 0x05
  MAG3110_OUT_Z_LSB,              // 0x06 
  MAG3110_WHO_AM_I,               // 0x07
  MAG3110_SYSMOD,                 // 0x08
  MAG3110_OFF_X_MSB,              // 0x09
  MAG3110_OFF_X_LSB,              // 0x0A
  MAG3110_OFF_Y_MSB,              // 0x0B
  MAG3110_OFF_Y_LSB,              // 0x0C
  MAG3110_OFF_Z_MSB,              // 0x0D
  MAG3110_OFF_Z_LSB,              // 0x0E 
  MAG3110_DIE_TEMP,               // 0x0f
  MAG3110_CTRL_REG1,              // 0x10
  MAG3110_CTRL_REG2,              // 0x11
 
};

/*
**  STATUS Registers
*/
#define STATUS_00_REG         0x00
//
#define ZYXOW_BIT             Bit._7
#define ZOW_BIT               Bit._6
#define YOR_BIT               Bit._5
#define XOR_BIT               Bit._4
#define ZYXDR_BIT             Bit._3
#define ZDR_BIT               Bit._2
#define YDR_BIT               Bit._1
#define XDR_BIT               Bit._0
//
#define ZYXOW_MASK            0x80
#define ZOW_MASK              0x40
#define YOR_MASK              0x20
#define XOR_MASK              0x10
#define ZYXDR_MASK            0x08
#define ZDR_MASK              0x04
#define YDR_MASK              0x02
#define XDR_MASK              0x01

/*
**  XYZ Data Registers
*/
#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06   

/*
**  WHO_AM_I Device ID Register
*/
#define WHO_AM_I_REG          0x07
#define MAG3110Q_ID           0xC4    

/*
**  SYSMOD System Mode Register
*/
#define SYSMOD_REG            0x08
//   
#define SYSMOD1_BIT           Bit._1
#define SYSMOD0_BIT           Bit._0
// 
#define SYSMOD1_MASK          0x02
#define SYSMOD0_MASK          0x01
#define SYSMOD_MASK           0x03
#define STANDBY_MASK          0x00
#define ACTIVE_MASK           0x01
#define CORRECT_MASK          0x02

/*
**  INT_SOURCE System Interrupt Status Register
*/
#define INT_SOURCE_REG        0x0C
//
#define SRC_ASLP_BIT          Bit._7
#define SRC_FIFO_BIT          Bit._6
#define SRC_TRANS_BIT         Bit._5
#define SRC_LNDPRT_BIT        Bit._4
#define SRC_PULSE_BIT         Bit._3
#define SRC_FF_MT_1_BIT       Bit._2
#define SRC_FF_MT_2_BIT       Bit._1
#define SRC_DRDY_BIT          Bit._0
//
#define SRC_ASLP_MASK         0x80
#define SRC_FIFO_MASK         0x40
#define SRC_TRANS_MASK        0x20
#define SRC_LNDPRT_MASK       0x10
#define SRC_PULSE_MASK        0x08
#define SRC_FF_MT_1_MASK      0x04
#define SRC_FF_MT_2_MASK      0x02
#define SRC_DRDY_MASK         0x01   

/*
**  XYZ Offset Correction Registers
*/
#define OFF_X_MSB             0x09
#define OFF_X_LSB             0x0A
#define OFF_Y_MSB             0x0B 
#define OFF_Y_LSB             0x0C
#define OFF_Z_MSB             0x0D
#define OFF_Z_LSB             0x0E 

#define DIE_TEMP              0x0F 


/*
**  CTRL_REG1 System Control 1 Register
*/
#define CTRL_REG1             0x10
//
#define DR2_BIT               Bit._7
#define DR1_BIT               Bit._6
#define DR0_BIT               Bit._5
#define OS1_BIT               Bit._4
#define OS0_BIT               Bit._3
#define FR_BIT                Bit._2
#define TM_BIT                Bit._1
#define AC_BIT                Bit._0
//
#define DR2_MASK              0x80
#define DR1_MASK              0x40
#define DR0_MASK              0x20
#define OS1_MASK              0x10
#define OS0_MASK              0x08
#define FR_MASK               0x04
#define TM_MASK               0x02
#define AC_MASK               0x01

#define ASLP_RATE_MASK        0xC0
#define DR_MASK               0x38
//                      
#define ASLP_RATE_20MS        0x00
#define ASLP_RATE_80MS        ASLP_RATE0_MASK
#define ASLP_RATE_160MS       ASLP_RATE1_MASK
#define ASLP_RATE_640MS       ASLP_RATE1_MASK+ASLP_RATE0_MASK
//
#define DATA_RATE_1250US      0x00
#define DATA_RATE_2500US      DR0_MASK
#define DATA_RATE_5MS         DR1_MASK
#define DATA_RATE_10MS        DR1_MASK+DR0_MASK
#define DATA_RATE_20MS        DR2_MASK
#define DATA_RATE_80MS        DR2_MASK+DR0_MASK
#define DATA_RATE_160MS       DR2_MASK+DR1_MASK
#define DATA_RATE_640MS       DR2_MASK+DR1_MASK+DR0_MASK

/*
**  CTRL_REG2 System Control 2 Register
*/
#define CTRL_REG2             0x11
//
#define AUTO_MRST_EN_BIT      Bit._7
#define RAW_BIT               Bit._5
#define MAG_RST_BIT           Bit._4  
//
#define AUTO_MRST_EN_MASK     0x80 
#define RAW_MASK              0x20
#define MAG_RST_MASK          0x10
 

/***********************************************************************************************\
* Public prototypes
\***********************************************************************************************/

extern void MAG3110_Init(void);
extern void MAG3110_Standby(void);
extern void MAG3110_Active(void);	 

extern void MAG_TST(void);
extern unsigned char IIC_start_MAG(void); //送起始位
extern void IIC_stop_MAG(void); //送停止位
extern void IIC_ack_main_MAG(unsigned char ack_main);//主应答 说明：ack_main=1:ack主应答；ack_main=0:no_ack无需应答
extern void send_byte_MAG(uchar c); //发送单个字节
extern uchar read_byte_MAG(void); //接收单个字节 说明：此函数需要配合主应答来用
 
extern void send_to_byte_MAG(uchar ad_main,uchar c);  
extern void send_to_nbyte_MAG(uchar ad_main,uchar ad_sub,uchar *buf,uchar num);	 
extern void read_from_byte_MAG(uchar ad_main,uchar *buf);
extern void read_from_nbyte_MAG(uchar ad_main,uchar ad_sub,uchar *buf,uchar num);	  	
extern unsigned char LQMAG_readbyte_MAG(unsigned char address);
extern void LQMAG_writebyte_MAG(unsigned char address, unsigned char thedata);


extern int MAG3110_DataProcess (int MAG3110_XData,int MAG3110_YData);

extern void MAG3110_STD(void);
extern void mag3110_tst(void);
extern void MAG_INIT(void);
extern int MAG_CAL(void) ;

extern int ang;
extern int ang0; //第一次初始化的时候的角度，电机还没有转动，好滴 必须是全局变量
extern int magI; // 磁偏角的积分。也必须是全局变量。
/*************************计算中用到的各个变量*****************************/ 



#endif

