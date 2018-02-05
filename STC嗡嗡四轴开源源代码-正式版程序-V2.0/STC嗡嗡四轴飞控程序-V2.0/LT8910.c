#include <STC15W4K48S4.h>	//STC15W4K48S4 单片机头文件
#include "LT8910.h"			//无线2.4G模块
uchar RegH;
uchar RegL;
/* ====================== LT8910端口定义 ====================== */ 
sbit RST     = P1^2;	//当RST_n 为高时，将开启芯片，寄存器将回复复位值
sbit SPI_SS  = P1^1;	//使能SPI 信号，低电平有效，也可以使芯片进入sleep mode
sbit SPI_CLK = P1^5;	//SPI/I2C 时钟输入脚
sbit MOSI    = P1^3;	//data 输入脚
sbit MISO    = P1^4;	//data 输出脚
sbit PKT     = P3^2;	//发射/接收状态标志位 可通过设置为高或低有效
/* ======================= us级延时函数 ======================= */ 
void Delay_us(uint n)
{
    for(;n>0;n--);
}
/* ========================= 写寄存器 ========================= */ 
void SPI_WriteReg(uchar addr,uchar H,uchar L)
{
    int i;
    SPI_SS = 0;
    for(i = 0; i < 8; ++ i)
 {
    MOSI = addr & 0x80;
    SPI_CLK = 1; 
    SPI_CLK = 0;
    addr = addr << 1; 
 }
    for(i = 0; i < 8; ++i)
 {
    MOSI = H & 0x80;
    SPI_CLK = 1;
    SPI_CLK = 0;
    H = H << 1;
 }
    for(i = 0; i < 8; ++i)
 {  
    MOSI = L & 0x80;
    SPI_CLK = 1;
    SPI_CLK = 0;
    L = L << 1;
 }
    SPI_SS = 1;
}
/* ========================= 读寄存器 ========================= */ 
void SPI_ReadReg(uchar addr)
{
    int i;	
    SPI_SS = 0;
    addr += 0x80;
    for(i = 0; i < 8; ++ i)
 {
    MOSI = addr & 0x80;
    SPI_CLK = 1;
    SPI_CLK = 0;
    addr = addr << 1;
 }
    for(i = 0; i < 8; ++ i)
 {
    SPI_CLK = 1;
    SPI_CLK = 0;
    RegH = RegH << 1;  
    RegH |= MISO;
 }
    for(i = 0; i < 8; ++ i)
 {
    SPI_CLK = 1;
    SPI_CLK = 0;
    RegL = RegL << 1; 
    RegL |= MISO;
 }
    SPI_SS = 1;
}
/* ========================== 初始化 ========================== */ 
void LT8910_Init(void)
{
    RST = 0;	   //复位 拉低
    Delay_us(500); //延时 500微秒
    RST = 1;	   //复位 拉高
    Delay_us(500); //延时 500微秒
    PKT = 1;	   //收发 标志位 拉高
    SPI_WriteReg( 0, 0x6f, 0xef );	//
    SPI_WriteReg( 1, 0x56, 0x81 );	//
    SPI_WriteReg( 2, 0x66, 0x17 );	//
    SPI_WriteReg( 4, 0x9c, 0xc9 );	//
    SPI_WriteReg( 5, 0x66, 0x37 );	//
    SPI_WriteReg( 7, 0x00, 0x00 );	//设定RF 频道，空中频率为：f=2402+ RF_PLL_CH_NO
    SPI_WriteReg( 8, 0x6c, 0x90 );	//
    SPI_WriteReg( 9, 0x48, 0x00 );	//PA 增益控制 -12.2dbm
    SPI_WriteReg(10, 0x7f, 0xfd );	//晶体振荡器
    SPI_WriteReg(11, 0x00, 0x08 );	//RSSI
    SPI_WriteReg(12, 0x00, 0x00 );	//校准VCO
    SPI_WriteReg(13, 0x48, 0xbd );	//
    SPI_WriteReg(22, 0x00, 0xff );	//
    SPI_WriteReg(23, 0x80, 0x05 );	//
    SPI_WriteReg(24, 0x00, 0x67 );	//
    SPI_WriteReg(25, 0x16, 0x59 );	//
    SPI_WriteReg(26, 0x19, 0xe0 );	//
    SPI_WriteReg(27, 0x13, 0x00 );	//
    SPI_WriteReg(28, 0x18, 0x00 );	//
    SPI_WriteReg(32, 0x58, 0x00 );	//LEN(重要参数)
    SPI_WriteReg(33, 0x3f, 0xc7 );	//
    SPI_WriteReg(34, 0x20, 0x00 );	//
    SPI_WriteReg(35, 0x0a, 0x00 );  //重发次数为9次 一共发送10次
    SPI_WriteReg(36, 0x00, 0x01 );	//SYNC_WORD[15:0]
    SPI_WriteReg(37, 0x00, 0x01 );	//SYNC_WORD[31:16]
    SPI_WriteReg(38, 0x00, 0x01 );	//SYNC_WORD[47:32]
    SPI_WriteReg(39, 0x00, 0x01 );	//SYNC_WORD[63:48]
    SPI_WriteReg(40, 0x44, 0x02 );	//FIFO 阈值
    SPI_WriteReg(41, 0xb8, 0x00 );  //CRC=ON;scramble=OFF;AUTO_ACK=ON
    SPI_WriteReg(42, 0xfd, 0xff );  //RSSI 扫描的信道数量
    SPI_WriteReg(43, 0x00, 0x0f );  //开始扫描RSSI
//    SPI_WriteReg(44, 0x10, 0x00 );  //
//    SPI_WriteReg(45, 0x05, 0x52 );  //	
}
/* ======================= 设置接收模式 ======================= */ 
void RX_model(void)
{
    SPI_WriteReg(52, 0x80, 0x80);
    SPI_WriteReg(7, 0x00, 0x80 + 0x20);
    Delay_us(200);	
}

/* ========================= 接收数据 ========================= */  
void js_shuju(uchar *rx_buf, uchar Num)
{
    uchar i=0;
	SPI_ReadReg(50);
	if(RegH==Num&&RegL==0)
	{
	 while(i<Num)
		 {
		    SPI_ReadReg(50);
		    rx_buf[i++]=RegH;
		    rx_buf[i++]=RegL;
		 }
	 }
}