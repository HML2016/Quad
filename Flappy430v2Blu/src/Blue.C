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
/****************************Blue.c****************************************/
// 使用蓝牙模块对Flappy430进行控制。所用模块型号：HC-06
// 控制数据流：&ch1ch2ch3ch4. ch:Rud--Thr--Ele--Ail
// 在FC.H中对使用的模式是遥控/蓝牙进行选择。
// 具备一定的失控保护。可以与有关的蓝牙软件-Flappy430安卓版本进行连接。
// 使用UCA0串口作为与蓝牙通信的串口。蓝牙模块使用HC-06. RXD--TXD--GND--VCC
// P4.6(TXD)--P4.7(RXD)--GND--VCC
// 主要使用PM部分的设置和定义。
// 使用串口中断部分接收数据，有自己的简单通信协议
//  也包含简单的AT指令对蓝牙模块进行设置。
/************************Blue.C*************************************/ 

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
#include "Blue.H"
void UCA0_TST(void);  //测试UCA0的数据发送正确否。
void ArmingRoutine_BLUE(void);          //判断5次以免误判 
void Blue_read (void);
void Port_Mapping_Blue(void);
void blue_hard_init(void);
void UCA0_UART_Init(void);
void Send_Char_SCIA0( char B);
void Send_Char_SCIA0_unsigned(unsigned  char B);
void Send_String_SCIA0(char *String);
void Send_String_SCIA0_unsigned (unsigned char *String);
char Get_Char_SCIA0(void);
unsigned char Get_Char_SCIA0_unsigned(void);
void SCIA0_Print_Float(float shu);
void HC_06_Init(void);

 char i=0; // 用来存储收到的数据计数

 char SCIA_RX_BUF0; // 使用全局变量来存储接收到的数据，与UCA1区别开
// unsigned char还是char?
/*********************************存储一下数据。全局变量。注意最后调用***********************/
int RxRUD_BLUE=0,RxTHR_BLUE=0,RxELE_BLUE=0,RxAIL_BLUE=0; //需要赋给初始值0
char Rxch1_blue,Rxch2_blue,Rxch3_blue,Rxch4_blue;
char Blue_BUFF[10]; // 用来存储接收到的四个通道数据
unsigned char ready;

unsigned char flag_blue_done=0; // 完成接收一个数据包
int error_flag=0;               // 接收过程中的错误计数。错误累积到一定程度则自动降落
int error_xieyi=0;  // 判断接收到底是不是&1234！的队列
int blue_balance=0;                // 每次中断完成加1.然后外面read-1.如果很多次没有
                                 // 大于0则一直没有中断完成。判断没有收到中断信号
                                 // 为了防止太大，需要限幅。
/******************************blue_hard_init************************************/
// 1 初始化PM，设置为串口
// 2 初始化UCA0.允许串口中断。
// 3 等待编写蓝牙模块的AT指令
// 4 全局变量等等待添加
// 5 115200波特率等
// 6 注意检查与UCA1有木有串了的地方
/******************************blue_hard_init***********************************/


void blue_hard_init(void)
{
  Port_Mapping_Blue(); // 初始化硬件PM，设置为串口用
  UCA0_UART_Init();    // UCA0串口的具体设置
}




/******************************Port_Mapping_Blue************************************/
// 使用PM功能使能P4.6，P4.7为UCA0的串口
// 顺便选择第二功能
//
/*******************************Port_Mapping_Blue***********************************/

void Port_Mapping_Blue(void)   // 使用PM功能，重定义P4.6，P4.7为UCA0的相关引脚
{
  __disable_interrupt();       // Disable Interrupts before altering Port Mapping registers
  PMAPPWD = 0x02D52;           // Enable Write-access to modify port mapping registers
  
  // #ifdef PORT_MAP_RECFG     // 直接使能             
  PMAPCTL = PMAPRECFG;         // Allow reconfiguration during runtime
  // #endif  
  
  P4MAP6 = PM_UCA0TXD;         // P4.6---UCA0TXD
  P4MAP7 = PM_UCA0RXD;         // P4.7---UCA0RXD
  
  // 还是与实际的连接符合。
  
  //P4MAP7 = PM_UCA0TXD;         // P4.6---UCA0TXD
  //P4MAP6 = PM_UCA0RXD;         // P4.7---UCA0RXD
 
  
  PMAPPWD = 0;                 // Disable Write-Access to modify port mapping registers
 //  #ifdef PORT_MAP_EINT      // 直接使能中断
  __enable_interrupt();        // Re-enable all interrupts
 //  #endif  
  P4SEL |=BIT6+BIT7;           // P4.6，P4.7选择第二功能--串口
  
}


/************************以下函数为基本与UCA1相同，注意区分***************************************/


/************************UCA0_UART_Init************************************/
// 初始化uart1。8bit，1stop，无校验
// 选择ACLK作为时钟源。需要将ACLK设置为20M.波特率是与20M匹配的115200
// 需要先关闭串口功能再设置寄存器
// 允许RX中断并且设计返回数据功能
/***********************UCA0_UART_Init*************************************/

void UCA0_UART_Init(void)
{


  P4SEL |= BIT6 + BIT7;                     // P4.6,7 = USCI_A0 TXD/RXD
  
  //第一步 引脚初始化
  
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**先关闭再去进行设置
  UCA0CTL1 |= UCSSEL_1;                     // ACLK作为时钟源，调整为20M.以便计算正确的波特率
 

  UCA0CTL0 &=~0XFF;                         //选择UART模式。
  //此寄存器也用于选择数据格式。8bit，1stop，0dd 校验。校验关闭
  
  
  UCA0BR0 = 173;                            // 20MHz 115200 (see User's Guide) 20000000/115200=173
  UCA0BR1 = 0;                        
  UCA0MCTL |= UCBRS_1 | UCBRF_0;           

  //UCA0IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt打开接收中断
  //比较关键的一句。打开中断  暂时没有设置用上位机发送数据回来改设置，暂时去掉
  UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**打开总开关
  // UCA0IE |= UCRXIE;必须是在UCA0CTL1 &= ~UCSWRST后面，不然是无效的
  UCA0IE |= UCRXIE;              // Enable USCI_A1 RX interrupt打开接收中断
  __bis_SR_register( GIE);       // interrupts enabled 打开总中断。
  
}


/*------------------------------------------*/
/*形式参数：char                            */
/*返回值:void				    */
/*函数描述:发送一个字符			    */
/*包括普通版本和unsigned 版本		    */
/*------------------------------------------*/
void Send_Char_SCIA0( char B)
{
    while (!(UCA0IFG&UCTXIFG));  // USCI_A1 TX buffer ready?
    UCA0TXBUF = B;               // TX -> character
    while ( (UCA0STAT&0X01) == 1);  //状态检测，等待忙碌标识为空
}

void Send_Char_SCIA0_unsigned(unsigned  char B)
{
    while (!(UCA0IFG&UCTXIFG));  // USCI_A1 TX buffer ready?
    UCA0TXBUF = B;               // TX -> character
    while ( (UCA0STAT&0X01) == 1);  //状态检测，等待忙碌标识为空
}

/*------------------------------------------*/
/*形式参数： char *String                   */
/*返回值:void				    */
/*函数描述:发送一个字符串		    */
/*包括普通版本和unsigned 版本		    */
/*------------------------------------------*/
void Send_String_SCIA0(char *String)
{
	while(*String !='\0')
  	{
        Send_Char_SCIA0(*String++);
  	}
}


void Send_String_SCIA0_unsigned (unsigned char *String)
{
	while(*String !='\0')
  	{
        Send_Char_SCIA0(*String++);
  	}
}

/*------------------------------------------*/
/*形式参数：void                            */
/*返回值:char				    */
/*函数描述:接收一个字符			    */
/*------------------------------------------*/
char Get_Char_SCIA0(void)
{
     UCA0IE &= ~UCRXIE ;//关闭接收中断
     while ( (UCA0STAT&0X01) == 1); //状态检测，等待忙碌标识为空  //等待接收完毕
     UCA0IE |= UCRXIE;   //开接收中断
     return (UCA0RXBUF); // 返回接收到的数据
}


unsigned char Get_Char_SCIA0_unsigned(void)
{
     UCA0IE &= ~UCRXIE ; //关闭接收中断 发送后面等到忙，接收前面等待
     while ( (UCA0STAT&0X01) == 1); //等待接收完毕
     UCA0IE |= UCRXIE;   //开接收中断
     return (UCA0RXBUF); // 返回接收到的数据
}


/*用串口以十进制字符形式发送浮点型数shu*/
/*用串口以十进制字符形式发送浮点型数shu*/
/*用于串口助手等用*/
/*由于最后的一位‘\0'难以用串口助手发出，谋求改成’e‘？*/

void SCIA0_Print_Float(float shu)
{
	unsigned char sz[32],i=0;  //
	long dd,ff;
    if(shu>=0)
        sz[0]='+';
    else
    {                   //
        sz[0]='-';
        shu=fabs(shu);  //fabs函数
    }
	if(shu<1.0)        //
	{
		sz[1]='0';    //数字和ASCII码之间由于相差了0X30
		sz[2]='.';
		dd=(long)(shu*1000); //
		sz[3]=dd/100+0x30;
		sz[4]=dd%100/10+0x30;//价格加OX30是因为，传送的是ASCII码
		sz[5]=dd%10+0x30;//就是一直扩大再除而已，其实拿一个数，例如0.356
		sz[6]='\0';       //精确到小数点后面第3位。
		//sz[5]='E';       //
	}                    //加到第5位的话，就乘以100000即可
	else				//12.678
	{
		dd=(long)shu;   //整数部分
		ff=dd;
    	for(i=1;;i++)  //不断的除以10，直到变成小数为止。其实13个的数组容量不仅小数有局限，整数部分也是有局限的
	    {
		 if(dd/10==0)
			   break;
			dd=dd/10;
		}
		sz[i+1]='.';
		dd=(long)(shu*1000)%1000;
		sz[i+2]=dd/100+0x30;
		sz[i+3]=dd%100/10+0x30;
		sz[i+4]=dd%10+0x30;
		sz[i+5]='\0';
		//sz[i+5]='E'; //换用‘e'作为标志位，算是自己的协议

		for(;;i--)
		{
			sz[i]=ff%10+0x30;
			if(ff/10==0)
			   break;
			ff=ff/10;    //这里既是将前面的整数部分还原至数组的前面了
		}
	}
	Send_String_SCIA0_unsigned (sz);
}



/***********************************************************************************************/
//  以下是接收中断
//  1 注意标志位清零，下次继续使用
//  2 接收协议~！仔细撰写判断     &-Rxch1-rxch2-rxch3-rxch4-！ 每次检测到&的时候就
//  3 与XINT的接收数据关系。        RXch1-rud RXch2-Thr RXCH3-Ele RXCH4 AIL
//  4 仔细配合，数据传输            软件当中也写成这样，与普通的XINT配合。
//  5 失控的话降落。？连续没有接受到数据？或者判断连续几次&没有出现
/***********************************************************************************************/


/************************USCI_A0_ISR************************************/
// &1234！
// 仔细看协议。注意判断
// 加入全局变量作为读取数据
// 加入cnt作为失控的判断
// 注意到主函数中添加相应的
// 注意到后面需要标志位清零 
// Blue_BUFF[i]=SCIA_RX_BUF0-30; // 转换成数字
// ASCII码表的转换~！非常关键的细节
/***********************USCI_A0_ISR*************************************/


#pragma vector=USCI_A0_VECTOR   //中断向量的名字
__interrupt void USCI_A0_ISR(void)
{
  
  
  
  switch(__even_in_range(UCA0IV,4))
{                                         //中断向量是集成式。要不断的case
   case 0:break;                             // Vector 0 - no interrupt
   case 2:                                   // Vector 2 - RXIFG
          
 	   UCA0IE &= ~UCRXIE ;//关闭接收中断
           
            __disable_interrupt();//关闭总中断。担心跟XINT接收冲突
            
 	   SCIA_RX_BUF0=UCA0RXBUF;  //读取SCI接收到的数据位 转换成数字。手机软件注意
           
           //UCA0TXBUF = SCIA_RX_BUF0;  //测试一下到底进没进中断
           //测试结果应该是进了才对。后面程序的清零肯定也是对的不然不会一串都是成的。
           UCA0IFG&=~UCRXIFG;   //手动清除RX中断标志位
           if(SCIA_RX_BUF0=='&')
           {
              i=0;
              ready=1;         // 准备接受数据，然后再开始接收后面的数据
           }
           if(SCIA_RX_BUF0!='&')               // 只区分两种情况，一种是&，一种不是&
           {
              if(ready==1)
              {
                 
                 Blue_BUFF[i]=SCIA_RX_BUF0; // 这里无需转换成数字 直接用char
 	         i++;  // 这里其实不需要清零。因为有前面的清零标志
                 if(i==4){i=0; ready=0;} // 手动回复之前的第一个数值。等待下一个&出现
              }
           }
           if(MODE==1)    // 输入到上位机处理
           {
           Debug[4]=Blue_BUFF[0]; //   Debug[4]=RxRud;
           Debug[5]=Blue_BUFF[1];
           Debug[6]=Blue_BUFF[2];
           Debug[7]=Blue_BUFF[3];}
           
           UCA0IFG&=~UCRXIFG;   //手动清除RX中断标志位只是清除这里的就可以了
	   UCA0IE |= UCRXIE;   //开接收中断  
        
           __enable_interrupt();        // Re-enable all interrupts
      

//////////////////////////////////////////////////////////////////////////////////

////////
	//以下函数用于接收完数据以后返回数据
	
    
        break;
        case 4:break;                             // Vector 4 - TXIFG
        default: break;
   }
}





/*************************************测试用串口代码*ECHOBACK***************************************/


/************************USCI_A0_ISR************************************/
// RX中断基础模式，返回一个数据
// 手动清除RXFLG.虽然实际上没有必要。读取后自动清零的。反而有不良影响
// UCA0IFG&=~UCRXIFG; 由上面推导出宏定义写法
// 仔细区分两个串口。U0的9600设置也是问题
// 用串口2与USB转串口配合。是对的，gafss等一长串都可以返回的
// 也可以发送啊。也不应该是中断被打断，有很好的设置的
//  比较两个函数的区别，仔细设施
/***********************USCI_A0_ISR*************************************/
// Echo back RXed character, confirm TX buffer is ready first

/*
#pragma vector=USCI_A0_VECTOR   
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
{                                        
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
   while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
   UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character记得这里是换成了UCA1的发送区域
    
   UCA0IFG&=~UCRXIFG;                      //手动清除RX中断标志位只是清除这里的就可以了
                                            //TXIV的需要清除否？
    
   break;
   case 4:break;                             // Vector 4 - TXIFG
   default: break;
 }
}
*/


void UCA0_TST(void)  //测试UCA0的数据发送正确否。
{
  char try1;
  try1=95;
  Send_Char_SCIA0(-try1);
  //Send_String_SCIA0("AT+NAMEFluB");
  DelayMs(500);
  DelayMs(500);
  
}














































/*************************************测试用串口代码*ECHOBACK***************************************/

/************************Blue_read************************************/
// 仔细check
// 通过开头不同的定义，决定在loop中是调用XINT的数据作为数据
// 还是调用RX-BLUE的数据作为数据
// 与PPMread交替使用
// 但是，其实还可能？完全没有数据过来。也就是？一次》4就应该下降了？
// 紧急降落的两种情况： 1 协议本身有错 2 一直没有串口中断。
// if((error_flag>100)||(blue_balance<-100))
/***********************Blue_read*************************************/

void Blue_read (void)  //将数据从BUFF中取出来。准备开始下一步的计算
{ 
   // 其实这样就可以额，中断里面有变化的话会自己更新的
   RxRUD_BLUE=Blue_BUFF[0]; // 要添加等等falg否？应该是不加。不然标志位难以发挥作用
   RxTHR_BLUE=Blue_BUFF[1];
   RxELE_BLUE=Blue_BUFF[2];
   RxAIL_BLUE=Blue_BUFF[3];
   
   //blue_balance--;    // 和中断里面相反，减少，所以如果balance是负数，说明，一直没有中断
                      // 也就是说，失控了，此时采取紧急降落。在后面的if里面。
   //if(blue_balance<-100){blue_balance=-101;} // 记得随时限幅，保持算法健康《-120与后面配合
   
    if(RxRUD_BLUE>125){RxRUD_BLUE=0;} // 对各个轴数据进行限幅处理。
    if(RxRUD_BLUE<-125){RxRUD_BLUE=0;}
    if(RxAIL_BLUE>125){RxAIL_BLUE=0;}
    if(RxAIL_BLUE<-125){RxAIL_BLUE=0;}
    if(RxELE_BLUE>125){RxELE_BLUE=0;}
    if(RxELE_BLUE<-125){RxELE_BLUE=0;}
   // 油门不一样，需要平稳降落
    if(RxTHR_BLUE>125){RxTHR_BLUE=25;}
    if(RxTHR_BLUE<0){RxTHR_BLUE=25;}
   // 此时说明出现了失控的情况，需要紧急降落了。
   // 注意！
   // 紧急降落的两种情况： 1 协议本身有错 2 一直没有串口中断。
    //if((error_flag>=100)||(blue_balance<=-100)) // 需要设置新的数据标志位吗？应该不需要吧？会一直保持的
   // 300hz下，balance有点太快而且难以预测，暂时不用了。注意限幅
   // 但是，不用balance的话，是去信号怎么破？有了真的安卓一直发送再说？这个。
   // if(error_flag>=100) // 需要设置新的数据标志位吗？应该不需要吧？会一直保持的
    //{
     
    //  RxRUD_BLUE=0;
    //  RxTHR_BLUE=25;  // 方向不偏转。油门调低。降落。
     // RxELE_BLUE=0;
     // RxAIL_BLUE=0;
   // }
  
 
   
   // 两种模式，仔细思考，识别，判断。
   // 具体测试与实际飞行的时候，都必须测试两种模式，
   // 以及互相之间的干扰等等，，，
   
   
   if(RxTHR_BLUE>45){debug_off=1;} //大于40 的时候，准备起飞，关闭debug
   if(RxTHR_BLUE<=45){debug_off=0;} 
   
  
}


void ArmingRoutine_BLUE(void)          //判断5次以免误判 
{       
	if((RxRUD_BLUE<-STICKGATE || RxRUD_BLUE>STICKGATE)&&(RxTHR_BLUE<RXthrGATE)) ArmCnt++; //油门+yaw进行判断
	else  ArmCnt=0;
			
	//Hold rud stick for a while, the num should mul main-loop cycle
	if(ArmCnt>5)// 还是奥连续发送5次，行自己		 
	{ 	
		if(InLock)
		{
		 	if((RxRUD_BLUE>STICKGATE)&&(RxTHR_BLUE<RXthrGATE)) 
			{
			 	//如果是油门低+yaw轴向右，解锁
				//如果是油门低+yaw轴向左，加锁
							
							
			 	InLock=0;	     
			}
		}
		if(InLock==0)
		{
                  if((RxRUD_BLUE<-STICKGATE)&&(RxTHR_BLUE<RXthrGATE)){InLock=1;}
		}
	}
}


void HC_06_Init(void) // 这个也很重要。一定要注意看
{
   // 设置波特率115200 自己地址，距离，加密。
   // 8bit数据位，等等。加油好~！
   // 第一步：设置自身波特率为9600.这样才可以通信。
  UCA0BRW = 2083; // 20MHz 9600 (see User's Guide) 20000000/9600=2083
   //UCA0BR1 = 0;    // UCA0BRW的写法对不对？试试。                  
  UCA0MCTL |= UCBRS_1 | UCBRF_0;   
  
  Send_String_SCIA0("AT+BAUD8");  // 设置波特率为115200
  DelayMs(500);                   // 消除错误的波特率的影响
  DelayMs(500);
  DelayMs(500);
  DelayMs(500);
  DelayUs(600);                            // 直接设置，发送的都是ASCII马。应该没有太大问题。
  
  UCA0BR0 = 173; // 20MHz 115200 (see User's Guide) 20000000/115200
  UCA0BR1 = 0;                        
  UCA0MCTL |= UCBRS_1 | UCBRF_0;   
  
  Send_String_SCIA0("AT+NAMEFlutB");
  DelayMs(500);  // 设置自身名字为Flappy430B
  DelayMs(500); 
  DelayMs(500); 
  Send_String_SCIA0("AT+PIN1235");       // 设置配对密码
  DelayMs(500);
  DelayMs(500); 
  DelayUs(600);  //设置，延时，准备接受数据，与上位蓝牙设备连接。


  
  
  
  
} //