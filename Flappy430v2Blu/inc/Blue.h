/* Flappy430-CopterM by HHY */


/* --COPYRIGHT--
 Flappy430-Copter by HHY
 * Copyright (c) 2014, HHY,Flappy430 Team, CHN
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
 Flappy430-Copter by HHY
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

/************************Blue.C**************************************/ 
// ʹ��UCA0������Ϊ������ͨ�ŵĴ��ڡ�����ģ��ʹ��HC-06. RXD--TXD--GND--VCC
// P4.6(TXD)--P4.7(RXD)--GND--VCC
// ��Ҫʹ��PM���ֵ����úͶ��塣
// ʹ�ô����жϲ��ֽ������ݣ����Լ��ļ�ͨ��Э��
//  Ҳ�����򵥵�ATָ�������ģ��������á�
// 
/************************Blue.C*************************************/ 
extern void ArmingRoutine_BLUE(void);          //�ж�5���������� 
extern void Blue_read (void);
extern void Port_Mapping_Blue(void);
extern void blue_hard_init(void);
extern void UCA0_UART_Init(void);
extern void Send_Char_SCIA0( char B);
extern void Send_Char_SCIA0_unsigned(unsigned  char B);
extern void Send_String_SCIA0(char *String);
extern void Send_String_SCIA0_unsigned (unsigned char *String);
extern char Get_Char_SCIA0(void);
extern unsigned char Get_Char_SCIA0_unsigned(void);
extern void SCIA0_Print_Float(float shu);
extern void HC_06_Init(void);

extern int RxRUD_BLUE,RxTHR_BLUE,RxELE_BLUE,RxAIL_BLUE;
extern char Rxch1_blue,Rxch2_blue,Rxch3_blue,Rxch4_blue;
extern char Blue_BUFF[10]; // �����洢���յ����ĸ�ͨ������

extern  char SCIA_RX_BUF0; // ʹ��ȫ�ֱ������洢���յ������ݣ���UCA1����
// unsigned char����char?

extern unsigned char flag_blue_done; // ��ɽ���һ�����ݰ�
extern int error_flag;               // ���չ����еĴ�������������ۻ���һ���̶����Զ�����

extern void UCA0_TST(void);  //����UCA0�����ݷ�����ȷ��