/*------------------------------------------------------------
´Ë²úÆ·ÊÇ»ùÓÚSTCºê¾§¹«Ë¾£¬ÔÚ2015Äê×îĞÂ¿ª·¢³öÒ»¿îSTC15W4K£¨IAP15W4K£©ÏµÁĞµÄ
8Î»µ¥Æ¬»ú¡£´ËÏµÁĞÔÚSTC12ÏµÁĞµÄ»ù´¡ÉÏ Ôö¼ÓÁË¿íµçÑ¹ºÍ¶àÂ·Ó²¼şPWM¹¦ÄÜ£¨15Î»£©¡£
´ËÌõ¼şÂú×ãÁËËÄÖá·ÉĞĞµÄ»ù±¾Éè¼ÆĞèÇó£¬¾­¹ı±¾ÈË¶à·½ÃæÑ§Ï°ÓëÑĞ¾¿µÄ¹ı³ÌÖĞ£¬
ÕÆÎÕÁËÃÔÄãËÄÖáµÄ·ÉĞĞÔ­ÀíºÍÖÆ×÷¾ßÌåÒªÇóÉè¼Æ²ÎÊıµÈ¡£

ÒÔÏÂÊÇ¡°ÎËÎËËÄÖá¡±µÄ³ÌĞò¸üÄÜËµÃ÷£º
µ¥Æ¬»ú²ÉÓÃSTC15W4K48S4-ÔöÇ¿ĞÍµ¥Æ¬»ú£¨IPAĞèĞŞ¸ÄEEPROM´úÂë£©
¹¤×÷µçÑ¹Îª£º5V	¾§ÕñÆµÂÊÎª£º24M	ÎŞÏßÍ¨Ñ¶Ä£¿éÎª£ºLT8910
±¾³ÌĞòÖĞ ²ÉÓÃÁËMCU6050×ö×ËÌ¬Êı¾İ²É¼¯¼ÓPIDËã·¨¸øÓ²¼ş4Â·PWM¿ØÖÆMOS¹ÜÇı¶¯¿ÕĞÄ±­µç»ú
×¢Òâ£º´Ë³ÌĞòÖ»¼æÈİ£º¿ÕĞÄ±­716µç»úÓë45MM×¨ÓÃÕı·´½°£¨×Ô¼ºµÄµç»ú»òĞèĞŞ¸ÄPID²ÎÊı£©
ÔÚºóÆÚµÄ³ÌĞò¸üĞÂÖĞ£¬»áÔö¼ÓÓ²¼ş´úÂë´ïµ½Ô¤ÆÚ·ÉĞĞ¹¦ÄÜ¡£¾´Çë¹Ø×¢¡£

¿ªÔ´³ÌĞò-¹ã´óËÄÖá°®ºÃÕß¶¼¿ÉÒÔ½øĞĞĞŞ¸ÄÓë¼¼Êõ½»Á÷¡£
ÄúÈç¹ûÔÚ´Ë»ù´¡ÉÏÄÜµ÷³ö¸üºÃµÄ·ÉĞĞĞ§¹û£¬¿ÉÓëÉè¼ÆÕßÁªÏµ£¬ÓĞ·áºñÊµÎï½±ÀøÅ¶¡£
Ô­´´Éè¼ÆÕß£ºÁªÏµ·½Ê½QQ£º250278069     STCÃÔÄãËÄÖá¼¼Êõ½»Á÷Èº£º421109612
ÌÔ±¦ÍøÕ¾£ºhttp://shop117773209.taobao.com/shop/view_shop.htm?spm=a1z0e.1.0.0.eqIhKz&mytmenu=mdianpu&utkn=g,meztsobuhe3to1427461542456&user_number_id=669230514&scm=1028.1.1.20001
ÖÆ×÷Íê³ÉÊ±¼ä£º2016Äê8ÔÂ8ÈÕ-V2.0°æ
------------------------------------------------------------*/
#include <STC15W4K48S4.h>	//STC15W4K48S4 ×¨ÓÃÍ·ÎÄ¼ş
#include <intrins.h>		//STCÌØÊâÃüÁî·ûºÅÉùÃ÷
#include <MPU6050.H>		//MPU6050Êı×ÖÍÓÂİÒÇ
#include <STC15W4K-PWM.H>	//µ¥Æ¬»úËùÓĞIO¿Ú³õÊ¼»¯-PWM¿Ú³õÊ¼»¯
#include <LT8910.h>			//ÎŞÏß2.4GÄ£¿é
//#include <STC15W4K-ADC.h>	//STC15W4K-ADC	Ó²¼şADCÄ£Êı×ª»»
#include <IMU.H>			//IMU·ÉĞĞºËĞÄËã·¨
#include <KalmanFilter.h>  //¿¨¶ûÂüÂË²¨Ëã·¨

//==================================================//
//  LEDµÆ Òı½Å¶¨Òå
//==================================================//
sbit	LEDR=P0^1; //LED ºìÉ« R
sbit	LEDG=P1^7; //LED ÂÌÉ« G
sbit	LEDB=P1^6; //LED À¶É« B

//sbit	KARX=P3^0; //ÅÅÕë½Ó¿Ú RXD
//sbit	KATX=P3^1; //ÅÅÕë½Ó¿Ú TXD
//==================================================//
//  ·ÉĞĞ¿ØÖÆ±äÁ¿
//==================================================//
unsigned char JieSuo;	//¶Ï¿ª/Á¬½Ó ½âËø±äÁ¿
unsigned char SSLL;     //Í¨Ñ¶×´Ì¬ ±äÁ¿
unsigned char ShiLian;	//Ê§Áª±äÁ¿
unsigned char SLJS;		//Ê§Áª¼ÆÊı±äÁ¿
unsigned int YouMen;	//ÓÍÃÅ±äÁ¿
unsigned int HangXiang;	//º½Ïò±äÁ¿
unsigned int HengGun;	//ºá¹ö±äÁ¿
unsigned int FuYang;	//¸©Ñö±äÁ¿
unsigned char  FYHG;	//¸©Ñöºá¹ö±äÁ¿
unsigned char Xiao1;	//CRC Ëã³öĞ§Ñé°ü
unsigned char Xiao2;	//CRC ½ÓÊÕĞ§Ñé°ü
/*------------------------------------------------
                  È«¾Öº¯Êı¶¨Òå
------------------------------------------------*/
unsigned char data RxBuf[20]; //ÉèÖÃ½ÓÊÕ³¤¶È£¬×î¸ßÎª32×Ö½Ú
//*****************·É¿Ø¿ØÖÆ²ÎÊı**************************
unsigned int YM=0;//ÓÍÃÅ±ä»¯ËÙ¶È¿ØÖÆ£¬²»ÕâÑù×öµÄ»°¿ìËÙ±ä»¯ÓÍÃÅÊ±ËÄÖá»áÊ§ËÙ·­×ª²¢GG
int speed0=0,speed1=0,speed2=0,speed3=0;   //µç»úËÙ¶È²ÎÊı
int PWM0=0,PWM1=0,PWM2=0,PWM3=0;//¼ÓÔØÖÁPWMÄ£¿éµÄ²ÎÊı
//double g_x=0,g_y=0,g_z=0;       //ÍÓÂİÒÇ½ÃÕı²ÎÊı
char a_x=0,a_y=0,a_z=0;         //½Ç¶È½ÃÕı²ÎÊı
float IMU_gz;
//*****************MPU-6050 ¼Ä´æÆ÷Êı¾İ²ÎÊı**************************
double Gyro_y=0,Gyro_x=0,Gyro_z=0;        //YÖáÍÓÂİÒÇÊı¾İÔİ´æ
int xdata Angle_ax=0,Angle_ay=0,Angle_az=0;	//ÓÉ¼ÓËÙ¶È¼ÆËãµÄ¼ÓËÙ¶È(»¡¶ÈÖÆ)
int idata Angle_gy=0,Angle_gx=0,Angle_gz=0;	//ÓÉ½ÇËÙ¶È¼ÆËãµÄ½ÇËÙÂÊ(½Ç¶ÈÖÆ)
int data  AngleX=0,AngleY=0;				//ËÄÔªÊı½âËã³öµÄÅ·À­½Ç  ,AngleZ=0
unsigned char data	tp[16];
//==================================================//
//   PIDËã·¨±äÁ¿
//==================================================//
float data PID_Output;				//PID×îÖÕÊä³öÁ¿

float xdata Last_Angle_gx=0;	//Íâ»·PIÊä³öÁ¿  ÉÏÒ»´ÎÍÓÂİÒÇÊı¾İ
float xdata Last_Angle_gy=0;

float xdata ERRORX_In=0;			//ÄÚ»·P ÄÚ»·I ÄÚ»·D ÄÚ»·Îó²î»ı·Ö
float xdata ERRORX_Out=0;			//Íâ»·P Íâ»·I       Íâ»·Îó²î»ı·Ö

float xdata ERRORY_In=0;
float xdata ERRORY_Out=0;

float xdata ERRORZ_Out=0;
float xdata Anglezlate=0;			//ZÖáÏà¹Ø

float xdata Last_Ax=0;
float xdata Last_Ay=0;

float xdata Last_gx=0;
float xdata Last_gy=0;

/////////////////////////////////////////////////////////////
float xdata Ax=0,Ay=0;Az=0;			//¼ÓÈëÒ£¿ØÆ÷¿ØÖÆÁ¿ºóµÄ½Ç¶È
int xdata  gx=0,gy=0;				//¼ÓÈëÒ£¿ØÆ÷¿ØÖÆÁ¿ºóµÄ½Ç¶
long  xdata g_x_aver=0;
long  xdata g_y_aver=0;
long  xdata g_z_aver=0;
int   delta_rc_x=0;
int   delta_rc_y=0;
int   delta_rc_z=0;
long idata PID_P;
long idata PID_I;
long idata PID_D;

#define Q15(X) \
   ((X < 0.0) ? (int)(32768*(X) - 0.5) : (int)(32767*(X) + 0.5))
#define KALMAN_Q	Q15(0.20)
#define KALMAN_R	Q15(0.80)
//==================================================//
//   PID ÊÖ¶¯Î¢µ÷²ÎÊıÖµ
//   ĞŞ¸ÄPIDÒÔÏÂ²ÎÊı ¿ÉÒÔ°É·É¿Øµ÷µÄ¸ü¼ÓÆ½ÎÈ
//   ¾ßÌåµ÷ÊÔ·½·¨¿Å²Î¿¼Ñ§Ï°×ÊÁÏ
//==================================================//
#define	Out_XP	12.0f	    //Íâ»·P
#define	Out_XI	Q15(0.12)   //Íâ»·I
#define	Out_XD	5.2f	    //Íâ»·D

#define	In_XP	Q15(0.31) 	//ÄÚ»·P 
#define	In_XI	Q15(0.012)	//ÄÚ»·I
#define	In_XD	4.3f		//ÄÚ»·D 


#define	Out_YP	12.0f	    //Íâ»·P 
#define	Out_YI	Q15(0.12)   //Íâ»·I
#define	Out_YD	5.2f	    //Íâ»·D

#define	In_YP	Q15(0.31) 	//ÄÚ»·P
#define	In_YI	Q15(0.012)	//ÄÚ»·I
#define	In_YD	4.3f		//ÄÚ»·D
				
#define	ZP	5.0f			//ZÖá P
#define ZI  Q15(0.24)		//ZÖá I
#define	ZD	4.0f			//ZÖá D

#define	ERR_MAX 800	 	   	//»ı·ÖÏŞ·ù


//--------------------------------------------------//
//  ¶¨Ê±Æ÷0 ³õÊ¼»¯º¯Êı 
//--------------------------------------------------//
void Timer0Init(void)//12ºÁÃë@24.000MHz
{
	AUXR &= 0x7F;	//¶¨Ê±Æ÷Ê±ÖÓ12TÄ£Ê½
	TMOD &= 0xF0;	//ÉèÖÃ¶¨Ê±Æ÷Ä£Ê½
	TL0 = 0x40;		//ÉèÖÃ¶¨Ê±³õÖµ
	TH0 = 0xA2;		//ÉèÖÃ¶¨Ê±³õÖµ
	TF0 = 0;		//Çå³ıTF0±êÖ¾
	TR0 = 1;		//¶¨Ê±Æ÷0¿ªÊ¼¼ÆÊ±
	ET0 = 1;        //Timer0 Interrupt Enable
}
//--------------------------------------------------//
//  Ê±¼äÑÓÊ± º¯Êı
//--------------------------------------------------//
void Delay(unsigned int x)
{unsigned int i,j;
	for(i=0;i<x;i++)
	for(j=0;j<250;j++);
}
//-------------------------------------------------//
//  PIDËã·¨·É¿Ø×ÔÆ½ºâ º¯Êı	¶¨Ê±Æ÷0ÖĞ¶Ï 12ºÁÃëÖ´ĞĞÒ»´Î
//--------------------------------------------------//
void Flight(void)  interrupt 1 
{

	RX_model();				//½ÓÊÕÄ£Ê½
	js_shuju(RxBuf,15);		//¶ÁÈ¡Êı¾İ°ü


	Read_MPU6050(tp);//Ö±½Ó¶ÁÈ¡MPU6050ÍÓÂİÒÇºÍ¼ÓËÙ¶ÈµÄÊı¾İ°ü

    Angle_ax=KalmanFilter_ax( ((int *)&tp)[0],KALMAN_Q,KALMAN_R);  //ÓÅ»¯Ëã·¨
	Angle_ay=KalmanFilter_ay( ((int *)&tp)[1],KALMAN_Q,KALMAN_R);
	Angle_az=KalmanFilter_az( ((int *)&tp)[2],KALMAN_Q,KALMAN_R);

	Angle_gx = ((float)(((int *)&tp)[4])) / 65.5;	//ÍÓÂİÒÇ´¦Àí	½á¹ûµ¥Î»ÊÇ +-¶È
	Angle_gy = ((float)(((int *)&tp)[5])) / 65.5;	//ÍÓÂİÒÇÁ¿³Ì +-500¶È/S, 1¶È/Ãë ¶ÔÓ¦¶ÁÊı 65.536
	Angle_gz=KalmanFilter_gyroz( ((int *)&tp)[6],Q15(0.2),Q15(0.8));
	IMU_gz=Angle_gz/65.5;
	Last_Angle_gx = Angle_gx;		//´¢´æÉÏÒ»´Î½ÇËÙ¶ÈÊı¾İ
	Last_Angle_gy = Angle_gy;

//*********************************** ËÄÔªÊı½âËã ***********************************
	IMUupdate(Angle_gx*0.0174533f, Angle_gy*0.0174533f, IMU_gz*0.0174533f, Angle_ax,Angle_ay,Angle_az);//×ËÌ¬½âËã£¬¾«¶È0.1¶È
	///·¢ËÍµ½Ò£¿ØÆ÷ 
//	TxBuf[0]=(AngleX+900)/0xff; // ÊıÖµÊÇ 48~1752 = 0-360¶È
//	TxBuf[1]=(AngleX+900)%0xff;	// ÊıÖµÊÇ 48~1752 = 0-360¶È
//	TxBuf[2]=(AngleY+900)/0xff;	// ÊıÖµÊÇ 48~1752 = 0-360¶È
//	TxBuf[3]=(AngleY+900)%0xff;	// ÊıÖµÊÇ 48~1752 = 0-360¶È
//****·É¿ØÊ§ÁªÅĞ¶Ï ×Ô¶¯½µÂäËã·¨***********************************************************************
//½ÓÊÕÒ£¿ØÆ÷·¢À´µÄ²»¶Ï¸üĞÂÊı¾İ ÅĞ¶ÏÁª»úÍ¨Ñ¶ÊÇ·ñÕı³£
	if(SSLL==ShiLian)	//Èç¹ûSSLLµÄÊı¾İÃ»ÓĞ¸üĞÂ¼´Ê§Áª
	{ if(++SLJS>=20)
		{	SLJS = 19;	    //×´Ì¬±êÊ¶
			HangXiang=128;	//º½Ïò±äÁ¿ 
			HengGun  =128;	//ºá¹ö±äÁ¿ 
			FuYang   =128;	//¸©Ñö±äÁ¿
			if(YM>20)YM--;//ÓÍÃÅÔÚÔ­ÖµÖğ½¥¼õĞ¡	
		}
	}
	else {SLJS=0;
	  if(YouMen>1001)YouMen=1000;//ÓÍÃÅÁ¿0-1000×î´óÖµ
	//ÓÍÃÅÓÅ»¯Ëã·¨ ¡¾½«ÓÍÃÅÒ¡¸ËµÄ¿ØÖÆ·ù¶È´Ó60%Ôö¼Óµ½90%¿ØÖÆËã·¨¡¿ÈçÏÂ
	  else
	    {
		 if(YouMen>50)			   //Ò¡¸ËÁ¿ÉÏ50Ö´ĞĞ
		    {YM=(YouMen+300)/1.3;} //Ò¡¸ËÔö·ùËã·¨
		 else  YM=YouMen; 		   //Ò¡¸ËµÍÓÚÖ±½Ó¸³Öµ
		} 
	 }
	ShiLian = SSLL;	//Ê§Áª±äÁ¿¸üĞÂ
	YM=YouMen;
//****ÇãĞ±½Ç¶È¼«ÏŞ¿ØÖÆ***********************************************************************
	//¼«ÏŞ½Ç¶ÈÖµ   30¶È
	     if((AngleX+900)>1200){LEDR=0;YM=0;}	//·É¿ØÏòÓÒÇãĞ±
	else if((AngleX+900)<500) {LEDR=0;YM=0;}	//·É¿ØÏò×óÇãĞ±
	else if((AngleY+900)>1200){LEDR=0;YM=0;}	//·É¿ØÏòÇ°ÇãĞ±
	else if((AngleY+900)<500) {LEDR=0;YM=0;}	//·É¿ØÏòºóÇãĞ±
	else LEDR=1;  //ºìÉ«

//****ÒÔÏÂÊÇ·ÉĞĞ¿ØÖÆËã·¨***********************************************************************
//************** MPU6050 XÖáÖ¸Ïò ***********************************************************
	delta_rc_x=((float)HengGun-128)*2;//µÃµ½ ºá¹öÊı¾İ±äÁ¿
	Ax =-AngleX-delta_rc_x+a_x*5;//
//	Ax =-AngleX+a_x*5;
	if(YM > 20)	ERRORX_Out += Ax;	//Íâ»·»ı·Ö(ÓÍÃÅĞ¡ÓÚÄ³¸öÖµÊ±²»»ı·Ö)
	else		ERRORX_Out = 0;		//ÓÍÃÅĞ¡ÓÚ¶¨ÖµÊ±Çå³ı»ı·ÖÖµ

	if(ERRORX_Out >  ERR_MAX)	ERRORX_Out =  ERR_MAX;	//»ı·ÖÏŞ·ù
	else if(ERRORX_Out < -ERR_MAX)	ERRORX_Out = -ERR_MAX;	//»ı·ÖÏŞ·ù	

	PID_P=(long)Ax*Out_YP;
	PID_I=((long)ERRORX_Out*Out_XI)>>15;
	PID_D=((Angle_gy+Last_Angle_gy)/2)*Out_XD;
	PID_Output = (PID_P+PID_I+PID_D+5)/10;	//Íâ»·PID

	Last_Ax=Ax;
	gx=PID_Output - Angle_gy;	   //Íâ»· -   ÍÓÂİÒÇYÖá

	if(YM > 20)	ERRORX_In += gx;	//ÄÚ»·»ı·Ö(ÓÍÃÅĞ¡ÓÚÄ³¸öÖµÊ±²»»ı·Ö)
	else		ERRORX_In  = 0; //ÓÍÃÅĞ¡ÓÚ¶¨ÖµÊ±Çå³ı»ı·ÖÖµ

	if(ERRORX_In >  ERR_MAX)	ERRORX_In =  ERR_MAX;
	else if(ERRORX_In < -ERR_MAX)	ERRORX_In = -ERR_MAX;	//»ı·ÖÏŞ·ù

	PID_P=((long)gx*In_XP)>>15;
	PID_I=((long)ERRORX_In*In_XI)>>15;
	PID_D=((long)gx - Last_gx)*In_XD;
	PID_Output =PID_P+PID_I+PID_D;	 //ÄÚ»·PID

	Last_gx=gx;

	if(PID_Output >  1000)	PID_Output =  1000;  //Êä³öÁ¿ÏŞ·ù
	if(PID_Output < -1000)	PID_Output = -1000;
	speed0 = 0 + PID_Output;	speed1 = 0 - PID_Output;
	speed3 = 0 + PID_Output;	speed2 = 0 - PID_Output;
//**************MPU6050 YÖáÖ¸Ïò**************************************************
	delta_rc_y=((float)FuYang-128)*2; //µÃµ½ ¸©ÑöÊı¾İ±äÁ¿
	Ay  =-AngleY-delta_rc_y-a_y*5;
//	Ay  =-AngleY-a_y*5;
	if(YM > 20)		ERRORY_Out += Ay;				//Íâ»·»ı·Ö(ÓÍÃÅĞ¡ÓÚÄ³¸öÖµÊ±²»»ı·Ö)
	else			ERRORY_Out = 0;					//ÓÍÃÅĞ¡ÓÚ¶¨ÖµÊ±Çå³ı»ı·ÖÖµ

	if(ERRORY_Out >  ERR_MAX)	ERRORY_Out =  ERR_MAX;
	else if(ERRORY_Out < -ERR_MAX)	ERRORY_Out = -ERR_MAX;	//»ı·ÖÏŞ·ù

	PID_P=(long)Ay*Out_YP;
	PID_I=((long)ERRORY_Out*Out_YI)>>15;
	PID_D=((Angle_gx+Last_Angle_gx)/2)*Out_YD;
	PID_Output = (PID_P +PID_I+PID_D+5)/10;	//Íâ»·PID£¬¡°+5¡±ÎªÁËËÄÉáÎåÈë±

	Last_Ay=Ay;
	gy=PID_Output - Angle_gx;

	if(YM > 20)ERRORY_In +=gy;//ÄÚ»·»ı·Ö(ÓÍÃÅĞ¡ÓÚÄ³¸öÖµÊ±²»»ı·Ö)
	else			ERRORY_In = 0;							//ÓÍÃÅĞ¡ÓÚ¶¨ÖµÊ±Çå³ı»ı·ÖÖµ

	if(ERRORY_In >  ERR_MAX)	ERRORY_In =  ERR_MAX;
	else if(ERRORY_In < -ERR_MAX)	ERRORY_In = -ERR_MAX;	//»ı·ÖÏŞ·ù

	PID_P=((long)gy*In_YP)>>15;
	PID_I=((long)ERRORY_In*In_YI)>>15;
	PID_D=((long)gy - Last_gy)*In_YD;
	PID_Output =PID_P+PID_I+PID_D;

	Last_gy=gy;

	if(PID_Output >  1000)	PID_Output =  1000;  //Êä³öÁ¿ÏŞ·ù
	if(PID_Output < -1000)	PID_Output = -1000;	
	speed0 = speed0 + PID_Output;	speed1 = speed1 + PID_Output;//¼ÓÔØµ½ËÙ¶È²ÎÊı
	speed3 = speed3 - PID_Output;	speed2 = speed2 - PID_Output;

//************** MPU6050 ZÖáÖ¸Ïò *****************************	
	delta_rc_z =-Angle_gz+((float)HangXiang-128)*65+a_y*20;	//µÃµ½ º½ÏòÊı¾İ±äÁ¿ ²Ù×÷Á¿	
	if(YM > 20)		ERRORZ_Out += delta_rc_z;
	else			ERRORZ_Out  = 0; 
	if(ERRORZ_Out >  50000)	ERRORZ_Out =  50000;
	else if(ERRORZ_Out < -50000)	ERRORZ_Out = -50000;	//»ı·ÖÏŞ·ù
	PID_P=((long)delta_rc_z)*ZP;
	PID_I=((long)ERRORZ_Out * ZI)>>15;
	PID_D=((long)delta_rc_z - Anglezlate) * ZD;
	PID_Output =( PID_P + PID_I + PID_D)>>6;

	Anglezlate = delta_rc_z;
	speed0 = speed0 + PID_Output;	speed1 = speed1 - PID_Output;
	speed3 = speed3 - PID_Output;	speed2 = speed2 + PID_Output;

//**************½«ËÙ¶È²ÎÊı¼ÓÔØÖÁPWMÄ£¿é*************************************************	
	//ËÙ¶È²ÎÊı¿ØÖÆ£¬·ÀÖ¹³¬¹ıPWM²ÎÊı·¶Î§0-1000£¨XĞÍÓĞĞ§£©
	PWM0=(YM+speed0);if(PWM0>1000)PWM0=1000;else if(PWM0<0)PWM0=0;
	PWM1=(YM+speed1);if(PWM1>1000)PWM1=1000;else if(PWM1<0)PWM1=0;
	PWM2=(YM+speed2);if(PWM2>1000)PWM2=1000;else if(PWM2<0)PWM2=0;
	PWM3=(YM+speed3);if(PWM3>1000)PWM3=1000;else if(PWM3<0)PWM3=0;
 	
	//Âú×ãÌõ¼ş£º£¨½âËø£º2.4G=5£»ÓÍÃÅ´óÓÚ30£©²ÅÄÜ¿ØÖÆµç»ú
	if(JieSuo==5&&YM>=30)
	  {PWM(1000-PWM1,1000-PWM2,1000-PWM3,1000-PWM0);} //Æô¶¯PWM
	else	  
	  {PWM(1000,1000,1000,1000);}                     //¹Ø±ÕPWM
} 
//--------------------------------------------------//
//  ³ÌĞò Ö÷º¯Êı
//--------------------------------------------------//
void main(void)
{
	PWMGO();		//³õÊ¼»¯PWM
	PWM(1000,1000,1000,1000); //¹Ø±Õµç»ú
	LEDR=0;LEDG=1;LEDB=1;	//3¿Å×´Ì¬µÆ
	Delay(100);			//ÑÓÊ±Ò»»á
	LEDR=1;LEDG=0;LEDB=1;	//3¿Å×´Ì¬µÆ
	Delay(100);			//ÑÓÊ±Ò»»á
	LEDR=1;LEDG=1;LEDB=0;	//3¿Å×´Ì¬µÆ
	Delay(100);			//ÑÓÊ±Ò»»á
	LEDR=1;LEDG=1;LEDB=1;	//3¿Å×´Ì¬µÆ
	Delay(10);    // ÑÓÊ± 100
//---------------------------------------//
//	InitADC();		//ADCÄ£Êı×ª»» ³õÊ¼»¯£¨ºóÆÚ¿ª·¢£©
//	Delay(10);    // ÑÓÊ± 100
	Init_MPU6050();	//³õÊ¼»¯MPU-6050
	Delay(10);    // ÑÓÊ± 100
	LT8910_Init();//ÎŞÏß2.4GÄ£¿é³õÊ¼»¯
	Delay(100);	  //ÑÓÊ±Ò»»á 1S
//---------------------------------------//
	Timer0Init(); //³õÊ¼»¯¶¨Ê±Æ÷
	Delay(100);	  //ÑÓÊ±Ò»»á 1S
//--Ä¬ÈÏÖµ³õÊ¼»¯-------------------------------------//
	YouMen =0;		//³õÊ¼»¯ÓÍÃÅ±äÁ¿ 
	HangXiang=128;	//³õÊ¼»¯º½Ïò±äÁ¿ 
	HengGun =128;	//³õÊ¼»¯ºá¹ö±äÁ¿ 
	FuYang  =128;	//³õÊ¼»¯¸©Ñö±äÁ¿
	LEDB=0;  //¿ªÆôÂÌµÆ
	a_x=0;  // ºá¹öÊÖ¶¯Öµ 
	a_y=0;  // ¸©ÑöÊÖ¶¯Öµ
	a_z=0;  // º½ÏòÊÖ¶¯Öµ

//	Flight();//±àÒëºó2¸ö¾¯¸æÊÇËµ ·É¿Øº¯ÊıÖĞ¶ÏÁ¿ ²»ÔÚÖ÷º¯ÊıÀï¡¾²»ĞèÒª¾À½á¡¿
	EA = 1;  //¿ª×ÜÖĞ¶Ï
while(1)
{
	//Òª·Åµ½¶¨Ê±Æ÷ÖĞ¶ÏÀïÔËĞĞ
//	RX_model();				//½ÓÊÕÄ£Ê½
//	js_shuju(RxBuf,15);		//¶ÁÈ¡Êı¾İ°ü

	Xiao1=RxBuf[0]+RxBuf[1]+RxBuf[2]+RxBuf[3]+RxBuf[4]+RxBuf[5]+RxBuf[6]+RxBuf[7]+RxBuf[8]+RxBuf[9];
	Xiao2=RxBuf[10];

	if(Xiao1==Xiao2)
	   {
		SSLL     =RxBuf[0];	 //½ÓÊÕ Ê§Áª±äÁ¿
		JieSuo   =RxBuf[1];	 //½ÓÊÕ ÃüÁîÖµ
		YouMen	 =RxBuf[2]*0xff+RxBuf[3];  //½ÓÊÕ ÓÍÃÅ±äÁ¿
		HangXiang=RxBuf[4];	 //½ÓÊÕ º½Ïò±äÁ¿ 
		HengGun  =RxBuf[5];	 //½ÓÊÕ ºá¹ö±äÁ¿ 
		FuYang   =RxBuf[6];	 //½ÓÊÕ ¸©Ñö±äÁ¿
		a_x=RxBuf[7]-128;	 //¶Á³ö XÖá±£´æÖµ
		a_y=RxBuf[8]-128;	 //¶Á³ö YÖá±£´æÖµ
		a_z=RxBuf[9]-128;	 //¶Á³ö ZÖá±£´æÖµ
		LEDG=1;	//LED ÂÌ
	   }
	 else 
	    LEDG=0; //LED ÂÌ

	if(JieSuo==1){LEDB=1;}//º½ÏòµÆ À¶É«
	if(JieSuo==5){LEDB=0;}//º½ÏòµÆ À¶É«
	Delay(3);	  //ÑÓÊ±Ò»»á 
/*----»ªÀöµÄ·Ö¸îÏß----»ªÀöµÄ·Ö¸îÏß----»ªÀöµÄ·Ö¸îÏß----»ªÀöµÄ·Ö¸îÏß----*/
	//ADCµçÑ¹µÍÑ¹¼ì²â×ÔÍ£±£»¤¹¦ÄÜ ºóÆÚ¿ª·¢
}}
