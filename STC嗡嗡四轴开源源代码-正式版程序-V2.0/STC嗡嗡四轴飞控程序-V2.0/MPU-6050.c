#include <STC15W4K48S4.h>	//STC15W4K48S4 µ¥Æ¬»úÍ·ÎÄ¼ş
#include <intrins.h>
#include <MPU6050.H>
sbit    SCL=P3^4;	//IICÊ±ÖÓÒı½Å¶¨Òå
sbit    SDA=P3^5;	//IICÊı¾İÒı½Å¶¨Òå

//**************************************
//ÑÓÊ±º¯Êı
//**************************************

void Delay2us()	
{
	unsigned char i;
	i = 2;
	while (--i);
}
//I^CÊ±ĞòÖĞÑÓÊ±ÉèÖÃ£¬¾ßÌå²Î¼û¸÷Ğ¾Æ¬µÄÊı¾İÊÖ²á  6050ÍÆ¼ö×îĞ¡1.3us µ«ÊÇ»á³öÎÊÌâ£¬ÕâÀïÑÓÊ±Êµ¼Ê1.9us×óÓÒ
//**************************************
//I2CÆğÊ¼ĞÅºÅ
//**************************************
void I2C_Start(void)
{
    SDA = 1;                    //À­¸ßÊı¾İÏß
    SCL = 1;                    //À­¸ßÊ±ÖÓÏß
    Delay2us();
	SDA = 0;                    //²úÉúÏÂ½µÑØ
	Delay2us(); 
	SCL = 0;                    //À­µÍÊ±ÖÓÏß
}

//**************************************
//I2CÍ£Ö¹ĞÅºÅ
//**************************************
void I2C_Stop(void)
{
    SDA = 0;                    //À­µÍÊı¾İÏß
    SCL = 1;                    //À­¸ßÊ±ÖÓÏß
    Delay2us();
	SDA = 1;                    //²úÉúÉÏÉıÑØ
	Delay2us();
}

//**************************************
//I2C½ÓÊÕÓ¦´ğĞÅºÅ
//**************************************
bit I2C_RecvACK(void)
{
    SCL = 1;                    //À­¸ßÊ±ÖÓÏß±
    Delay2us();
	CY = SDA;                   //¶ÁÓ¦´ğĞÅºÅ
    SCL = 0;                    //À­µÍÊ±ÖÓÏß
    Delay2us();
	return CY;
}

//**************************************
//ÏòI2C×ÜÏß·¢ËÍÒ»¸ö×Ö½ÚÊı¾İ
//**************************************
void I2C_SendByte(unsigned char dat)
{
    unsigned char i;
    for (i=0; i<8; i++)         //8Î»¼ÆÊıÆ÷
    {
        dat <<= 1;              //ÒÆ³öÊı¾İµÄ×î¸ßÎ»
        SDA = CY;               //ËÍÊı¾İ¿Ú
        SCL = 1;                //À­¸ßÊ±ÖÓÏß
        Delay2us();
		SCL = 0;                //À­µÍÊ±ÖÓÏß
        Delay2us();
    }
    I2C_RecvACK();
}

//**************************************
//´ÓI2C×ÜÏß½ÓÊÕÒ»¸ö×Ö½ÚÊı¾İ
//**************************************
unsigned char I2C_RecvByte(void)
{
    unsigned char i;
    unsigned char dat = 0;
    SDA = 1;                    //Ê¹ÄÜÄÚ²¿ÉÏÀ­,×¼±¸¶ÁÈ¡Êı¾İ,
    for (i=0; i<8; i++)         //8Î»¼ÆÊıÆ÷
    {
        dat <<= 1;
        SCL = 1;                //À­¸ßÊ±ÖÓÏß
        Delay2us();
		dat |= SDA;             //¶ÁÊı¾İ
        SCL = 0;                //À­µÍÊ±ÖÓÏß
		Delay2us();
    }
    return dat;
}

//**************************************
//ÏòI2CÉè±¸Ğ´ÈëÒ»¸ö×Ö½ÚÊı¾İ
//**************************************
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data)
{
    I2C_Start();                  //ÆğÊ¼ĞÅºÅ
    I2C_SendByte(SlaveAddress);   //·¢ËÍÉè±¸µØÖ·+Ğ´ĞÅºÅ
    I2C_SendByte(REG_Address);    //ÄÚ²¿¼Ä´æÆ÷µØÖ·£¬
    I2C_SendByte(REG_data);       //ÄÚ²¿¼Ä´æÆ÷Êı¾İ£¬
    I2C_Stop();                   //·¢ËÍÍ£Ö¹ĞÅºÅ
}
//**************************************
//´ÓI2CÉè±¸¶ÁÈ¡Ò»¸ö×Ö½ÚÊı¾İ
//**************************************
/*
unsigned char Single_ReadI2C(unsigned char REG_Address)
{
	unsigned char REG_data;
	I2C_Start();                   //ÆğÊ¼ĞÅºÅ
	I2C_SendByte(SlaveAddress);    //·¢ËÍÉè±¸µØÖ·+Ğ´ĞÅºÅ
	I2C_SendByte(REG_Address);     //·¢ËÍ´æ´¢µ¥ÔªµØÖ·£¬´Ó0¿ªÊ¼
	I2C_Start();                   //ÆğÊ¼ĞÅºÅ
	I2C_SendByte(SlaveAddress+1);  //·¢ËÍÉè±¸µØÖ·+¶ÁĞÅºÅ
	REG_data=I2C_RecvByte();       //¶Á³ö¼Ä´æÆ÷Êı¾İ
	
	SDA = 1;                    //Ğ´Ó¦´ğĞÅºÅ
	SCL = 1;                    //À­¸ßÊ±ÖÓÏß
	Delay2us();                 //ÑÓÊ±
	SCL = 0;                    //À­µÍÊ±ÖÓÏß
	Delay2us();                 //ÑÓÊ±
	
	I2C_Stop();                    //Í£Ö¹ĞÅºÅ
	return REG_data;
}
*/

//**************************************
//³õÊ¼»¯MPU6050
//**************************************
void Init_MPU6050(void)
{
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//½â³ıĞİÃß×´Ì¬
	Single_WriteI2C(SMPLRT_DIV, 0x07);  //ÍÓÂİÒÇ125hz
	Single_WriteI2C(CONFIG, 0x04);      //21HZÂË²¨ ÑÓÊ±A8.5ms G8.3ms  ´Ë´¦È¡ÖµÓ¦Ïàµ±×¢Òâ£¬ÑÓÊ±ÓëÏµÍ³ÖÜÆÚÏà½üÎªÒË
	Single_WriteI2C(GYRO_CONFIG, 0x08); //ÍÓÂİÒÇ500¶È/S 65.5LSB/g
	Single_WriteI2C(ACCEL_CONFIG, 0x18);//¼ÓËÙ¶È+-16g  8192LSB/g
}

//**************************************
//ºÏ³ÉÊı¾İ
//**************************************
/*
int GetData(unsigned char REG_Address)
{
	char H,L;
	H = Single_ReadI2C(REG_Address);
	L = Single_ReadI2C(REG_Address+1);
//	return (H<<8)+L;   //ºÏ³ÉÊı¾İ
	return (((u16)H << 8) + L);   //ºÏ³ÉÊı¾İ
}
*/
//**************************************
//Ö±½Ó¶ÁÈ¡Êı¾İ
//**************************************
void Read_MPU6050(unsigned char *buf)
{
	unsigned char	i;
	
	I2C_Start();                  //ÆğÊ¼ĞÅºÅ
	I2C_SendByte(SlaveAddress);   //·¢ËÍÉè±¸µØÖ·+Ğ´ĞÅºÅ
	I2C_SendByte(ACCEL_XOUT_H);    //ÄÚ²¿¼Ä´æÆ÷µØÖ·£¬
	I2C_Start();                   //ÆğÊ¼ĞÅºÅ
	I2C_SendByte(SlaveAddress+1);  //·¢ËÍÉè±¸µØÖ·+¶ÁĞÅºÅ
	for(i=0; i<13; i++)
	{
		buf[i] = I2C_RecvByte();    //¶Á³ö¼Ä´æÆ÷Êı¾İ
		SDA = 0;                    //Ğ´Ó¦´ğĞÅºÅ
		SCL = 1;                    //À­¸ßÊ±ÖÓÏß
		Delay2us();
		SCL = 0;                    //À­µÍÊ±ÖÓÏß
		Delay2us();
	}
		buf[i] = I2C_RecvByte();	//×îºóÒ»¸ö×Ö½Ú
		SDA = 1;                    //Ğ´·ÇÓ¦´ğĞÅºÅ
		SCL = 1;                    //À­¸ßÊ±ÖÓÏß
		Delay2us();
		SCL = 0;                    //À­µÍÊ±ÖÓÏß
		Delay2us();
		I2C_Stop();                    //Í£Ö¹ĞÅºÅ
}
