extern unsigned char TX_DATA[4];  	 //显示据缓存区
extern unsigned char BUF[10];       //接收数据缓存区
extern char  test; 				 //IIC用到
extern int T_X,T_Y,T_Z,T_T,A_X,A_Y,A_Z; //X,Y,Z轴，温度 acc

extern void I2C_delay(void);
extern void delay5ms(void);
extern void I2C_SendByte(unsigned char SendByte); //数据从高位到低位//
extern unsigned char I2C_WaitAck(void); 	
extern void I2C_NoAck(void);
extern void I2C_Ack(void);
extern void I2C_Stop(void); //对于IIC的整体运行速度有很大的作用
extern unsigned char I2C_Start(void);
extern unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
extern unsigned char I2C_RadeByte(void);  //数据从高位到低位//
extern unsigned char Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
extern void Init_ITG3205(void); //主要牵扯到LSB的问题。这样量程是对的吗？
extern void READ_ITG3205(void);
extern void  Init_ADXL345(void);
extern void read_ADXL345(void);
extern int Gyro_Acc_Filter_1(char flitercnt); //关键的数字滤波函数。没有滤波的数值有时候真觉得无法实用