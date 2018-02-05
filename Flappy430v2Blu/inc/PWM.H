extern unsigned char Motor1,Motor2,Motor3,Motor4; //用于最后输出moto数值
extern void PWM_Pin_Tst(void);//测试P1.1--1.4
extern void PWM_Init_50(void);//初始化50hz的pwm波
extern void PWM_out_50(unsigned char ch, int period,unsigned char mode);
extern void PWM_Init_500(void);//初始化50hz的pwm波
extern void PWM_out_500(unsigned char ch, int period,unsigned char mode); //四个通道。
extern void PWM_out_500_brush(unsigned char ch, int period); //四个通道。
extern void MotorOut_Brush(unsigned char mode); //0是反向输出 1是正常输出
extern void MotorOut_Brushless(unsigned char mode); //dasizhou
extern void MotorOut_Brushless_down(unsigned char mode); //上锁的时候输出较小的转速
extern int MotorLimitValue(int v);