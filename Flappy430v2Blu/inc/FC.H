/*********************函数名称******************************************/
/******************控制模式*****************************/
#define MODE 1
// 0--普通2.4G遥控器
// 1--蓝牙+手机
//
/******************控制模式*****************************/
extern void plane_tst(void);
extern int GyroCompe(int gyro,signed char pn);
//extern int Filter(int *value);
extern int GyroIntegral(int v,int delta);
//extern void Gyro-Acc-Filter(char flitercnt); //可以自定义滤波几次
extern void GyroGainRead(unsigned char gainrol,unsigned char gainpit,unsigned char gainyaw);
extern int GainAdj(int gyro,unsigned char gain);

extern void MotorControl_kalman_zhonglidiantst(void);

/*********************kalman滤波计算***********************************/
extern void Mean_kalman(void);
extern void MotorControl_kalman(void);
extern void Base_Tst(void);//检测是否解锁，中立点建立。
extern void Gyro_Acc_Read(void); //数据基础来源
extern void AxisMixer_Loop(void);
/*********************函数名称******************************************/
/**********************************************/
extern int ReadingNetNick; //新增积分中立点，使得积分更加精确
extern int ReadingNetRoll; //正确的积分加正确的PID叙述
/**********************************************/

/*********************全局变量*****************************************/

extern unsigned char  InLock;	//model is in lock 模型处于锁定状态
extern unsigned char  ArmCnt;	//Count for arm/disarm
/*********************全局变量*****************************************/

extern int GyroRol,GyroPit,GyroYaw;//三轴陀螺仪数据，改为由数字传感器输入
extern int GyroBaseRol,GyroBasePit,GyroBaseYaw;//三轴陀螺仪基准。起着自动矫正数值的作用。开始的时候采集200次
//这里必须是int，因为数据LSB扩大到了-250-250.char已经表示不了；了。注意限幅
extern int GyroAccRol,GyroAccPit,GyroAccYaw; //新增加速度计的三轴数据。具体的匹配和方向正负看后面的函数
extern int GyroBaseAccRol,GyroBaseAccPit,GyroBaseAccYaw; //这里也采用多次采集，自动减去初始值的设置

extern void PID_Mixer (void); //输出到大四周。5%-10%占空比

extern void PWM_Out_Da(void); //输出大四周的5%-10%的波形
extern void PWM_Out_xiao(void); //输出大四周的5%-10%的波形
extern void _loop(char mode);  // mode=0 小四 mode=1 大四 主循环 有待完善

extern void AxisMixer(void);
extern int balance_number;
extern int Reading_GyroYaw,Reading_GyroRoll,Reading_GyroNick;
extern int Mean_AccNick,Mean_AccRoll,Mean_AccTop;

extern int Reading_IntegralGyroRoll2,Reading_IntegralGyroRoll,Reading_IntegralGyroNick2,Reading_IntegralGyroNick;
extern int IntegralAccNick, IntegralAccRoll;//这个数据应该比M小很多，而且积分上限也没有超过3000.可以尝试int
//long是4字节的，然后double是8字节。之前的那个函数改为double是不对的。修改为float吧
extern int Reading_IntegralGyroNick, Reading_IntegralGyroNick2;
extern int Reading_IntegralGyroRoll, Reading_IntegralGyroRoll2;//reading与2
extern float  AttitudeCorrectionRoll, AttitudeCorrectionNick;     //长期融合的关键更正
//经常是0.xx。所以设置为float这样readinf在修正以后也是float/拖慢速度？
extern int IntegralNick,IntegralNick2;
extern int IntegralRoll,IntegralRoll2;//积分与2  这个使用int/float

extern float MeanIntegralNick; //后面做融合用，再次怀疑这个int范围不够
extern float MeanIntegralRoll; //这个是积分的积分。所以必须使用float了
 //extern unsigned char Motor1,Motor2,Motor3,Motor4;
extern int labs(int shu);
extern void AxisMixer_Loop_TST(void);
extern int Debug[23];          //用来存储debug数据

/*********************全局变量******************************************/