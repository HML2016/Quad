extern void PpmReadSignal(void);
extern void PpmWaitSignal(void);
extern  int  LimitPpmValue(int v);
extern void Led_Init(void);
extern void Looprate_tst(void);
extern void Led_On(void);//led on;
extern void Led_Off(void);//led off
extern void Led_Toggle(unsigned char cnt);//led shanshuo 范围0-510
extern void Led_Toggle_short(void);//led shanshuo 短短的闪一下
extern void Led_Toggle_long(void);//led shanshuo 较长的闪一下
extern void PWM_Pin_Init_soft(void); //软件PWM。
extern void Led_tst(void);  // 不断变化闪烁pinl，测试led
extern void PWM_Pin_Init_hard(void); //硬件PWM 
extern void IIC_Pin_Init(void);
extern void SDA_In(void);// Set P6.1--sda-In
extern void SDA_Out(void);// Set P6.1--sda-out
extern void RC_Pin_Init(void);
extern unsigned char debug_off;//用来表示准备起飞，然后关闭degug
extern unsigned int RxChStart1,RxChStart2,RxChStart3,RxChStart4;
extern int  RxCh1,RxCh2,RxCh3,RxCh4; 
extern unsigned char InLock;
extern void IIC_Pin_DeInit(void);
  
extern        int  RxAil,RxEle,RxRud;  //由于有了形参传递，直接放到里面
       
extern        int  RxThr,RxThrLow;
extern  void ArmingRoutine(void);

extern unsigned char InLock;	//model is in lock 模型处于锁定状态
extern unsigned char ArmCnt;	//Count for arm/disarm

/*************************摇杆系列门限*******************************/

#define	STICKGATE 95  // Stick move gate when setting 设置时的摇杆摆动门限
                      // 实际测试95比较合适
#define RXthrGATE 40  //作为rxthr的门限。