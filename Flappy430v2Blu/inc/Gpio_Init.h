extern void PpmReadSignal(void);
extern void PpmWaitSignal(void);
extern  int  LimitPpmValue(int v);
extern void Led_Init(void);
extern void Looprate_tst(void);
extern void Led_On(void);//led on;
extern void Led_Off(void);//led off
extern void Led_Toggle(unsigned char cnt);//led shanshuo ��Χ0-510
extern void Led_Toggle_short(void);//led shanshuo �̶̵���һ��
extern void Led_Toggle_long(void);//led shanshuo �ϳ�����һ��
extern void PWM_Pin_Init_soft(void); //���PWM��
extern void Led_tst(void);  // ���ϱ仯��˸pinl������led
extern void PWM_Pin_Init_hard(void); //Ӳ��PWM 
extern void IIC_Pin_Init(void);
extern void SDA_In(void);// Set P6.1--sda-In
extern void SDA_Out(void);// Set P6.1--sda-out
extern void RC_Pin_Init(void);
extern unsigned char debug_off;//������ʾ׼����ɣ�Ȼ��ر�degug
extern unsigned int RxChStart1,RxChStart2,RxChStart3,RxChStart4;
extern int  RxCh1,RxCh2,RxCh3,RxCh4; 
extern unsigned char InLock;
extern void IIC_Pin_DeInit(void);
  
extern        int  RxAil,RxEle,RxRud;  //���������βδ��ݣ�ֱ�ӷŵ�����
       
extern        int  RxThr,RxThrLow;
extern  void ArmingRoutine(void);

extern unsigned char InLock;	//model is in lock ģ�ʹ�������״̬
extern unsigned char ArmCnt;	//Count for arm/disarm

/*************************ҡ��ϵ������*******************************/

#define	STICKGATE 95  // Stick move gate when setting ����ʱ��ҡ�˰ڶ�����
                      // ʵ�ʲ���95�ȽϺ���
#define RXthrGATE 40  //��Ϊrxthr�����ޡ�