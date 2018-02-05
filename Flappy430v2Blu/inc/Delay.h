
extern void DelayUs(unsigned int us); //较精确的微妙秒级延时，最多255us，以免溢出
extern void DelayMs(unsigned int ms); //较精确的毫秒级延时，最多255ms
// extern void delayMs(unsigned int ms);

extern void Delay_no_accu(void); //简单函数，循环256个周期
extern void Delay_no_accu_1(unsigned char cnt); //简单函数，循环256个周期
extern void Delay_no_accu_2(unsigned char cnt); //简单函数，循环256个周期
extern void Delay_no_accu_3(void); //简单函数，循环256个周期
