/*----------------------------

----------------------------*/
extern void          _nop_     (void);
#define ADC_POWER   0x80            //ADC电源控制位
#define ADC_SPEEDLL 0x00            //540个时钟
/////-----初始化ADC-----/////
void InitADC(void)
{
    P1ASF = 0xC7;                   //设置为AD口:P1.0,P1.1,P1.2,P1.6,P1.7.
    ADC_RES = 0;                    //清除结果寄存器
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL;
//  Delay(2);                       //ADC上电并延时
}
/*----------------------------
读取ADC结果
extern void          _pop_     (unsigned char _sfr);
extern void          _nop_     (void);
----------------------------*/
typedef unsigned char BYTE;
#define ADC_FLAG    0x10            //ADC完成标志
#define ADC_START   0x08            //ADC起始控制位
/////-----读取ADC某一个端口-----/////
unsigned int GetADCResult(BYTE ch)
{	unsigned int ADC10;
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL | ch | ADC_START;
    _nop_();                        //等待4个NOP
    _nop_();
    _nop_();
    _nop_();
    while (!(ADC_CONTR & ADC_FLAG));//等待ADC转换完成
    ADC_CONTR &= ~ADC_FLAG;         //Close ADC
	ADC10=ADC_RES*4+ADC_RESL; 
    return ADC10;                   //返回ADC结果
}
