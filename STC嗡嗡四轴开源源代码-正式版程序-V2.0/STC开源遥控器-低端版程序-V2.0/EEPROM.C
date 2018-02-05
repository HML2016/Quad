#include <STC15F2K60S2.h>	//STC15W4K48S4 专用头文件
#include <EEPROM.h>
#include <intrins.h>
#define CMD_IDLE    0            
#define CMD_READ    1              
#define CMD_PROGRAM 2               
#define CMD_ERASE   3              
#define ENABLE_IAP 0x80           //if SYSCLK<30MHz

void Idle(void)
{
    IAP_CONTR = 0;                 
    IAP_CMD = 0;                    
    IAP_TRIG = 0;                  
    IAP_ADDRH = 0x80;              
    IAP_ADDRL = 0;
}
unsigned char ReadByte(unsigned int addr) //读取1字节
{
    unsigned char dat;                  
    IAP_CONTR = ENABLE_IAP;       
    IAP_CMD = CMD_READ;           
    IAP_ADDRL = addr;             
    IAP_ADDRH = addr >> 8;          
    IAP_TRIG = 0x5a;             
    IAP_TRIG = 0xa5;             
    _nop_();                      
    dat = IAP_DATA;
    Idle();	
    return dat;                   
}

void ProgramByte(unsigned int addr, unsigned char dat) //写入1字节
{
    IAP_CONTR = ENABLE_IAP;        
    IAP_CMD = CMD_PROGRAM;        
    IAP_ADDRL = addr;             
    IAP_ADDRH = addr >> 8;         
    IAP_DATA = dat;                
    IAP_TRIG = 0x5a;              
    IAP_TRIG = 0xa5;                
    _nop_();    
    Idle();	
}

void EraseSector(unsigned int addr) //扇区擦除
{
    IAP_CONTR = ENABLE_IAP;        
    IAP_CMD = CMD_ERASE;          
    IAP_ADDRL = addr;             
    IAP_ADDRH = addr >> 8;         
    IAP_TRIG = 0x5a;              
    IAP_TRIG = 0xa5;               
    _nop_(); 
    Idle();	
}
void EEPROM_Xie(unsigned char *DAROM)   //将数组写入EEPROM
{
unsigned char i;
EraseSector(0); //扇区擦除

for(i=0;i<5;i++)ProgramByte(i,DAROM[i]);;//写入数据至EEPROM
}
void EEPROM_Du(unsigned char *DAROM) //将数组读出EEPROM
{
unsigned char i;
for(i=0;i<5;i++)DAROM[i]=ReadByte(i);//读出数据至EEPROM
}
