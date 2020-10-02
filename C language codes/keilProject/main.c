#include "reg51.h"
#include "intrins.h"



sfr     P_SW2       =   0xba;//外设端口切换寄存器2（功能脚切换）
//可以通过配置这个寄存器和P_SW1实现串口分时复用，以及IIC分时复用等，见教材P74

sfr     PWMCFG      =   0xf1; //增强型PWM配置寄存器
sfr     PWMIF       =   0xf6;//增强型PWM中断标志寄存器
sfr     PWMFDCR     =   0xf7;//PWM异常检测控制寄存器
sfr     PWMCR       =   0xfe;//PWM控制寄存器

sfr     AUXR        =   0x8e;
sfr     T2H         =   0xd6;
sfr     T2L         =   0xd7;
sfr     S2CON       =   0x9a;
sfr     S2BUF       =   0x9b;
sfr     IE2         =   0xaf;

#define FOSC            11059200UL
#define BRT             (65536 - FOSC / 9600 / 4)

//注：PWMxTn没有分为H和L
#define PWMC            (*(unsigned int volatile xdata *)0xfff0)
#define PWMCKS          (*(unsigned char volatile xdata *)0xfff2) //PWM时钟选择
#define TADCP           (*(unsigned int volatile xdata *)0xfff3)  //触发ADC计数值
#define PWM0T1          (*(unsigned int volatile xdata *)0xff00)  //PWM0T1计数值
#define PWM0T2          (*(unsigned int volatile xdata *)0xff02)  //PWM0T2计数值
#define PWM0CR          (*(unsigned char volatile xdata *)0xff04) //PWM0控制寄存器
#define PWM0HLD         (*(unsigned char volatile xdata *)0xff05) //PWM0电平保持控制寄存器
#define PWM1T1          (*(unsigned int volatile xdata *)0xff10)
#define PWM1T2          (*(unsigned int volatile xdata *)0xff12)
#define PWM1CR          (*(unsigned char volatile xdata *)0xff14)
#define PWM1HLD         (*(unsigned char volatile xdata *)0xff15)
#define PWM2T1          (*(unsigned int volatile xdata *)0xff20)
#define PWM2T2          (*(unsigned int volatile xdata *)0xff22)
#define PWM2CR          (*(unsigned char volatile xdata *)0xff24)
#define PWM2HLD         (*(unsigned char volatile xdata *)0xff25)
#define PWM3T1          (*(unsigned int volatile xdata *)0xff30)
#define PWM3T2          (*(unsigned int volatile xdata *)0xff32)
#define PWM3CR          (*(unsigned char volatile xdata *)0xff34)
#define PWM3HLD         (*(unsigned char volatile xdata *)0xff35)
#define PWM4T1          (*(unsigned int volatile xdata *)0xff40)
#define PWM4T2          (*(unsigned int volatile xdata *)0xff42)
#define PWM4CR          (*(unsigned char volatile xdata *)0xff44)
#define PWM4HLD         (*(unsigned char volatile xdata *)0xff45)
#define PWM5T1          (*(unsigned int volatile xdata *)0xff50)
#define PWM5T2          (*(unsigned int volatile xdata *)0xff52)
#define PWM5CR          (*(unsigned char volatile xdata *)0xff54)
#define PWM5HLD         (*(unsigned char volatile xdata *)0xff55)
#define PWM6T1          (*(unsigned int volatile xdata *)0xff60)
#define PWM6T2          (*(unsigned int volatile xdata *)0xff62)
#define PWM6CR          (*(unsigned char volatile xdata *)0xff64)
#define PWM6HLD         (*(unsigned char volatile xdata *)0xff65)
#define PWM7T1          (*(unsigned int volatile xdata *)0xff70)
#define PWM7T2          (*(unsigned int volatile xdata *)0xff72)
#define PWM7CR          (*(unsigned char volatile xdata *)0xff74)
#define PWM7HLD         (*(unsigned char volatile xdata *)0xff75)

void Delay10ms()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	_nop_();
	i = 144;
	j = 157;
	do
	{
		while (--j);
	} while (--i);
}

void delay_100ms(void)
{
  int i=1;
  for(;i<=10;i++)
  Delay10ms();
}
void delay_1s(void)
{
  int i=1;
  for(;i<=100;i++)
  Delay10ms();
}





/* 使用PWM步骤
步骤：
0.  P_SW2 |= 0x80;（不知道干啥用的）  
1.  设置PWM时钟（PWMCKS）
2.  设置PWM周期（PWMC）
3.  设置PWM的功能脚以及使能PWM（PWMnCR）
4.  设置PWM的翻转点（如果包含了"STC8.h"，参照头文件里面的变量符号，如果没有包含，参照自己写的宏定义）
5.  P_SW2 &=~0x80;（不知道干啥用的）
6.  启动整个PWM模块，同时决定是否要开启PWM计数器归零中断

修改翻转点的方法：
1.  P_SW2 |= 0x80;
2.  修改PWMnT1和PWMnT2的值
3.  P_SW2 &=~0x80;

即先将EAXFR置为1，再进行翻转点的赋值，赋值完成后再将EAXFR设置为0

翻转点说明：
计数与设置的第一个翻转点相等时，PWM输出波形翻转为低电平
计数与设置的第二个翻转点相同时，PWM输出波形翻转为高电平

ps：
可通过PWMnHLD寄存器控制电平输出（强制为高电平，强制为低电平，正常输出）

*/


//配置四个电机的四个PWM信号，分别为PWM0~3，默认都为高电平
//全速
void fullSpeed(void){
  PWM0T1= 0x0000;
  PWM0T2= 0x0000;
  PWM1T1= 0x0000;
  PWM1T2= 0x0000;
  PWM2T1= 0x0000;
  PWM2T2= 0x0000;  
  PWM3T1= 0x0000;
  PWM3T2= 0x0000;
}




//半速（还没写！！！）
void halfSpeed(void){
  PWM0T1= 0x0500;
  PWM0T2= 0x0F00;
  PWM1T1= 0x0500;
  PWM1T2= 0x0F00;
  PWM2T1= 0x0500;
  PWM2T2= 0x0F00;  
  PWM3T1= 0x0500;
  PWM3T2= 0x0F00;
}



void initPWMs1(void){
  
  P_SW2 |= 0x80;                              
  //0b 1000 0000 EAXFR为1（关于读写片外数据,不知道干啥用的）
  //其他的不变（为了不影响串口2，3，4的功能脚，最好不要直接对P_SW2赋值）  

  PWMCKS = 0x00;                                // PWM时钟为系统时钟
  PWMC = 0x1000;                                //设置PWM周期为1000H个PWM时钟（对应于SG90的周期为20ms）  
  
  //初始PWM信号，四个都一样，小车保持静止，满速
  halfSpeed();
  
  //配置PWM0
  PWM0CR |= 0x80;                               //使能PWM0输出
  PWM0CR &= 0xE7;                               //0b11100111  PWM0输出脚为P2.0 
  
  //配置PWM1
  PWM1CR |= 0x80;
  PWM1CR &= 0xE7;                               //PWM1输出脚为P2.1
  
  //配置PWM2
  PWM2CR |= 0x80;
  PWM2CR &= 0xE7;
  
  //配置PWM3
  PWM3CR |= 0x80;
  PWM3CR &= 0xE7;  
  
  P_SW2 &=~0x80;
  PWMCR = 0x80;                                 
  //0b 1000 0000   ENPWM==1,启动PWM模块（整个PWM模块的控制位）
  //ECBI==0,关闭PWM计数器归零中断
}

//关闭所有PWM输出
//100ms的延时是为了避免突然反转，对轮子造成损害
void forbiddenAll(void){
  PWM0CR &= ~0x80;
  PWM1CR &= ~0x80;
  PWM2CR &= ~0x80;
  PWM3CR &= ~0x80;
  delay_100ms();
}
void headForwards(){
  P_SW2 |=0x80;
  forbiddenAll();
  PWM1CR |= 0x80;
  PWM3CR |= 0x80;
  P_SW2 &=~0x80;
}
void headBackwards(void){
  P_SW2 |=0x80;
  forbiddenAll();
  PWM0CR |= 0x80;
  PWM2CR |= 0x80;
  P_SW2 &=~0x80;
}
void turnLeft(void){
  P_SW2 |=0x80;
  forbiddenAll();
  PWM0CR |= 0x80;
  PWM3CR |= 0x80;
  P_SW2 &=~0x80;
  
}  

void turnRight(void){
  P_SW2 |=0x80;
  forbiddenAll();
  PWM1CR |= 0x80;
  PWM2CR |= 0x80;
  P_SW2 &=~0x80;
}

void stop(void){
  P_SW2 |=0x80;
  forbiddenAll();
  P_SW2 &=~0x80;
}
/*
配置两个SG90舵机的PWM信号
*/


//全局变量
bit busyGY=0;
bit busyBLE;
char dataFromBLE;
char dataFromGY[16];
char wptr=0x00;

/*
配置串口1和串口2
串口1用定时器1，串口2用定时器2
步骤：
1.  选择数据位和波特率是否可变（SCON）
2.  选择定时器时钟，以及串口选择哪个定时器作为波特率发生器（AUXR或者AUXR2）
3.  设置定时器的初值
4.  禁止定时器中断
5.  打开定时器
*/
void uartInit(void)		//9600bps@11.0592MHz
{
  //配置串口1（见P315~P319）
  SCON = 0x50;
  //TCON = 
  TMOD = 0x00;
  TL1 = BRT;
  TH1 = BRT >> 8;
  AUXR =0x00;
  TR1 = 1;
	ET1 = 0;		//禁止定时器1中断
	TR1 = 1;		//启动定时器1
  //AUXR = 0x40;
  AUXR |= 0x40;
  AUXR &= 0xFE;////使用T1作为波特率发生器
  
  //配置串口2（见P323）
  S2CON = 0x10;
  T2L = BRT;
  T2H = BRT >> 8;
  AUXR |= 0x04;   //T2R=1（T2开始计数），T2_C/T=1（T2用作定时器）  ，见P263
  AUXR |= 0x10; 
  IE2 |= 0x01;    //打开串口2的中断
  EA = 1;         //打开总中断
  
}

//串口1的中断函数（蓝牙）
void uart1Interrrupt() interrupt 4{
  if(RI){   //如果是接收一个字节的数据完毕
    RI=0;
    busyBLE=0;
    dataFromBLE=SBUF;
  }
      
}
void uart1SendChar(char sendData){
  while(busyBLE);
  busyBLE=1;
  SBUF=sendData;
  while(TI==0);
  TI=0;
  busyBLE=0;
}


//串口2的中断函数（GY-952）
void uart2Interrrupt() interrupt 8{

  if (S2CON & 0x01)   //如果是接收一个字节的数据完毕
  {
      S2CON &= ~0x01;
      dataFromGY[wptr++] = S2BUF;
      wptr &= 0x0f;   //最多接收16个数据，如果超过了，就重写前面的数据
      busyGY=0;
  }
  
  if (S2CON & 0x02)   //如果是接收一个字节的数据完毕
  {
      S2CON &= ~0x02;
      busyGY=0;
  }
}

//发送a5 75 1a，之后将wptr置为0x00
void uart2SendChar(char sendData){
  while(busyGY);
  busyGY=1;
  S2BUF=sendData;
}




void initGY952(void){
}

void main()
{
  initPWMs1();
  headForwards();
  
}

