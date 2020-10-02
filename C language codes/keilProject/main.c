#include "reg51.h"
#include "intrins.h"



sfr     P_SW2       =   0xba;//����˿��л��Ĵ���2�����ܽ��л���
//����ͨ����������Ĵ�����P_SW1ʵ�ִ��ڷ�ʱ���ã��Լ�IIC��ʱ���õȣ����̲�P74

sfr     PWMCFG      =   0xf1; //��ǿ��PWM���üĴ���
sfr     PWMIF       =   0xf6;//��ǿ��PWM�жϱ�־�Ĵ���
sfr     PWMFDCR     =   0xf7;//PWM�쳣�����ƼĴ���
sfr     PWMCR       =   0xfe;//PWM���ƼĴ���

sfr     AUXR        =   0x8e;
sfr     T2H         =   0xd6;
sfr     T2L         =   0xd7;
sfr     S2CON       =   0x9a;
sfr     S2BUF       =   0x9b;
sfr     IE2         =   0xaf;

#define FOSC            11059200UL
#define BRT             (65536 - FOSC / 9600 / 4)

//ע��PWMxTnû�з�ΪH��L
#define PWMC            (*(unsigned int volatile xdata *)0xfff0)
#define PWMCKS          (*(unsigned char volatile xdata *)0xfff2) //PWMʱ��ѡ��
#define TADCP           (*(unsigned int volatile xdata *)0xfff3)  //����ADC����ֵ
#define PWM0T1          (*(unsigned int volatile xdata *)0xff00)  //PWM0T1����ֵ
#define PWM0T2          (*(unsigned int volatile xdata *)0xff02)  //PWM0T2����ֵ
#define PWM0CR          (*(unsigned char volatile xdata *)0xff04) //PWM0���ƼĴ���
#define PWM0HLD         (*(unsigned char volatile xdata *)0xff05) //PWM0��ƽ���ֿ��ƼĴ���
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





/* ʹ��PWM����
���裺
0.  P_SW2 |= 0x80;����֪����ɶ�õģ�  
1.  ����PWMʱ�ӣ�PWMCKS��
2.  ����PWM���ڣ�PWMC��
3.  ����PWM�Ĺ��ܽ��Լ�ʹ��PWM��PWMnCR��
4.  ����PWM�ķ�ת�㣨���������"STC8.h"������ͷ�ļ�����ı������ţ����û�а����������Լ�д�ĺ궨�壩
5.  P_SW2 &=~0x80;����֪����ɶ�õģ�
6.  ��������PWMģ�飬ͬʱ�����Ƿ�Ҫ����PWM�����������ж�

�޸ķ�ת��ķ�����
1.  P_SW2 |= 0x80;
2.  �޸�PWMnT1��PWMnT2��ֵ
3.  P_SW2 &=~0x80;

���Ƚ�EAXFR��Ϊ1���ٽ��з�ת��ĸ�ֵ����ֵ��ɺ��ٽ�EAXFR����Ϊ0

��ת��˵����
���������õĵ�һ����ת�����ʱ��PWM������η�תΪ�͵�ƽ
���������õĵڶ�����ת����ͬʱ��PWM������η�תΪ�ߵ�ƽ

ps��
��ͨ��PWMnHLD�Ĵ������Ƶ�ƽ�����ǿ��Ϊ�ߵ�ƽ��ǿ��Ϊ�͵�ƽ�����������

*/


//�����ĸ�������ĸ�PWM�źţ��ֱ�ΪPWM0~3��Ĭ�϶�Ϊ�ߵ�ƽ
//ȫ��
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




//���٣���ûд��������
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
  //0b 1000 0000 EAXFRΪ1�����ڶ�дƬ������,��֪����ɶ�õģ�
  //�����Ĳ��䣨Ϊ�˲�Ӱ�촮��2��3��4�Ĺ��ܽţ���ò�Ҫֱ�Ӷ�P_SW2��ֵ��  

  PWMCKS = 0x00;                                // PWMʱ��Ϊϵͳʱ��
  PWMC = 0x1000;                                //����PWM����Ϊ1000H��PWMʱ�ӣ���Ӧ��SG90������Ϊ20ms��  
  
  //��ʼPWM�źţ��ĸ���һ����С�����־�ֹ������
  halfSpeed();
  
  //����PWM0
  PWM0CR |= 0x80;                               //ʹ��PWM0���
  PWM0CR &= 0xE7;                               //0b11100111  PWM0�����ΪP2.0 
  
  //����PWM1
  PWM1CR |= 0x80;
  PWM1CR &= 0xE7;                               //PWM1�����ΪP2.1
  
  //����PWM2
  PWM2CR |= 0x80;
  PWM2CR &= 0xE7;
  
  //����PWM3
  PWM3CR |= 0x80;
  PWM3CR &= 0xE7;  
  
  P_SW2 &=~0x80;
  PWMCR = 0x80;                                 
  //0b 1000 0000   ENPWM==1,����PWMģ�飨����PWMģ��Ŀ���λ��
  //ECBI==0,�ر�PWM�����������ж�
}

//�ر�����PWM���
//100ms����ʱ��Ϊ�˱���ͻȻ��ת�������������
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
��������SG90�����PWM�ź�
*/


//ȫ�ֱ���
bit busyGY=0;
bit busyBLE;
char dataFromBLE;
char dataFromGY[16];
char wptr=0x00;

/*
���ô���1�ʹ���2
����1�ö�ʱ��1������2�ö�ʱ��2
���裺
1.  ѡ������λ�Ͳ������Ƿ�ɱ䣨SCON��
2.  ѡ��ʱ��ʱ�ӣ��Լ�����ѡ���ĸ���ʱ����Ϊ�����ʷ�������AUXR����AUXR2��
3.  ���ö�ʱ���ĳ�ֵ
4.  ��ֹ��ʱ���ж�
5.  �򿪶�ʱ��
*/
void uartInit(void)		//9600bps@11.0592MHz
{
  //���ô���1����P315~P319��
  SCON = 0x50;
  //TCON = 
  TMOD = 0x00;
  TL1 = BRT;
  TH1 = BRT >> 8;
  AUXR =0x00;
  TR1 = 1;
	ET1 = 0;		//��ֹ��ʱ��1�ж�
	TR1 = 1;		//������ʱ��1
  //AUXR = 0x40;
  AUXR |= 0x40;
  AUXR &= 0xFE;////ʹ��T1��Ϊ�����ʷ�����
  
  //���ô���2����P323��
  S2CON = 0x10;
  T2L = BRT;
  T2H = BRT >> 8;
  AUXR |= 0x04;   //T2R=1��T2��ʼ��������T2_C/T=1��T2������ʱ����  ����P263
  AUXR |= 0x10; 
  IE2 |= 0x01;    //�򿪴���2���ж�
  EA = 1;         //�����ж�
  
}

//����1���жϺ�����������
void uart1Interrrupt() interrupt 4{
  if(RI){   //����ǽ���һ���ֽڵ��������
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


//����2���жϺ�����GY-952��
void uart2Interrrupt() interrupt 8{

  if (S2CON & 0x01)   //����ǽ���һ���ֽڵ��������
  {
      S2CON &= ~0x01;
      dataFromGY[wptr++] = S2BUF;
      wptr &= 0x0f;   //������16�����ݣ���������ˣ�����дǰ�������
      busyGY=0;
  }
  
  if (S2CON & 0x02)   //����ǽ���һ���ֽڵ��������
  {
      S2CON &= ~0x02;
      busyGY=0;
  }
}

//����a5 75 1a��֮��wptr��Ϊ0x00
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

