#include "reg51.h"
#include "intrins.h"
#include<math.h>
//ע�⣡��������Ҫѡ��hexģʽ������ͬ��������01��
//���ú��ı�ģʽ��hexģʽ���ͺ���յ�������ʵ���ϲ�һ����
//	Uart1��T1��Uart3��T3��Uart2��T2�������ʷ�����

//
//���Ծ���Ƶ��Ϊ11.0592MHz
//������Ϊ9600Hz
#define FOSC            11059200UL 
#define BRT             (65536 - FOSC / 9600 / 4)
sfr     AUXR        =   0x8e;
sfr     T4T3M       =   0xd1;
sfr     T4H         =   0xd2;
sfr     T4L         =   0xd3;
sfr     T3H         =   0xd4;
sfr     T3L         =   0xd5;
sfr     T2H         =   0xd6;
sfr     T2L         =   0xd7;
sfr     S2CON       =   0x9a;
sfr     S2BUF       =   0x9b;//    
sfr     S3CON       =   0xac;
sfr     S3BUF       =   0xad;
sfr     S4CON       =   0x84;
sfr     S4BUF       =   0x85;  
sfr     IE2         =   0xaf;

sfr     P_SW2       =   0xba;//����˿��л��Ĵ���2�����ܽ��л��� switch to register2 
//����ͨ����������Ĵ�����P_SW1ʵ�ִ��ڷ�ʱ���ã��Լ�IIC��ʱ���õȣ����̲�P74

sfr     PWMCFG      =   0xf1; //��ǿ��PWM���üĴ��� 
sfr     PWMIF       =   0xf6;//��ǿ��PWM�жϱ�־�Ĵ��� PWM interrupt
sfr     PWMFDCR     =   0xf7;//PWM�쳣�����ƼĴ���
sfr     PWMCR       =   0xfe;//PWM���ƼĴ���

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
bit busy_1;
bit busy_2;
bit busy_3;
unsigned char uart1_received;
//char uart2_received[3];
char uart2_received[3];
unsigned char uart3_received[30];
unsigned char uart3_RPY[6];
double angleYAW=0.0;
unsigned char uart1_temp;
unsigned char uart2_temp;
unsigned char wptr_2=0;
unsigned char wptr_3=0;
char speed=9;               //Ĭ��ȫ��
char newSpeed=2;            //��¼�µ��ٶ�
char directionSign=0x01;    //��־��ǰ��ǰ������0x01~0x04�ֱ�Ϊǰ������
char lastDirection=0x01;    //ָ����תָ���Ƕ�֮ǰ����ʻ����(Ϊ��ʵ�־�ֹʱ��ĳ�Ƕȵ�ԭ����ת)
//char uart3_received;
//char wptr_2=0;
int pitchAngle[5]={900,777,654,531,408};   //�ӵ͵��ߣ�����Ķ��������Ƕ�
int yawAngle[9]={553,799,1045,1291,1537,1783,2029,2275,2521};    //���ҵ�������Ķ��9���Ƕȣ��ұ�5�������3��
void uart1_init();
void uart3_init();
void uart2_init();
void uart1_send_char(unsigned  char dat);
void uart1_send_str(unsigned  char* p);
void uart3_send_char(unsigned  char dat);
void uart3_send_str(unsigned  char *p);
void uart2_send_char(unsigned  char  dat);
void uart2_send_str(unsigned  char* p);

void uart1_isr() interrupt 4//isr: Interrupt Service Routine
{
  if(RI)
  {
    RI=0;
    busy_1=0;
    uart1_received=SBUF;//���������������ݴ������uart1_received
    // wptr_1++;
    //wptr_1&=0x07;//����8���ֽھͻḲ��ǰ�������
    //if(wptr_1>=4){wptr_1=0;}//uart1_received�涨Ϊ8λ
  }
}
 
void uart2_isr() interrupt 8  
{
  if(S2CON&0x02)
  {
    S2CON&=~0x02;//~0x02==0xFD
    busy_2=0;//������һ֡���ݣ�busy_2λ��0���Ա㷢����һ֡����
  }
  if(S2CON&0x01)
  {
    S2CON&=~0x01;
    if(wptr_2<3)
    {
      uart2_received[wptr_2]=S2BUF;
      wptr_2++;
    }
    else
      uart2_temp=S2BUF;   //������һ���Է��͵����ݶ�������ʱ�ͷŵ����������
    busy_2=0;
  }
}  
void uart3_isr() interrupt 17
{
    if (S3CON & 0x02)
    {
        S3CON &= ~0x02;
        busy_3=0;
    }
    if (S3CON & 0x01)
    {
        busy_3=0;
        S3CON &= ~0x01;
        uart3_received[wptr_3++] = S3BUF;
        if(wptr_3>=30){
          wptr_3=0;
        }
    }
}
//3����ʱ���� delay functions
void delay_10ms()		//@11.0592MHz
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
  delay_10ms();
}
void delay_1s(void)
{
  int i=1;
  for(;i<=100;i++)
    delay_10ms();
}




void clear_uart3_received(void){
  int i=1;
  for(;i<=30;i++){
    uart3_received[i-1]=0;
  }
  wptr_3=0;
}

//�ر��������Ϊ��ѯ�����û�з������ݣ� Stop inputing
void closeOutput(void){
    uart3_send_char(0xa5);
    uart3_send_char(0x75);
    uart3_send_char(0x1a); 
}
//У׼��������û�з������ݣ� Calibrate the sensor
void sensorsCallibration(void){
    uart3_send_char(0xa5);
    uart3_send_char(0x57);
    uart3_send_char(0xfc); 

}
//����RAW�ǣ�û�з������ݣ�
void clearYAW(void){
    uart3_send_char(0xa5);
    uart3_send_char(0x58);
    uart3_send_char(0xfd);   
}
//��ʼ������ Initial the six-axis sensor
void initGY952(void){
  closeOutput();
  sensorsCallibration();
  clearYAW();
}
//��ȡ���ٶȣ�����11���ֽڣ� Get the acc information
void getAcc(void){
    clear_uart3_received();
    uart3_send_char(0xa5);
    uart3_send_char(0xc5);
    uart3_send_char(0x6a); 
}
//��ȡ���������ݣ�����11���ֽڣ�Get gyroscope  
void getGYRO(void){
    clear_uart3_received();
    uart3_send_char(0xa5);
    uart3_send_char(0xd5);
    uart3_send_char(0x7a); 
}
//��ȡŷ���ǣ�����11���ֽڣ� Get euler's Angle
void sendRPYRequest(void){
  clear_uart3_received();
  uart3_send_char(0xa5);
  uart3_send_char(0x95);
  uart3_send_char(0x3a);  
  delay_100ms();  //�ʼ�õ�10ms���������ݲ�����
}
//��ȡ��Ԫ��������13���ֽڣ�Get the quaternion
void getTCB(void){
    clear_uart3_received();
    uart3_send_char(0xa5);
    uart3_send_char(0xb5);
    uart3_send_char(0x5a); 
}
void refreshRPY(void){
    int i,j;
    int temp0;
    double temp1,temp2;
    sendRPYRequest();
    for(i=0;i<30;i++){
    //�ж��Ƿ�Ϊһ֡��Ч���ݣ���֪��Ϊʲô����ú�����������
      if(uart3_received[i]==0x5a&&uart3_received[i+1]==0x5a&&uart3_received[i+2]==0x45&&uart3_received[i+3]==0x06){
        for(j=i+4;j<i+10;j++)
          uart3_RPY[j-i-4]=uart3_received[j];
        break;
      }    
    }
    
    temp0=uart3_RPY[4]<<8|uart3_RPY[5];
    //uart1_send_char(uart3_RPY[4]<<8|uart3_RPY[5]);
    temp1=((int)(temp0-0xffff)-1);
    temp1/=100;
    temp2=temp0/100;    //�жϵ�ǰ��YAW����ֵ���Ǹ�ֵ
    if(fabs(temp1)<=180)
      angleYAW=temp1;
    else if(fabs(temp2)<=180)
        angleYAW=temp2;
    delay_10ms();      
}


/* ʹ��PWM���� Steps of Using the PWM
//���ݱ���speed�����ٶ�
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

void getSpeed(void){
  //�ٶ�Ϊ1~9�ŵ���ʵ��speed<3��ʱ��ʹ�������
  // 0x8a40ԼΪ0.1����0x5666
  //PWMnTx��Ϊ0x5666����Ϊ0x0000��һ������ǰ����Ϊ��һ���ģ�
  PWM0T1= 0x5666-0x8a40*(speed-1);
  PWM0T2= 0x0000;
  PWM1T1= 0x5666-0x8a40*(speed-1);
  PWM1T2= 0x0000;
  PWM2T1= 0x5666-0x8a40*(speed-1);
  PWM2T2= 0x0000;
  PWM3T1= 0x5666-0x8a40*(speed-1);
  PWM3T2= 0x0000;
  if (speed==9)
  {
    PWM0T1=0x0000;
    PWM1T1=0x0000;
    PWM2T1=0x0000;
    PWM3T1=0x0000;
  }
}





//�ı�pitch��yaw�ĽǶȣ�����1Ϊpitch������2Ϊyaw�����ֻ��ı�һ���Ƕȣ�����һ��������Ϊ10�����߸���ֵ������
int changeAngle(int,int); 
//��ʼ��6��PWM Initialize the PWM
void initPWMs(void)
{
  P_SW2 |= 0x80;                              
  //0b 1000 0000 EAXFRΪ1�����ڶ�дƬ������,��֪����ɶ�õģ�
  //�����Ĳ��䣨Ϊ�˲�Ӱ�촮��2��3��4�Ĺ��ܽţ���ò�Ҫֱ�Ӷ�P_SW2��ֵ��  
  
  PWMCKS = 0x0a;                                // PWMʱ��Ϊϵͳʱ�ӵ�1/10
  PWMC = 0x5666;                               
  
  //��ʼPWM�źţ��ĸ�PWM�����ã�С�����־�ֹ������
  getSpeed();
  
  //����PWM0
  PWM0CR |= 0x80;                               //ʹ��PWM0���
  PWM0CR &= 0xE7;                               //0b11100111  PWM0�����ΪP2.0 
  PWM0T1=0x0000;
  PWM0T2=0x0000;
  
  //����PWM1
  PWM1CR |= 0x80;
  PWM1CR &= 0xE7;                               //PWM1�����ΪP2.1                             
  PWM1T1=0x0000;
  PWM1T2=0x0000;
  
  //����PWM2
  PWM2CR |= 0x80;
  PWM2CR &= 0xE7;                             
  PWM2T1=0x0000;
  PWM2T2=0x0000;
  
  //����PWM3
  PWM3CR |= 0x80;
  PWM3CR &= 0xE7;                           
  PWM3T1=0x0000;
  PWM3T2=0x0000;  
  
  //����PWM4
  PWM4T1=yawAngle[5];           //���ұ�553  �м�1783
  PWM4T2 = 0x5666;
  PWM4CR |= 0x80;
  PWM4CR &= 0xE7;
  
  //����PWM5
  PWM5T1=pitchAngle[0];       //0x5666*0.5/20=553��Ϊ0��      10/90/20*0x5666=123  ÿ����ת��10����Ҫ��123
  PWM5T2 = 0x5666;
  PWM5CR |= 0x80;
  PWM5CR &= 0xE7;
  
  P_SW2 &=~0x80;
  PWMCR = 0x80;  
  delay_1s();
  
}
//�ر�����PWM��������������������PWM��Close all the PWM output
void forbiddenAll(void){
  PWM0CR &= ~0x80;
  PWM1CR &= ~0x80;
  PWM2CR &= ~0x80;
  PWM3CR &= ~0x80;
}
void stop(void){
  P_SW2 |=0x80;
  forbiddenAll();
  if(speed!=newSpeed){
    uart2_send_str("stop\n");
    speed=newSpeed;
    getSpeed();
  }
  P_SW2 &=~0x80;
  //lastDirection=directionSign;
  directionSign=0;
}
void headForwards(){
  stop();
  delay_100ms();
  P_SW2 |=0x80;
  PWM1CR |= 0x80;
  PWM3CR |= 0x80;
  P_SW2 &=~0x80;
  //lastDirection=directionSign;
  directionSign=0x01;
}
void headBackwards(void){
  stop();
  delay_100ms();
  P_SW2 |=0x80;
  PWM0CR |= 0x80;
  PWM2CR |= 0x80;
  P_SW2 &=~0x80;
  //lastDirection=directionSign;
  directionSign=0x02;
}
void turnLeft(void){
  stop();
  delay_100ms();
  P_SW2 |=0x80;
  PWM0CR |= 0x80;
  PWM3CR |= 0x80;
  P_SW2 &=~0x80;
  //lastDirection=directionSign;
  directionSign=0x03;
  
}  

void turnRight(void){
  stop();
  delay_100ms();
  P_SW2 |=0x80;
  PWM1CR |= 0x80;
  PWM2CR |= 0x80;
  P_SW2 &=~0x80;
  //lastDirection=directionSign;
  directionSign=0x04;
}

//ע��������������ĺ�С�������Ĳ��غϣ�������ʵ��ת�ĽǶȺͷ��͵�ָ��������в�ͬ
//��һ����ȷ��ת����Ҫ������ѧ�Ƶ�����С������ת�ǶȻ�����������ת�Ƕ�

char turnLeftDegree(char theta){
  double targetAngleYAW=angleYAW+theta;
  if(fabs(theta)>90)
    return 0;
  turnLeft();
  while(angleYAW<targetAngleYAW){
    refreshRPY();
    delay_10ms();
    uart2_send_char(0x33);
  }
  switch(lastDirection)
  {
    case 0:
      stop();
      break;
    case 1:
      headForwards();
      break;
    case 2:
      headBackwards();
      break;
    case 3:
      turnLeft();
      break;
    case 4:
      turnRight();
      break;
  }
  uart2_send_char(0x59);
  return 1;
}
char turnRightDegree(char theta){
  double targetAngleYAW=angleYAW-theta;
  if(fabs(theta)>90)
    return 0;
  turnRight();
  while(angleYAW>targetAngleYAW){
    uart2_send_char(0x44);
    refreshRPY();
    delay_10ms();
  }
  switch(lastDirection)
  {
    case 0:
      stop();
      break;
    case 1:
      headForwards();
      break;
    case 2:
      headBackwards();
      break;
    case 3:
      turnLeft();
      break;
    case 4:
      turnRight();
      break;
  }
  uart2_send_char(0x58);
  return 1;
}

void refreshSpeed(){
  //ֻ���ٶȸ���ʱ��ִ��
  if(speed!=newSpeed){
    //��stop()������speed��ֵ
    switch(directionSign){
      case 0x01:
        headForwards();
        break;
      case 0x02:
        headBackwards();
        break;
      case 0x03:
        turnLeft();
        break;
      case 0x04:
        turnRight();
        break;
      default:
        break;
    }
  }
}
void main()
{
  int i,j;
  char temp[3]={0};
  delay_100ms();
  uart1_init();//ע��Ĵ���AUXR��ֵ�Ƿ��ظ��Ķ�����initʧ��
  uart2_init();
  uart3_init();
  ES=1;       //�򿪴���1���ж�
  IE2 = 0x09; //�򿪴���3(0x08)���жϺʹ���2���ж�(0x01) 
  EA = 1;//�����ж�
  initGY952(); //��������һ��Ҫ���ڴ��ڵĳ�ʼ��֮�󣨰���TMOD,SCON,THn,TLn,AUXR�ʹ����ж���صļĴ���ES,IE2,EA�ȣ�
  initPWMs();    
  //�����жദ��ʱ�����������λ����������̫��
  //�����ݱ�����֮ǰ�ͱ������ˣ����ܻᵼ�·��͵����ݱȽ��յ�������
  //������λ����ȡ���ݵļ����ò�Ҫ����100ms
  while(1)                          //���������յ������ݽ����ж� 
  {
    wptr_2=0;
    delay_10ms();                   //ע��delay_10msһ��Ҫ��wptr_2=0�ĺ��棬��Ϊ�������û������ʱ֮�б�������Ļ�
                                    //����wptr_2��0�ˣ��Ḳ��֮ǰ��ĳЩ���ݡ�
    while(uart2_received[0]!=0x00)  //��֪��Ϊʲô��if��׽����
    {          
      delay_10ms();                //ȷ��ʣ�µ���������Ҳ��������
                                   //uart1_send_char(wptr_2);
      for(i=0;i<3;i++)
        temp[i]=uart2_received[i];
                                   //JDY-16������ֵ��bluepy�ж����ˣ���Ҫ����ķ�����
      /*if(temp[0]==0||temp[1]==0||temp[2]==0)
      {
        uart2_send_char(0x88);
        for(i=0;i<3;i++)
          uart2_received[i]=0;    //����uart2_received������������Ļ�û�з�������ʱ����2��һֱ����0x88
        wptr_2=0;
        continue;
      }*/
      for(i=0;i<3;i++)
        uart2_received[i]=0;      //һ��ȡ������֮���������uart2_received��whileѭ����������Ҳ�У�
      refreshRPY();               //ÿ�ν���ָ��ǰ������RPY��
      switch(temp[0]){
        case 0x01:                //һֱǰ��
          uart2_send_char(0x01);
          headForwards();
          clearYAW();
          uart2_send_char(directionSign);
          break;    
        case 0x02:                //һֱ����
          uart2_send_char(0x02);
          headBackwards();
          clearYAW();
          uart2_send_char(directionSign);
          break;    
        case 0x03:                //һֱ����ת
          uart2_send_char(0x03);
          turnLeft();
          clearYAW();
          uart2_send_char(directionSign);
          break;
        case 0x04:               //һֱ����ת
          uart2_send_char(0x04);
          turnRight();
          clearYAW();            //���ܰ�clearYAW()����turnRight���棬��Ϊ����ĳ�Ƕ���תʱ���õ�֮ǰ��YAW�Ƕ�
          uart2_send_char(directionSign);
          break;
        case 0x05:
          lastDirection=directionSign;
          if(temp[1]!=0){
            uart2_send_char(7);
            turnLeftDegree(temp[1]);
            clearYAW(); 
          }
          break;
        case 0x06:
          lastDirection=directionSign;
          if(temp[1]!=0){
            uart2_send_char(0x06);
            turnRightDegree(temp[1]);
            clearYAW(); 
          }
          break;
        case 0x07:            //�ı��ٶ�
          uart2_send_char(7);
          newSpeed=temp[1];
          refreshSpeed();
          break;
        case 0x09:            //�ı���������ĽǶ�
          changeAngle(temp[1]-1,temp[2]-1);    //temp[i]����Ϊ0�����Ըı����Ƕ�ʱ�������±�+1
          break;
        default:            //ֹͣ
          stop();
          break;
      }
    }
  }
}
void uart1_init()
{
  SCON=0x50;//��ģʽ0������ͬʱѡ��T1��Ϊ~
  TMOD=0x00;//16λ�Զ���װ
  TL1=BRT;
  TH1=BRT>>8;
  TR1=1;//T1��ʼ����
  AUXR =0x00;
  AUXR |= 0x40;
  AUXR &= 0xFE;   //ʹ��T1��Ϊ�����ʷ�����
  busy_1=0;
  uart1_received=0x00;
}
 
void uart1_send_char(unsigned  char dat)
{
  while(busy_1);
  busy_1=1;
  SBUF=dat;
  while(TI==0);
  TI=0;//ֱ���ڷ��ͺ����ｫTI��0�������жϺ�������0
  busy_1=0;//�ʼ���ûд
}
void uart1_send_str(unsigned  char* p)
{
  while(*p)
  {
    uart1_send_char(*p++);
  }
}


void uart2_init()
{
  S2CON=0x10;//S4ST4==0��ʹ��T2��Ϊ~����S4REN=1�������������
  //Ϊ�˲�����������ʱ����ʹ�ã���ò�ȡλ���㣬ֻ����T2�йص�λ��ֵ
  T2L=BRT;
  T2H=BRT>>8;//�ȸ�ֵ�ټ���
  AUXR|=0x10;//T2R=1��T2��ʼ����
  AUXR|=0x04;//T2x12=1,����Ƶ
  busy_2=0;
}
void uart2_send_char(unsigned  char  dat)
{
  while(busy_2);
  busy_2=1;
  S2BUF=dat;//ע�ⲻ��SBUF
}
void uart2_send_str(unsigned  char* p)
{
  while(*p)
  {
    uart2_send_char(*p++);
  }
}

void uart3_init()
{
    S3CON = 0x50; 
    //0b  0101 0000  ��P326�����ɱ䲨���� 8λ��ѡ��T3Ϊ~��
    //��Ҫȷ������ģʽ�Ͷ�ʱ��������               
    T3L = BRT;
    T3H = BRT >> 8;//baud_rateΪ9600
    T4T3M = 0x0a; //0b 0000 1��T3R��T3��ʼ������001
    busy_3 = 0;
}
void uart3_send_char(unsigned  char dat)
{
    while (busy_3);
    busy_3 = 1;
    S3BUF = dat; //�򴮿�3�����ݻ�����д������
}
void uart3_send_str(unsigned  char *p)
{
    while (*p)
    {
        uart3_send_char(*p++);
    }
} 

int changeAngle(int i, int j)
{
  //����������charʱ������������ִ��
  uart2_send_char(i);
  uart2_send_char(j);
  P_SW2 |= 0x80;
  if(i<5&&i>=0)
    PWM5T1=pitchAngle[i];
  if(j<9&&j>=0)
    PWM4T1=yawAngle[j];
  P_SW2 &= ~0x80; 
  return 1;  
}