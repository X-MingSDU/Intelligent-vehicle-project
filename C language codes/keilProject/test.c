#include "reg51.h"
#include "intrins.h"
#include<math.h>
//注意！发送数据要选择hex模式！比如同样的数据01，
//采用和文本模式和hex模式发送后接收到的数据实际上不一样！
//	Uart1用T1，Uart3用T3，Uart2用T2做波特率发生器

//
//测试晶振频率为11.0592MHz
//波特率为9600Hz
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

sfr     P_SW2       =   0xba;//外设端口切换寄存器2（功能脚切换） switch to register2 
//可以通过配置这个寄存器和P_SW1实现串口分时复用，以及IIC分时复用等，见教材P74

sfr     PWMCFG      =   0xf1; //增强型PWM配置寄存器 
sfr     PWMIF       =   0xf6;//增强型PWM中断标志寄存器 PWM interrupt
sfr     PWMFDCR     =   0xf7;//PWM异常检测控制寄存器
sfr     PWMCR       =   0xfe;//PWM控制寄存器

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
char speed=9;               //默认全速
char newSpeed=2;            //记录新的速度
char directionSign=0x01;    //标志当前的前进方向0x01~0x04分别为前后左右
char lastDirection=0x01;    //指在旋转指定角度之前的行驶方向(为了实现静止时的某角度的原地旋转)
//char uart3_received;
//char wptr_2=0;
int pitchAngle[5]={900,777,654,531,408};   //从低到高，上面的舵机的五个角度
int yawAngle[9]={553,799,1045,1291,1537,1783,2029,2275,2521};    //从右到左，下面的舵机9个角度，右边5个，左边3个
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
    uart1_received=SBUF;//将读缓存器的数据存进变量uart1_received
    // wptr_1++;
    //wptr_1&=0x07;//多于8个字节就会覆盖前面的数据
    //if(wptr_1>=4){wptr_1=0;}//uart1_received规定为8位
  }
}
 
void uart2_isr() interrupt 8  
{
  if(S2CON&0x02)
  {
    S2CON&=~0x02;//~0x02==0xFD
    busy_2=0;//发送完一帧数据，busy_2位置0，以便发送下一帧数据
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
      uart2_temp=S2BUF;   //当蓝牙一次性发送的数据多于三个时释放掉多余的数据
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
//3个延时函数 delay functions
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

//关闭输出（改为查询输出，没有返回数据） Stop inputing
void closeOutput(void){
    uart3_send_char(0xa5);
    uart3_send_char(0x75);
    uart3_send_char(0x1a); 
}
//校准传感器（没有返回数据） Calibrate the sensor
void sensorsCallibration(void){
    uart3_send_char(0xa5);
    uart3_send_char(0x57);
    uart3_send_char(0xfc); 

}
//清零RAW角（没有返回数据）
void clearYAW(void){
    uart3_send_char(0xa5);
    uart3_send_char(0x58);
    uart3_send_char(0xfd);   
}
//初始化六轴 Initial the six-axis sensor
void initGY952(void){
  closeOutput();
  sensorsCallibration();
  clearYAW();
}
//获取加速度（返回11个字节） Get the acc information
void getAcc(void){
    clear_uart3_received();
    uart3_send_char(0xa5);
    uart3_send_char(0xc5);
    uart3_send_char(0x6a); 
}
//获取陀螺仪数据（返回11个字节）Get gyroscope  
void getGYRO(void){
    clear_uart3_received();
    uart3_send_char(0xa5);
    uart3_send_char(0xd5);
    uart3_send_char(0x7a); 
}
//获取欧拉角（返回11个字节） Get euler's Angle
void sendRPYRequest(void){
  clear_uart3_received();
  uart3_send_char(0xa5);
  uart3_send_char(0x95);
  uart3_send_char(0x3a);  
  delay_100ms();  //最开始用的10ms，发现数据不完整
}
//获取四元数（返回13个字节）Get the quaternion
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
    //判断是否为一帧有效数据（不知道为什么这段用函数代替会出错）
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
    temp2=temp0/100;    //判断当前的YAW是正值还是负值
    if(fabs(temp1)<=180)
      angleYAW=temp1;
    else if(fabs(temp2)<=180)
        angleYAW=temp2;
    delay_10ms();      
}


/* 使用PWM步骤 Steps of Using the PWM
//根据变量speed设置速度
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

void getSpeed(void){
  //速度为1~9九档，实测speed<3的时候就带不动了
  // 0x8a40约为0.1倍的0x5666
  //PWMnTx设为0x5666和设为0x0000不一样（以前还以为是一样的）
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





//改变pitch和yaw的角度，参数1为pitch，参数2为yaw。如果只想改变一个角度，另外一个参数置为10（或者更大值）即可
int changeAngle(int,int); 
//初始化6个PWM Initialize the PWM
void initPWMs(void)
{
  P_SW2 |= 0x80;                              
  //0b 1000 0000 EAXFR为1（关于读写片外数据,不知道干啥用的）
  //其他的不变（为了不影响串口2，3，4的功能脚，最好不要直接对P_SW2赋值）  
  
  PWMCKS = 0x0a;                                // PWM时钟为系统时钟的1/10
  PWMC = 0x5666;                               
  
  //初始PWM信号，四个PWM都启用，小车保持静止，满速
  getSpeed();
  
  //配置PWM0
  PWM0CR |= 0x80;                               //使能PWM0输出
  PWM0CR &= 0xE7;                               //0b11100111  PWM0输出脚为P2.0 
  PWM0T1=0x0000;
  PWM0T2=0x0000;
  
  //配置PWM1
  PWM1CR |= 0x80;
  PWM1CR &= 0xE7;                               //PWM1输出脚为P2.1                             
  PWM1T1=0x0000;
  PWM1T2=0x0000;
  
  //配置PWM2
  PWM2CR |= 0x80;
  PWM2CR &= 0xE7;                             
  PWM2T1=0x0000;
  PWM2T2=0x0000;
  
  //配置PWM3
  PWM3CR |= 0x80;
  PWM3CR &= 0xE7;                           
  PWM3T1=0x0000;
  PWM3T2=0x0000;  
  
  //配置PWM4
  PWM4T1=yawAngle[5];           //最右边553  中间1783
  PWM4T2 = 0x5666;
  PWM4CR |= 0x80;
  PWM4CR &= 0xE7;
  
  //配置PWM5
  PWM5T1=pitchAngle[0];       //0x5666*0.5/20=553，为0度      10/90/20*0x5666=123  每向上转动10度需要减123
  PWM5T2 = 0x5666;
  PWM5CR |= 0x80;
  PWM5CR &= 0xE7;
  
  P_SW2 &=~0x80;
  PWMCR = 0x80;  
  delay_1s();
  
}
//关闭所有PWM输出（不包括两个舵机的PWM）Close all the PWM output
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

//注：由于六轴的质心和小车的质心不重合，所以真实旋转的角度和发送的指令可能略有不同
//进一步精确的转弯需要进行数学推导，把小车的旋转角度换算成六轴的旋转角度

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
  //只有速度更改时才执行
  if(speed!=newSpeed){
    //在stop()更新了speed的值
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
  uart1_init();//注意寄存器AUXR的值是否被重复改动导致init失败
  uart2_init();
  uart3_init();
  ES=1;       //打开串口1的中断
  IE2 = 0x09; //打开串口3(0x08)的中断和串口2的中断(0x01) 
  EA = 1;//打开总中断
  initGY952(); //发送数据一定要放在串口的初始化之后（包括TMOD,SCON,THn,TLn,AUXR和串口中断相关的寄存器ES,IE2,EA等）
  initPWMs();    
  //由于有多处延时，所以如果上位机发送数据太快
  //在数据被处理之前就被覆盖了，可能会导致发送的数据比接收的数据少
  //所以上位机读取数据的间隔最好不要超过100ms
  while(1)                          //对蓝牙接收到的数据进行判断 
  {
    wptr_2=0;
    delay_10ms();                   //注意delay_10ms一定要在wptr_2=0的后面，因为如果数据没有在延时之中被接收完的话
                                    //由于wptr_2置0了，会覆盖之前的某些数据。
    while(uart2_received[0]!=0x00)  //不知道为什么用if捕捉不到
    {          
      delay_10ms();                //确保剩下的两个数据也发过来了
                                   //uart1_send_char(wptr_2);
      for(i=0;i<3;i++)
        temp[i]=uart2_received[i];
                                   //JDY-16的特征值在bluepy中读不了，不要下面的反馈了
      /*if(temp[0]==0||temp[1]==0||temp[2]==0)
      {
        uart2_send_char(0x88);
        for(i=0;i<3;i++)
          uart2_received[i]=0;    //不对uart2_received进行清零操作的话没有发送数据时串口2会一直发送0x88
        wptr_2=0;
        continue;
      }*/
      for(i=0;i<3;i++)
        uart2_received[i]=0;      //一获取到数据之后马上清空uart2_received（while循环的最后清空也行）
      refreshRPY();               //每次接收指令前都更新RPY角
      switch(temp[0]){
        case 0x01:                //一直前进
          uart2_send_char(0x01);
          headForwards();
          clearYAW();
          uart2_send_char(directionSign);
          break;    
        case 0x02:                //一直后退
          uart2_send_char(0x02);
          headBackwards();
          clearYAW();
          uart2_send_char(directionSign);
          break;    
        case 0x03:                //一直向左转
          uart2_send_char(0x03);
          turnLeft();
          clearYAW();
          uart2_send_char(directionSign);
          break;
        case 0x04:               //一直向右转
          uart2_send_char(0x04);
          turnRight();
          clearYAW();            //不能把clearYAW()放在turnRight里面，因为按照某角度旋转时会用到之前的YAW角度
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
        case 0x07:            //改变速度
          uart2_send_char(7);
          newSpeed=temp[1];
          refreshSpeed();
          break;
        case 0x09:            //改变两个舵机的角度
          changeAngle(temp[1]-1,temp[2]-1);    //temp[i]不能为0，所以改变舵机角度时，传入下标+1
          break;
        default:            //停止
          stop();
          break;
      }
    }
  }
}
void uart1_init()
{
  SCON=0x50;//以模式0工作，同时选择T1作为~
  TMOD=0x00;//16位自动重装
  TL1=BRT;
  TH1=BRT>>8;
  TR1=1;//T1开始计数
  AUXR =0x00;
  AUXR |= 0x40;
  AUXR &= 0xFE;   //使用T1作为波特率发生器
  busy_1=0;
  uart1_received=0x00;
}
 
void uart1_send_char(unsigned  char dat)
{
  while(busy_1);
  busy_1=1;
  SBUF=dat;
  while(TI==0);
  TI=0;//直接在发送函数里将TI置0，不在中断函数里置0
  busy_1=0;//最开始这句没写
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
  S2CON=0x10;//S4ST4==0（使用T2作为~），S4REN=1，允许接收数据
  //为了不干涉其他定时器的使用，最好采取位运算，只对与T2有关的位赋值
  T2L=BRT;
  T2H=BRT>>8;//先赋值再计数
  AUXR|=0x10;//T2R=1，T2开始计数
  AUXR|=0x04;//T2x12=1,不分频
  busy_2=0;
}
void uart2_send_char(unsigned  char  dat)
{
  while(busy_2);
  busy_2=1;
  S2BUF=dat;//注意不是SBUF
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
    //0b  0101 0000  （P326），可变波特率 8位，选择T3为~，
    //主要确定工作模式和定时器发生器               
    T3L = BRT;
    T3H = BRT >> 8;//baud_rate为9600
    T4T3M = 0x0a; //0b 0000 1（T3R，T3开始计数）001
    busy_3 = 0;
}
void uart3_send_char(unsigned  char dat)
{
    while (busy_3);
    busy_3 = 1;
    S3BUF = dat; //向串口3的数据缓存器写入数据
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
  //函数参数是char时函数不能正常执行
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