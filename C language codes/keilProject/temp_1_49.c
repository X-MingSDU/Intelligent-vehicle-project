#include "reg51.h"
#include "intrins.h"

#define FOSC            11059200UL
#define BRT             (65536 - FOSC / 9600 / 4)

sfr     T4T3M       =   0xd1;
sfr     T4H         =   0xd2;
sfr     T4L         =   0xd3;
sfr     T3H         =   0xd4;
sfr     T3L         =   0xd5;
sfr     S3CON       =   0xac;
sfr     S3BUF       =   0xad;
sfr     IE2         =   0xaf;

bit busy;
char wptr;
char rptr;
char buffer[16];

void Uart3Isr() interrupt 17
{
    if (S3CON & 0x02)
    {
        S3CON &= ~0x02;
        busy = 0;
    }
    if (S3CON & 0x01)
    {
        S3CON &= ~0x01;
        buffer[wptr++] = S3BUF;
        wptr &= 0x0f;
    }
}

void Uart3Init()
{
    S3CON = 0x50;
    T3L = BRT;
    T3H = BRT >> 8;
    T4T3M = 0x0a;
    wptr = 0x00;
    rptr = 0x00;
    busy = 0;
}

void Uart3Send(char dat)
{
    while (busy);
    busy = 1;
    S3BUF = dat;
}

void Uart3SendStr(char *p)
{
    while (*p)
    {
        Uart3Send(*p++);
    }
}

void main()
{
    Uart3Init();
    IE2 = 0x08;
    EA = 1;
    Uart3SendStr("Uart Test !\r\n");

    while (1)
    {
      Uart3SendStr("Uart Test !\r\n");
    }
}

