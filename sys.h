#ifndef _SYS_H
#define _SYS_H

#include "stc8h.h"
#include "intrins.h"

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned int ushort;
typedef unsigned long ulong;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;

typedef char  s8;
typedef int   s16;
typedef long  s32;


#define FOSC     40000000UL

void delay_ms(u16 ms);
void delay_us(u16 us);

#endif

