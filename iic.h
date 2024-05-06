#ifndef _IIC_H
#define _IIC_H


#include "sys.h"


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

sbit SCL=P1^5;//SCL
sbit SDA=P1^4;//SDA
sbit RES=P1^6;//RES


void IIC_delay(void);

void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendACK(bit ack);
bit I2C_RecvACK();
void I2C_SendByte(u8 dat);
u8 I2C_RecvByte(bit n);


//OLED控制用函数
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_Set_Pos(u8 x, u8 y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 sizey);
u32 oled_pow(u8 m,u8 n);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 sizey);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 sizey);
void OLED_ShowChinese(u8 x,u8 y,u8 no,u8 sizey);
void OLED_DrawBMP(u8 x,u8 y,u8 sizex, u8 sizey,u8 BMP[]);
void OLED_Init(void);



#endif


