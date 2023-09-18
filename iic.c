#include "iic.h"
#include "oledfont.h"


//*************************************************************************************************
//I2C��ʼ�ź�
//*************************************************************************************************
void I2C_Start()
{
	SCL = 1;                    //����ʱ����
    SDA = 1;                    //����������
    IIC_delay();                 //��ʱ
    SDA = 0;                    //�����½���
    IIC_delay();                 //��ʱ
    SCL = 0;                    //����ʱ����
}
//*************************************************************************************************
//I2Cֹͣ�ź�
//*************************************************************************************************
void I2C_Stop()
{
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    IIC_delay();                 //��ʱ
    SDA = 1;                    //����������
    IIC_delay();                 //��ʱ
}
//**************************************************************************************************
//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************************************************************************
void I2C_SendACK(bit ack)
{
    SCL=0;
	SDA=ack;
	IIC_delay();
	SCL=1;
	IIC_delay();
	SCL=0;
}
//****************************************************************************************************
//I2C����Ӧ���ź�
//****************************************************************************************************
bit I2C_RecvACK()
{
	u16 time=0;
	SDA=1;
	delay_us(1);
	SCL=1;
	delay_us(1);
	while(SDA)
	{
		time++;
		if(time>250*5)
		{
			//I2C_Stop();
			return 1;
		}
	}
	SCL=0;
	return 0;
}
//*****************************************************************************************************
//��I2C���߷���һ���ֽ�����
//***************************************************************************************************** 
void I2C_SendByte(u8 dat)
{
    char i;
	SCL=0;
	SDA=0;
	IIC_delay();
	for(i=7;i>=0;i--)
	{
		if(dat&(1<<i))
			SDA=1;
		else
			SDA=0;
		IIC_delay();
		SCL=1;
		IIC_delay();
		SCL=0;
		IIC_delay();
	}
}
//*****************************************************************************************************
//��I2C���߽���һ���ֽ�����
//n��ʾ�Ƿ���Ӧ   0��ʾ��ӦACK   1��ʾ����ӦNACK
//******************************************************************************************************
u8 I2C_RecvByte(bit n)
{
    u8 dat=0;
	char i;
	
	for(i=7;i>=0;i--)
	{
		SCL=0;
		IIC_delay();
		SCL=1;
		if(SDA)
			dat|=1<<i;
		IIC_delay();
	}
	I2C_SendACK(n);  
	return dat;
}


//����һ���ֽ�
//��SSD1306д��һ���ֽڡ�
//mode:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 mode)
{
	I2C_Start();
	I2C_SendByte(0x78);
	I2C_RecvACK();
	if(mode){I2C_SendByte(0x40);}
	else{I2C_SendByte(0x00);}
	I2C_RecvACK();
	I2C_SendByte(dat);
	I2C_RecvACK();
	I2C_Stop();
}


//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   


//���Ժ���
void OLED_ColorTurn(u8 i)
{
	if(i==0)
		{
			OLED_WR_Byte(0xA6,OLED_CMD);//������ʾ
		}
	if(i==1)
		{
			OLED_WR_Byte(0xA7,OLED_CMD);//��ɫ��ʾ
		}
}

//��Ļ��ת180��
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
		{
			OLED_WR_Byte(0xC8,OLED_CMD);//������ʾ
			OLED_WR_Byte(0xA1,OLED_CMD);
		}
	if(i==1)
		{
			OLED_WR_Byte(0xC0,OLED_CMD);//��ת��ʾ
			OLED_WR_Byte(0xA0,OLED_CMD);
		}
}


//��������

void OLED_Set_Pos(u8 x, u8 y) 
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD);
}   	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(0x0,OLED_DATA); 
	} //������ʾ
	
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63				 
//sizey:ѡ������ 6x8  8x16
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 sizey)
{      	
	u8 c=0,sizex=sizey/2;
	u16 i=0,size1;
	if(sizey==8)size1=6;
	else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
	c=chr-' ';//�õ�ƫ�ƺ��ֵ
	OLED_Set_Pos(x,y);
	for(i=0;i<size1;i++)
	{
		if(i%sizex==0&&sizey!=8) OLED_Set_Pos(x,y++);
		if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8�ֺ�
		else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16�ֺ�
//		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//�û�����ֺ�
		else return;
	}
}




//m^n����
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//��ʾ����
//x,y :�������
//num:Ҫ��ʾ������
//len :���ֵ�λ��
//sizey:�����С		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 sizey)
{         	
	u8 t,temp,m=0;
	u8 enshow=0;
	if(sizey==8)m=2;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
				continue;
			}else enshow=1;
		}
	 	OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
	}
}
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 sizey)
{
	u8 j=0;
	while (chr[j]!='\0')
	{
		OLED_ShowChar(x,y,chr[j++],sizey);
		if(sizey==8)x+=6;
		else x+=sizey/2;
	}
}
//��ʾ����
void OLED_ShowChinese(u8 x,u8 y,u8 no,u8 sizey)
{
	u16 i,size1=(sizey/8+((sizey%8)?1:0))*sizey;
	for(i=0;i<size1;i++)
	{
		if(i%sizey==0) OLED_Set_Pos(x,y++);
		if(sizey==16) OLED_WR_Byte(Hzk[no][i],OLED_DATA);//16x16�ֺ�
//		else if(sizey==xx) OLED_WR_Byte(xxx[c][i],OLED_DATA);//�û�����ֺ�
		else return;
	}				
}


//��ʾͼƬ
//x,y��ʾ����
//sizex,sizey,ͼƬ����
//BMP��Ҫ��ʾ��ͼƬ
void OLED_DrawBMP(u8 x,u8 y,u8 sizex, u8 sizey,u8 BMP[])
{ 	
  u16 j=0;
	u8 i,m;
	sizey=sizey/8+((sizey%8)?1:0);
	for(i=0;i<sizey;i++)
	{
		OLED_Set_Pos(x,i+y);
    for(m=0;m<sizex;m++)
		{      
			OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
		}
	}
} 


void SSD1306_WRITECOMMAND(u8 dat)
{
	OLED_WR_Byte(dat,OLED_CMD);
}


//��ʼ��				    
void OLED_Init(void)
{
    delay_ms(200);

//	
//	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
//	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
//	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
//	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
//	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
//	OLED_WR_Byte(0x81,OLED_CMD); // contract control
//	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
//	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
//	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
//	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
//	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
//	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
//	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
//	OLED_WR_Byte(0x00,OLED_CMD);//
//	
//	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
//	OLED_WR_Byte(0x80,OLED_CMD);//
//	
//	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
//	OLED_WR_Byte(0x05,OLED_CMD);//
//	
//	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
//	OLED_WR_Byte(0xF1,OLED_CMD);//
//	
//	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
//	OLED_WR_Byte(0x12,OLED_CMD);//
//	
//	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
//	OLED_WR_Byte(0x20,OLED_CMD);//
//	
//	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
//	OLED_WR_Byte(0x14,OLED_CMD);//
//	
//	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
	
 /* Init LCD */
    SSD1306_WRITECOMMAND(0xAE); // Set display off
    SSD1306_WRITECOMMAND(0xA8); // Set multiplex ratio
    SSD1306_WRITECOMMAND(0x1F); // -- from default 63 to 31 (i.e. 32MUX)
    SSD1306_WRITECOMMAND(0xD3); // Set display offset
    SSD1306_WRITECOMMAND(0x00); // -- no offset
    SSD1306_WRITECOMMAND(0x40); // Set display start line
    SSD1306_WRITECOMMAND(0xA0); // Set segment re-map, column address 127 is mapped to SEG0   0xa0���ҷ�  0xa1����
    SSD1306_WRITECOMMAND(0xC0); // Set COM output scan direction - remapped mode              0xc0���·�  0xc8����
    SSD1306_WRITECOMMAND(0x81); // Set contrast control for BANK0
    SSD1306_WRITECOMMAND(0x7F); // -- range 0x00 to 0xFF => 50%
    SSD1306_WRITECOMMAND(0xA4); // Enable display outputs according to the GDDRAM contents.
    SSD1306_WRITECOMMAND(0xA6); // Set normal display
    SSD1306_WRITECOMMAND(0xD5); // Set display clock divide ration and oscillator frequency
    SSD1306_WRITECOMMAND(0x80); // -- frequency (1000 - default); display clock divide ratio (0000 - divide ration 1)
    SSD1306_WRITECOMMAND(0x8D); // Charge pump setting
    SSD1306_WRITECOMMAND(0x14); // -- enable charge pump

    SSD1306_WRITECOMMAND(0x2E); // Deactivate scroll
    SSD1306_WRITECOMMAND(0x20); // Set memory addressing mode
    SSD1306_WRITECOMMAND(0x10); // -- Page Addressing Mode (RESET)
    SSD1306_WRITECOMMAND(0xDA); // Set COM pins hardware configuration
    SSD1306_WRITECOMMAND(0x02); // --
    SSD1306_WRITECOMMAND(0xD9); // Set pre-charge period
    SSD1306_WRITECOMMAND(0x22); // --
    SSD1306_WRITECOMMAND(0xDB); // Set Vcomh deselect level
    SSD1306_WRITECOMMAND(0x20); // -- 0.77 x Vcc (RESET)

    SSD1306_WRITECOMMAND(0xB0); // Set page start address for page addressing mode
    SSD1306_WRITECOMMAND(0x00); // Set lower column start address for page addressing mode
    SSD1306_WRITECOMMAND(0x10); // Set higher column start address for page addressing mode

    SSD1306_WRITECOMMAND(0xAF); // Set display on
	
	OLED_Clear();
}




