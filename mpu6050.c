#include "mpu6050.h"


//*****************************************************************************************************
//��I2C�豸д��һ���ֽ�����
//*****************************************************************************************************
u8 MPU_Write_Byte(u8 REG_Address,u8 REG_data)
{
    I2C_Start();                  //��ʼ�ź�
    I2C_SendByte((MPU_ADDR<<1)|0);   //�����豸��ַ+д�ź�
	I2C_RecvACK();	//�ȴ�Ӧ��
    I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
	I2C_RecvACK();	//�ȴ�Ӧ��
    I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
	I2C_RecvACK();	//�ȴ�Ӧ��
    I2C_Stop();                   //����ֹͣ�ź�
	return 0;
}
//*******************************************************************************************************
//��I2C�豸��ȡһ���ֽ�����
//*******************************************************************************************************
u8 MPU_Read_Byte(u8 REG_Address)
{
	u8 REG_data;
	I2C_Start();                   //��ʼ�ź�
	I2C_SendByte((MPU_ADDR<<1)|0);    //�����豸��ַ+д�ź�
	I2C_RecvACK();	//�ȴ�Ӧ��
	I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ	
	I2C_RecvACK();	//�ȴ�Ӧ��
	I2C_Start();                   //��ʼ�ź�
	I2C_SendByte((MPU_ADDR<<1)|1);  //�����豸��ַ+���ź�
	I2C_RecvACK();	//�ȴ�Ӧ��
 	REG_data=I2C_RecvByte(1);       //�����Ĵ�������
	I2C_Stop();                    //ֹͣ�ź�
	return REG_data;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t dat=0;
	if(lpf>=188)dat=1;
	else if(lpf>=98)dat=2;
	else if(lpf>=42)dat=3;
	else if(lpf>=20)dat=4;
	else if(lpf>=10)dat=5;
	else dat=6; 
	return MPU_Write_Byte(MPU_CFG_REG,dat);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t dat;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	dat=1000/rate-1;
	dat=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,dat);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}


//******************************************************************************************************
//��ʼ��MPU6050
//******************************************************************************************************
u8 InitMPU6050(void)
{
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0XFF);	//FIFOȫ��
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(200);						//���ò�����Ϊ200Hz
 	}else return 1;
	return 0;
}
//******************************************************************************************************
//�ϳ�����
//******************************************************************************************************
u16 GetData(u8 REG_Address)
{
	u8 H,L;
	H=MPU_Read_Byte(REG_Address);
	L=MPU_Read_Byte(REG_Address+1);
	return ((H<<8)+L);   //�ϳ�����
}


