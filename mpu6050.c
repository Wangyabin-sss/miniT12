#include "mpu6050.h"


//*****************************************************************************************************
//向I2C设备写入一个字节数据
//*****************************************************************************************************
u8 MPU_Write_Byte(u8 REG_Address,u8 REG_data)
{
    I2C_Start();                  //起始信号
    I2C_SendByte((MPU_ADDR<<1)|0);   //发送设备地址+写信号
	I2C_RecvACK();	//等待应答
    I2C_SendByte(REG_Address);    //内部寄存器地址，
	I2C_RecvACK();	//等待应答
    I2C_SendByte(REG_data);       //内部寄存器数据，
	I2C_RecvACK();	//等待应答
    I2C_Stop();                   //发送停止信号
	return 0;
}
//*******************************************************************************************************
//从I2C设备读取一个字节数据
//*******************************************************************************************************
u8 MPU_Read_Byte(u8 REG_Address)
{
	u8 REG_data;
	I2C_Start();                   //起始信号
	I2C_SendByte((MPU_ADDR<<1)|0);    //发送设备地址+写信号
	I2C_RecvACK();	//等待应答
	I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始	
	I2C_RecvACK();	//等待应答
	I2C_Start();                   //起始信号
	I2C_SendByte((MPU_ADDR<<1)|1);  //发送设备地址+读信号
	I2C_RecvACK();	//等待应答
 	REG_data=I2C_RecvByte(1);       //读出寄存器数据
	I2C_Stop();                    //停止信号
	return REG_data;
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t dat=0;
	if(lpf>=188)dat=1;
	else if(lpf>=98)dat=2;
	else if(lpf>=42)dat=3;
	else if(lpf>=20)dat=4;
	else if(lpf>=10)dat=5;
	else dat=6; 
	return MPU_Write_Byte(MPU_CFG_REG,dat);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t dat;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	dat=1000/rate-1;
	dat=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,dat);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


//******************************************************************************************************
//初始化MPU6050
//******************************************************************************************************
u8 InitMPU6050(void)
{
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0XFF);	//FIFO全开
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(200);						//设置采样率为200Hz
 	}else return 1;
	return 0;
}
//******************************************************************************************************
//合成数据
//******************************************************************************************************
u16 GetData(u8 REG_Address)
{
	u8 H,L;
	H=MPU_Read_Byte(REG_Address);
	L=MPU_Read_Byte(REG_Address+1);
	return ((H<<8)+L);   //合成数据
}


