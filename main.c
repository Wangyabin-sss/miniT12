
//#include "iic.h"
#include "mpu6050.h"

sbit LED    = P3^5;
sbit SWITCH = P3^6;
sbit KEY1   = P3^3;
sbit KEY2   = P3^2;

u8 _K1 = 0,_K2 = 0;
u8 KEY1_DOWN=0,KEY2_DOWN=0;

//�˷ŵ�ѹ���¶ȶ�Ӧ��  0  50  100  150  200  250  300  350  400  450
static u16 temp_val[10] = {0};

void gpio_init(void);
void ADC_init(void);
u16 ADC_get_val(u8 channel);
int get_pwmval_with_pid(u16 adcvalt12, u16 adcvalwant, u16 pwmmax);
void Timer0_Init(void);

int main()
{
	u16 PWMVAL = 0;
	float powerval=0;
	u16 pwmtime = 0;
	u16 systime = 0;
	u32 t12adc_val = 0;
	u32 kes = 0;
	int z_data=0;
	
	gpio_init();
	OLED_Init();
	ADC_init();
	Timer0_Init();
	InitMPU6050();
	delay_ms(10);
	EA = 1;
	SWITCH = 0;
	
	powerval = ADC_get_val(1);
	powerval = (powerval*3300)/4096;
	powerval = powerval/10000*100;
	//OLED_ShowNum(0,2,powerval*10,8,16);
	
	
	while(1)
	{
		if(pwmtime<PWMVAL)
		{
			SWITCH = 1;
			LED = 0;
		}
		else
		{
			SWITCH = 0;
			LED = 1;
		}
		delay_us(50);
		pwmtime++;
		if(pwmtime==2020)  //50*2000=100000us = 100ms����
		{
			pwmtime = 0;
			//5s����ˢ��һ�ε�Դ��ѹ
			if(systime%50==0)
			{
				powerval = ADC_get_val(1);
				powerval = (powerval*3300)/4096;
				powerval = powerval/10000*100;
				//OLED_ShowNum(0,2,powerval*10,8,16);
			}
			t12adc_val = ADC_get_val(0);
			t12adc_val = (t12adc_val*3300)/4096;
			PWMVAL = get_pwmval_with_pid(t12adc_val,750,2000);
			//OLED_ShowNum(0,0,t12adc_val,8,16);
			
			z_data = GetData(MPU_GYRO_XOUTH_REG);
			if(z_data<0)
				z_data = -z_data;
			OLED_ShowNum(54,0,z_data,6,16);
			z_data = GetData(MPU_GYRO_YOUTH_REG);
			if(z_data<0)
				z_data = -z_data;
			OLED_ShowNum(54,2,z_data,6,16);
			z_data = GetData(MPU_TEMP_OUTH_REG);
			if(z_data<0)
				z_data = -z_data;
			OLED_ShowNum(0,2,z_data,6,16);
			
			if(KEY1_DOWN)
			{
				KEY1_DOWN = 0;
				OLED_ShowNum(54,0,kes++,6,16);
			}
			if(KEY2_DOWN)
			{
				KEY2_DOWN = 0;
				
			}
			
			systime++;
		}
	}
}


void gpio_init(void)
{
	P_SW2 |= 0x80;     //ʹ�ܷ��� XFR
	
	P3M0 |= (3<<5);    //���� P3.5  P3.6Ϊ����ģʽ   LED  SWITCH
	P3M1 &= ~(3<<5);
	
	P1M0 |= (3<<4);    //���� P1.4  P1.5Ϊ��©ģʽ   IIC
	P1M1 |= (3<<4);
	
	P1M0 &= ~(3<<0);   //���� P1.0  P1.1Ϊ��������   ADC
	P1M1 |= (3<<0);
	
	P3M0 &= ~(3<<2);    //���� P3.2  P3.3Ϊ˫���ģʽ   KEY
	P3M1 &= ~(3<<2);
	
	KEY1=1;
	KEY2=1;
	SCL=1;
	SDA=1;
}

void ADC_init(void)
{
	ADCTIM = 0x3f;//���� ADC �ڲ�ʱ��
	ADCCFG = 0x0f;//���� ADC ʱ��Ϊϵͳʱ��/2/16
	ADC_CONTR = 0x80;//ʹ�� ADC ģ��
}


u16 ADC_get_val(u8 channel)
{
	ADC_CONTR |= 0x40;        //���� AD ת��
	
	ADC_CONTR &= ~(0xf);
	ADC_CONTR |= channel;
	
	_nop_();
	_nop_();
	while (!(ADC_CONTR & 0x20));//��ѯ ADC ��ɱ�־
	ADC_CONTR &= ~0x20;         //����ɱ�־
	P2 = ADC_RES;               //��ȡ ADC ���
	return (ADC_RES<<4)|(ADC_RESL>>4);
}


#define PVAL  15.0F
#define IVAL  0.0F
#define DVAL  0.0F
int get_pwmval_with_pid(u16 adcvalt12, u16 adcvalwant, u16 pwmmax)
{
	float pwm_val = 0;
	int diff = adcvalwant - adcvalt12;
	pwm_val = PVAL*diff;
	if(pwm_val<0)
		pwm_val = -pwm_val;
	if(pwm_val > pwmmax)
		pwm_val = pwmmax;
	if(adcvalt12>adcvalwant)
		return 100;
	return pwm_val;
}

void Timer0_Isr(void) interrupt 1
{
	if(KEY1==0){
		_K1+=1;
	}else{
		_K1=0;
	}
	if(KEY2==0){
		_K2+=1;
	}else{
		_K2=0;
	}
	if(_K1>120)
	{
		_K1=0;
		KEY1_DOWN=1;
	}
	if(_K2>120)
	{
		_K2=0;
		KEY2_DOWN=1;
	}
}

void Timer0_Init(void)		//1000΢��@24.000MHz
{
	AUXR |= 0x80;			//��ʱ��ʱ��1Tģʽ
	TMOD &= 0xF0;			//���ö�ʱ��ģʽ
	TL0 = 0x40;				//���ö�ʱ��ʼֵ
	TH0 = 0xA2;				//���ö�ʱ��ʼֵ
	TF0 = 0;				//���TF0��־
	TR0 = 1;				//��ʱ��0��ʼ��ʱ
	ET0 = 1;				//ʹ�ܶ�ʱ��0�ж�
}


