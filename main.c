
//#include "iic.h"
#include "mpu6050.h"

sbit LED    = P3^5;
sbit SWITCH = P3^6;
sbit KEY1   = P3^3;
sbit KEY2   = P3^2;

u8 _K1 = 0,_K2 = 0;
u8 KEY1_DOWN=0,KEY2_DOWN=0;

//�˷ŵ�ѹ���¶ȶ�Ӧ��  0  50  100  150  200  250  300  350  400  450
//  adc
//  |
//	|____temp
//
#define TEMPMAPNUM 10
static struct temperature_map{
	int temp;
	int adc;
	float k;
	float b;
}temp_map[TEMPMAPNUM] = {{0,0},
						{50,100},
						{100,200},
						{150,300},
						{200,400},
						{250,500},
						{300,600},
						{350,750},
						{400,800},
						{450,850}};
	
#define ADCARRAYNUM 6

void gpio_init(void);
void ADC_init(void);
u16 ADC_get_val(u8 channel);
u16 temp2adcval(u16 temperature);
u16 adc2tempval(u16 adcval);
int get_pwmval_with_pid(u16 adcvalt12, u16 adcvalwant, u16 pwmmax);
void Timer0_Init(void);

int main()
{
	u16 PWMVAL = 0;
	float powerval=0;
	u16 pwmtime = 0;
	u32 t12adc_val[ADCARRAYNUM] = {0}, t12adc_max, t12adc_min, t12adc_all, t12adc_average, t12adc_i=0;   //adc��ֵ�˲�
	u16 temp_want = 350,adc_want;
	s16 mpu_data=0,mpu_diff=0,mpu_time=0;
	u8 i;
	
	for(i=0;i<TEMPMAPNUM-1;i++)
	{
		temp_map[i].k = (temp_map[i+1].adc-temp_map[i].adc)/50.0f;
		temp_map[i].b = temp_map[i].adc-temp_map[i].k*temp_map[i].temp;
	}
	
	gpio_init();
	OLED_Init();
	ADC_init();
	Timer0_Init();
	InitMPU6050();
	
	EA = 1;
	SWITCH = 0;

	OLED_ShowString(72,0,"Set:",8);
	OLED_ShowString(72,1,"Pow:",8);
	OLED_ShowNum(102,0,temp_want,3,8);
	
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
		if(pwmtime==2001)  //50*2000=100000us = 100ms����
		{
			pwmtime = 0;
			//���㵱ǰ��Դ��ѹ����ʾ
			powerval = ADC_get_val(1);
			powerval = (powerval*3300)/4096;
			powerval = powerval/10000*100;
			OLED_ShowNum(102,1,powerval*10,3,8);
			
			//��ȡmpu6050����
			if((2000-PWMVAL)>800)
			{
				mpu_data = GetData(MPU_GYRO_XOUTH_REG);
			}

			//�����趨���¶�תadcֵ
			adc_want = temp2adcval(temp_want);

			//adc�˲�����ȥ�������Сֵ�����ֵ
			t12adc_max=t12adc_val[0];
			t12adc_min=t12adc_val[0];
			t12adc_all = 0;
			for(i=0;i<ADCARRAYNUM;i++)
			{
				if(t12adc_val[i]<t12adc_min)
					t12adc_min = t12adc_val[i];
				if(t12adc_val[i]>t12adc_max)
					t12adc_max = t12adc_val[i];
				t12adc_all += t12adc_val[i];
			}
			t12adc_average = (t12adc_all-t12adc_max-t12adc_min)/(ADCARRAYNUM-2);
			PWMVAL = get_pwmval_with_pid(t12adc_average,adc_want,2000);
			OLED_ShowNum(32,0,adc2tempval(t12adc_average),3,16);

			//��������Ŵ��������ѹ���������
			t12adc_val[t12adc_i] = ADC_get_val(0);
			t12adc_val[t12adc_i] = (t12adc_val[t12adc_i]*3300)/4096;
			t12adc_i++;
			if(t12adc_i==ADCARRAYNUM)
				t12adc_i = 0;
			
			//�������
			if(KEY1_DOWN)
			{
				OLED_ShowNum(102,0,temp_want+=KEY1_DOWN,3,8);
				KEY1_DOWN = 0;
			}
			if(KEY2_DOWN)
			{
				OLED_ShowNum(102,0,temp_want-=KEY2_DOWN,3,8);
				KEY2_DOWN = 0;
			}
		}
	}
}


void gpio_init(void)
{
	P_SW2 |= 0x80;     //ʹ�ܷ��� XFR
	
	P3M0 |= (3<<5);    //���� P3.5  P3.6Ϊ����ģʽ   LED  SWITCH
	P3M1 &= ~(3<<5);
	
	P1M0 |= (3<<4);    //���� P1.4  P1.5Ϊ��©ģʽ   IIC ����������
	P1M1 |= (3<<4);
	
	P1M0 &= ~(3<<0);   //���� P1.0  P1.1Ϊ��������   ADC
	P1M1 |= (3<<0);
	
	P3M0 &= ~(3<<2);    //���� P3.2  P3.3Ϊ׼˫��ģʽ   KEY
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


u16 temp2adcval(u16 temperature)
{
	u8 i=0;
	
	for(i=0;i<TEMPMAPNUM-1;i++)
	{
		if(temperature>=temp_map[i].temp&&temperature<temp_map[i+1].temp)
		{
			return temp_map[i].k*temperature+temp_map[i].b;
		}
	}
}

u16 adc2tempval(u16 adcval)
{
	u8 i=0;
	
	for(i=0;i<TEMPMAPNUM-1;i++)
	{
		if(adcval>=temp_map[i].adc&&adcval<temp_map[i+1].adc)
		{
			return (adcval-temp_map[i].b)/temp_map[i].k;
		}
	}
}


#define PVAL  20.0F
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
	if(_K1>70)
	{
		_K1=0;
		KEY1_DOWN+=1;
	}
	if(_K2>70)
	{
		_K2=0;
		KEY2_DOWN+=1;
	}
}

void Timer0_Init(void)		//1000΢��@40.000MHz
{
	AUXR |= 0x80;			//��ʱ��ʱ��1Tģʽ
	TMOD &= 0xF0;			//���ö�ʱ��ģʽ
	TL0 = 0xC0;				//���ö�ʱ��ʼֵ
	TH0 = 0x63;				//���ö�ʱ��ʼֵ
	TF0 = 0;				//���TF0��־
	TR0 = 1;				//��ʱ��0��ʼ��ʱ
	ET0 = 1;				//ʹ�ܶ�ʱ��0�ж�
}


