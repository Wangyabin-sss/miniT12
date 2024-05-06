
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
// ��ǰ�¶�����Ϊ����ͷ
#define TEMPMAPNUM 10
static struct temperature_map{
	int temp;
	int adc;
	float k;
	float b;
}temp_map[TEMPMAPNUM] = {{0,0},
						{50,90},
						{100,254},
						{150,350},
						{200,488},
						{250,620},
						{300,786},
						{350,922},
						{400,1060},
						{450,1250}};
	
#define ADCARRAYNUM 5       //t12 adc����
#define SLEEPTIME   300     //����ʱ�䣨�룩
#define CLOSETIME   600     //�ر�ʱ�䣨�룩
#define MPUGRYLIEMT 20      //mpu6050�𶯷�Χ���жϾ���״̬��
#define PWMHZ       10      //��ǰ����Ƶ��


void gpio_init(void);
void ADC_init(void);
u16 ADC_get_val(u8 channel);
u16 temp2adcval(u16 temperature);
u16 adc2tempval(u16 adcval);
int get_pwmval_with_pid(u16 adcvalt12, u16 adcvalwant, s16 pwmmax);
void Timer0_Init(void);
s16 abs(s16 num);
						
						
#if 1
int main()
{
	s16 PWMVAL = 0, pwmtime = 0;
	float powerval=0;
	u32 t12adc_val[ADCARRAYNUM] = {0}, t12adc_max, t12adc_min, t12adc_all, t12adc_average, t12adc_i=0;   //adc��ֵ�˲�  adc��ѹ��λmV
	u16 temp_want = 350,temp_set=350,adc_want;
	s16 mpu_data=0,mpu_data_diff=0,mpu_data_last=0,mpu_time,mpu_temp;
	u8 i;
	

	for(i=0;i<TEMPMAPNUM;i++)
	{
		if(i==TEMPMAPNUM-1)
        {
            temp_map[i].k = temp_map[i-1].k;
            temp_map[i].b = temp_map[i-1].b;
            break;
        }
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
	OLED_ShowString(72,2,"Slp:",8);
	OLED_ShowNum(102,0,temp_want,3,8);


	//���㵱ǰ��Դ��ѹ����ʾ
	powerval = ADC_get_val(1);
	powerval = (powerval*3300)/4096;
	powerval = powerval/1000*11.2;
	OLED_ShowNum(102,1,powerval*10,3,8);
	
	
	
	while(1)
	{
		//����
		if(pwmtime<PWMVAL)
		{
			SWITCH = 1;
			LED = 0;
		}
		//�Ͽ�
		else
		{
			SWITCH = 0;
			LED = 1;
		}
		delay_us(40);
		pwmtime++;
		if(pwmtime==2501)  //40*2500=100000us = 100ms = 10Hz
		{
			pwmtime = 0;
			//��ֹʱ����
			mpu_data = GetData(MPU_GYRO_YOUTH_REG);
			mpu_data_diff = mpu_data - mpu_data_last;
			mpu_data_last = mpu_data;
			if(abs(mpu_data_diff)<MPUGRYLIEMT)
			{
				mpu_time++;
			}
			else
				mpu_time=0;
			
			//���ߡ��رա�����״̬�л�
			if(mpu_time/PWMHZ>CLOSETIME)
				temp_want = 0;
			else if(mpu_time/PWMHZ>SLEEPTIME)
				temp_want = 200;
			else
				temp_want = temp_set;
				
			
			//��ȫ�ټ���ʱ��ʾһЩ�Ǳ�Ҫ����
			if((2500-PWMVAL)>1000)
			{
//				//���㵱ǰ��Դ��ѹ����ʾ
//				powerval = ADC_get_val(1);
//				powerval = (powerval*3300)/4096;
//				powerval = powerval/10000*100;
//				OLED_ShowNum(102,1,powerval*10,3,8);
//				//mpu6050�¶ȴ������¶�
//				mpu_temp = MPU_Get_Temperature();
//				OLED_ShowNum(102,2,mpu_temp,3,8);
				//��ʾ����ʱ��
				OLED_ShowNum(102,2,mpu_time/PWMHZ,3,8);
			}

			//�����趨���¶Ȼ�ȡT12�ȵ�ż��ѹֵ��adcֵ��
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
			//PID����PWM����ռ�ձ�
			PWMVAL = get_pwmval_with_pid(t12adc_average,adc_want,2500);
			//��ʾ��ǰ�¶�
			OLED_ShowNum(32,0,adc2tempval(t12adc_average),3,16);
			//adc
			//OLED_ShowNum(32,0,t12adc_average,4,16);

			//��������Ŵ��������ѹ && �������
			t12adc_val[t12adc_i] = ADC_get_val(0);
			t12adc_val[t12adc_i] = (t12adc_val[t12adc_i]*3300)/4096;
			t12adc_i++;
			if(t12adc_i==ADCARRAYNUM)
				t12adc_i = 0;
			
			//�������
			if(KEY1_DOWN&&KEY2_DOWN)  //ͬʱ����
			{
				KEY1_DOWN = 0;
				KEY2_DOWN = 0;
				
			}
			if(KEY1_DOWN)   //KEY1����
			{
				temp_want+=KEY1_DOWN;
				if(temp_want>450)
					temp_want = 450;
				temp_set = temp_want;
				OLED_ShowNum(102,0,temp_want,3,8);
				KEY1_DOWN = 0;
			}
			if(KEY2_DOWN)  //KEY2����
			{
				temp_want-=KEY2_DOWN;
				if(temp_want<0)
					temp_want = 0;
				temp_set = temp_want;
				OLED_ShowNum(102,0,temp_want,3,8);
				KEY2_DOWN = 0;
			}
		}
	}
}
#else
//Ӳ������
int main()
{
	u8 flag=0,ret;
	s16 mpu_data,mpu_temp;
	float powerval;
	
	gpio_init();
	OLED_Init();
	ADC_init();
	Timer0_Init();
	ret = InitMPU6050();
	OLED_ShowNum(10,2,ret,3,8);
	
	EA = 1;
	SWITCH = 0;
	
	while(1)
	{
		mpu_data = GetData(MPU_GYRO_YOUTH_REG);
		mpu_temp = MPU_Get_Temperature();

		if(mpu_data<0)
		{
			mpu_data = -mpu_data;
			OLED_ShowString(0,0,"-",8);
		}
		else
		{
			OLED_ShowString(0,0,"+",8);
		}
			
		OLED_ShowNum(10,0,mpu_data,5,8);
		OLED_ShowNum(72,0,mpu_temp,4,8);
		
		
		//���㵱ǰ��Դ��ѹ����ʾ
		powerval = ADC_get_val(1);
		powerval = (powerval*3300)/4096;
		powerval = powerval/10000*100;
		OLED_ShowNum(10,1,powerval*10,3,8);
		
		if(flag==1)
		{
			SWITCH = 1;
			LED = 0;
			flag = 0;
		}
		else
		{
			SWITCH = 0;
			LED = 1;
			flag = 1;
		}
		
		delay_ms(60);
	}
}
#endif


void gpio_init(void)
{
	P_SW2 |= 0x80;     //ʹ�ܷ��� XFR
	
	P3M0 |= (3<<5);    //���� P3.5  P3.6Ϊ����ģʽ   LED & SWITCH
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
	return temp_map[i].k*temperature+temp_map[i].b;
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
	return (adcval-temp_map[i].b)/temp_map[i].k;
}


#define PVAL  25.0F
#define IVAL  0.82F
#define DVAL  1.20F
#define INTEGRAL 1500
int get_pwmval_with_pid(u16 adcvalt12, u16 adcvalwant, s16 pwmmax)
{
	static s16  lasterror=0, integralval=0;
	s16 error,derror, pwmval=0;
	error = adcvalwant - adcvalt12;
	integralval += error;
	if(integralval>INTEGRAL)
		integralval = INTEGRAL;
	if(integralval<-INTEGRAL)
		integralval = -INTEGRAL;
	
	derror = error - lasterror;
	pwmval = PVAL*error + IVAL*integralval + DVAL*derror;
	lasterror = error;
	
	if(pwmval > pwmmax)
		pwmval = pwmmax;
	return pwmval;
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
	if(_K1>80)
	{
		_K1=0;
		KEY1_DOWN+=1;
	}
	if(_K2>80)
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


s16 abs(s16 num)
{
	if(num<0)
		return -num;
	else
		return num;
}



