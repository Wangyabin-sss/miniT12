
//#include "iic.h"
#include "mpu6050.h"

sbit LED    = P3^5;
sbit SWITCH = P3^6;
sbit KEY1   = P3^3;
sbit KEY2   = P3^2;

u8 _K1 = 0,_K2 = 0;
u8 KEY1_DOWN=0,KEY2_DOWN=0;

//运放电压与温度对应表  0  50  100  150  200  250  300  350  400  450
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
	u32 t12adc_val[ADCARRAYNUM] = {0}, t12adc_max, t12adc_min, t12adc_all, t12adc_average, t12adc_i=0;   //adc均值滤波
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
		if(pwmtime==2001)  //50*2000=100000us = 100ms周期
		{
			pwmtime = 0;
			//计算当前电源电压并显示
			powerval = ADC_get_val(1);
			powerval = (powerval*3300)/4096;
			powerval = powerval/10000*100;
			OLED_ShowNum(102,1,powerval*10,3,8);
			
			//获取mpu6050数据
			if((2000-PWMVAL)>800)
			{
				mpu_data = GetData(MPU_GYRO_XOUTH_REG);
			}

			//根据设定的温度转adc值
			adc_want = temp2adcval(temp_want);

			//adc滤波数组去掉最大最小值，求均值
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

			//计算运算放大器输出电压，填充数组
			t12adc_val[t12adc_i] = ADC_get_val(0);
			t12adc_val[t12adc_i] = (t12adc_val[t12adc_i]*3300)/4096;
			t12adc_i++;
			if(t12adc_i==ADCARRAYNUM)
				t12adc_i = 0;
			
			//按键检测
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
	P_SW2 |= 0x80;     //使能访问 XFR
	
	P3M0 |= (3<<5);    //设置 P3.5  P3.6为推挽模式   LED  SWITCH
	P3M1 &= ~(3<<5);
	
	P1M0 |= (3<<4);    //设置 P1.4  P1.5为开漏模式   IIC 带上拉电阻
	P1M1 |= (3<<4);
	
	P1M0 &= ~(3<<0);   //设置 P1.0  P1.1为高阻输入   ADC
	P1M1 |= (3<<0);
	
	P3M0 &= ~(3<<2);    //设置 P3.2  P3.3为准双向模式   KEY
	P3M1 &= ~(3<<2);
	
	KEY1=1;
	KEY2=1;
	SCL=1;
	SDA=1;
}

void ADC_init(void)
{
	ADCTIM = 0x3f;//设置 ADC 内部时序
	ADCCFG = 0x0f;//设置 ADC 时钟为系统时钟/2/16
	ADC_CONTR = 0x80;//使能 ADC 模块
}


u16 ADC_get_val(u8 channel)
{
	ADC_CONTR |= 0x40;        //启动 AD 转换
	
	ADC_CONTR &= ~(0xf);
	ADC_CONTR |= channel;
	
	_nop_();
	_nop_();
	while (!(ADC_CONTR & 0x20));//查询 ADC 完成标志
	ADC_CONTR &= ~0x20;         //清完成标志
	P2 = ADC_RES;               //读取 ADC 结果
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

void Timer0_Init(void)		//1000微秒@40.000MHz
{
	AUXR |= 0x80;			//定时器时钟1T模式
	TMOD &= 0xF0;			//设置定时器模式
	TL0 = 0xC0;				//设置定时初始值
	TH0 = 0x63;				//设置定时初始值
	TF0 = 0;				//清除TF0标志
	TR0 = 1;				//定时器0开始计时
	ET0 = 1;				//使能定时器0中断
}


