
//#include "iic.h"
#include "mpu6050.h"

sbit LED    = P3^5;
sbit SWITCH = P3^6;
sbit KEY1   = P3^3;
sbit KEY2   = P3^2;

u8 _K1 = 0,_K2 = 0;
u8 KEY1_DOWN=0,KEY2_DOWN=0;

//运放电压与温度对应表  0  50  100  150  200  250  300  350  400  450
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
		if(pwmtime==2020)  //50*2000=100000us = 100ms周期
		{
			pwmtime = 0;
			//5s计算刷新一次电源电压
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
	P_SW2 |= 0x80;     //使能访问 XFR
	
	P3M0 |= (3<<5);    //设置 P3.5  P3.6为推挽模式   LED  SWITCH
	P3M1 &= ~(3<<5);
	
	P1M0 |= (3<<4);    //设置 P1.4  P1.5为开漏模式   IIC
	P1M1 |= (3<<4);
	
	P1M0 &= ~(3<<0);   //设置 P1.0  P1.1为高阻输入   ADC
	P1M1 |= (3<<0);
	
	P3M0 &= ~(3<<2);    //设置 P3.2  P3.3为双向口模式   KEY
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

void Timer0_Init(void)		//1000微秒@24.000MHz
{
	AUXR |= 0x80;			//定时器时钟1T模式
	TMOD &= 0xF0;			//设置定时器模式
	TL0 = 0x40;				//设置定时初始值
	TH0 = 0xA2;				//设置定时初始值
	TF0 = 0;				//清除TF0标志
	TR0 = 1;				//定时器0开始计时
	ET0 = 1;				//使能定时器0中断
}


