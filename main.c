
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
// 当前温度曲线为西安头
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
	
#define ADCARRAYNUM 5       //t12 adc数组
#define SLEEPTIME   300     //休眠时间（秒）
#define CLOSETIME   600     //关闭时间（秒）
#define MPUGRYLIEMT 20      //mpu6050震动范围（判断静置状态）
#define PWMHZ       10      //当前加热频率


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
	u32 t12adc_val[ADCARRAYNUM] = {0}, t12adc_max, t12adc_min, t12adc_all, t12adc_average, t12adc_i=0;   //adc均值滤波  adc电压单位mV
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


	//计算当前电源电压并显示
	powerval = ADC_get_val(1);
	powerval = (powerval*3300)/4096;
	powerval = powerval/1000*11.2;
	OLED_ShowNum(102,1,powerval*10,3,8);
	
	
	
	while(1)
	{
		//加热
		if(pwmtime<PWMVAL)
		{
			SWITCH = 1;
			LED = 0;
		}
		//断开
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
			//静止时间检测
			mpu_data = GetData(MPU_GYRO_YOUTH_REG);
			mpu_data_diff = mpu_data - mpu_data_last;
			mpu_data_last = mpu_data;
			if(abs(mpu_data_diff)<MPUGRYLIEMT)
			{
				mpu_time++;
			}
			else
				mpu_time=0;
			
			//休眠、关闭、加热状态切换
			if(mpu_time/PWMHZ>CLOSETIME)
				temp_want = 0;
			else if(mpu_time/PWMHZ>SLEEPTIME)
				temp_want = 200;
			else
				temp_want = temp_set;
				
			
			//非全速加热时显示一些非必要数据
			if((2500-PWMVAL)>1000)
			{
//				//计算当前电源电压并显示
//				powerval = ADC_get_val(1);
//				powerval = (powerval*3300)/4096;
//				powerval = powerval/10000*100;
//				OLED_ShowNum(102,1,powerval*10,3,8);
//				//mpu6050温度传感器温度
//				mpu_temp = MPU_Get_Temperature();
//				OLED_ShowNum(102,2,mpu_temp,3,8);
				//显示静置时长
				OLED_ShowNum(102,2,mpu_time/PWMHZ,3,8);
			}

			//根据设定的温度获取T12热电偶电压值（adc值）
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
			//PID控制PWM加热占空比
			PWMVAL = get_pwmval_with_pid(t12adc_average,adc_want,2500);
			//显示当前温度
			OLED_ShowNum(32,0,adc2tempval(t12adc_average),3,16);
			//adc
			//OLED_ShowNum(32,0,t12adc_average,4,16);

			//计算运算放大器输出电压 && 填充数组
			t12adc_val[t12adc_i] = ADC_get_val(0);
			t12adc_val[t12adc_i] = (t12adc_val[t12adc_i]*3300)/4096;
			t12adc_i++;
			if(t12adc_i==ADCARRAYNUM)
				t12adc_i = 0;
			
			//按键检测
			if(KEY1_DOWN&&KEY2_DOWN)  //同时按下
			{
				KEY1_DOWN = 0;
				KEY2_DOWN = 0;
				
			}
			if(KEY1_DOWN)   //KEY1按下
			{
				temp_want+=KEY1_DOWN;
				if(temp_want>450)
					temp_want = 450;
				temp_set = temp_want;
				OLED_ShowNum(102,0,temp_want,3,8);
				KEY1_DOWN = 0;
			}
			if(KEY2_DOWN)  //KEY2按下
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
//硬件测试
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
		
		
		//计算当前电源电压并显示
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
	P_SW2 |= 0x80;     //使能访问 XFR
	
	P3M0 |= (3<<5);    //设置 P3.5  P3.6为推挽模式   LED & SWITCH
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


s16 abs(s16 num)
{
	if(num<0)
		return -num;
	else
		return num;
}



