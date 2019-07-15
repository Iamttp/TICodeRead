首先用了git控制了最原始代码，然后替换了徐师兄程序

#### 传感器测试程序

	7月12 
	1.测试飞控底板成功，能读取传感器数据，未起飞测试
	2.发现匿名这款程序时间片不仅使用了定时器，还利用icm20602的中断引脚
	(ttp注： Drv_icm20602.c里为GPIOB2注册中断 执行INT_1ms_Task函数)（这个
	芯片的中断引脚可以配置1ms往外输出一个脉冲），在程序里面加一个外部中断函数
	优先级最高（在这个函数里面获取数据，可能这样传感器数据时序更稳定？）因为买
	的传感器没有将icm20602的中断引脚引出来，所以直接强行把这个中断函数删了，时
	间片的方案直接用之前的那个。
	3.匿名新ti飞控，由于芯片引脚太少，没有用遥控器协议没有用之前的pwm模式，这次
	用了PPM模式，也就是意味着之前的遥控器都不能用了，（少部分带ppm输出的可以用）
	这个需要注意一下，不过可以自己写个接口，直接用外部的stm32加个nrf24l01或者蓝
	牙控制开关就可以了

这里先记录师兄更改的代码（更改了三个文件）

* Drv_Bsp.c 注释了ADC\光流\数传\OPMV\GPS 添加了launchpad板载串口

```cpp
	//ADC初始化
//	Drv_AdcInit();
	//滴答时钟初始化
	SysTick_Init();	
	//串口初始化
	Drv_Uart1Init(115200);//徐 添加launchpad板载串口
//	Drv_Uart4Init(500000);	//接光流
//	Drv_Uart2Init(500000);	//接数传
//	Drv_Uart3Init(500000);  //接OPMV
//	Drv_GpsPin_Init();//
	
```

* Ano_Scheduler.h 取消了向任务调度执行函数传入的时间参数，取消了rate_hz的注释（但是这个好像没有用）

```cpp
/////////////////////////////////////////////-----old
typedef struct
{
	void(*task_func)(u32 dT_us);
//	u16 rate_hz;
	u32 interval_ticks;
	u32 last_run;
}sched_task_t;

/////////////////////////////////////////////-----new
typedef struct
{
	void(*task_func)(void);
	u16 rate_hz;
	u32 interval_ticks;
	u32 last_run;
}sched_task_t;
```

* Ano_Scheduler.c 

```cpp
//////////////////////////////////////////------ 根据上面的解释注释传感器时间片， 
/////////////////////////////////////////------- TODO 但是LED的程序还没有移植到一毫秒时间片 灯光驱动 LED_1ms_DRV();

	//查询1ms任务是否需要执行
//	if(lt0_run_flag!=0)
//	{
//		//
//		lt0_run_flag--;
//		Loop_Task_0();
//	}

//////////////////////////////////////////------ 添加了1ms时间片

	//任务n,    周期us,   上次时间us
	{Loop_Task_0 ,  1000,  0 },

///////////////////////////////////////// -------- 注释了50ms里面的三个任务，TODO 恒温控制（不能直接注释掉，否则开机过不了校准）?

static void Loop_Task_9(void)	//50ms执行一次
{
	//
	/*电压相关任务*/
//	Power_UpdateTask(50);
//	//恒温控制（不能直接注释掉，否则开机过不了校准）
//	Thermostatic_Ctrl_Task(50);
//	//	/*延时存储任务*/
//	Ano_Parame_Write_task(50);
}

。。。。。 // 应该是为了先测试传感器，还注释了一些（具体看log Ano_Scheduler.c）
```

#### 起飞程序

* 取消了对Ano_Scheduler.c的部分注释, 所以，主要是注释了数传和OPMV


* 在Ano_Power.c里面直接对电压赋值

```cpp
Plane_Votage = 15;//@徐 硬件原因直接赋值
```

* 在Ano_RC.c中可能需要根据遥控器更改通道值，这里没有改
```cpp
//		CH_N[1]=-CH_N[1];//@徐，遥控器俯仰通道反了，所以加个负号
```

#### 起飞程序1.1

Drv_PwmOut.c 更改了

```cpp
/////////////////////////////////////------ old pwm输出
	if(Motor == 0)
		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,tempval);
	if(Motor == 1)
		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,tempval);

/////////////////////////////////////------ new pwm输出的0，1号电机由4，5路输出， 注释了4，5，6，7路电机
	if(Motor == 0)
		ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_4,tempval);
	if(Motor == 1)
		ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,tempval);
	if(Motor == 2)	
		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,tempval);
	if(Motor == 3)	
		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,tempval);
```

Ano_RC.c 取消了注释

```cpp
		CH_N[1]=-CH_N[1];//@徐，遥控器俯仰通道反了，所以加个负号
```