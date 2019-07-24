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

### 核心功能添加记录

* 1.0 数传功能

在Ano_Scheduler.c里面添加100ms执行函数发送数据，需要初始化串口Drv_Uart5。

```c
static void Loop_Task_10(void)	//100ms执行一次
{
	u8 _cnt = 0;
	u16 t_pit = (u16)(imu_data.pit*100+18000);
	u16 t_rol = (u16)(imu_data.rol*100+18000);
	u16 t_yaw = (u16)(imu_data.yaw*100+18000);
	my_data_to_send[_cnt++]=0xaa;
	my_data_to_send[_cnt++]=t_pit/256;
	my_data_to_send[_cnt++]=t_pit%256;
	my_data_to_send[_cnt++]=t_rol/256;
	my_data_to_send[_cnt++]=t_rol%256;
	my_data_to_send[_cnt++]=t_yaw/256;
	my_data_to_send[_cnt++]=t_yaw%256;
	my_data_to_send[_cnt++]=CH_N[CH_ROL]/256;
	my_data_to_send[_cnt++]=CH_N[CH_ROL]%256;
	my_data_to_send[_cnt++]=CH_N[CH_PIT]/256;
	my_data_to_send[_cnt++]=CH_N[CH_PIT]%256;
	my_data_to_send[_cnt++]=CH_N[CH_THR]/256;
	my_data_to_send[_cnt++]=CH_N[CH_THR]%256;
	my_data_to_send[_cnt++]=CH_N[CH_YAW]/256;
	my_data_to_send[_cnt++]=CH_N[CH_YAW]%256;
	my_data_to_send[_cnt++]=opmv.cb.pos_x/256;
	my_data_to_send[_cnt++]=opmv.cb.pos_x%256;
	my_data_to_send[_cnt++]=opmv.cb.pos_y/256;
	my_data_to_send[_cnt++]=opmv.cb.pos_y%256;
	my_data_to_send[_cnt++]=(int)(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0]*100)/256;
	my_data_to_send[_cnt++]=(int)(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0]*100)%256;
	my_data_to_send[_cnt++]=(int)(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1]*100)/256;
	my_data_to_send[_cnt++]=(int)(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1]*100)%256;
	my_data_to_send[_cnt++]=(int)(fs.speed_set_h[X]*100)/256;
	my_data_to_send[_cnt++]=(int)(fs.speed_set_h[X]*100)%256;
	my_data_to_send[_cnt++]=(int)(fs.speed_set_h[Y]*100)/256;
	my_data_to_send[_cnt++]=(int)(fs.speed_set_h[Y]*100)%256;
	my_data_to_send[_cnt++]=(int)(motor[0])/256;
	my_data_to_send[_cnt++]=(int)(motor[0])%256;
	my_data_to_send[_cnt++]=(int)(motor[1])/256;
	my_data_to_send[_cnt++]=(int)(motor[1])%256;
	my_data_to_send[_cnt++]=(int)(motor[2])/256;
	my_data_to_send[_cnt++]=(int)(motor[2])%256;
	my_data_to_send[_cnt++]=(int)(motor[3])/256;
	my_data_to_send[_cnt++]=(int)(motor[3])%256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_thr)/256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_thr)%256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_yaw)/256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_yaw)%256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_rol)/256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_rol)%256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_pit)/256;
	my_data_to_send[_cnt++]=(int)(mc.ct_val_pit)%256;
	my_data_to_send[_cnt++]=(int)(my_jig)/256;
	my_data_to_send[_cnt++]=(int)(my_jig)%256;
	u16 res = ref_height_used;
	my_data_to_send[_cnt++]=(int)(res)/256;
	my_data_to_send[_cnt++]=(int)(res)%256;
	my_data_to_send[_cnt++]=wcz_hei_fus.out;
	my_data_to_send[_cnt++]=0xfe;
	
	Drv_Uart5SendBuf(my_data_to_send, _cnt);

}
```

* 2.0 一键起飞降落

在Ano_FlightCtrl.c里面添加一键起飞降落程序，在1ms程序里面调用。

注意注释掉Ano_FlightCtrl.c里面的设置flag.taking_off = 1;的代码。

```c
static u8 FixHeight_flag=0;
void Fly_FixHeight()
{
	// 一键降落
	if(CH_N[7]<200&&FixHeight_flag==0)
	{
		one_key_land();
	//	flag.unlock_cmd = 0;
		Ano_Parame.set.auto_take_off_height=0;
		FixHeight_flag=1;
	}
	// 解锁
	if(CH_N[7]>400&&FixHeight_flag==1)
	{
		Ano_Parame.set.auto_take_off_height=160;
		Ano_Parame.set.auto_take_off_speed=200;

		one_key_take_off();
		FixHeight_flag=0;
	}
	// 急停
	if(CH_N[4]>400)
	{
		flag.unlock_cmd = 0;
	}
	// 解锁后起飞
	if(CH_N[5]>400&&FixHeight_flag==0)
	{
		flag.taking_off = 1;
	}
//	else if(CH_N[5]<200)
//	{
//		flag.unlock_cmd = 1;
//	}
}
```

* 3.0 启用openmv判断程序

Ano_OPMV_CBTracking_Ctrl.c里面取消对光流的要求
```c
////////////////////////////////////////////////////old
	if(switchs.of_flow_on && switchs.opmv_on)
////////////////////////////////////////////////////new
	if(switchs.opmv_on)
///////////////////////////////////////////////////2019/7/24 new 添加对高度的要求
	if(switchs.opmv_on && ref_height_used > 50)

```

Ano_FlightCtrl.c里面取消对LOC_HOLD模式的要求
```c
///////////////////////////////////////////////////old
	if(opmv.offline==0 && flag.flight_mode == LOC_HOLD)
///////////////////////////////////////////////////new
	if(opmv.offline==0)
```

* 4.0 添加激光测距，将原本光流的串口用于接受激光模块，气压计高度数据直接被激光数据替换

```c
	Drv_Uart5Init(115200);	// 自定义数传
	Drv_Uart4Init(115200);	// 接激光
	Drv_Uart2Init(500000);	// 接数传，未用
	Drv_Uart3Init(500000);  // 接OPMV
```

Drv_Uart.c里面直接替换UART4_IRQHandler(void)的数据
```c
	// AnoOF_GetOneByte(com_data);
	
	if(com_data==0x5a && my_flag == 0)
		my_flag = 1;
	else if(com_data==0x5a && my_flag == 1)
		my_flag = 2;
	else if(com_data==0x15 && my_flag == 2)
		my_flag = 3;
	else if(com_data==0x03 && my_flag == 3)
		my_flag = 4;
	else if(my_flag == 4)
	{
		my_jig = (com_data << 8);
		my_flag++;
	}
	else if(my_flag == 5)
	{
		my_jig |= (com_data);
		my_flag = 0;
	}		
	else
		my_flag = 0;
```

Ano_FlightDataCal.c里面直接设置高度数据
```c
	ref_height_used = my_jig/10.0;
```