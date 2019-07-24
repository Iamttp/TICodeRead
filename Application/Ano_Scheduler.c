/******************** (C) COPYRIGHT 2019 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "Drv_Bsp.h"
#include "Drv_icm20602.h"
#include "Ano_LED.h"
#include "Ano_FlightDataCal.h"
#include "Ano_Sensor_Basic.h"

#include "Drv_gps.h"
#include "Ano_DT.h"
#include "Ano_RC.h"
#include "Ano_Parameter.h"
#include "Drv_led.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "Ano_FlightCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_MagProcess.h"
#include "Ano_Power.h"
#include "Ano_OF.h"
#include "Drv_heating.h"
#include "Ano_FlyCtrl.h"
#include "Ano_UWB.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Drv_Uart.h"
#include "Ano_MotionCal.h"
#include "Ano_FlightDataCal.h"



#define CIRCLE_NUM 20
static u8 lt0_run_flag;
static u8 circle_cnt[2];
void INT_1ms_Task()
{	
//	if(fc_sta.start_ok == 0) return;
	
	//标记1ms执行
	lt0_run_flag ++;
	//灯光驱动
	LED_1ms_DRV();

	//循环计数
	circle_cnt[0] ++;
	//20次循环
	circle_cnt[0] %= CIRCLE_NUM;
	//
	if(!circle_cnt[0])
	{
		//
		
	}
}


static void Loop_Task_0(void)//1ms执行一次
{
	//	
	/*传感器数据读取*/
	Fc_Sensor_Get();
	/*惯性传感器数据准备*/
	Sensor_Data_Prepare(1);
	
	/*姿态解算更新*/
	IMU_Update_Task(1);
	
	/*获取WC_Z加速度*/
	WCZ_Acc_Get_Task();
	WCXY_Acc_Get_Task();
	
	/*飞行状态任务*/
	Flight_State_Task(1,CH_N);
	
	/*开关状态任务*/
	Swtich_State_Task(1);
	

	/*数传数据交换*/
	// ANO_DT_Data_Exchange();
    Fly_FixHeight();	
}

static void Loop_Task_1(void)	//2ms执行一次
{
//////////////////////////////////////////////////////////////////////	
//	float t1_dT_s;
//	t1_dT_s = (float)dT_us *1e-6f;
	//========================
	/*姿态角速度环控制*/
	Att_1level_Ctrl(2*1e-3f);
	
	/*电机输出控制*/
	Motor_Ctrl_Task(2);	

//////////////////////////////////////////////////////////////////////	
}

static void Loop_Task_2(void)	//6ms执行一次
{
//////////////////////////////////////////////////////////////////////	
//	float t2_dT_s;
//	t2_dT_s = (float)dT_us *1e-6f;
	//========================
	/*获取姿态角（ROLL PITCH YAW）*/
	calculate_RPY();
	
	/*姿态角度环控制*/
	Att_2level_Ctrl(6e-3f,CH_N);

	//
	
//////////////////////////////////////////////////////////////////////	
}


static void Loop_Task_5(void)	//11ms执行一次
{	
//////////////////////////////////////////////////////////////////////	
//	float t2_dT_s = (float)dT_us *1e-6f;//0.008f;//
	//========================
	/*遥控器数据处理*/
	RC_duty_task(11);
	
	/*飞行模式设置任务*/
	Flight_Mode_Set(11);
	

	
	/*高度数据融合任务*/
	WCZ_Fus_Task(11);
	GPS_Data_Processing_Task(11);
	
	/*高度速度环控制*/
	Alt_1level_Ctrl(11e-3f);
	
	/*高度环控制*/
	Alt_2level_Ctrl(11e-3f);
	
	/*--*/	
	AnoOF_DataAnl_Task(11);

	/*灯光控制*/	
	LED_Task2(11);


//////////////////////////////////////////////////////////////////////		
}



static void Loop_Task_8(void)	//20ms执行一次
{
	u8 dT_ms = 20;//(u8)(dT_us *1e-3f);
	//==========================
	//
	/*罗盘数据处理任务*/
	Mag_Update_Task(20);
	/*程序指令控制*/
	FlyCtrl_Task(20);
	/*--*/
	// Ano_UWB_Data_Calcu_Task(20);
	/*位置速度环控制*/
	Loc_1level_Ctrl(20,CH_N);
//	/*OPMV检测是否掉线*/
	OpenMV_Offline_Check(20);
//	/*OPMV色块追踪数据处理任务*/
	ANO_CBTracking_Task(20);
//	/*OPMV色块追踪控制任务*/	
	ANO_CBTracking_Ctrl_Task(20);
}

static u8 my_data_to_send[50];	//发送数据缓存
static void Loop_Task_9(void)	//50ms执行一次
{
	//
	/*电压相关任务*/
	Power_UpdateTask(50);
	//恒温控制（不能直接注释掉，否则开机过不了校准）
	Thermostatic_Ctrl_Task(50);
	//	/*延时存储任务*/
	Ano_Parame_Write_task(50);	
}

extern u16 my_jig;
extern s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;

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


//////////////////////////
//调度器程序
//////////////////////////


//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] = 
{
	//任务n,    周期us,   上次时间us
	{Loop_Task_0 ,  1000,  0 },
	{Loop_Task_1 ,  2000,  0 },
	{Loop_Task_2 ,  6000,  0 },
//	{Loop_Task_2 ,  2500,  0 },
//	{Loop_Task_3 ,  2500,  0 },
//	{Loop_Task_4 ,  2500,  0 },
	{Loop_Task_5 ,  11000,  0 },
//	{Loop_Task_6 ,  9090,  0 },
//	{Loop_Task_7 ,  9090,  0 },
	{Loop_Task_8 , 20000,  0 },
	{Loop_Task_9 , 50000,  0 },
	{Loop_Task_10,100000,  0 },
};

//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

u8 Main_Task(void)
{
	uint8_t index = 0;
	
	//查询1ms任务是否需要执行
//	if(lt0_run_flag!=0)
//	{
//		//
//		lt0_run_flag--;
//		Loop_Task_0();
//	}

	//循环判断其他所有线程任务，是否应该执行
	uint32_t time_now,delta_time_us;
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位US
		 time_now = GetSysRunTimeUs();//SysTick_GetTick();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(time_now - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			delta_time_us = (u32)(time_now - sched_tasks[index].last_run);

			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = time_now;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();

		}	 
	}
	
	return 0;
}

/******************* (C) COPYRIGHT 2019 ANO TECH *****END OF FILE************/
	

