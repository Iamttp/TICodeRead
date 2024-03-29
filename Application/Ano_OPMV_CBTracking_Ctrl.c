/*==========================================================================
 * 描述    ：对OPMV传回的数据进行处理，并解除因机体俯仰、横滚旋转而造成追踪
 目标坐标变化的耦合，也称作“旋转解耦”或“旋转补偿”。
 
 * 更新时间：2019-07-03 
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
============================================================================*/

//默认引用
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_Math.h"
#include "Ano_ProgramCtrl_User.h"
//
//数据接口定义：
//=========mapping===============
//需要引用的文件：
#include "ANO_IMU.h"
#include "Ano_OF.h"
#include "Ano_MotionCal.h"
//需要调用引用的外部变量：
#define IMU_ROL                 (imu_data.rol)     //横滚角
#define IMU_PIT                 (imu_data.pit)     //俯仰角
#define RELATIVE_HEIGHT_CM           (wcz_hei_fus.out)  //相对高度
#define VELOCITY_CMPS_X              (OF_DX2FIX)   //载体运动速度h_x
#define VELOCITY_CMPS_Y              (OF_DY2FIX)   //载体运动速度h_y

#define CBT_KP                  (0.66f)  //比例项
#define CBT_KD                  (0.00f)  //微分项
#define CBT_KF                  (0.50f)  //前馈项

//需要操作赋值的外部变量：


//===============================
//全局变量：
static u16 target_loss_hold_time;
_ano_opmv_cbt_ctrl_st ano_opmv_cbt_ctrl;
static float ref_carrier_velocity[2];
static float decou_pos_pixel_lpf[2][2];

//参数设定：
#define PIXELPDEG_X    2.4f  //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。//3.2f->(160*120,3.6mm)  2.4f->(160*120,2.8mm)
#define PIXELPDEG_Y    2.4f  //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。
#define CMPPIXEL_X     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。//目前粗略标定
#define CMPPIXEL_Y     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。
#define TLH_TIME       1000   //判断目标丢失的保持时间。



	
/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Task
*功能说明: 匿名科创色块跟踪任务
*参    数: 周期时间(ms)
*返 回 值: 无
**********************************************************************************************************/
void ANO_CBTracking_Task(u8 dT_ms)
{
	//跟踪数据旋转解耦合
	ANO_CBTracking_Decoupling(&dT_ms,IMU_ROL,IMU_PIT);
	//跟踪数据计算
	ANO_CBTracking_Calcu(&dT_ms,RELATIVE_HEIGHT_CM);
}

/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Decoupling
*功能说明: 匿名科创色块跟踪解耦合
*参    数: 周期时间(形参ms)，横滚角度，俯仰角度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_CBTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
	float dT_s = (*dT_ms) *1e-3f;

	//有识别到目标
	if(opmv.cb.sta != 0)//(opmv.cb.color_flag!=0)
	{
		//
		ano_opmv_cbt_ctrl.target_loss = 0;
		target_loss_hold_time = 0;
	}
	else
	{
		//延迟一定时间
		if(target_loss_hold_time<TLH_TIME)
		{
			target_loss_hold_time += *dT_ms;
		}
		else
		{
			//目标丢失标记置位
			ano_opmv_cbt_ctrl.target_loss = 1;
		}
	}
	//换到飞控坐标系
	ano_opmv_cbt_ctrl.opmv_pos[0] =  opmv.cb.pos_y;
	ano_opmv_cbt_ctrl.opmv_pos[1] = -opmv.cb.pos_x;
	//
	if(opmv.cb.sta != 0)
	{
		//更新姿态量对应的偏移量
		ano_opmv_cbt_ctrl.rp2pixel_val[0] = -PIXELPDEG_X *pit_degs;
		ano_opmv_cbt_ctrl.rp2pixel_val[1] = -PIXELPDEG_Y *rol_degs;
		ano_opmv_cbt_ctrl.rp2pixel_val[0] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[0],-60,60);//高度120pixel
		ano_opmv_cbt_ctrl.rp2pixel_val[1] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[1],-80,80);//宽度160pixel
		//赋值参考的载体运动速度
		ref_carrier_velocity[0] = VELOCITY_CMPS_X;
		ref_carrier_velocity[1] = VELOCITY_CMPS_Y;		
	}
	else
	{
		//姿态量对应偏移量保持不变
		//参考的载体运动速度复位0
		ref_carrier_velocity[0] = 0;
		ref_carrier_velocity[1] = 0;
	}

	//
	if(ano_opmv_cbt_ctrl.target_loss==0) //有效，没丢失
	{
		//得到平移偏移量，并低通滤波
		decou_pos_pixel_lpf[0][0] += 0.2f *((ano_opmv_cbt_ctrl.opmv_pos[0] - ano_opmv_cbt_ctrl.rp2pixel_val[0]) - decou_pos_pixel_lpf[0][0]);
		decou_pos_pixel_lpf[0][1] += 0.2f *((ano_opmv_cbt_ctrl.opmv_pos[1] - ano_opmv_cbt_ctrl.rp2pixel_val[1]) - decou_pos_pixel_lpf[0][1]);
		//再滤一次
		decou_pos_pixel_lpf[1][0] += 0.2f *(decou_pos_pixel_lpf[0][0] - decou_pos_pixel_lpf[1][0]);
		decou_pos_pixel_lpf[1][1] += 0.2f *(decou_pos_pixel_lpf[0][1] - decou_pos_pixel_lpf[1][1]);
		//赋值
		ano_opmv_cbt_ctrl.decou_pos_pixel[0] = decou_pos_pixel_lpf[1][0];
		ano_opmv_cbt_ctrl.decou_pos_pixel[1] = decou_pos_pixel_lpf[1][1];
	}
	else //丢失目标
	{
//		ano_opmv_cbt_ctrl.decou_pos_pixel[0] = ano_opmv_cbt_ctrl.decou_pos_pixel[1] = 0;
		//低通复位到0
		LPF_1_(0.2f,dT_s,0,ano_opmv_cbt_ctrl.decou_pos_pixel[0]);
		LPF_1_(0.2f,dT_s,0,ano_opmv_cbt_ctrl.decou_pos_pixel[1]);

	}
	//

}

/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Calcu
*功能说明: 匿名科创色块跟踪计算处理
*参    数: 周期时间(形参ms)，相对高度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_CBTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
	static float relative_height_cm_valid;
	static float g_pos_err_old[2];
	//相对高度赋值
	if(relative_height_cm<500)
	{
		relative_height_cm_valid = relative_height_cm;
	}
	else
	{
		//null
	}
	//记录历史值
	g_pos_err_old[0] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0];
	g_pos_err_old[1] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1];
	//得到地面偏差，单位厘米
	ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] = CMPPIXEL_X *relative_height_cm_valid *ano_opmv_cbt_ctrl.decou_pos_pixel[0];
	ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] = CMPPIXEL_Y *relative_height_cm_valid *ano_opmv_cbt_ctrl.decou_pos_pixel[1];
	//计算微分偏差，单位厘米每秒
	ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] - g_pos_err_old[0])*(1000/(*dT_ms));
	ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] - g_pos_err_old[1])*(1000/(*dT_ms));
	//计算目标的地面速度，单位厘米每秒
	ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0] = (ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] + ref_carrier_velocity[0]) ;
	ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1] = (ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] + ref_carrier_velocity[1]) ;
}

/**********************************************************************************************************
*函 数 名: ANO_CBTracking_Ctrl_Task
*功能说明: 匿名科创色块跟踪控制任务
*参    数: 周期时间(ms)
*返 回 值: 无
**********************************************************************************************************/
extern s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;

void ANO_CBTracking_Ctrl_Task(u8 dT_ms)
{
	//开启控制的条件，可以自己修改
	if(switchs.opmv_on && ref_height_used > 50)
	{
		//距离偏差PD控制和速度前馈
		ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0]\
		= CBT_KF *ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0]\
		+ CBT_KP *(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0])\
		+ CBT_KD *ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0];
		
		ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]\
		= CBT_KF *ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1]\
		+ CBT_KP *(ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1])\
		+ CBT_KD *ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1];
	}
	else
	{
		ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0] = ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1] = 0;
	}
	//调用用户程控函数赋值控制量
	Program_Ctrl_User_Set_HXYcmps(ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0],ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]);
}	



