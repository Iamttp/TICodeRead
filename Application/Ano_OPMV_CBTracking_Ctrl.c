/*==========================================================================
 * ����    ����OPMV���ص����ݽ��д������������帩���������ת�����׷��
 Ŀ������仯����ϣ�Ҳ��������ת�������ת��������
 
 * ����ʱ�䣺2019-07-03 
 * ����		 �������ƴ�-Jyoun
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
============================================================================*/

//Ĭ������
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Drv_OpenMV.h"
#include "Ano_Math.h"
#include "Ano_ProgramCtrl_User.h"
//
//���ݽӿڶ��壺
//=========mapping===============
//��Ҫ���õ��ļ���
#include "ANO_IMU.h"
#include "Ano_OF.h"
#include "Ano_MotionCal.h"
//��Ҫ�������õ��ⲿ������
#define IMU_ROL                 (imu_data.rol)     //�����
#define IMU_PIT                 (imu_data.pit)     //������
#define RELATIVE_HEIGHT_CM           (wcz_hei_fus.out)  //��Ը߶�
#define VELOCITY_CMPS_X              (OF_DX2FIX)   //�����˶��ٶ�h_x
#define VELOCITY_CMPS_Y              (OF_DY2FIX)   //�����˶��ٶ�h_y

#define CBT_KP                  (0.66f)  //������
#define CBT_KD                  (0.00f)  //΢����
#define CBT_KF                  (0.50f)  //ǰ����

//��Ҫ������ֵ���ⲿ������


//===============================
//ȫ�ֱ�����
static u16 target_loss_hold_time;
_ano_opmv_cbt_ctrl_st ano_opmv_cbt_ctrl;
static float ref_carrier_velocity[2];
static float decou_pos_pixel_lpf[2][2];

//�����趨��
#define PIXELPDEG_X    2.4f  //ÿ1�Ƕȶ�Ӧ�����ظ�������ֱ��ʺͽ����йأ���Ҫ���Ա궨��//3.2f->(160*120,3.6mm)  2.4f->(160*120,2.8mm)
#define PIXELPDEG_Y    2.4f  //ÿ1�Ƕȶ�Ӧ�����ظ�������ֱ��ʺͽ����йأ���Ҫ���Ա궨��
#define CMPPIXEL_X     0.01f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��//Ŀǰ���Ա궨
#define CMPPIXEL_Y     0.01f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��
#define TLH_TIME       1000   //�ж�Ŀ�궪ʧ�ı���ʱ�䡣



	
/**********************************************************************************************************
*�� �� ��: ANO_CBTracking_Task
*����˵��: �����ƴ�ɫ���������
*��    ��: ����ʱ��(ms)
*�� �� ֵ: ��
**********************************************************************************************************/
void ANO_CBTracking_Task(u8 dT_ms)
{
	//����������ת�����
	ANO_CBTracking_Decoupling(&dT_ms,IMU_ROL,IMU_PIT);
	//�������ݼ���
	ANO_CBTracking_Calcu(&dT_ms,RELATIVE_HEIGHT_CM);
}

/**********************************************************************************************************
*�� �� ��: ANO_CBTracking_Decoupling
*����˵��: �����ƴ�ɫ����ٽ����
*��    ��: ����ʱ��(�β�ms)������Ƕȣ������Ƕ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void ANO_CBTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs)
{
	float dT_s = (*dT_ms) *1e-3f;

	//��ʶ��Ŀ��
	if(opmv.cb.sta != 0)//(opmv.cb.color_flag!=0)
	{
		//
		ano_opmv_cbt_ctrl.target_loss = 0;
		target_loss_hold_time = 0;
	}
	else
	{
		//�ӳ�һ��ʱ��
		if(target_loss_hold_time<TLH_TIME)
		{
			target_loss_hold_time += *dT_ms;
		}
		else
		{
			//Ŀ�궪ʧ�����λ
			ano_opmv_cbt_ctrl.target_loss = 1;
		}
	}
	//�����ɿ�����ϵ
	ano_opmv_cbt_ctrl.opmv_pos[0] =  opmv.cb.pos_y;
	ano_opmv_cbt_ctrl.opmv_pos[1] = -opmv.cb.pos_x;
	//
	if(opmv.cb.sta != 0)
	{
		//������̬����Ӧ��ƫ����
		ano_opmv_cbt_ctrl.rp2pixel_val[0] = -PIXELPDEG_X *pit_degs;
		ano_opmv_cbt_ctrl.rp2pixel_val[1] = -PIXELPDEG_Y *rol_degs;
		ano_opmv_cbt_ctrl.rp2pixel_val[0] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[0],-60,60);//�߶�120pixel
		ano_opmv_cbt_ctrl.rp2pixel_val[1] = LIMIT(ano_opmv_cbt_ctrl.rp2pixel_val[1],-80,80);//���160pixel
		//��ֵ�ο��������˶��ٶ�
		ref_carrier_velocity[0] = VELOCITY_CMPS_X;
		ref_carrier_velocity[1] = VELOCITY_CMPS_Y;		
	}
	else
	{
		//��̬����Ӧƫ�������ֲ���
		//�ο��������˶��ٶȸ�λ0
		ref_carrier_velocity[0] = 0;
		ref_carrier_velocity[1] = 0;
	}

	//
	if(ano_opmv_cbt_ctrl.target_loss==0) //��Ч��û��ʧ
	{
		//�õ�ƽ��ƫ����������ͨ�˲�
		decou_pos_pixel_lpf[0][0] += 0.2f *((ano_opmv_cbt_ctrl.opmv_pos[0] - ano_opmv_cbt_ctrl.rp2pixel_val[0]) - decou_pos_pixel_lpf[0][0]);
		decou_pos_pixel_lpf[0][1] += 0.2f *((ano_opmv_cbt_ctrl.opmv_pos[1] - ano_opmv_cbt_ctrl.rp2pixel_val[1]) - decou_pos_pixel_lpf[0][1]);
		//����һ��
		decou_pos_pixel_lpf[1][0] += 0.2f *(decou_pos_pixel_lpf[0][0] - decou_pos_pixel_lpf[1][0]);
		decou_pos_pixel_lpf[1][1] += 0.2f *(decou_pos_pixel_lpf[0][1] - decou_pos_pixel_lpf[1][1]);
		//��ֵ
		ano_opmv_cbt_ctrl.decou_pos_pixel[0] = decou_pos_pixel_lpf[1][0];
		ano_opmv_cbt_ctrl.decou_pos_pixel[1] = decou_pos_pixel_lpf[1][1];
	}
	else //��ʧĿ��
	{
//		ano_opmv_cbt_ctrl.decou_pos_pixel[0] = ano_opmv_cbt_ctrl.decou_pos_pixel[1] = 0;
		//��ͨ��λ��0
		LPF_1_(0.2f,dT_s,0,ano_opmv_cbt_ctrl.decou_pos_pixel[0]);
		LPF_1_(0.2f,dT_s,0,ano_opmv_cbt_ctrl.decou_pos_pixel[1]);

	}
	//

}

/**********************************************************************************************************
*�� �� ��: ANO_CBTracking_Calcu
*����˵��: �����ƴ�ɫ����ټ��㴦��
*��    ��: ����ʱ��(�β�ms)����Ը߶�
*�� �� ֵ: ��
**********************************************************************************************************/
static void ANO_CBTracking_Calcu(u8 *dT_ms,s32 relative_height_cm)
{
	static float relative_height_cm_valid;
	static float g_pos_err_old[2];
	//��Ը߶ȸ�ֵ
	if(relative_height_cm<500)
	{
		relative_height_cm_valid = relative_height_cm;
	}
	else
	{
		//null
	}
	//��¼��ʷֵ
	g_pos_err_old[0] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0];
	g_pos_err_old[1] = ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1];
	//�õ�����ƫ���λ����
	ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] = CMPPIXEL_X *relative_height_cm_valid *ano_opmv_cbt_ctrl.decou_pos_pixel[0];
	ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] = CMPPIXEL_Y *relative_height_cm_valid *ano_opmv_cbt_ctrl.decou_pos_pixel[1];
	//����΢��ƫ���λ����ÿ��
	ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[0] - g_pos_err_old[0])*(1000/(*dT_ms));
	ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] = (ano_opmv_cbt_ctrl.ground_pos_err_h_cm[1] - g_pos_err_old[1])*(1000/(*dT_ms));
	//����Ŀ��ĵ����ٶȣ���λ����ÿ��
	ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[0] = (ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[0] + ref_carrier_velocity[0]) ;
	ano_opmv_cbt_ctrl.target_gnd_velocity_cmps[1] = (ano_opmv_cbt_ctrl.ground_pos_err_d_h_cmps[1] + ref_carrier_velocity[1]) ;
}

/**********************************************************************************************************
*�� �� ��: ANO_CBTracking_Ctrl_Task
*����˵��: �����ƴ�ɫ����ٿ�������
*��    ��: ����ʱ��(ms)
*�� �� ֵ: ��
**********************************************************************************************************/
extern s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;

void ANO_CBTracking_Ctrl_Task(u8 dT_ms)
{
	//�������Ƶ������������Լ��޸�
	if(switchs.opmv_on && ref_height_used > 50)
	{
		//����ƫ��PD���ƺ��ٶ�ǰ��
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
	//�����û��̿غ�����ֵ������
	Program_Ctrl_User_Set_HXYcmps(ano_opmv_cbt_ctrl.exp_velocity_h_cmps[0],ano_opmv_cbt_ctrl.exp_velocity_h_cmps[1]);
}	



