//Ĭ�����ã�
#include "Drv_OpenMV.h"
#include "Ano_Math.h"
//�趨
#define OPMV_OFFLINE_TIME_MS  1000  //����

//ȫ�ֱ���
u16 offline_check_time;
u8 openmv_buf[20];
_openmv_data_st opmv;
/**********************************************************************************************************
*�� �� ��: OpenMV_Byte_Get
*����˵��: OpenMV�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
**********************************************************************************************************/
void OpenMV_Byte_Get(u8 bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	
	//
	openmv_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(1)//(bytedata==0x29)δȷ��
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(bytedata==0x05)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(bytedata==0x41 || bytedata==0x42)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		//
		len = bytedata;
		if(len<20)
		{
			rec_sta++;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+5))
	{
		//
		for(u8 i=0;i<len+5;i++)
		{
			check_val += openmv_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//�����ɹ�
			OpenMV_Data_Analysis(openmv_buf,len+6);
			//
			rec_sta=0;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else
	{
		//	
		rec_sta++;
	}
	
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Data_Analysis
*����˵��: OpenMV���ݽ���
*��    ��: �������ݣ��βΣ�������
*�� �� ֵ: ��
**********************************************************************************************************/
extern s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;
s16 angle_right = 0;
extern float my_speed, speed_set_tmp[2];
static s16 get_real(s16 angle)
{
	if(angle > 90)
		return angle - 180;
	else
		return angle;
}

static void OpenMV_Data_Analysis(u8 *buf_data,u8 len)
{
	static int i = 0;
	if(*(buf_data+3)==0x41)
	{
		i++;
		if(i%10 == 0)
		{
			opmv.lt.angle = 0;
			angle_right = 0;
			Program_Ctrl_User_Set_YAWdps(0);	
			// Program_Ctrl_User_Set_HXYcmps(0,0);
		}
		opmv.cb.color_flag = *(buf_data+5);
		opmv.cb.sta = *(buf_data+6);
		opmv.cb.pos_x = (s16)((*(buf_data+7)<<8)|*(buf_data+8));
		opmv.cb.pos_y = (s16)((*(buf_data+9)<<8)|*(buf_data+10));
	}
	if(*(buf_data+3)==0x42)
	{
		i = 0;
		angle_right = (s16)((*(buf_data+9)<<8)|*(buf_data+10));
		if(ref_height_used > 90)
		{				
			if(opmv.lt.angle!=0 && angle_right != 0 && ABS(get_real(angle_right) - get_real(opmv.lt.angle)) < 35)
			{
					Program_Ctrl_User_Set_YAWdps(get_real(angle_right*0.8 + 0.2*opmv.lt.angle)*1.2 \
																		+ (get_real(angle_right) - get_real(opmv.lt.angle))*0.12);	
			}
			// TODO 
//			if(angle_right - opmv.lt.angle > 40)
//				my_speed = -50;
//			else
//				my_speed = 50;
			if(ABS(get_real(angle_right)) > 10)
			{
				my_speed = 0;
				speed_set_tmp[X] = 0;
			}
			else
			{
				my_speed = 20;
			}
			opmv.lt.angle = angle_right;
		}
	}
	//
	OpenMV_Check_Reset();
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Offline_Check
*����˵��: OpenMV���߼�⣬�������Ӳ���Ƿ�����
*��    ��: ʱ�䣨���룩
*�� �� ֵ: ��
**********************************************************************************************************/
void OpenMV_Offline_Check(u8 dT_ms)
{
	if(offline_check_time<OPMV_OFFLINE_TIME_MS)
	{
		offline_check_time += dT_ms;
	}
	else
	{
		opmv.offline = 1;
	}
	
}

/**********************************************************************************************************
*�� �� ��: OpenMV_Check_Reset
*����˵��: OpenMV���߼�⸴λ��֤��û�е���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void OpenMV_Check_Reset()
{
	offline_check_time = 0;
	opmv.offline = 0;
}


