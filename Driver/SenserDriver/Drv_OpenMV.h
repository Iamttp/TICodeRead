#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"
#include "Ano_ProgramCtrl_User.h"

//==定义
typedef struct
{
	//
	u8 color_flag;
	u8 sta;
	s16 pos_x;
	s16 pos_y;

}_openmv_color_block_st;

typedef struct
{
	//
	u8 angle;
	u8 sta;
	s16 pos_x;
	s16 pos_y;

}_openmv_line_tracking_st;

typedef struct
{
	u8 offline;
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
}_openmv_data_st;
//==数据声明
extern _openmv_data_st opmv;

//==函数声明

//static
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len);
static void OpenMV_Check_Reset(void);

//public
void OpenMV_Offline_Check(u8 dT_ms);
void OpenMV_Byte_Get(u8 bytedata);


#endif

