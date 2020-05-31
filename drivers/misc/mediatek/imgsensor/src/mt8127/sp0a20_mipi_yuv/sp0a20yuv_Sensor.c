/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   Sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/



/*#####################################################


  superpix    sensor   30m  SP0A20 .   sensorID = 0X2b       SLAVE ADDR= 0X42 


#####################################################*/


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "sp0a20yuv_Sensor.h"
#include "sp0a20yuv_Camera_Sensor_para.h"
#include "sp0a20yuv_CameraCustomized.h"

static MSDK_SENSOR_CONFIG_STRUCT SP0A20SensorConfigData;
static struct SP0A20_Sensor_Struct SP0A20_Sensor_Driver;




#define SP0A20YUV_DEBUG
#ifdef SP0A20YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif


#define __SENSOR_CONTROL__
#ifdef __SENSOR_CONTROL__
#define CAMERA_CONTROL_FLOW(para1,para2) printk("[%s:%d]::para1=0x%x,para1=0x%x\n\n",__FUNCTION__,__LINE__,para1,para2)
#else
#define CAMERA_CONTROL_FLOW(para1, para2)
#endif


kal_uint8 isBanding = 1; // 0: 50hz  1:60hz


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*************************************************************************
 * FUNCTION
 *    SP0A20_write_cmos_sensor
 *
 * DESCRIPTION
 *    This function wirte data to CMOS sensor through I2C
 *
 * PARAMETERS
 *    addr: the 16bit address of register
 *    para: the 8bit value of register
 *
 * RETURNS
 *    None
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static void SP0A20_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
	kal_uint8 out_buff[2];

	out_buff[0] = addr;
	out_buff[1] = para;

	iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), SP0A20_WRITE_ID); 

#if (defined(__SP0A20_DEBUG_TRACE__))
	if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
 * FUNCTION
 *    SP0A20_read_cmos_sensor
 *
 * DESCRIPTION
 *    This function read data from CMOS sensor through I2C.
 *
 * PARAMETERS
 *    addr: the 16bit address of register
 *
 * RETURNS
 *    8bit data read through I2C
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static kal_uint8 SP0A20_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint8 in_buff[1] = {0xFF};
	kal_uint8 out_buff[1];

	out_buff[0] = addr;

	if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), SP0A20_WRITE_ID)) {
		SENSORDB("ERROR: SP0A20_read_cmos_sensor \n");
	}

#if (defined(__SP0A20_DEBUG_TRACE__))
	if (size != rt) printk("I2C read %x error\n", addr);
#endif

	return in_buff[0];
}

//#define DEBUG_SENSOR 	//T-card 调试开关

#ifdef DEBUG_SENSOR

kal_uint8 fromsd = 0;
//kal_uint16 SP0A20_write_cmos_sensor(kal_uint8 addr, kal_uint8 para);

#define SP0A20_OP_CODE_INI		0x00		/* Initial value. */
#define SP0A20_OP_CODE_REG		0x01		/* Register */
#define SP0A20_OP_CODE_DLY		0x02		/* Delay */
#define SP0A20_OP_CODE_END		0x03		/* End of initial setting. */


typedef struct
{
	u16 init_reg;
	u16 init_val;	/* Save the register value and delay tick */
	u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
} SP0A20_initial_set_struct;

SP0A20_initial_set_struct SP0A20_Init_Reg[1000];

u32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
				(base==16 && *nptr>='a' && *nptr<='f') || 
				(base>=10 && *nptr>='0' && *nptr<='9') ||
				(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

u8 SP0A20_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/storage/sdcard1/sp0a20_sd.dat", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		printk("create file error 0\n");
		fp = filp_open("/storage/sdcard0/sp0a20_sd.dat", O_RDONLY , 0); 
		if (IS_ERR(fp)) { 
			printk("create file error 1\n"); 
			return 2;//-1; 
		}
	} 
	else
		printk("SP0A20_Initialize_from_T_Flash Open File Success\n");

	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	filp_close(fp, NULL); 
	set_fs(fs);



	printk("1\n");

	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_REG;

			SP0A20_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			SP0A20_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_DLY;

			SP0A20_Init_Reg[i].init_reg = 0xFF;
			SP0A20_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}
	printk("2\n");
	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	SP0A20_Init_Reg[i].op_code = SP0A20_OP_CODE_END;
	SP0A20_Init_Reg[i].init_reg = 0xFF;
	SP0A20_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
	//printk(" %x  ==  %x\n",SP0A20_Init_Reg[j].init_reg, SP0A20_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
#if 1
	for (j=0; j<i; j++)
	{
		if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_DLY)
		{
			msleep(SP0A20_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP0A20_Init_Reg[j].op_code == SP0A20_OP_CODE_REG)
		{

			SP0A20_write_cmos_sensor((kal_uint8)SP0A20_Init_Reg[j].init_reg, (kal_uint8)SP0A20_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif

	printk("3\n");
	return 1;	
}

#endif



static void SP0A20_Set_Dummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
}   /*  SP0A20_Set_Dummy    */


/*************************************************************************
 * FUNCTION
 *	SP0A20_NightMode
 *
 * DESCRIPTION
 *	This function night mode of SP0A20.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED   ///070312101020  
 *
 *************************************************************************/
static void SP0A20_night_mode(kal_bool bEnable)
{
	// kal_uint8 temp = SP0A20_read_cmos_sensor(0x3B);



	if (!SP0A20_Sensor_Driver.MODE_CAPTURE) { 
		if(bEnable)//night mode
		{ 
			SP0A20_Sensor_Driver.bNight_mode = KAL_TRUE;
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0xcd,0x20);
			SP0A20_write_cmos_sensor(0xce,0x1f);

			if(SP0A20_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
			{
				if(isBanding== 0)
				{
					printk("video 50Hz night\n");	
#if 0
					//Video record night 24M 50hz 10.02-10fps maxgain				                     
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0x50);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xf4);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x4b);
					SP0A20_write_cmos_sensor(0x02 , 0x0e);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x4b);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x1a);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x1a);
					SP0A20_write_cmos_sensor(0xd1 , 0x04);
#else//Video record night 24M 50hz 8-8fps maxgain	
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x00);
					SP0A20_write_cmos_sensor(0x04,0x50);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x02);
					SP0A20_write_cmos_sensor(0x0a,0xf4);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x4b);
					SP0A20_write_cmos_sensor(0x02,0x0e);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x4b);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0x1a);
					SP0A20_write_cmos_sensor(0xbf,0x04);
					SP0A20_write_cmos_sensor(0xd0,0x1a);
					SP0A20_write_cmos_sensor(0xd1,0x04);
#endif

					//dbg_print(" video 50Hz night\r\n");
				}
				else if(isBanding == 1)
				{
#if 0
					//Video record night 24M 60Hz 10.1002-10FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0xfc);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x06);
					SP0A20_write_cmos_sensor(0x0a , 0x01);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x2a);
					SP0A20_write_cmos_sensor(0x02 , 0x0c);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x2a);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xf8);
					SP0A20_write_cmos_sensor(0xbf , 0x01);
					SP0A20_write_cmos_sensor(0xd0 , 0xf8);
					SP0A20_write_cmos_sensor(0xd1 , 0x01);
#else//Video record night 24M 60hz 8-8fps maxgain	
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x00);
					SP0A20_write_cmos_sensor(0x04,0xcc);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x08);
					SP0A20_write_cmos_sensor(0x0a,0x31);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x22);
					SP0A20_write_cmos_sensor(0x02,0x0f);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x22);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0xfe);
					SP0A20_write_cmos_sensor(0xbf,0x01);
					SP0A20_write_cmos_sensor(0xd0,0xfe);
					SP0A20_write_cmos_sensor(0xd1,0x01);
#endif

					printk(" video 60Hz night\r\n");
				}
			}	
			else 
			{
				//	dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
				if(isBanding== 0)
				{
					//capture preview night 24M 50hz 20.0401-6fps maxgain:	 
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0x50);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xf4);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x4b);
					SP0A20_write_cmos_sensor(0x02 , 0x0e);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x4b);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x1a);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x1a);
					SP0A20_write_cmos_sensor(0xd1 , 0x04);

					printk(" priview 50Hz night\r\n");	
				}  
				else if(isBanding== 1)
				{
					//capture preview night 24M 60hz 20.2004-6FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x7a);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xe7);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x3f);
					SP0A20_write_cmos_sensor(0x02 , 0x11);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x3f);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x2f);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x2f);

					printk(" priview 60Hz night\r\n");	
				}
			} 		
		}
		else    // daylight mode
		{
			SP0A20_Sensor_Driver.bNight_mode = KAL_FALSE;
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0xcd,0x10);
			SP0A20_write_cmos_sensor(0xce,0x1f);  
			if(SP0A20_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
			{
				//dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
				if(isBanding== 0)
				{
#if 1
					//Video record daylight 24M 50hz 15.0301-15FPS maxgain:                     
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0x50);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xf4);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x4b);
					SP0A20_write_cmos_sensor(0x02 , 0x0e);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x4b);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x1a);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x1a);
					SP0A20_write_cmos_sensor(0xd1 , 0x04);
#else//Video record night 24M 50hz 8-8fps maxgain	
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x00);
					SP0A20_write_cmos_sensor(0x04,0x50);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x02);
					SP0A20_write_cmos_sensor(0x0a,0xf4);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x4b);
					SP0A20_write_cmos_sensor(0x02,0x0e);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x4b);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0x1a);
					SP0A20_write_cmos_sensor(0xbf,0x04);
					SP0A20_write_cmos_sensor(0xd0,0x1a);
					SP0A20_write_cmos_sensor(0xd1,0x04);
#endif


					printk(" video 50Hz normal\r\n");				
				}
				else if(isBanding == 1)
				{
#if 1
					//Video record daylight 24M 60Hz 15.1503-15FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x7a);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xe7);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x3f);
					SP0A20_write_cmos_sensor(0x02 , 0x08);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x3f);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0xf8);
					SP0A20_write_cmos_sensor(0xbf , 0x01);
					SP0A20_write_cmos_sensor(0xd0 , 0xf8);
					SP0A20_write_cmos_sensor(0xd1 , 0x01);
#else//Video record night 24M 60hz 8-8fps maxgain	
					SP0A20_write_cmos_sensor(0xfd,0x00);
					SP0A20_write_cmos_sensor(0x03,0x00);
					SP0A20_write_cmos_sensor(0x04,0xcc);
					SP0A20_write_cmos_sensor(0x05,0x00);
					SP0A20_write_cmos_sensor(0x06,0x00);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x00);
					SP0A20_write_cmos_sensor(0x09,0x08);
					SP0A20_write_cmos_sensor(0x0a,0x31);
					SP0A20_write_cmos_sensor(0xfd,0x01);
					SP0A20_write_cmos_sensor(0xf0,0x00);
					SP0A20_write_cmos_sensor(0xf7,0x22);
					SP0A20_write_cmos_sensor(0x02,0x0f);
					SP0A20_write_cmos_sensor(0x03,0x01);
					SP0A20_write_cmos_sensor(0x06,0x22);
					SP0A20_write_cmos_sensor(0x07,0x00);
					SP0A20_write_cmos_sensor(0x08,0x01);
					SP0A20_write_cmos_sensor(0x09,0x00);
					SP0A20_write_cmos_sensor(0xfd,0x02);
					SP0A20_write_cmos_sensor(0xbe,0xfe);
					SP0A20_write_cmos_sensor(0xbf,0x01);
					SP0A20_write_cmos_sensor(0xd0,0xfe);
					SP0A20_write_cmos_sensor(0xd1,0x01);
#endif

					printk(" video 60Hz normal\r\n");	
				}
			}
			else 
			{
				//	dbg_print(" SP0A20_banding=%x\r\n",SP0A20_banding);
				if(isBanding== 0)
				{
#if 1
					//capture preview daylight 24M 50hz 15-7FPS maxgain:   
						//ae setting 24M 7-15fps
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0x50);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xf4);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x4b);
					SP0A20_write_cmos_sensor(0x02 , 0x0e);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x4b);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x1a);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x1a);
					SP0A20_write_cmos_sensor(0xd1 , 0x04);
#else
					//capture preview daylight 24M 50hz 20.0401-6fps maxgain:	 
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x00);
					SP0A20_write_cmos_sensor(0x04 , 0x50);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xf4);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x4b);
					SP0A20_write_cmos_sensor(0x02 , 0x0e);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x4b);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x1a);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x1a);
					SP0A20_write_cmos_sensor(0xd1 , 0x04);
#endif


					printk(" priview 50Hz normal\r\n");
				}
				else if(isBanding== 1)
				{
#if 1
					//capture preview daylight 24M 60hz 15-7FPS maxgain                         					  
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0x7a);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x02);
					SP0A20_write_cmos_sensor(0x0a , 0xe7);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x3f);
					SP0A20_write_cmos_sensor(0x02 , 0x11);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x3f);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x2f);
					SP0A20_write_cmos_sensor(0xbf , 0x04);
					SP0A20_write_cmos_sensor(0xd0 , 0x2f);
					SP0A20_write_cmos_sensor(0xd1 , 0x04);
#else
					//capture preview daylight 24M 60hz 20.2004-6FPS maxgain:
					SP0A20_write_cmos_sensor(0xfd , 0x00);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x04 , 0xf8);
					SP0A20_write_cmos_sensor(0x05 , 0x00);
					SP0A20_write_cmos_sensor(0x06 , 0x00);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x00);
					SP0A20_write_cmos_sensor(0x09 , 0x01);
					SP0A20_write_cmos_sensor(0x0a , 0x5a);
					SP0A20_write_cmos_sensor(0xfd , 0x01);
					SP0A20_write_cmos_sensor(0xf0 , 0x00);
					SP0A20_write_cmos_sensor(0xf7 , 0x54);
					SP0A20_write_cmos_sensor(0x02 , 0x14);
					SP0A20_write_cmos_sensor(0x03 , 0x01);
					SP0A20_write_cmos_sensor(0x06 , 0x54);
					SP0A20_write_cmos_sensor(0x07 , 0x00);
					SP0A20_write_cmos_sensor(0x08 , 0x01);
					SP0A20_write_cmos_sensor(0x09 , 0x00);
					SP0A20_write_cmos_sensor(0xfd , 0x02);
					SP0A20_write_cmos_sensor(0xbe , 0x90);
					SP0A20_write_cmos_sensor(0xbf , 0x06);
					SP0A20_write_cmos_sensor(0xd0 , 0x90);
					SP0A20_write_cmos_sensor(0xd1 , 0x06);
#endif

					printk(" priview 60Hz normal\r\n");
				}
			}

		}  
	}
}	/*	SP0A20_NightMode	*/

static void SP0A20_Sensor_Driver_Init(void)
{
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x02);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x0c,0x00);
	SP0A20_write_cmos_sensor(0x92,0x11);
	SP0A20_write_cmos_sensor(0x99,0x05);
	SP0A20_write_cmos_sensor(0x1b,0x27);
	SP0A20_write_cmos_sensor(0x12,0x02);
	SP0A20_write_cmos_sensor(0x13,0x2f);
	SP0A20_write_cmos_sensor(0x6d,0x32);
	SP0A20_write_cmos_sensor(0x6c,0x32);
	SP0A20_write_cmos_sensor(0x6f,0x33);
	SP0A20_write_cmos_sensor(0x6e,0x34);
	SP0A20_write_cmos_sensor(0x99,0x04);
	SP0A20_write_cmos_sensor(0x16,0x38);
	SP0A20_write_cmos_sensor(0x17,0x38);
	SP0A20_write_cmos_sensor(0x70,0x3a);
	SP0A20_write_cmos_sensor(0x14,0x02);
	SP0A20_write_cmos_sensor(0x15,0x20);
	SP0A20_write_cmos_sensor(0x71,0x23);
	SP0A20_write_cmos_sensor(0x69,0x25);
	SP0A20_write_cmos_sensor(0x6a,0x1a);
	SP0A20_write_cmos_sensor(0x72,0x1c);
	SP0A20_write_cmos_sensor(0x75,0x1e);
	SP0A20_write_cmos_sensor(0x73,0x3c);
	SP0A20_write_cmos_sensor(0x74,0x21);
	SP0A20_write_cmos_sensor(0x79,0x00);
	SP0A20_write_cmos_sensor(0x77,0x10);
	SP0A20_write_cmos_sensor(0x1a,0x4d);
	SP0A20_write_cmos_sensor(0x1c,0x07);
	SP0A20_write_cmos_sensor(0x1e,0x15);
	SP0A20_write_cmos_sensor(0x21,0x08);
	SP0A20_write_cmos_sensor(0x22,0x28);
	SP0A20_write_cmos_sensor(0x26,0x66);
	SP0A20_write_cmos_sensor(0x28,0x0b);
	SP0A20_write_cmos_sensor(0x37,0x4a);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x01,0x80);
	SP0A20_write_cmos_sensor(0x52,0x10);
	SP0A20_write_cmos_sensor(0x54,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x41,0x00);
	SP0A20_write_cmos_sensor(0x42,0x00);
	SP0A20_write_cmos_sensor(0x43,0x00);
	SP0A20_write_cmos_sensor(0x44,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x00);
SP0A20_write_cmos_sensor(0x03,0x00);
SP0A20_write_cmos_sensor(0x04,0x50);
	SP0A20_write_cmos_sensor(0x05,0x00);
	SP0A20_write_cmos_sensor(0x06,0x00);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x00);
	SP0A20_write_cmos_sensor(0x09,0x02);
	SP0A20_write_cmos_sensor(0x0a,0xf4);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xf0,0x00);
	SP0A20_write_cmos_sensor(0xf7,0x4b);
	SP0A20_write_cmos_sensor(0x02,0x0e);
	SP0A20_write_cmos_sensor(0x03,0x01);
	SP0A20_write_cmos_sensor(0x06,0x4b);
	SP0A20_write_cmos_sensor(0x07,0x00);
	SP0A20_write_cmos_sensor(0x08,0x01);
	SP0A20_write_cmos_sensor(0x09,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xbe,0x1a);
	SP0A20_write_cmos_sensor(0xbf,0x04);
	SP0A20_write_cmos_sensor(0xd0,0x1a);
	SP0A20_write_cmos_sensor(0xd1,0x04);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x5a,0x40);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xbc,0x70);
	SP0A20_write_cmos_sensor(0xbd,0x50);
	SP0A20_write_cmos_sensor(0xb8,0x66);
	SP0A20_write_cmos_sensor(0xb9,0x8f);
	SP0A20_write_cmos_sensor(0xba,0x30);
	SP0A20_write_cmos_sensor(0xbb,0x45);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xe0,0x50);
	SP0A20_write_cmos_sensor(0xe1,0x3e);
	SP0A20_write_cmos_sensor(0xe2,0x36);
	SP0A20_write_cmos_sensor(0xe3,0x30);
	SP0A20_write_cmos_sensor(0xe4,0x30);
	SP0A20_write_cmos_sensor(0xe5,0x2e);
	SP0A20_write_cmos_sensor(0xe6,0x2e);
	SP0A20_write_cmos_sensor(0xe7,0x2c);
	SP0A20_write_cmos_sensor(0xe8,0x2c);
	SP0A20_write_cmos_sensor(0xe9,0x2c);
	SP0A20_write_cmos_sensor(0xea,0x2a);
	SP0A20_write_cmos_sensor(0xf3,0x2a);
	SP0A20_write_cmos_sensor(0xf4,0x2a);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x04,0xa0);
	SP0A20_write_cmos_sensor(0x05,0x2a);
	SP0A20_write_cmos_sensor(0x0a,0xa0);
	SP0A20_write_cmos_sensor(0x0b,0x2a);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xeb,0x80);
	SP0A20_write_cmos_sensor(0xec,0x80);
	SP0A20_write_cmos_sensor(0xed,0x04);
	SP0A20_write_cmos_sensor(0xee,0x0a);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xf2,0x4d);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x5b,0x05);
	SP0A20_write_cmos_sensor(0x5c,0xa0);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x26,0x80);
	SP0A20_write_cmos_sensor(0x27,0x4f);
	SP0A20_write_cmos_sensor(0x28,0x00);
	SP0A20_write_cmos_sensor(0x29,0x20);
	SP0A20_write_cmos_sensor(0x2a,0x77);
	SP0A20_write_cmos_sensor(0x2b,0x03);
	SP0A20_write_cmos_sensor(0x2c,0x77);
	SP0A20_write_cmos_sensor(0x2d,0x20);
	SP0A20_write_cmos_sensor(0x30,0x00);
	SP0A20_write_cmos_sensor(0x31,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xa1,0x28);
	SP0A20_write_cmos_sensor(0xa2,0x28);
	SP0A20_write_cmos_sensor(0xa3,0x28);
	SP0A20_write_cmos_sensor(0xa4,0x28);
	SP0A20_write_cmos_sensor(0xa5,0x1c);
	SP0A20_write_cmos_sensor(0xa6,0x1e);
	SP0A20_write_cmos_sensor(0xa7,0x20);
	SP0A20_write_cmos_sensor(0xa8,0x20);
	SP0A20_write_cmos_sensor(0xa9,0x1a);
	SP0A20_write_cmos_sensor(0xaa,0x1c);
	SP0A20_write_cmos_sensor(0xab,0x20);
	SP0A20_write_cmos_sensor(0xac,0x20);
	SP0A20_write_cmos_sensor(0xad,0x09);
	SP0A20_write_cmos_sensor(0xae,0x06);
	SP0A20_write_cmos_sensor(0xaf,0x09);
	SP0A20_write_cmos_sensor(0xb0,0x06);
	SP0A20_write_cmos_sensor(0xb1,0x00);
	SP0A20_write_cmos_sensor(0xb2,0x00);
	SP0A20_write_cmos_sensor(0xb3,0x00);
	SP0A20_write_cmos_sensor(0xb4,0x00);
	SP0A20_write_cmos_sensor(0xb5,0x02);
	SP0A20_write_cmos_sensor(0xb6,0x02);
	SP0A20_write_cmos_sensor(0xb7,0x02);
	SP0A20_write_cmos_sensor(0xb8,0x02);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x08,0x00);
	SP0A20_write_cmos_sensor(0x09,0x06);
	SP0A20_write_cmos_sensor(0x1d,0x03);
	SP0A20_write_cmos_sensor(0x1f,0x05);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x32,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x26,0xac);
	SP0A20_write_cmos_sensor(0x27,0xad);
	SP0A20_write_cmos_sensor(0x10,0x00);
	SP0A20_write_cmos_sensor(0x11,0xdd);
	SP0A20_write_cmos_sensor(0x1b,0x80);
	SP0A20_write_cmos_sensor(0x1a,0x80);
	SP0A20_write_cmos_sensor(0x18,0x27);
	SP0A20_write_cmos_sensor(0x19,0x26);
	SP0A20_write_cmos_sensor(0x2a,0x00);
	SP0A20_write_cmos_sensor(0x2b,0x00);
	SP0A20_write_cmos_sensor(0x28,0xf8);
	SP0A20_write_cmos_sensor(0x29,0x08);
	SP0A20_write_cmos_sensor(0x66,0x40);
	SP0A20_write_cmos_sensor(0x67,0x62);
	SP0A20_write_cmos_sensor(0x68,0xd4);
	SP0A20_write_cmos_sensor(0x69,0xf4);
	SP0A20_write_cmos_sensor(0x6a,0xa5);
	SP0A20_write_cmos_sensor(0x7c,0x20);
	SP0A20_write_cmos_sensor(0x7d,0x4b);
	SP0A20_write_cmos_sensor(0x7e,0xf4);
	SP0A20_write_cmos_sensor(0x7f,0x26);
	SP0A20_write_cmos_sensor(0x80,0xa6);
	SP0A20_write_cmos_sensor(0x70,0x26);
	SP0A20_write_cmos_sensor(0x71,0x4a);
	SP0A20_write_cmos_sensor(0x72,0x25);
	SP0A20_write_cmos_sensor(0x73,0x46);
	SP0A20_write_cmos_sensor(0x74,0xaa);
	SP0A20_write_cmos_sensor(0x6b,0x0a);
	SP0A20_write_cmos_sensor(0x6c,0x31);
	SP0A20_write_cmos_sensor(0x6d,0x2e);
	SP0A20_write_cmos_sensor(0x6e,0x52);
	SP0A20_write_cmos_sensor(0x6f,0xaa);
	SP0A20_write_cmos_sensor(0x61,0xfb);
	SP0A20_write_cmos_sensor(0x62,0x13);
	SP0A20_write_cmos_sensor(0x63,0x48);
	SP0A20_write_cmos_sensor(0x64,0x6e);
	SP0A20_write_cmos_sensor(0x65,0x6a);
	SP0A20_write_cmos_sensor(0x75,0x80);
	SP0A20_write_cmos_sensor(0x76,0x09);
	SP0A20_write_cmos_sensor(0x77,0x02);
	SP0A20_write_cmos_sensor(0x24,0x25);
	SP0A20_write_cmos_sensor(0x0e,0x16);
	SP0A20_write_cmos_sensor(0x3b,0x09);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xde,0x0f);
	SP0A20_write_cmos_sensor(0xd7,0x18);
	SP0A20_write_cmos_sensor(0xd8,0x18);
	SP0A20_write_cmos_sensor(0xd9,0x10);
	SP0A20_write_cmos_sensor(0xda,0x14);
	SP0A20_write_cmos_sensor(0xe8,0x1a);
	SP0A20_write_cmos_sensor(0xe9,0x18);
	SP0A20_write_cmos_sensor(0xea,0x11);
	SP0A20_write_cmos_sensor(0xeb,0x18);
	SP0A20_write_cmos_sensor(0xec,0x11);
	SP0A20_write_cmos_sensor(0xed,0x11);
	SP0A20_write_cmos_sensor(0xee,0x11);
	SP0A20_write_cmos_sensor(0xef,0x11);
	SP0A20_write_cmos_sensor(0xd3,0x08);
	SP0A20_write_cmos_sensor(0xd4,0x10);
	SP0A20_write_cmos_sensor(0xd5,0x10);
	SP0A20_write_cmos_sensor(0xd6,0x08);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xd1,0x20);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xdc,0x05);
	SP0A20_write_cmos_sensor(0x05,0x20);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x81,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xfc,0x00);
	SP0A20_write_cmos_sensor(0x7d,0x05);
	SP0A20_write_cmos_sensor(0x7e,0x05);
	SP0A20_write_cmos_sensor(0x7f,0x09);
	SP0A20_write_cmos_sensor(0x80,0x08);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0xdd,0x0f);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x6d,0x08);
	SP0A20_write_cmos_sensor(0x6e,0x08);
	SP0A20_write_cmos_sensor(0x6f,0x10);
	SP0A20_write_cmos_sensor(0x70,0x18);
	SP0A20_write_cmos_sensor(0x86,0x18);
	SP0A20_write_cmos_sensor(0x71,0x10);
	SP0A20_write_cmos_sensor(0x72,0x10);
	SP0A20_write_cmos_sensor(0x73,0x10);
	SP0A20_write_cmos_sensor(0x74,0x10);
	SP0A20_write_cmos_sensor(0x75,0x10);
	SP0A20_write_cmos_sensor(0x76,0x10);
	SP0A20_write_cmos_sensor(0x77,0x10);
	SP0A20_write_cmos_sensor(0x78,0x10);
	SP0A20_write_cmos_sensor(0x79,0x35);
	SP0A20_write_cmos_sensor(0x7a,0x34);
	SP0A20_write_cmos_sensor(0x7b,0x34);
	SP0A20_write_cmos_sensor(0x7c,0x22);
	SP0A20_write_cmos_sensor(0x81,0x0d);
	SP0A20_write_cmos_sensor(0x82,0x18);
	SP0A20_write_cmos_sensor(0x83,0x20);
	SP0A20_write_cmos_sensor(0x84,0x24);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x83,0x12);
	SP0A20_write_cmos_sensor(0x84,0x14);
	SP0A20_write_cmos_sensor(0x86,0x04);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x61,0x60);
	SP0A20_write_cmos_sensor(0x62,0x28);
	SP0A20_write_cmos_sensor(0x8a,0x10);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x8b,0x00);
	SP0A20_write_cmos_sensor(0x8c,0x0b);
	SP0A20_write_cmos_sensor(0x8d,0x18);
	SP0A20_write_cmos_sensor(0x8e,0x26);
	SP0A20_write_cmos_sensor(0x8f,0x31);
	SP0A20_write_cmos_sensor(0x90,0x47);
	SP0A20_write_cmos_sensor(0x91,0x59);
	SP0A20_write_cmos_sensor(0x92,0x66);
	SP0A20_write_cmos_sensor(0x93,0x74);
	SP0A20_write_cmos_sensor(0x94,0x84);
	SP0A20_write_cmos_sensor(0x95,0x92);
	SP0A20_write_cmos_sensor(0x96,0xa0);
	SP0A20_write_cmos_sensor(0x97,0xab);
	SP0A20_write_cmos_sensor(0x98,0xb7);
	SP0A20_write_cmos_sensor(0x99,0xc0);
	SP0A20_write_cmos_sensor(0x9a,0xca);
	SP0A20_write_cmos_sensor(0x9b,0xd2);
	SP0A20_write_cmos_sensor(0x9c,0xdb);
	SP0A20_write_cmos_sensor(0x9d,0xe3);
	SP0A20_write_cmos_sensor(0x9e,0xeb);
	SP0A20_write_cmos_sensor(0x9f,0xf5);
	SP0A20_write_cmos_sensor(0xa0,0xff);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x15,0xc0);
	SP0A20_write_cmos_sensor(0x16,0x8c);
	SP0A20_write_cmos_sensor(0xa0,0xa0);
	SP0A20_write_cmos_sensor(0xa1,0x03);
	SP0A20_write_cmos_sensor(0xa2,0xdd);
	SP0A20_write_cmos_sensor(0xa3,0xe2);
	SP0A20_write_cmos_sensor(0xa4,0xab);
	SP0A20_write_cmos_sensor(0xa5,0xf4);
	SP0A20_write_cmos_sensor(0xa6,0xf0);
	SP0A20_write_cmos_sensor(0xa7,0xc7);
	SP0A20_write_cmos_sensor(0xa8,0xca);
	SP0A20_write_cmos_sensor(0xa9,0x30);
	SP0A20_write_cmos_sensor(0xaa,0x33);
	SP0A20_write_cmos_sensor(0xab,0x0f);
	SP0A20_write_cmos_sensor(0xac,0x9d);
	SP0A20_write_cmos_sensor(0xad,0xf0);
	SP0A20_write_cmos_sensor(0xae,0xf3);
	SP0A20_write_cmos_sensor(0xaf,0xdd);
	SP0A20_write_cmos_sensor(0xb0,0xc9);
	SP0A20_write_cmos_sensor(0xb1,0xda);
	SP0A20_write_cmos_sensor(0xb2,0xed);
	SP0A20_write_cmos_sensor(0xb3,0x7d);
	SP0A20_write_cmos_sensor(0xb4,0x16);
	SP0A20_write_cmos_sensor(0xb5,0x3c);
	SP0A20_write_cmos_sensor(0xb6,0x33);
	SP0A20_write_cmos_sensor(0xb7,0x1f);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xd3,0x66);
	SP0A20_write_cmos_sensor(0xd4,0x58);
	SP0A20_write_cmos_sensor(0xd5,0x4a);
	SP0A20_write_cmos_sensor(0xd6,0x40);
	SP0A20_write_cmos_sensor(0xd7,0x66);
	SP0A20_write_cmos_sensor(0xd8,0x58);
	SP0A20_write_cmos_sensor(0xd9,0x4a);
	SP0A20_write_cmos_sensor(0xda,0x40);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xdd,0x30);
	SP0A20_write_cmos_sensor(0xde,0x10);
	SP0A20_write_cmos_sensor(0xdf,0xff);
	SP0A20_write_cmos_sensor(0x00,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xc2,0xaa);
	SP0A20_write_cmos_sensor(0xc3,0x88);
	SP0A20_write_cmos_sensor(0xc4,0x77);
	SP0A20_write_cmos_sensor(0xc5,0x66);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0xcd,0x10);
	SP0A20_write_cmos_sensor(0xce,0x1f);
	SP0A20_write_cmos_sensor(0xcf,0x30);
	SP0A20_write_cmos_sensor(0xd0,0x45);
	SP0A20_write_cmos_sensor(0xfd,0x02);
	SP0A20_write_cmos_sensor(0x31,0x60);
	SP0A20_write_cmos_sensor(0x32,0x60);
	SP0A20_write_cmos_sensor(0x33,0xc0);
	SP0A20_write_cmos_sensor(0x35,0x60);
	SP0A20_write_cmos_sensor(0x37,0x13);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x0e,0x80);
	SP0A20_write_cmos_sensor(0x0f,0x20);
	SP0A20_write_cmos_sensor(0x10,0x80);
	SP0A20_write_cmos_sensor(0x11,0x80);
	SP0A20_write_cmos_sensor(0x12,0x80);
	SP0A20_write_cmos_sensor(0x13,0x80);
	SP0A20_write_cmos_sensor(0x14,0x8d);
	SP0A20_write_cmos_sensor(0x15,0x86);
	SP0A20_write_cmos_sensor(0x16,0x84);
	SP0A20_write_cmos_sensor(0x17,0x80);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x31,0x06);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x32,0x15);
	SP0A20_write_cmos_sensor(0x33,0xef);
	SP0A20_write_cmos_sensor(0x34,0x07);
	SP0A20_write_cmos_sensor(0xd2,0x01);
	SP0A20_write_cmos_sensor(0xfb,0x25);
	SP0A20_write_cmos_sensor(0xf2,0x49);
	SP0A20_write_cmos_sensor(0x35,0x40);
	SP0A20_write_cmos_sensor(0x5d,0x11);
	SP0A20_write_cmos_sensor(0xfd,0x01);
	SP0A20_write_cmos_sensor(0x36,0x00);
	SP0A20_write_cmos_sensor(0xfd,0x00);
	SP0A20_write_cmos_sensor(0x92,0x11);
	SP0A20_write_cmos_sensor(0xfd,0x01);

}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
 * FUNCTION
 *	SP0A20Open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 SP0A20Open(void)

{
	kal_uint16 sensor_id=0; 
	int retry = 10; 

	SENSORDB("SP0A20Open_start \n");

	//	SP0A20_Sensor_Driver.i2c_clit.addr=SP0A20_WRITE_ID;
	//	SP0A20_Sensor_Driver.i2c_clit = i2c_clit;
	//    SP0A20_Sensor_Driver.i2c_clit->addr = SP0A20_WRITE_ID;

#if 0 
	SP0A20_write_cmos_sensor(0x12, 0x80);
	mDELAY(10);
#endif 

	// check if sensor ID correct
	do {

		SP0A20_write_cmos_sensor(0xfd,0x00);
		sensor_id=SP0A20_read_cmos_sensor(0x02);
		if (sensor_id == 0x2b) {
			break; 
		}
		SENSORDB("Read Sensor ID Fail = 0x%x\n", sensor_id); 

		retry--; 
	}while (retry > 0); 

	if (sensor_id != 0x2b) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}



	memset(&SP0A20_Sensor_Driver, 0, sizeof(struct SP0A20_Sensor_Struct)); 
	SP0A20_Sensor_Driver.MPEG4_encode_mode=KAL_FALSE;
	SP0A20_Sensor_Driver.dummy_pixels=0;
	SP0A20_Sensor_Driver.dummy_lines=0;
	SP0A20_Sensor_Driver.extra_exposure_lines=0;
	SP0A20_Sensor_Driver.exposure_lines=0;
	SP0A20_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;

	SP0A20_Sensor_Driver.bNight_mode =KAL_FALSE; // to distinguish night mode or auto mode, default: auto mode setting
	SP0A20_Sensor_Driver.bBanding_value = AE_FLICKER_MODE_50HZ; // to distinguish between 50HZ and 60HZ.

	SP0A20_Sensor_Driver.fPV_PCLK = 24; //26;
	SP0A20_Sensor_Driver.iPV_Pixels_Per_Line = 0;

	//	SP0A20_set_isp_driving_current(1);
	// initail sequence write in
	//    SP0A20_write_cmos_sensor(0x12, 0x80);
	mDELAY(10);

#ifdef DEBUG_SENSOR  //gepeiwei   120903
	//判断手机对应目录下是否有名为sp0a20_sd 的文件,没有默认参数

	//介于各种原因，本版本初始化参数在_s_fmt中。
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	//static char buf[10*1024] ;

	printk("sp0a20_sd Open File Start\n");
	fp = filp_open("/storage/sdcard1/sp0a20_sd.dat", O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		printk("open file error 0\n");
		fp = filp_open("/storage/sdcard0/sp0a20_sd.dat", O_RDONLY , 0); 
		if (IS_ERR(fp)) { 
			fromsd = 0;   
			printk("open file error 1\n");
		}
		else{
			printk("open file success 1\n");
			fromsd = 1;
			filp_close(fp, NULL); 
		}
	} 
	else 
	{
		printk("open file success 0\n");
		fromsd = 1;
		//SP0A20_Initialize_from_T_Flash();
		filp_close(fp, NULL); 
	}
	//set_fs(fs);
	if(fromsd == 1)//是否从SD读取//gepeiwei   120903
		SP0A20_Initialize_from_T_Flash();//从SD卡读取的主要函数
	else
		SP0A20_Sensor_Driver_Init();  
#else
	SP0A20_Sensor_Driver_Init();  
#endif


	SENSORDB("SP0A20Open_end \n");
	return ERROR_NONE;
}   /* SP0A20Open  */



/*************************************************************************
 * FUNCTION
 *	SP0A20_GetSensorID
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 SP0A20_GetSensorID(kal_uint32 *sensorID)

{

	SENSORDB("SP0A20GetSensorID\n");	
	//	Read sensor ID to adjust I2C is OK?
	SP0A20_write_cmos_sensor(0xfd,0x00);
	*sensorID = SP0A20_read_cmos_sensor(0x02);
	SENSORDB("SP0A20 Sensor Read ID %x\n",*sensorID);
	if (*sensorID != 0x2b) 
	{
		*sensorID =0xffffffff;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	SP0A20Close
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 SP0A20Close(void)
{
	kal_uint8 tmp1;
	// tmp1 = closed;
	//CAMERA_CONTROL_FLOW(tmp1,closed++);
	SENSORDB("SP0A20Close\n");
	return ERROR_NONE;
}   /* SP0A20Close */


/*************************************************************************
 * FUNCTION
 * SP0A20_Preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 SP0A20_Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
	SP0A20_Sensor_Driver.fPV_PCLK=24000000;//26000000
	SP0A20_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;

	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO){
		SP0A20_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE;  // MPEG4 Encode Mode
	}else{
		SP0A20_Sensor_Driver.MPEG4_encode_mode = KAL_FALSE;  
	}

	image_window->GrabStartX= IMAGE_SENSOR_VGA_INSERTED_PIXELS;
	image_window->GrabStartY= IMAGE_SENSOR_VGA_INSERTED_LINES;
	image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

	if(KAL_TRUE == SP0A20_Sensor_Driver.bNight_mode) // for nd 128 noise,decrease color matrix
	{
	}

	// copy sensor_config_data
	memcpy(&SP0A20SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;

}   /*  SP0A20_Preview   */

/*************************************************************************
 * FUNCTION
 *	SP0A20_Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static kal_uint32 SP0A20GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	kal_uint8 tmp1;
	//    tmp1 = res;
	//	CAMERA_CONTROL_FLOW(tmp1,res++);

	pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;

	return ERROR_NONE;
}	/* SP0A20GetResolution() */

static kal_uint32 SP0A20GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
		MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
		MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("SP0A20GetInfo \n");
	pSensorInfo->SensorPreviewResolutionX = IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY = IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX = IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorFullResolutionY = IMAGE_SENSOR_PV_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=30;
	pSensorInfo->SensorWebCamCaptureFrameRate=30;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;

	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	//	   pSensorInfo->SensorDriver3D = 0;   // the sensor driver is 2D
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

	pSensorInfo->CaptureDelayFrame = 3;
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 3; 	
	pSensorInfo->SensorMasterClockSwitch = 0; 


	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			//		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:

		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:

			//		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		default:			
			pSensorInfo->SensorClockFreq=24;//26;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 1; 
			pSensorInfo->SensorGrabStartY = 1;		   
			break;
	}

	memcpy(pSensorConfigData, &SP0A20SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

	return ERROR_NONE;
}	/* SP0A20GetInfo() */


static kal_uint32 SP0A20Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
		MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	CAMERA_CONTROL_FLOW(ScenarioId,ScenarioId);

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			//		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			SP0A20_Preview(pImageWindow, pSensorConfigData);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			//SP0A20_Capture(pImageWindow, pSensorConfigData);
			SP0A20_Preview(pImageWindow, pSensorConfigData);
			break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	return TRUE;
}	/* MT9P012Control() */



static BOOL SP0A20_set_param_wb(UINT16 para)
{
	kal_uint8  temp_reg;


	if(SP0A20_Sensor_Driver.u8Wb_value==para)
		return FALSE;

	SP0A20_Sensor_Driver.u8Wb_value = para;

	switch (para)
	{
		case AWB_MODE_OFF:
			//SP0A20_write_cmos_sensor(0xfd,0x00);				   
			//SP0A20_write_cmos_sensor(0x32,0x05);	   
			break;

		case AWB_MODE_AUTO:
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xac);																
			SP0A20_write_cmos_sensor(0x27,0xad);
			SP0A20_write_cmos_sensor(0xfd,0x01);	// AUTO 3000K~7000K 	   
			SP0A20_write_cmos_sensor(0x32,0x15);
			SP0A20_write_cmos_sensor(0xfd,0x00);
			break;

		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			SP0A20_write_cmos_sensor(0xfd,0x01);	 //7000K									 
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xdb);																
			SP0A20_write_cmos_sensor(0x27,0x90);																
			SP0A20_write_cmos_sensor(0xfd,0x00);											   
			break;

		case AWB_MODE_DAYLIGHT: //sunny
			SP0A20_write_cmos_sensor(0xfd,0x01);	//6500K 									
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xe8);																
			SP0A20_write_cmos_sensor(0x27,0xc8);																
			SP0A20_write_cmos_sensor(0xfd,0x00);														   
			break;

		case AWB_MODE_INCANDESCENT: //office
			// SP0A20_reg_WB_auto 
			SP0A20_write_cmos_sensor(0xfd,0x01);	//2800K~3000K									  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0x88);																
			SP0A20_write_cmos_sensor(0x27,0xd0);																
			SP0A20_write_cmos_sensor(0xfd,0x00);															
			break;

		case AWB_MODE_TUNGSTEN: //home
			// SP0A20_reg_WB_auto 
			SP0A20_write_cmos_sensor(0xfd,0x01);	//4000K 								  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xac);																
			SP0A20_write_cmos_sensor(0x27,0xbe);																
			SP0A20_write_cmos_sensor(0xfd,0x00);														   
			break;

		case AWB_MODE_FLUORESCENT:
			// SP0A20_reg_WB_auto 
			SP0A20_write_cmos_sensor(0xfd,0x01);	//4200-5000K 								  
			SP0A20_write_cmos_sensor(0x32,0x05);															
			SP0A20_write_cmos_sensor(0xfd,0x02);															
			SP0A20_write_cmos_sensor(0x26,0xbf);																
			SP0A20_write_cmos_sensor(0x27,0xcc);																
			SP0A20_write_cmos_sensor(0xfd,0x00);														   
			break;

		default:
			return FALSE;
	}


	return TRUE;
} /* SP0A20_set_param_wb */


static BOOL SP0A20_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;

	if(para==SP0A20_Sensor_Driver.u8Effect_value)
		return FALSE;


	SP0A20_Sensor_Driver.u8Effect_value = para;
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x36,0x02);
			SP0A20_write_cmos_sensor(0xfd,0x00);
	switch (para)
	{
		case MEFFECT_OFF:  
mDELAY(200);
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x00);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0x80);
			break;

		case MEFFECT_SEPIA:
mDELAY(200);  
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x98);
			SP0A20_write_cmos_sensor(0x68,0x58);


			break;

		case MEFFECT_NEGATIVE: 
mDELAY(200);
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x04);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0x80);

			break;

		case MEFFECT_SEPIAGREEN:
mDELAY(200);
			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x50);
			SP0A20_write_cmos_sensor(0x68,0x50);

			break;

		case MEFFECT_SEPIABLUE:
mDELAY(200);

			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x10);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0xb0);

			break;

		case MEFFECT_MONO: //B&W
mDELAY(200);

			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x66,0x20);
			SP0A20_write_cmos_sensor(0x67,0x80);
			SP0A20_write_cmos_sensor(0x68,0x80);


			break;
		default:
			return FALSE;
	}


			SP0A20_write_cmos_sensor(0xfd,0x01);
			SP0A20_write_cmos_sensor(0x36,0x00);
	return ret;

} /* SP0A20_set_param_effect */

static void SP0A20_set_banding_for_50Hz(void)
{
	printk("SP0A20_set_banding_for_50Hz\n");

}


static void SP0A20_set_banding_for_60Hz(void)
{
	printk("SP0A20_set_banding_for_60Hz\n");

}

static BOOL SP0A20_set_param_banding(UINT16 para)
{
	//if(SP0A20_Sensor_Driver.bBanding_value == para)
	//	return TRUE;

	SP0A20_Sensor_Driver.bBanding_value = para;

	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			isBanding = 0;
			printk("SP0A20_set_param_banding_50hz\n");
			//SP0A20_set_banding_for_50Hz();
			break;
		case AE_FLICKER_MODE_60HZ:
			isBanding = 1;
			printk("SP0A20_set_param_banding_60hz\n");
			//SP0A20_set_banding_for_60Hz();
			break;
		default:
			return FALSE;
	}

	return TRUE;
} /* SP0A20_set_param_banding */
static BOOL SP0A20_set_param_exposure(UINT16 para)
{


	if(para == SP0A20_Sensor_Driver.u8Ev_value)
		return FALSE;

	SP0A20_Sensor_Driver.u8Ev_value = para;

	switch (para)
	{
		case AE_EV_COMP_n13:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xc0);
			break;

		case AE_EV_COMP_n10:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xd0);
			break;

		case AE_EV_COMP_n07:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xe0);
			break;

		case AE_EV_COMP_n03:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0xf0);
			break;

		case AE_EV_COMP_00:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x00);//0xfa before
			break;

		case AE_EV_COMP_03:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x10);
			break;

		case AE_EV_COMP_07:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x20);
			break;

		case AE_EV_COMP_10:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x30);
			break;

		case AE_EV_COMP_13:
			SP0A20_write_cmos_sensor(0xfd, 0x01);
			SP0A20_write_cmos_sensor(0xdb, 0x40);
			break;

		default:
			return FALSE;
	}


	return TRUE;
} /* SP0A20_set_param_exposure */

static kal_uint32 SP0A20_YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
#ifdef DEBUG_SENSOR
	return TRUE;
#endif

	switch (iCmd) {
		case FID_SCENE_MODE:
			if (iPara == SCENE_MODE_OFF){
				SP0A20_night_mode(FALSE); 
			}else if (iPara == SCENE_MODE_NIGHTSCENE){
				SP0A20_night_mode(TRUE); 
			}	    

			break; 
		case FID_AWB_MODE:
			SP0A20_set_param_wb(iPara);
			break;
		case FID_COLOR_EFFECT:
			SP0A20_set_param_effect(iPara);
			break;
		case FID_AE_EV:	
			SP0A20_set_param_exposure(iPara);
			break;
		case FID_AE_FLICKER:
			SP0A20_set_param_banding(iPara);
			//whl120717 test
			if (SP0A20_Sensor_Driver.bNight_mode == KAL_FALSE){
				SP0A20_night_mode(FALSE); 
			}else if (SP0A20_Sensor_Driver.bNight_mode == KAL_TRUE){
				SP0A20_night_mode(TRUE); 

			}	

			break;
		default:
			break;
	}

	return TRUE;
}   /* SP0A20_YUVSensorSetting */

static kal_uint32 SP0A20_YUVSetVideoMode(UINT16 u2FrameRate)
{
	kal_uint8 temp ;//= SP0A20_read_cmos_sensor(0x3B);
	SP0A20_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE; 

	if (u2FrameRate == 30)
	{
	}
	else if (u2FrameRate == 15)       
	{
	}
	else 
	{
		printk("Wrong frame rate setting \n");
	}   

	printk("\n SP0A20_YUVSetVideoMode:u2FrameRate=%d\n\n",u2FrameRate);
	return TRUE;
}

UINT32 SP0A20SetSoftwarePWDNMode(kal_bool bEnable)
{
#if 0
	SENSORDB("[SP0A20SetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);

	if(bEnable) {   // enable software sleep mode   
		SP0A20_write_cmos_sensor(0x09, 0x10);
	} else {
		SP0A20_write_cmos_sensor(0x09, 0x03);  
	}
#endif
	return TRUE;
}

/*************************************************************************
 * FUNCTION
 *    SP0A20_get_size
 *
 * DESCRIPTION
 *    This function return the image width and height of image sensor.
 *
 * PARAMETERS
 *    *sensor_width: address pointer of horizontal effect pixels of image sensor
 *    *sensor_height: address pointer of vertical effect pixels of image sensor
 *
 * RETURNS
 *    None
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static void SP0A20_get_size(kal_uint16 *sensor_width, kal_uint16 *sensor_height)
{
	*sensor_width = IMAGE_SENSOR_FULL_WIDTH; /* must be 4:3 */
	*sensor_height = IMAGE_SENSOR_FULL_HEIGHT;
}

/*************************************************************************
 * FUNCTION
 *    SP0A20_get_period
 *
 * DESCRIPTION
 *    This function return the image width and height of image sensor.
 *
 * PARAMETERS
 *    *pixel_number: address pointer of pixel numbers in one period of HSYNC
 *    *line_number: address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static void SP0A20_get_period(kal_uint16 *pixel_number, kal_uint16 *line_number)
{
	*pixel_number = VGA_PERIOD_PIXEL_NUMS+SP0A20_Sensor_Driver.dummy_pixels;
	*line_number = VGA_PERIOD_LINE_NUMS+SP0A20_Sensor_Driver.dummy_lines;
}

/*************************************************************************
 * FUNCTION
 *    SP0A20_feature_control
 *
 * DESCRIPTION
 *    This function control sensor mode
 *
 * PARAMETERS
 *    id: scenario id
 *    image_window: image grab window
 *    cfg_data: config data
 *
 * RETURNS
 *    error code
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static kal_uint32 SP0A20FeatureControl(MSDK_SENSOR_FEATURE_ENUM id, kal_uint8 *para, kal_uint32 *len)
{
	UINT32 *pFeatureData32=(UINT32 *) para;

	switch (id)
	{
		case SENSOR_FEATURE_GET_RESOLUTION: /* no use */
			SP0A20_get_size((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			SP0A20_get_period((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*(kal_uint32 *)para = SP0A20_Sensor_Driver.fPV_PCLK;
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE: 
			SP0A20_night_mode((kal_bool)*(kal_uint16 *)para);
			break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			SP0A20_write_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER: /* 10 */
			((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData = SP0A20_read_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&SP0A20_Sensor_Driver.eng.CCT, para, sizeof(SP0A20_Sensor_Driver.eng.CCT));
			break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_GET_CONFIG_PARA: /* no use */
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
		case SENSOR_FEATURE_GET_GROUP_INFO: /* 20 */
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			/*
			 * get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			 * if EEPROM does not exist in camera module.
			 */
			*(kal_uint32 *)para = LENS_DRIVER_ID_DO_NOT_CARE;
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			//		SP0A20_YUVSensorSetting((FEATURE_ID)(UINT32 *)para, (UINT32 *)(para+1));

			SP0A20_YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
#if 0		    		
		case SENSOR_FEATURE_QUERY:
			SP0A20_Query(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
			break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
			break;		
#endif 			
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			SP0A20_YUVSetVideoMode(*para);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			SP0A20_GetSensorID(pFeatureData32); 
			break; 	
		case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
			SP0A20SetSoftwarePWDNMode((BOOL)*pFeatureData32);        	        	
			break;
		default:
			break;
	}
	return ERROR_NONE;
}




#if 0
image_sensor_func_struct image_sensor_driver_SP0A20=
{
	SP0A20Open,
	SP0A20Close,
	SP0A20GetResolution,
	SP0A20GetInfo,
	SP0A20Control,
	SP0A20FeatureControl
};
void image_sensor_func_config(void)
{
	extern image_sensor_func_struct *image_sensor_driver;

	image_sensor_driver = &image_sensor_driver_SP0A20;
}

#endif

SENSOR_FUNCTION_STRUCT	SensorFuncSP0A20=
{
	SP0A20Open,
	SP0A20GetInfo,
	SP0A20GetResolution,
	SP0A20FeatureControl,
	SP0A20Control,
	SP0A20Close
};

UINT32 SP0A20_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{

	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP0A20;

	return ERROR_NONE;
}	/* SensorInit() */




