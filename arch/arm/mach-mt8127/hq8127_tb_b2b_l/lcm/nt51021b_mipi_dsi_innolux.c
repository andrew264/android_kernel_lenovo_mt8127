/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
*  permission of MediaTek Inc. (C) 2008
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
#ifndef BUILD_LK
#include <linux/string.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>

#elif (defined BUILD_UBOOT)
#include <asm/arch/mt6577_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif
#include "lcm_drv.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define SN65DSI_DEBUG
#define FRAME_WIDTH  (600)
#define FRAME_HEIGHT (1024)


#define LVDS_LCM_STBY       GPIO118

#define GPIO_LCM1990_RESET      GPIO89
#define GPIO_LCM1990_STBY       GPIO83


#define LCM_DSI_6589_PLL_CLOCK_201_5 0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (mt_set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY 0xAB

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)			lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)											lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)						lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define REGFLAG_END_OF_TABLE 0xFE // END OF REGISTERS MARKER

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

typedef unsigned char    kal_uint8;
static struct LCM_setting_table lcm_initialization_setting[] = {
{0x83, 1, {0x00}},
{0x84, 1, {0x00}},
                  
{0x85, 1, {0x04}},
{0x86, 1, {0x08}},
                  
{0x8C, 1, {0x8E}},
                  
{0xC5, 1, {0x23}},
{0xC7, 1, {0x23}},
                  
{0xFD, 1, {0x5B}},
{0xFA, 1, {0x14}},
                  
{0x83, 1, {0xAA}},
                  

{0x84, 1, {0x11}},
{0xC0, 1, {0x1F}},
{0xC1, 1, {0x20}},
{0xC2, 1, {0x28}},
{0xC3, 1, {0x36}},
{0xC4, 1, {0x46}},
{0xC5, 1, {0x4E}},
{0xC6, 1, {0x54}},
{0xC7, 1, {0x58}},
{0xC8, 1, {0x5E}},

{0xC9, 1, {0xD1}},
{0xCA, 1, {0xDB}},
{0xCB, 1, {0xF8}},
{0xCC, 1, {0x05}},
{0xCD, 1, {0x0A}},
{0xCE, 1, {0x08}},
{0xCF, 1, {0x0A}},
{0xD0, 1, {0x0D}},
{0xD1, 1, {0x1E}},
{0xD2, 1, {0x2F}},

{0xD3, 1, {0x4B}},
{0xD4, 1, {0x51}},
{0xD5, 1, {0xB1}},
{0xD6, 1, {0xB5}},
{0xD7, 1, {0xBD}},
{0xD8, 1, {0xC5}},
{0xD9, 1, {0xCE}},
{0xDA, 1, {0xD8}},
{0xDB, 1, {0xE3}},
{0xDC, 1, {0xF1}},

{0xDD, 1, {0xFF}},
{0xDE, 1, {0xF8}},
{0xDF, 1, {0x2F}},
{0xE0, 1, {0x20}},
{0xE1, 1, {0x21}},
{0xE2, 1, {0x2A}},
{0xE3, 1, {0x39}},
{0xE4, 1, {0x48}},
{0xE5, 1, {0x4F}},
{0xE6, 1, {0x57}},
                  

{0xE7, 1, {0x5B}},
{0xE8, 1, {0x61}},
{0xE9, 1, {0xD8}},
{0xEA, 1, {0xE4}},
{0xEB, 1, {0x02}},
{0xEC, 1, {0x10}},
{0xED, 1, {0x15}},
{0xEE, 1, {0x14}},
{0xEF, 1, {0x16}},
{0xF0, 1, {0x1D}},
{0xF1, 1, {0x2F}},
{0xF2, 1, {0x40}},
{0xF3, 1, {0x5F}},
{0xF4, 1, {0x6A}},
{0xF5, 1, {0xBD}},
{0xF6, 1, {0xBE}},
{0xF7, 1, {0xC9}},
{0xF8, 1, {0xD1}},
{0xF9, 1, {0xDA}},
{0xFA, 1, {0xE4}},
{0xFB, 1, {0xEE}},
{0xFC, 1, {0xFD}},
{0xFD, 1, {0xFF}},
{0xFE, 1, {0xFC}},
{0xFF, 1, {0x2F}},
                  
{0x83, 1, {0xBB}},
{0x84, 1, {0x22}},
{0xC0, 1, {0x1F}},
{0xC1, 1, {0x20}},
{0xC2, 1, {0x28}},
{0xC3, 1, {0x36}},
{0xC4, 1, {0x46}},
{0xC5, 1, {0x4E}},
{0xC6, 1, {0x54}},
{0xC7, 1, {0x58}},
{0xC8, 1, {0x5E}},
{0xC9, 1, {0xD1}},
{0xCA, 1, {0xDB}},
{0xCB, 1, {0xF8}},
{0xCC, 1, {0x05}},
{0xCD, 1, {0x0A}},
{0xCE, 1, {0x08}},
{0xCF, 1, {0x0A}},
{0xD0, 1, {0x0D}},
{0xD1, 1, {0x1E}},
{0xD2, 1, {0x2F}},
{0xD3, 1, {0x4B}},
{0xD4, 1, {0x30}},
{0xD5, 1, {0x9F}},
{0xD6, 1, {0xB5}},
{0xD7, 1, {0xBD}},
{0xD8, 1, {0xC5}},
{0xD9, 1, {0xCE}},
{0xDA, 1, {0xD8}},
{0xDB, 1, {0xE3}},
{0xDC, 1, {0xF1}},
{0xDD, 1, {0xFF}},
{0xDE, 1, {0xF8}},
{0xDF, 1, {0x2F}},
{0xE0, 1, {0x20}},
{0xE1, 1, {0x21}},
{0xE2, 1, {0x2A}},
{0xE3, 1, {0x39}},
{0xE4, 1, {0x48}},
{0xE5, 1, {0x4F}},
{0xE6, 1, {0x57}},
{0xE7, 1, {0x5B}},
{0xE8, 1, {0x61}},
{0xE9, 1, {0xD7}},
{0xEA, 1, {0xE3}},
{0xEB, 1, {0x01}},
{0xEC, 1, {0x0F}},
{0xED, 1, {0x14}},
{0xEE, 1, {0x13}},
{0xEF, 1, {0x15}},
{0xF0, 1, {0x1C}},
{0xF1, 1, {0x0D}},
{0xF2, 1, {0x0E}},
{0xF3, 1, {0x00}},
{0xF4, 1, {0xE4}},
{0xF5, 1, {0x65}},
{0xF6, 1, {0x66}},
{0xF7, 1, {0x67}},
{0xF8, 1, {0x69}},
{0xF9, 1, {0x6B}},
{0xFA, 1, {0x6C}},
{0xFB, 1, {0x6D}},
{0xFC, 1, {0x6E}},
{0xFD, 1, {0x6F}},
{0xFE, 1, {0xFC}},
{0xFF, 1, {0x27}},
                  
{0x83, 1, {0xCC}},
{0x84, 1, {0x33}},
{0xC0, 1, {0x1F}},
{0xC1, 1, {0x20}},
{0xC2, 1, {0x28}},
{0xC3, 1, {0x36}},
{0xC4, 1, {0x46}},
{0xC5, 1, {0x4E}},
{0xC6, 1, {0x54}},
{0xC7, 1, {0x58}},
{0xC8, 1, {0x5E}},
{0xC9, 1, {0xD1}},
{0xCA, 1, {0xDB}},
{0xCB, 1, {0xF8}},
{0xCC, 1, {0x05}},
{0xCD, 1, {0x0A}},
{0xCE, 1, {0x08}},
{0xCF, 1, {0x0A}},
{0xD0, 1, {0x0D}},
{0xD1, 1, {0x1E}},
{0xD2, 1, {0x2F}},
{0xD3, 1, {0x4B}},
{0xD4, 1, {0x51}},
{0xD5, 1, {0xB1}},
{0xD6, 1, {0xB5}},
{0xD7, 1, {0xBD}},
{0xD8, 1, {0xC5}},
{0xD9, 1, {0xCE}},
{0xDA, 1, {0xD8}},
{0xDB, 1, {0xE3}},
{0xDC, 1, {0xF1}},
{0xDD, 1, {0xFF}},
{0xDE, 1, {0xF8}},
{0xDF, 1, {0x2F}},
{0xE0, 1, {0x20}},
{0xE1, 1, {0x21}},
{0xE2, 1, {0x2A}},
{0xE3, 1, {0x39}},
{0xE4, 1, {0x48}},
{0xE5, 1, {0x4F}},
{0xE6, 1, {0x57}},
{0xE7, 1, {0x5B}},
{0xE8, 1, {0x61}},
{0xE9, 1, {0xD8}},
{0xEA, 1, {0xE4}},
{0xEB, 1, {0x02}},
{0xEC, 1, {0x10}},
{0xED, 1, {0x15}},
{0xEE, 1, {0x14}},
{0xEF, 1, {0x16}},
{0xF0, 1, {0x1D}},
{0xF1, 1, {0x2F}},
{0xF2, 1, {0x40}},
{0xF3, 1, {0x5F}},
{0xF4, 1, {0x6A}},
{0xF5, 1, {0xBD}},
{0xF6, 1, {0xBE}},
{0xF7, 1, {0xC9}},
{0xF8, 1, {0xD1}},
{0xF9, 1, {0xDA}},
{0xFA, 1, {0xE4}},
{0xFB, 1, {0xEE}},
{0xFC, 1, {0xFD}},
{0xFD, 1, {0xFF}},
{0xFE, 1, {0xFC}},
{0xFF, 1, {0x2F}},
                  
{0x83, 1, {0xAA}},
{0x84, 1, {0x11}},
{0xA9, 1, {0x4B}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
	
        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;
       case REGFLAG_END_OF_TABLE :
          break;
        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}



static struct sn65dsi8x_setting_table {
    unsigned char cmd;
    unsigned char data;
};

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{	

	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   =BURST_VDO_MODE;// BURST_VDO_MODE;

	//params->dsi.mode   = SYNC_PULSE_VDO_MODE;
	//params->dsi.mode   =SYNC_EVENT_VDO_MODE ;
	// DSI
	/* Command mode setting */  
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.word_count=1024*3;
	// params->dsi.word_count=480*3;

	params->dsi.vertical_sync_active= 1;
	params->dsi.vertical_backporch= 25 ;
	params->dsi.vertical_frontporch= 35 ;
	params->dsi.vertical_active_line= FRAME_HEIGHT;//hight

	params->dsi.horizontal_sync_active				= 20 ;
	params->dsi.horizontal_backporch				= 60 ;
	params->dsi.horizontal_frontporch				=  80 ;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;//=wight

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.pll_select=0;	//0: MIPI_PLL; 1: LVDS_PLL
	params->dsi.PLL_CLOCK = 156;//this value must be in MTK suggested table
	params->dsi.cont_clock = 1;//if not config this para, must config other 7 or 3 paras to gen. PLL
}

static void lcm_init(void)
{

	unsigned int data_array[64];

	mt_set_gpio_mode(GPIO_LCM1990_STBY, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM1990_STBY, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ONE);
	MDELAY(5);

	mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);	  
	mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_STBY
	MDELAY(15);
	mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);    
	mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ZERO); // LCM_RST -h
	MDELAY(40);
	mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);    
	mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_RST -h

	MDELAY(15);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000083;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000084;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000485;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000886;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00008e8c;
 dsi_set_cmdq(data_array, 2, 1);
 
 data_array[0] = 0x00023902;
 data_array[1] = 0x00002bc5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002bc7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005BFD;
 dsi_set_cmdq(data_array, 2, 1);   

 data_array[0] = 0x00023902;
 data_array[1] = 0x000014FA;
 dsi_set_cmdq(data_array, 2, 1);              

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000AA83;
 dsi_set_cmdq(data_array, 2, 1);
                  
 data_array[0] = 0x00023902;
 data_array[1] = 0x00001184;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001Fc0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000020c1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000028c2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000036c3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000046c4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004Ec5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000054c6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000058c7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005Ec8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D1c9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000DBcA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F8cB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000005cC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000AcD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000008cE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000AcF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000DD0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001ED1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FD2;
 dsi_set_cmdq(data_array, 2, 1);
                  
 data_array[0] = 0x00023902;
 data_array[1] = 0x00004BD3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000051D4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B1D5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B5D6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BDD7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000C5D8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000CED9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D8DA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000E3DB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F1DC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FFDD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F8DE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FDF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000020E0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000021E1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002AE2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000039E3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000048E4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004FE5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000057E6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005BE7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000061E8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D8E9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000E4EA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000002EB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000010EC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000015ED;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000014EE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000016EF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001DF0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FF1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000040F2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005FF3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00006AF4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BDF5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BEF6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000C9F7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D1F8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000DAF9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000E4FA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000EEFB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FDFC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FFFD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FCFE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FFF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BB83;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002284;
 dsi_set_cmdq(data_array, 2, 1);
                 
data_array[0] = 0x00023902;
 data_array[1] = 0x000019c0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001Ec1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000026c2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000034c3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000044c4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004Cc5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000052c6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000056c7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005cc8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000cfc9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D6cA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F2cB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FDcC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000000cD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FBcE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FDcF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000fbD0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FFD1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000006D2;
 dsi_set_cmdq(data_array, 2, 1);
                  
 data_array[0] = 0x00023902;
 data_array[1] = 0x000011D3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000CD4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000088D5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00008AD6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00008ED7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000092D8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000098D9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00009DDA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000A4DB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000ABDC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B9DD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000010DE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002EDF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001EE0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001FE1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000028E2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000037E3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000046E4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004DE5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000055E6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000059E7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005FE8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D5E9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000DEEA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FBEB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000007EC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000AED;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000006EE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000008EF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000AF0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000FF1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000016F2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000022F3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001DF4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00008EF5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000092F6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000099F7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00009DF8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000A3F9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000A8FA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000ADFB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B5FC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B7FD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F8FE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FFF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000CC83;
 dsi_set_cmdq(data_array, 2, 1);
                  
 data_array[0] = 0x00023902;
 data_array[1] = 0x00003384;
 dsi_set_cmdq(data_array, 2, 1);   

data_array[0] = 0x00023902;
 data_array[1] = 0x00001Fc0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000020c1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000028c2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000036c3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000046c4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004Ec5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000054c6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000058c7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005Ec8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D1c9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000DBcA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F8cB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000005cC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000AcD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000008cE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000AcF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00000DD0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001ED1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FD2;
 dsi_set_cmdq(data_array, 2, 1);
                  
 data_array[0] = 0x00023902;
 data_array[1] = 0x00004BD3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000051D4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B1D5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000B5D6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BDD7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000C5D8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000CED9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D8DA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000E3DB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F1DC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FFDD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000F8DE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FDF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000020E0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000021E1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002AE2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000039E3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000048E4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004FE5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000057E6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005BE7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000061E8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D8E9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000E4EA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000002EB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000010EC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000015ED;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000014EE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000016EF;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001DF0;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FF1;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x000040F2;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00005FF3;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00006AF4;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BDF5;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000BEF6;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000C9F7;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000D1F8;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000DAF9;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000E4FA;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000EEFB;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FDFC;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FFFD;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x0000FCFE;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00002FFF;
 dsi_set_cmdq(data_array, 2, 1);              

                  
 data_array[0] = 0x00023902;
 data_array[1] = 0x0000AA83;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00001184;
 dsi_set_cmdq(data_array, 2, 1);

 data_array[0] = 0x00023902;
 data_array[1] = 0x00004BA9;
 dsi_set_cmdq(data_array, 2, 1);
}


static void lcm_suspend(void)
{
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ZERO); 
	mt_set_gpio_mode(GPIO_LCM1990_STBY, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM1990_STBY, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ZERO);
}


static void lcm_resume(void)
{
	lcm_init();	
}
static unsigned int lcm_compare_id(void)
{
#if defined(BUILD_LK)
		printf("NT51021B  lcm_compare_id \n");
#endif

    return 1;
}

LCM_DRIVER nt51021b_mipi_dsi_lcm_drv = 
{
	.name		= "NT51021B",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};

