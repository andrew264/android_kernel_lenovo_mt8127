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
#include <platform/mt_pmic.h>
#include <platform/upmu_common.h>
#include <platform/mt_i2c.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif
#include "lcm_drv.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1024)
#define FRAME_HEIGHT (600)



//#define GPIO_LCD_PWR_EN      GPIO3
#define GPIO_LCD_3V3_EN                     GPIO83
#define GPIO_LCD_PWR_EN                     GPIO44
#define GPIO_LCD_RST_EN                     GPIO89
#define GPIO_LCD_STB_EN                  	GPIO88

#define HSYNC_PULSE_WIDTH 10 
#define HSYNC_BACK_PORCH  160
#define HSYNC_FRONT_PORCH 160
#define VSYNC_PULSE_WIDTH 10
#define VSYNC_BACK_PORCH  23
#define VSYNC_FRONT_PORCH 12

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    


static void lcd_power_en(unsigned char enabled)
{
#ifndef BUILD_LK
	printk("[IND][K] %s : %s\n", __func__, enabled ? "on" : "off");
#else
	printf("[IND][LK] %s : %s\n", __func__, enabled ? "on" : "off");
#endif

	if (enabled)
	{
		mt_set_gpio_mode(GPIO_LCD_3V3_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_3V3_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_3V3_EN, GPIO_OUT_ONE);
		MDELAY(5);
		mt_set_gpio_mode(GPIO_LCD_PWR_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_PWR_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_LCD_3V3_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_3V3_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_3V3_EN, GPIO_OUT_ZERO);
		MDELAY(5);
		mt_set_gpio_mode(GPIO_LCD_PWR_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_PWR_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
	}
}

#if 1
static void lcd_reset(unsigned char enabled)
{
#ifndef BUILD_LK
    printk("[IND][K] %s : %s\n", __func__, enabled ? "on" : "off");
#else
    printf("[IND][LK] %s : %s\n", __func__, enabled ? "on" : "off");
#endif

    if (enabled)
    {
        mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
		MDELAY(1);
		mt_set_gpio_mode(GPIO_LCD_STB_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_STB_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_STB_EN, GPIO_OUT_ONE);
    }
    else
    {	
        mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
		MDELAY(1);
        mt_set_gpio_mode(GPIO_LCD_STB_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_STB_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_STB_EN, GPIO_OUT_ZERO);   
    }
}
#endif

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DPI;
	params->ctrl   = LCM_CTRL_SERIAL_DBI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->io_select_mode = 0; 

// div0_real = div0==0 ? 1:
//			div0==1 ? 2: 4;
// div1_real = div1==0 ? 1:
//			div1==1 ? 2: 4;
// freq = 26*mipi_pll_clk_ref/2^24/div0_real/div1_real/8
#if 0
	/* RGB interface configurations */
	params->dpi.mipi_pll_clk_ref  = 536870912;
	params->dpi.mipi_pll_clk_div1 = 0;	// div0=0,1,2,3;div0_real=1,2,4,4
	params->dpi.mipi_pll_clk_div2 = 0;	// div1=0,1,2,3;div1_real=1,2,4,4
	params->dpi.dpi_clk_div 	  = 4;			 //{4,2}, pll/4=51.025M
	params->dpi.dpi_clk_duty	  = 2;
#endif
	params->dpi.PLL_CLOCK = 52;  //67MHz

    params->dpi.clk_pol           = LCM_POLARITY_FALLING;
    params->dpi.de_pol            = LCM_POLARITY_RISING;
    params->dpi.vsync_pol         = LCM_POLARITY_FALLING;
    params->dpi.hsync_pol         = LCM_POLARITY_FALLING;

	params->dpi.hsync_pulse_width = HSYNC_PULSE_WIDTH;
	params->dpi.hsync_back_porch  = HSYNC_BACK_PORCH;
	params->dpi.hsync_front_porch = HSYNC_FRONT_PORCH;
	params->dpi.vsync_pulse_width = VSYNC_PULSE_WIDTH;
	params->dpi.vsync_back_porch  = VSYNC_BACK_PORCH;
	params->dpi.vsync_front_porch = VSYNC_FRONT_PORCH;
	
    params->dpi.lvds_tx_en = 1;
    params->dpi.ssc_disable = 1;
    params->dpi.format            = LCM_DPI_FORMAT_RGB888;   // format is 24 bit
    params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;
    params->dpi.is_serial_output  = 0;

    params->dpi.intermediat_buffer_num = 0;

    params->dpi.io_driving_current = LCM_DRIVING_CURRENT_2MA;
}

extern void DSI_clk_HS_mode(unsigned char enter);

//extern void DSI_Continuous_HS(void);

static void init_lcm_registers(void)
{
	  
#ifndef BUILD_LK
    printk("[IND][K] %s\n", __func__);
#else
    printf("[IND][LK] %s\n", __func__);
#endif
	  
}

static void lcm_init(void)
{
#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);
#else
	printf("[IND][LK] %s\n", __func__);
#endif


	
	//upmu_set_rg_vgp1_vosel(0x5);
	//upmu_set_rg_vgp1_en(0x1);
hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"lcm");
	MDELAY(20);

	lcd_reset(0);
	MDELAY(50);
	lcd_reset(1);
	MDELAY(50);
	lcd_power_en(1);
	MDELAY(50);

}
//////////////////////////

static void lcm_init_pwer(void)
{
#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);
#else
	printf("[IND][LK] %s\n", __func__);
#endif


	
	//upmu_set_rg_vgp1_vosel(0x5);
	//upmu_set_rg_vgp1_en(0x1);

	//MDELAY(20);

	//lcd_reset(0);
	//MDELAY(50);
	//lcd_reset(1);
	//MDELAY(50);
	//lcd_power_en(1);
	//MDELAY(50);

}

//////////////////////////
static void lcm_suspend(void)
{
#ifndef BUILD_LK
    printk("[IND][K] %s\n", __func__);
#else
    printf("[IND][LK] %s\n", __func__);
#endif
    
    //lcd_reset(0);
    //MDELAY(5);
    //lcd_power_en(0);
    //MDELAY(30);
	
//#ifndef BUILD_LK //kernel
	//hwPowerDown(MT6323_POWER_LDO_VGP1,"lcm");
//MDELAY(30);
//hwPowerDown(MT6323_POWER_LDO_VGP1,"lcm");
//#else//lk
 	//upmu_set_rg_vgp1_en(0x0);
//#endif

   // MDELAY(50);
    //lcd_reset(0);
   // lcd_power_en(0);
}


static void lcm_resume(void)
{

	//lcd_reset(0);
	//MDELAY(10);
	//lcd_reset(1);

#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);
	//hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"lcm");
#else
	printf("[IND][LK] %s\n", __func__);
#endif
	//MDELAY(20);
	//lcd_power_en(1);
	//MDELAY(10);
}
/////////////////////////////////////////////////////
static void lcm_resume_power(void)
{



#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);
	hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"lcm");
#else
	printf("[IND][LK] %s\n", __func__);
#endif
	MDELAY(10);
	lcd_reset(0);
	MDELAY(10);
	lcd_reset(1);
	MDELAY(50);
	lcd_power_en(1);
	MDELAY(10);
}
//////////////////
static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
    printk("[IND][K] %s\n", __func__);
#else
    printf("[IND][LK] %s\n", __func__);
#endif
    //hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"lcm");
	MDELAY(5);
    lcd_reset(0);
    MDELAY(85);
    printk("111");
    lcd_power_en(0);
    MDELAY(50);
	
//#ifndef BUILD_LK //kernel
	printk("222");
	hwPowerDown(MT6323_POWER_LDO_VGP1,"lcm");
	printk("333");
//#else//lk
 	// upmu_set_rg_vgp1_en(0x0);
//#endif

//    MDELAY(5);
  //  lcd_reset(0);
   // lcd_power_en(0);
}
/////////////////////////////////////////////////////
static int lcm_compare_id(void)
{
	return 1;
}


LCM_DRIVER hx8282_a01_lvds_dpi_vdo_lcm_drv = 
{
	.name		= "hx8282_a01_lvds_dpi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params 	= lcm_get_params,
	.init			= lcm_init,
	.suspend		= lcm_suspend,
	.resume 		= lcm_resume,

	//.init_power    = lcm_init_pwer,
	.suspend_power = lcm_suspend_power,
	.resume_power = lcm_resume_power,

	.compare_id    = lcm_compare_id,

};

