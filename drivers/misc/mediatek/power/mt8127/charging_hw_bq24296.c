/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "bq24296.h"
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_charging.h>
 
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mach/diso.h>
#include "cust_diso.h"
#include <linux/of.h>
#include <mach/irqs.h>
#ifdef MTK_DISCRETE_SWITCH
#include <mach/eint.h>
#include <cust_eint.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#endif
#endif


 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //
extern int g_runin_battery_test_flag;
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

#if !defined(GPIO_CHR_CE_PIN)
#ifdef GPIO_SWCHARGER_EN_PIN 
#define GPIO_CHR_CE_PIN GPIO_SWCHARGER_EN_PIN
#else
#define  GPIO_CHR_CE_PIN (19 | 0x80000000)
#endif
#endif
int gpio_off_dir  = GPIO_DIR_OUT;
int gpio_off_out  = GPIO_OUT_ONE;
int gpio_on_dir   = GPIO_DIR_OUT;
int gpio_on_out   = GPIO_OUT_ZERO;

kal_bool charging_type_det_done = KAL_TRUE;

#if defined(CONFIG_HQ_LENOVO_A1992_CHARGING_SUPPORT)
const kal_uint32 VBAT_CV_VTH[]=
{
	3504000,    3520000,    3536000,    3552000,
	3568000,    3584000,    3600000,    3616000,
	3632000,    3648000,    3664000,    3680000,
	3696000,    3712000,	3728000,    3744000,
	3760000,    3776000,    3792000,    3808000,
	3824000,    3840000,    3856000,    3872000,
	3888000,    3904000,    3920000,    3936000,
	3952000,    3968000,    3984000,    4000000,
	4016000,    4032000,    4048000,    4064000,
	4080000,    4096000,    4112000,    4128000,
	4144000,    4160000,    4176000,    4192000,	
	4208000,    4224000,    4240000,    4256000,
	4272000,    4288000,    4304000,    4320000,
	4336000,    4352000,    4368000
};
#else
const kal_uint32 VBAT_CV_VTH[]=
{
	3504000,    3520000,    3536000,    3552000,
	3568000,    3584000,    3600000,    3616000,
	3632000,    3648000,    3664000,    3680000,
	3696000,    3712000,	3728000,    3744000,
	3760000,    3776000,    3792000,    3808000,
	3824000,    3840000,    3856000,    3872000,
	3888000,    3904000,    3920000,    3936000,
	3952000,    3968000,    3984000,    4000000,
	4016000,    4032000,    4048000,    4064000,
	4080000,    4096000,    4112000,    4128000,
	4144000,    4160000,    4176000,    4192000,	
	4208000,    4224000,    4240000,    4256000,
	4272000,    4288000,    4304000
};
#endif

const kal_uint32 CS_VTH[]=
{
	51200,  57600,  64000,  70400,
	76800,  83200,  89600,  96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000
}; 

const kal_uint32 INPUT_CS_VTH[]=
{
	CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_150_00_MA,	CHARGE_CURRENT_500_00_MA,  CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_2000_00_MA,  CHARGE_CURRENT_MAX
}; 

const kal_uint32 VCDT_HV_VTH[]=
{
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
};

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifndef CUST_GPIO_VIN_SEL
#define CUST_GPIO_VIN_SEL 18
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD 100 //100 ms
#define MSEC_TO_NSEC(x)		(x * 1000000UL)
  
  static DEFINE_MUTEX(diso_polling_mutex);
  static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
  static struct hrtimer diso_kthread_timer;
  static kal_bool diso_thread_timeout = KAL_FALSE;
  static struct delayed_work diso_polling_work;
  static void diso_polling_handler(struct work_struct *work);
  static DISO_Polling_Data DISO_Polling;
#else
  DISO_IRQ_Data DISO_IRQ;
#endif
  int g_diso_state	= 0;
  int vin_sel_gpio_number	= (CUST_GPIO_VIN_SEL | 0x80000000); 
  static kal_bool g_diso_otg = KAL_FALSE;
  
  static char *DISO_state_s[8] = {
		  "IDLE",
		  "OTG_ONLY",
		  "USB_ONLY",
		  "USB_WITH_OTG",
		  "DC_ONLY",
		  "DC_WITH_OTG",
		  "DC_WITH_USB",
		  "DC_USB_OTG",
  };
#endif


// ============================================================ //
// function prototype
// ============================================================ //


// ============================================================ //
//extern variable
// ============================================================ //

// ============================================================ //
//extern function
// ============================================================ //
extern void do_chrdet_int_task(void);
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern bool mt_usb_is_device(void);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern void mt_power_off(void);

static kal_uint32 charging_error = false;
static kal_uint32 charging_get_error_state(void);
static kal_uint32 charging_set_error_state(void *data);
 // ============================================================ //
kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

    battery_xlog_printk(BAT_LOG_CRTI, "array_size = %d \r\n", array_size);
    
	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    battery_xlog_printk(BAT_LOG_CRTI, "NO register value match. val=%d\r\n", val);
	//TODO: ASSERT(0);	// not find the value
	return 0;
}


static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
{
	kal_uint32 i;
	kal_uint32 max_value_in_last_element;

	if(pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if(max_value_in_last_element == KAL_TRUE)
	{
		for(i = (number-1); i != 0; i--)	 //max value in the last element
		{
			if(pList[i] <= level)
			{
				return pList[i];
			}	  
		}

		battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		return pList[0];
		//return CHARGE_CURRENT_0_00_MA;
	}
	else
	{
		for(i = 0; i< number; i++)  // max value in the first element
		{
			if(pList[i] <= level)
			{
				return pList[i];
			}	  
		}

		battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
		return pList[number -1];
		//return CHARGE_CURRENT_0_00_MA;
	}
}

static void hw_bc11_dump_register(void)
{
	kal_uint32 reg_val = 0;
	kal_uint32 reg_num = CHR_CON18;
	kal_uint32 i = 0;

	for(i=reg_num ; i<=CHR_CON19 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		battery_xlog_printk(BAT_LOG_FULL, "Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
	}
}


static void hw_bc11_init(void)
{
	Charger_Detect_Init();

	//RG_BC11_BIAS_EN=1	
	upmu_set_rg_bc11_bias_en(0x1);
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//BC11_RST=1
	upmu_set_rg_bc11_rst(0x1);
	//BC11_BB_CTRL=1
	upmu_set_rg_bc11_bb_ctrl(0x1);

	//msleep(10);
	mdelay(50);

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_init() \r\n");
		hw_bc11_dump_register();
	}	

}
 
 
static U32 hw_bc11_DCD(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_IPD_EN[1.0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=01
	upmu_set_rg_bc11_vref_vth(0x1);
	//RG_BC11_CMP_EN[1.0] = 10
	upmu_set_rg_bc11_cmp_en(0x2);

	//msleep(20);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_DCD() \r\n");
		hw_bc11_dump_register();
	}

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return wChargerAvail;
}
 
 
static U32 hw_bc11_stepA1(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 10
	upmu_set_rg_bc11_cmp_en(0x2);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA1() \r\n");
		hw_bc11_dump_register();
	}

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);

	return  wChargerAvail;
}
 
 
static U32 hw_bc11_stepB1(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 01
	//upmu_set_rg_bc11_ipu_en(0x1);
	upmu_set_rg_bc11_ipd_en(0x1);      
	//RG_BC11_VREF_VTH = [1:0]=10
	//upmu_set_rg_bc11_vref_vth(0x2);
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB1() \r\n");
		hw_bc11_dump_register();
	}

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}
 
 
static U32 hw_bc11_stepC1(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepC1() \r\n");
		hw_bc11_dump_register();
	}

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}
 
 
static U32 hw_bc11_stepA2(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_VSRC_EN[1.0] = 10 
	upmu_set_rg_bc11_vsrc_en(0x2);
	//RG_BC11_IPD_EN[1:0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA2() \r\n");
		hw_bc11_dump_register();
	}

	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);

	return  wChargerAvail;
}
 
 
static U32 hw_bc11_stepB2(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1:0]=10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x1);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB2() \r\n");
		hw_bc11_dump_register();
	}

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}
 
 
static void hw_bc11_done(void)
{
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=0
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_BIAS_EN=0
	upmu_set_rg_bc11_bias_en(0x0); 

	Charger_Detect_Release();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_done() \r\n");
		hw_bc11_dump_register();
	}

}

static kal_uint32 charging_hw_init(void *data)
{
	kal_uint32 status = STATUS_OK;

	upmu_set_rg_bc11_bb_ctrl(1);    //BC11_BB_CTRL    
	upmu_set_rg_bc11_rst(1);        //BC11_RST

	
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	mt_set_gpio_mode(vin_sel_gpio_number,0); // 0:GPIO mode
	mt_set_gpio_dir(vin_sel_gpio_number,0); // 0: input, 1: output
#endif

#ifdef GPIO_CHR_PSEL_PIN
	//pull PSEL low
	mt_set_gpio_mode(GPIO_CHR_PSEL_PIN,GPIO_MODE_GPIO);  
	mt_set_gpio_dir(GPIO_CHR_PSEL_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CHR_PSEL_PIN,GPIO_OUT_ZERO);
#endif    

#ifdef GPIO_CHR_CE_PIN 
	//pull CE low
	mt_set_gpio_mode(GPIO_CHR_CE_PIN,GPIO_MODE_GPIO);  
	mt_set_gpio_dir(GPIO_CHR_CE_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CHR_CE_PIN,GPIO_OUT_ZERO);    
#endif

	//battery_xlog_printk(BAT_LOG_FULL, "gpio_number=0x%x,gpio_on_mode=%d,gpio_off_mode=%d\n", gpio_number, gpio_on_mode, gpio_off_mode);	
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT) && defined(MTK_LOAD_SWITCH_FPF3040)
	bq24296_set_chg_config(0x0); // charger disable
	bq24296_set_en_hiz(0x1);
#else
	bq24296_set_en_hiz(0x0);
#endif

	bq24296_set_vindpm(0xA); //VIN DPM check 4.68V
	bq24296_set_reg_rst(0x0);
	bq24296_set_wdt_rst(0x1); //Kick watchdog	
	bq24296_set_sys_min(0x5); //Minimum system voltage 3.5V	
	bq24296_set_iprechg(0x0); //Precharge current 512mA //my 1900 is 128ma!!!
	bq24296_set_iterm(0x0); //Termination current 128mA

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    bq24296_set_vreg(0x35); //VREG 4.352V
#else
	bq24296_set_vreg(0x2C); //VREG 4.208V
#endif    

	bq24296_set_batlowv(0x1); //BATLOWV 3.0V
	bq24296_set_vrechg(0x00); //VRECHG 0.1V for 1900,rechargV = -300mv
	bq24296_set_en_term(0x1); //Enable termination
	bq24296_set_watchdog(0x1); //WDT 40s
	bq24296_set_en_timer(0x0); //Disable charge timer
	bq24296_set_int_mask(0x0); //Disable fault interrupt

	return status;
}


static kal_uint32 charging_dump_register(void *data)
{
	kal_uint32 status = STATUS_OK;

	battery_xlog_printk(BAT_LOG_CRTI, "charging_dump_register\r\n");

	bq24296_dump_register();

	return status;
}	


static kal_uint32 charging_enable(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
	bq24296_set_en_hiz(0x0);	            	
	bq24296_set_chg_config(0x1); // charger enable
	}
	else
	{

	#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if(mt_usb_is_device())
	#endif 			
		{
			if(g_runin_battery_test_flag==1)
			    bq24296_set_en_hiz(0x1);//for 1900
			bq24296_set_chg_config(0x0);
		}
	}

	return status;
}


static kal_uint32 charging_set_cv_voltage(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);	
	
#if defined(CONFIG_HQ_LENOVO_A1992_CHARGING_SUPPORT)
   if(cv_value == BATTERY_VOLT_04_200000_V)
	{
		cv_value = 4208000;
 	}
	else
	{
		cv_value = 4352000;
	}
#else
	if(cv_value == BATTERY_VOLT_04_200000_V)
	{
	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
		//highest of voltage will be 4.3V, because powerpath limitation
		cv_value = 4304000;		
	#else
		//use nearest value
		cv_value = 4208000;
#endif	    
 	}
#endif
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), cv_value);
	bq24296_set_vreg(register_value); 

	return status;
} 	


static kal_uint32 charging_get_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	//kal_uint32 array_size;
	//kal_uint8 reg_value;

	kal_uint8 ret_val=0;    
	kal_uint8 ret_force_20pct=0;

	//Get current level
	bq24296_read_interface(bq24296_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

	//Get Force 20% option
	bq24296_read_interface(bq24296_CON2, &ret_force_20pct, CON2_FORCE_20PCT_MASK, CON2_FORCE_20PCT_SHIFT);

	//Parsing
	ret_val = (ret_val*64) + 512;

	if (ret_force_20pct == 0)
	{
		//Get current level
		//array_size = GETARRAYNUM(CS_VTH);
		//*(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
		*(kal_uint32 *)data = ret_val;
	}   
	else
	{
		//Get current level
		//array_size = GETARRAYNUM(CS_VTH_20PCT);
		//*(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
		//return (int)(ret_val<<1)/10;
		*(kal_uint32 *)data = (int)(ret_val<<1)/10;
	}   

	return status;
}  
  


static kal_uint32 charging_set_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;

	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
	bq24296_set_ichg(register_value);

	return status;
} 	
 

static kal_uint32 charging_set_input_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;

	array_size = GETARRAYNUM(INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(kal_uint32 *)data);
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);	

	bq24296_set_iinlim(register_value);

	return status;
} 	


static kal_uint32 charging_get_charging_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq24296_get_chrg_stat();

	if(ret_val == 0x3)
		*(kal_uint32 *)data = KAL_TRUE;
	else
		*(kal_uint32 *)data = KAL_FALSE;

	return status;
} 	


static kal_uint32 charging_reset_watch_dog_timer(void *data)
{
	kal_uint32 status = STATUS_OK;

	battery_xlog_printk(BAT_LOG_CRTI, "charging_reset_watch_dog_timer\r\n");

	bq24296_set_wdt_rst(0x1); //Kick watchdog

	return status;
}
 
 
static kal_uint32 charging_set_hv_threshold(void *data)
{
	kal_uint32 status = STATUS_OK;

	kal_uint32 set_hv_voltage;
	kal_uint32 array_size;
	kal_uint16 register_value;
	kal_uint32 voltage = *(kal_uint32*)(data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
	upmu_set_rg_vcdt_hv_vth(register_value);

	return status;
}
 
 
static kal_uint32 charging_get_hv_status(void *data)
{
	kal_uint32 status = STATUS_OK;

	*(kal_bool*)(data) = upmu_get_rgs_vcdt_hv_det();

	return status;
}


static kal_uint32 charging_get_battery_status(void *data)
{
	kal_uint32 status = STATUS_OK;

	upmu_set_baton_tdet_en(1);	
	upmu_set_rg_baton_en(1);
	*(kal_bool*)(data) = upmu_get_rgs_baton_undet();

	return status;
}


static kal_uint32 charging_get_charger_det_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_xlog_printk(BAT_LOG_CRTI,"[charging_get_charger_det_status] charger exist for bring up.\n"); 
#else    
	#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		val = upmu_get_rgs_chrdet();
	#else
		if(((g_diso_state >> 1) & 0x3) != 0x0 || (upmu_get_rgs_chrdet() && !g_diso_otg))
			val = KAL_TRUE;
		else
			val = KAL_FALSE;
	#endif

#endif

	*(kal_bool*)(data) = val;
	if(val == 0)
		g_charger_type = CHARGER_UNKNOWN;

	return status;
}



kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}

extern CHARGER_TYPE hw_charger_type_detection(void);

static kal_uint32 charging_get_charger_type(void *data)
{
	kal_uint32 status = STATUS_OK;
	CHARGER_TYPE charger_type = CHARGER_UNKNOWN;
#if defined(CONFIG_POWER_EXT)
	*(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
	charging_type_det_done = KAL_FALSE;

	charger_type = hw_charger_type_detection();
	battery_xlog_printk(BAT_LOG_CRTI, "charging_get_charger_type = %d\r\n", charger_type);

	*(CHARGER_TYPE*)(data) = charger_type;

#if 0
	/********* Step initial  ***************/		 
	hw_bc11_init();

	/********* Step DCD ***************/  
	if(1 == hw_bc11_DCD())
	{
		/********* Step A1 ***************/
		if(1 == hw_bc11_stepA1())
		{
			/********* Step B1 ***************/
			if(1 == hw_bc11_stepB1())
			{
				//*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
				//battery_xlog_printk(BAT_LOG_CRTI, "step B1 : Non STANDARD CHARGER!\r\n");				
				*(CHARGER_TYPE*)(data) = APPLE_2_1A_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step B1 : Apple 2.1A CHARGER!\r\n");
			}	 
			else
			{
				//*(CHARGER_TYPE*)(data) = APPLE_2_1A_CHARGER;
				//battery_xlog_printk(BAT_LOG_CRTI, "step B1 : Apple 2.1A CHARGER!\r\n");
				*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step B1 : Non STANDARD CHARGER!\r\n");
			}	 
		}
		else
		{
			/********* Step C1 ***************/
			if(1 == hw_bc11_stepC1())
			{
				*(CHARGER_TYPE*)(data) = APPLE_1_0A_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step C1 : Apple 1A CHARGER!\r\n");
			}	 
			else
			{
				*(CHARGER_TYPE*)(data) = APPLE_0_5A_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step C1 : Apple 0.5A CHARGER!\r\n");			 
			}	 
		}

	}
	else
	{
		/********* Step A2 ***************/
		if(1 == hw_bc11_stepA2())
		{
			/********* Step B2 ***************/
			if(1 == hw_bc11_stepB2())
			{
				*(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\r\n");
			}
			else
			{
				*(CHARGER_TYPE*)(data) = CHARGING_HOST;
				battery_xlog_printk(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			}
		}
		else
		{
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			battery_xlog_printk(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		}

	}

	/********* Finally setting *******************************/
	hw_bc11_done();
#endif

	charging_type_det_done = KAL_TRUE;
	g_charger_type = *(CHARGER_TYPE*)(data);

#endif
	return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
    kal_uint32 status = STATUS_OK;

    if(slp_get_wake_reason() == WR_PCM_TIMER)
        *(kal_bool*)(data) = KAL_TRUE;
    else
        *(kal_bool*)(data) = KAL_FALSE;

    battery_xlog_printk(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
       
    return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_platform_reset\n");
 
    //arch_reset(0,NULL);
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    *(kal_uint32*)(data) = get_boot_mode();

    battery_xlog_printk(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_power_off\n");
    mt_power_off();
         
    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
    kal_uint32 status = STATUS_OK;

#if 0 //#if defined(MTK_POWER_EXT_DETECT)
    if (MT_BOARD_PHONE == mt_get_board_type())
        *(kal_bool *)data = KAL_FALSE;
    else
        *(kal_bool *)data = KAL_TRUE;
#else
        *(kal_bool *)data = KAL_FALSE;
#endif

    return status;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;  
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
void set_diso_otg(bool enable)
{
	g_diso_otg = enable;
}

void set_vusb_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vusb_polling_measure.notify_irq_en = enable;
	DISO_Polling.vusb_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
#else
	kal_uint16 threshold = 0;
	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vusb_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vusb_measure_channel.rising_threshold;

		threshold = (threshold *R_DISO_VBUS_PULL_DOWN)/(R_DISO_VBUS_PULL_DOWN + R_DISO_VBUS_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vusb_measure_channel.number, threshold, \
	 	DISO_IRQ.vusb_measure_channel.period, DISO_IRQ.vusb_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vusb_measure_channel.number);
	}
#endif
	battery_xlog_printk(BAT_LOG_FULL, " [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}
 
void set_vdc_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vdc_polling_measure.notify_irq_en = enable;
	DISO_Polling.vdc_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
#else
	kal_uint16 threshold = 0;
	if(enable) {
		if(flag == 0)
		threshold = DISO_IRQ.vdc_measure_channel.falling_threshold;
		else
		threshold = DISO_IRQ.vdc_measure_channel.rising_threshold;

		threshold = (threshold *R_DISO_DC_PULL_DOWN)/(R_DISO_DC_PULL_DOWN + R_DISO_DC_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vdc_measure_channel.number, threshold, \
		DISO_IRQ.vdc_measure_channel.period, DISO_IRQ.vdc_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vdc_measure_channel.number);
	}
#endif
	battery_xlog_printk(BAT_LOG_FULL, " [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}
 
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
static void diso_polling_handler(struct work_struct *work)
{
	int trigger_channel = -1;
	int trigger_flag = -1;

	if(DISO_Polling.vdc_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VDC_CHANNEL;
	else if(DISO_Polling.vusb_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VUSB_CHANNEL;

	battery_xlog_printk(BAT_LOG_CRTI, "[DISO]auxadc handler triggered\n" );
	switch(trigger_channel)
	{
		case AP_AUXADC_DISO_VDC_CHANNEL:
			trigger_flag = DISO_Polling.vdc_polling_measure.notify_irq;
			battery_xlog_printk(BAT_LOG_CRTI, "[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag );
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			if (trigger_flag == DISO_IRQ_RISING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[2]);
			}
#else //for load switch OTG leakage handle
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
			if (trigger_flag == DISO_IRQ_RISING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[5]);
			} else if (trigger_flag == DISO_IRQ_FALLING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[1]);
			}
			else
				battery_xlog_printk(BAT_LOG_CRTI, "[%s] wrong trigger flag!\n",__func__);
#endif
			break;
		case AP_AUXADC_DISO_VUSB_CHANNEL:
			trigger_flag = DISO_Polling.vusb_polling_measure.notify_irq;
			battery_xlog_printk(BAT_LOG_CRTI, "[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			if(trigger_flag == DISO_IRQ_FALLING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[4]);
			} else if (trigger_flag == DISO_IRQ_RISING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[6]);
			}
			else
				battery_xlog_printk(BAT_LOG_CRTI, "[%s] wrong trigger flag!\n",__func__);
				set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
			break;
		default:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			battery_xlog_printk(BAT_LOG_CRTI, "[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
			return; /* in error or unexecpt state just return */
	}

	g_diso_state = *(int*)&DISO_data.diso_state;
	battery_xlog_printk(BAT_LOG_CRTI, "[DISO]g_diso_state: 0x%x\n", g_diso_state);
	DISO_data.irq_callback_func(0, NULL);

	return ;
}
#else
static irqreturn_t diso_auxadc_irq_handler(int irq, void *dev_id)
{
	int trigger_channel = -1;
	int trigger_flag = -1;
	trigger_channel = mt_auxadc_getCurrentChannel();
	battery_xlog_printk(BAT_LOG_CRTI, "[DISO]auxadc handler triggered\n" );
	switch(trigger_channel)
	{
		case AP_AUXADC_DISO_VDC_CHANNEL:
			trigger_flag = mt_auxadc_getCurrentTrigger();
			battery_xlog_printk(BAT_LOG_CRTI, "[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag );
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			if (trigger_flag == DISO_IRQ_RISING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[2]);
			}
#else //for load switch OTG leakage handle
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
			if (trigger_flag == DISO_IRQ_RISING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[5]);
				} else {
				DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[1]);
			}
#endif
			break;
		case AP_AUXADC_DISO_VUSB_CHANNEL:
			trigger_flag = mt_auxadc_getCurrentTrigger();
			battery_xlog_printk(BAT_LOG_CRTI, "[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag);
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			if(trigger_flag == DISO_IRQ_FALLING) {
				DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[4]);
			} else {
				DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
				DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
				DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
				DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " cur diso_state is %s!\n",DISO_state_s[6]);
			}

			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
			break;
		default:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			battery_xlog_printk(BAT_LOG_CRTI, "[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
			return IRQ_HANDLED; /* in error or unexecpt state just return */
	}
	g_diso_state = *(int*)&DISO_data.diso_state;
	return IRQ_WAKE_THREAD;
}
#endif
 
#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
void vdc_eint_handler()
{
	battery_xlog_printk(BAT_LOG_CRTI, "[diso_eint] vdc eint irq triger\n");
	DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
	mt_eint_mask(CUST_EINT_VDC_NUM); 
	do_chrdet_int_task();
}
#endif
 
static kal_uint32 diso_get_current_voltage(int Channel)
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, times = 5;

	if( IMM_IsAdcInitReady() == 0 ) {
		battery_xlog_printk(BAT_LOG_CRTI, "[DISO] AUXADC is not ready");
		return 0;
	}

	i = times;
	while (i--)
	{
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if(ret_value == 0) {
			ret += ret_temp;
		} else {
			times = times > 1 ? times - 1 : 1;
			battery_xlog_printk(BAT_LOG_CRTI, "[diso_get_current_voltage] ret_value=%d, times=%d\n",
			ret_value, times);
		}
	}

	ret = ret*1500/4096 ;
	ret = ret/times;

	return  ret;
}
 
static void _get_diso_interrupt_state(void)
{
	int vol = 0;
	int diso_state =0;
	int check_times = 30;
	kal_bool vin_state = KAL_FALSE;

#ifndef VIN_SEL_FLAG
	mdelay(AUXADC_CHANNEL_DELAY_PERIOD);
#endif

	vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vol =(R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vol/(R_DISO_DC_PULL_DOWN)/100;
	battery_xlog_printk(BAT_LOG_CRTI, "[DISO]	Current DC voltage mV = %d\n", vol);

#ifdef VIN_SEL_FLAG
	/* set gpio mode for kpoc issue as DWS has no default setting */
	mt_set_gpio_mode(vin_sel_gpio_number,0); // 0:GPIO mode
	mt_set_gpio_dir(vin_sel_gpio_number,0); // 0: input, 1: output

	if (vol > VDC_MIN_VOLTAGE/1000 && vol < VDC_MAX_VOLTAGE/1000) {
		/* make sure load switch already switch done */
		do{
			check_times--;
#ifdef VIN_SEL_FLAG_DEFAULT_LOW
			vin_state = mt_get_gpio_in(vin_sel_gpio_number);
#else
			vin_state = mt_get_gpio_in(vin_sel_gpio_number);
			vin_state = (~vin_state) & 0x1;
#endif
			if(!vin_state)
			mdelay(5);
		} while ((!vin_state) && check_times);
		battery_xlog_printk(BAT_LOG_CRTI, "[DISO] i==%d  gpio_state= %d\n",
		check_times, mt_get_gpio_in(vin_sel_gpio_number));

		if (0 == check_times)
			diso_state &= ~0x4; //SET DC bit as 0
		else
			diso_state |= 0x4; //SET DC bit as 1
	} else {
		diso_state &= ~0x4; //SET DC bit as 0
	}
#else
	mdelay(SWITCH_RISING_TIMING + LOAD_SWITCH_TIMING_MARGIN); /* force delay for switching as no flag for check switching done */
	if (vol > VDC_MIN_VOLTAGE/1000 && vol < VDC_MAX_VOLTAGE/1000)
		diso_state |= 0x4; //SET DC bit as 1
	else
		diso_state &= ~0x4; //SET DC bit as 0
#endif


	vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vol =(R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vol/(R_DISO_VBUS_PULL_DOWN)/100;
	battery_xlog_printk(BAT_LOG_CRTI, "[DISO]	Current VBUS voltage  mV = %d\n",vol);

	if (vol > VBUS_MIN_VOLTAGE/1000 && vol < VBUS_MAX_VOLTAGE/1000) {
		if(!mt_usb_is_device()) {
			diso_state |= 0x1; //SET OTG bit as 1
			diso_state &= ~0x2; //SET VBUS bit as 0
		} else {
			diso_state &= ~0x1; //SET OTG bit as 0
			diso_state |= 0x2; //SET VBUS bit as 1;
		}

	} else {
		diso_state &= 0x4; //SET OTG and VBUS bit as 0
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[DISO] DISO_STATE==0x%x \n",diso_state);
	g_diso_state = diso_state;
	return;
}
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
int _get_irq_direction(int pre_vol, int cur_vol)
{
	int ret = -1;

	//threshold 1000mv
	if((cur_vol - pre_vol) > 1000)
		ret = DISO_IRQ_RISING;
	else if((pre_vol - cur_vol) > 1000) 
		ret = DISO_IRQ_FALLING;

	return ret;
}
 
static void _get_polling_state(void)
{
	int vdc_vol = 0, vusb_vol = 0;
	int vdc_vol_dir = -1;
	int vusb_vol_dir = -1;

	DISO_polling_channel* VDC_Polling = &DISO_Polling.vdc_polling_measure;
	DISO_polling_channel* VUSB_Polling = &DISO_Polling.vusb_polling_measure;

	vdc_vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vdc_vol =(R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vdc_vol/(R_DISO_DC_PULL_DOWN)/100;

	vusb_vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vusb_vol =(R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vusb_vol/(R_DISO_VBUS_PULL_DOWN)/100;

	VDC_Polling->preVoltage = VDC_Polling->curVoltage;
	VUSB_Polling->preVoltage = VUSB_Polling->curVoltage;
	VDC_Polling->curVoltage = vdc_vol;
	VUSB_Polling->curVoltage = vusb_vol;

	if (DISO_Polling.reset_polling)
	{
		DISO_Polling.reset_polling = KAL_FALSE;
		VDC_Polling->preVoltage = vdc_vol;
		VUSB_Polling->preVoltage = vusb_vol;

		if(vdc_vol > 1000)
		vdc_vol_dir = DISO_IRQ_RISING;
		else
		vdc_vol_dir = DISO_IRQ_FALLING;

		if(vusb_vol > 1000)
		vusb_vol_dir = DISO_IRQ_RISING;
		else
		vusb_vol_dir = DISO_IRQ_FALLING;
	}
	else
	{
		//get voltage direction
		vdc_vol_dir = _get_irq_direction(VDC_Polling->preVoltage, VDC_Polling->curVoltage);
		vusb_vol_dir = _get_irq_direction(VUSB_Polling->preVoltage, VUSB_Polling->curVoltage);
	}

	if(VDC_Polling->notify_irq_en && 
	(vdc_vol_dir == VDC_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10*HZ/1000); //10ms
		battery_xlog_printk(BAT_LOG_CRTI, "[%s] ready to trig VDC irq, irq: %d\n",
		__func__,VDC_Polling->notify_irq);
	} else if(VUSB_Polling->notify_irq_en && 
	(vusb_vol_dir == VUSB_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10*HZ/1000);
		battery_xlog_printk(BAT_LOG_CRTI, "[%s] ready to trig VUSB irq, irq: %d\n",
		__func__, VUSB_Polling->notify_irq);
	} else if((vdc_vol == 0) && (vusb_vol == 0)) {
		VDC_Polling->notify_irq_en = 0;
		VUSB_Polling->notify_irq_en = 0;
	}

	return;
}
 
enum hrtimer_restart diso_kthread_hrtimer_func(struct hrtimer *timer)
{
	diso_thread_timeout = KAL_TRUE;
	wake_up(&diso_polling_thread_wq);

	return HRTIMER_NORESTART;
}
 
int diso_thread_kthread(void *x)
{
	/* Run on a process content */
	while (1) {
		wait_event(diso_polling_thread_wq, (diso_thread_timeout == KAL_TRUE));

		diso_thread_timeout = KAL_FALSE;

		mutex_lock(&diso_polling_mutex);

		_get_polling_state();

		if (DISO_Polling.vdc_polling_measure.notify_irq_en ||
		DISO_Polling.vusb_polling_measure.notify_irq_en)
		hrtimer_start(&diso_kthread_timer,ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),HRTIMER_MODE_REL);
		else
		hrtimer_cancel(&diso_kthread_timer);

		mutex_unlock(&diso_polling_mutex);
	}

	return 0;
}
#endif
#endif


static kal_uint32 charging_get_error_state(void)
{
	return charging_error;
}

static kal_uint32 charging_set_error_state(void *data)
{
	kal_uint32 status = STATUS_OK;
	charging_error = *(kal_uint32*)(data);

	return status;
}

static kal_uint32 charging_diso_init(void *data)
{	 
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state	 = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state	 = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;
	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	diso_kthread_timer.function = diso_kthread_hrtimer_func;
	INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

	kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
	battery_xlog_printk(BAT_LOG_CRTI, "[%s] done\n", __func__);
#else
	struct device_node *node;
	int ret;

	//Initial AuxADC IRQ
	DISO_data.irq_line_number = LOWBATTERY_IRQ_ID;
	DISO_IRQ.vdc_measure_channel.number   = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number  = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period   = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period  = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce	  = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce  = AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine*/
	DISO_IRQ.vusb_measure_channel.falling_threshold  = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold   = VDC_MIN_VOLTAGE/1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold  = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold	  = VDC_MIN_VOLTAGE/1000;

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler, \
	pDISO_data->irq_callback_func, IRQF_ONESHOT  , "DISO_ADC_IRQ", NULL);

	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI, "[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		battery_xlog_printk(BAT_LOG_CRTI, "[diso_adc]: diso_init success.\n");
	}
#endif

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
	battery_xlog_printk(BAT_LOG_CRTI, "[diso_eint]vdc eint irq registitation\n");
	mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
	mt_eint_mask(CUST_EINT_VDC_NUM); 
#endif
#endif

	return status;
}

static kal_uint32 charging_get_diso_state(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
	if(((diso_state >> 1) & 0x3) != 0x0)
	{
		switch (diso_state){
			case USB_ONLY:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
				mt_eint_unmask(CUST_EINT_VDC_NUM); 
#else
				set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
				pDISO_data->diso_state.cur_vusb_state  = DISO_ONLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_OFFLINE;
				break;
			case DC_ONLY:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
				pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_ONLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_OFFLINE;
				break;
			case DC_WITH_USB:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_ENABLE,DISO_IRQ_FALLING);
				pDISO_data->diso_state.cur_vusb_state  = DISO_ONLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_ONLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_OFFLINE;
				break;
			case DC_WITH_OTG:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
				pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_ONLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_ONLINE;
				break;
			default: // OTG only also can trigger vcdt IRQ
				pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_ONLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " switch load vcdt irq triggerd by OTG Boost!\n");
				break; // OTG plugin no need battery sync action
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return status;
}


static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
{
	charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
	,charging_diso_init
	,charging_get_diso_state
};

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	kal_int32 status;
	if(cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd](data);
	else
		return STATUS_UNSUPPORTED;

	return status;
}


