#ifndef __GSL_TS_DRIVER_H__
#define __GSL_TS_DRIVER_H__
/*********************************/
#define TPD_HAVE_BUTTON		//按键宏
#define GSL_ALG_ID		//有没有id算法
#define GSL_DEBUG			//调试
#define TPD_PROC_DEBUG		//adb调试
#define GSL_TIMER				//定时器宏
//#define TPD_PROXIMITY			//距离传感器宏
#define GSL_GPIO_IDT_TP
#define GSL_GESTURE
#define GSL9XX_VDDIO_1800		1
#define TPD_DEBUG_TIME	0x20130424
struct gsl_touch_info
{
	int x[10];
	int y[10];
	int id[10];
	int finger_num;	
};

struct gsl_ts_data {
	struct i2c_client *client;
	struct workqueue_struct *wq;
	struct work_struct work;
	unsigned int irq;
	struct early_suspend pm;
};

/*button*/
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{80,1030,120,80},{240,1030,120,80},{460,1030,120,80}}


#ifdef GSL_ALG_ID 
extern unsigned int gsl_mask_tiaoping(void);
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);
#endif
/* Fixme mem Alig */
/*
struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};
*/
#include "gsl_ts_fw.h"
static unsigned char gsl_cfg_index = 0;

struct fw_config_type
{
	const struct fw_data *fw;
	unsigned int fw_size;
	unsigned int *data_id;
	unsigned int data_size;
};
static struct fw_config_type gsl_cfg_table[9] = {
/*0*/{GSLX68X_FW_1,(sizeof(GSLX68X_FW_1)/sizeof(struct fw_data)),
	gsl_config_data_id_1,(sizeof(gsl_config_data_id_1)/4)},
/*1*/{GSLX68X_FW_2,(sizeof(GSLX68X_FW_2)/sizeof(struct fw_data)),
	gsl_config_data_id_2,(sizeof(gsl_config_data_id_2)/4)},
};

#endif
