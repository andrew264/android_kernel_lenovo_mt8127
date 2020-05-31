/******************************************************************************

  Copyright (C), 2010-2012, Silead, Inc.

 ******************************************************************************
Filename      : gsl1680-d0.c
Version       : R2.0
Aurthor       : mark_huang
Creattime     : 2012.6.20
Description   : Driver for Silead I2C touchscreen.

 ******************************************************************************/
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/upmu_common.h>

#include "gsl_ts_driver.h"
int gesture_power_flage=0;
extern struct tpd_device *tpd;

static struct gsl_ts_data *ddata = NULL;

static int boot_mode = NORMAL_BOOT;

#define GSL_DEV_NAME "gsl1680"

#define I2C_TRANS_SPEED 400	//100 khz or 400 khz
#define TPD_REG_BASE 0x00

////////////////
//#define KEY_SVCHANGED 271
//////////////////////
static const struct i2c_device_id gsl_device_id[] = {{"gsl_tp",0},{}};
static unsigned short force[] = {0,0x80,I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gsl_tp", (0x80>>1))};

static volatile int gsl_halt_flag = 0;
static struct mutex gsl_i2c_lock;
#ifdef GSL_GESTURE
typedef enum{
	GE_DISABLE = 0,
	GE_ENABLE = 1,
	GE_WAKEUP = 2,
	GE_NOWORK =3,
}GE_T;
static GE_T gsl_gesture_status = GE_DISABLE;
static unsigned int gsl_gesture_flag = 1;
static char gsl_gesture_c = 0;
extern int gsl_obtain_gesture(void);
extern void gsl_FunIICRead(unsigned int (*fun) (unsigned int *,unsigned int,unsigned int));
extern void gsl_GestureExternInt(unsigned int* model,int len);
unsigned int gsl_model_extern[] ={
	0x10,'N',
	0x04031800,0x07064e33,0x0808856a,0x0808bba0,0x0707f2d7,0x0e0ad0eb,0x141199b4,0x1e18637e,
	0x34282d47,0x63481218,0x917e321e,0xa19d684d,0xa0a29f84,0xa49fd5ba,0xc8affef1,0xffe3fcff,
	0x10,'N',
	0x1d28ebfe,0x0c14bfd6,0x020692a9,0x0000657b,0x0e053a4f,0x2a1a1526,0x543d0209,0x816a0200,
	0xaa97190b,0xbfb7432c,0xc6c4705a,0xc5c69e87,0xbbc1cab5,0xa7b0f7e0,0xd1bafbfc,0xffe8fffb,
	0x10,'E',
	0x9e858784,0xd2b87a82,0xf8e9566c,0xf6ff243c,0xcbe20813,0x97b10103,0x637d0601,0x364b1e0f,
	0x14234731,0x020a785e,0x0200ac92,0x200ed6c3,0x4f36f0e6,0x8269fcf8,0xb69cfeff,0xead0f5fb,

}
;


#endif

#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/wakelock.h>
static u8 tpd_proximity_flag = 0; //flag whether start alps
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
static struct wake_lock ps_lock;
static u8 gsl_psensor_data[8]={0};
static int gsl_ps_enable = 0;
#endif

#ifdef GSL_TIMER
#define GSL_TIMER_CHECK_CIRCLE        200
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static char int_1st[4];
static char int_2nd[4];
#endif

#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

#define GSL_TEST_TP //wu add rawdata test 2015.11.5
#ifdef GSL_TEST_TP
extern void gsl_write_test_config(unsigned int cmd,int value);
extern unsigned int gsl_read_test_config(unsigned int cmd);
extern int gsl_obtain_array_data_ogv(int*ogv,int i_max,int j_max);
extern int gsl_obtain_array_data_dac(unsigned int *dac,int i_max,int j_max);
extern int gsl_tp_module_test(char *buf,int size);
static int open_short_row;
static int open_short_col;
#define GSL_PARENT_PROC_NAME "touchscreen"
#define GSL_OPENHSORT_PROC_NAME "ctp_openshort_test"
#define GSL_RAWDATA_PROC_NAME "ctp_rawdata"
#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct task_struct *thread = NULL;
static int tpd_flag = 0;

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
		do{                              \
		    printk("[tp-gsl][%s]"fmt,__func__, ##args);     \
		}while(0)
#else
#define print_info(fmt, args...)
#endif

#ifdef TPD_HAVE_BUTTON
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

static int gsl_read_interface(struct i2c_client *client,
        u8 reg, u8 *buf, u32 num)
{
	int err = 0;
	int i;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (temp+8)&0x5c;
			i2c_master_send(client,&temp,1);
			err = i2c_master_recv(client,&buf[0],4);
		temp = reg;
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],4);
	}
	for(i=0;i<num;)
	{
		temp = reg + i;
		i2c_master_send(client,&temp,1);
		if((i+8)<num)
			err = i2c_master_recv(client,(buf+i),8);
		else
			err = i2c_master_recv(client,(buf+i),(num-i));
		i+=8;
	}
	mutex_unlock(&gsl_i2c_lock);

	return err;
}

static int gsl_write_interface(struct i2c_client *client,
        const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1] = {0};
	int err;
	u8 tmp_buf[num + 1];

	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = 0;//client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	xfer_msg[0].timing = 400;//I2C_TRANS_SPEED;
	mutex_lock(&gsl_i2c_lock);

	err = i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);

	
	return err;
}
#ifdef GSL_TEST_TP
void gsl_I2C_ROnePage(unsigned int addr, char *buf)
{
	u8 tmp_buf[4]={0};
	tmp_buf[3]=(u8)(addr>>24);
	tmp_buf[2]=(u8)(addr>>16);
	tmp_buf[1]=(u8)(addr>>8);
	tmp_buf[0]=(u8)(addr);
	gsl_write_interface(ddata->client,0xf0,tmp_buf,4);
	gsl_read_interface(ddata->client,0,buf,128);
}
EXPORT_SYMBOL(gsl_I2C_ROnePage);
void gsl_I2C_RTotal_Address(unsigned int addr,unsigned int *data)
{
	u8 tmp_buf[4]={0};	
	tmp_buf[3]=(u8)((addr/0x80)>>24);
	tmp_buf[2]=(u8)((addr/0x80)>>16);
	tmp_buf[1]=(u8)((addr/0x80)>>8);
	tmp_buf[0]=(u8)((addr/0x80));
	gsl_write_interface(ddata->client,0xf0,tmp_buf,4);
	gsl_read_interface(ddata->client,addr%0x80,tmp_buf,4);
	*data = tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
}
EXPORT_SYMBOL(gsl_I2C_RTotal_Address);
#endif
#ifdef GSL_TEST_TP
static ssize_t gsl_test_show(void)
{
	static int gsl_test_flag = 0; 
	char *tmp_buf;
	int err;
	int result = 0;
	printk("enter gsl_test_show start::gsl_test_flag  = %d\n",gsl_test_flag);
	if(gsl_test_flag == 1){
		return 0;	
	}
	gsl_test_flag = 1;
	tmp_buf = kzalloc(3*1024,GFP_KERNEL);
	if(!tmp_buf){
		printk(" kzalloc  kernel  fail\n");
		return 0;
		}
	err = gsl_tp_module_test(tmp_buf,3*1024);
	printk("enter gsl_test_show end\n");
	if(err > 0){
		printk("tp test pass\n");
		result = 1;
	}else{
		printk("tp test failure\n");
		result = 0;
	}
	kfree(tmp_buf);
	gsl_test_flag = 0; 
	return result;
}
static s32 gsl_openshort_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}
static s32 gsl_openshort_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	//char *ptr = buf;
	int test_result  = 0;
	if(*ppos)
	{
		printk("[%s]:tp test again return\n",__func__);
		return 0;
	}
	*ppos += 16;
	test_result = gsl_test_show();
	memset(buf,'\0',16);
	count = 16;
	if(1 == test_result)
	{
		printk("[%s]:tp test pass\n",__func__);
		return sprintf(buf, "result=%d\n", 1);
	}
	else
	{
		printk("[%s]:tp test failure\n",__func__);
		return sprintf(buf, "result=%d\n", 0);
	}
	//return count;
}
static s32 gsl_rawdata_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
	{
		int i,j,ret;
		static short* gsl_ogv;
		ssize_t read_buf_chars = 0; 
		gsl_ogv = kzalloc(24*14*2,GFP_KERNEL);
		static short* gsl_dac;
		gsl_dac = kzalloc(4*14*2,GFP_KERNEL);
		if(!gsl_ogv){
			return -1;
		}  
		if(*ppos)
		{
			printk("[%s]:tp test again return\n",__func__);
			return 0;
		}
		ret=gsl_test_show();
		gsl_obtain_array_data_ogv(gsl_ogv,24,14);
		for(i=0;i<24*14;i++)
		{
			read_buf_chars += sprintf(&(buf[read_buf_chars])," _%u_ ",gsl_ogv[i]);
			if(!((i+1)%14))
			{
				buf[read_buf_chars++] = '\n';
			}
		}
		for(i=0;i<24;i++)
		{
			for(j=0;j<14;j++){
			printk("%4d ",gsl_ogv[i*14+j]);
						}
			printk("\n");
		 }
		buf[read_buf_chars-1] = '\n';
		gsl_obtain_array_data_dac(gsl_dac,4,14);
		printk("gsl dac value:\n");
		for(i=0;i<4;i++)
		{
			for(j=0;j<14;j++)
			{
				read_buf_chars += sprintf(&(buf[read_buf_chars])," _%u_ ",gsl_dac[i*14+j]);
				if(!((i*14+j+1)%14))
				{
					buf[read_buf_chars++] = '\n';
				}
			}
		}
		for(i=0;i<4;i++)
		{
			for(j=0;j<14;j++){
			printk("%4d ",gsl_dac[i*14+j]);
						}
			printk("\n");
		 }
		printk("\n");
		*ppos += read_buf_chars; 
		return read_buf_chars; 
	}
static s32 gsl_rawdata_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}
static const struct file_operations gsl_rawdata_procs_fops =
{
	.write = gsl_rawdata_proc_write,
	.read = gsl_rawdata_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static const struct file_operations gsl_openshort_procs_fops =
{
    .write = gsl_openshort_proc_write,
    .read = gsl_openshort_proc_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};
void create_ctp_proc(void)
{
	struct proc_dir_entry *gsl_device_proc = NULL;
	struct proc_dir_entry *gsl_openshort_proc = NULL;
	struct proc_dir_entry *gsl_rawdata_proc = NULL;
	gsl_device_proc = proc_mkdir(GSL_PARENT_PROC_NAME, NULL);
	if(gsl_device_proc == NULL)
    	{
        	printk("gsl915: create parent_proc fail\n");
        	return;
    	}
	gsl_openshort_proc = proc_create(GSL_OPENHSORT_PROC_NAME, 0666, gsl_device_proc, &gsl_openshort_procs_fops);
    	if (gsl_openshort_proc == NULL)
    	{
        	printk("gsl915: create openshort_proc fail\n");
    	}
	gsl_rawdata_proc = proc_create(GSL_RAWDATA_PROC_NAME, 0777, gsl_device_proc, &gsl_rawdata_procs_fops);
    	if (gsl_rawdata_proc == NULL)
    	{
        	printk("gsl915: create ctp_rawdata_proc fail\n");
    	}
}
#endif
#ifdef GSL_GPIO_IDT_TP
static int gsl_read_TotalAdr(struct i2c_client *client,u32 addr,u32 *data)
{
	u8 buf[4];
	int err;
	buf[3]=(u8)((addr/0x80)>>24);
	buf[2]=(u8)((addr/0x80)>>16);
	buf[1]=(u8)((addr/0x80)>>8);
	buf[0]=(u8)((addr/0x80));
	gsl_write_interface(client,0xf0,buf,4);
	err = gsl_read_interface(client,addr%0x80,buf,4);
	if(err > 0){
		*data = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
	}
	return err;
}
static int gsl_write_TotalAdr(struct i2c_client *client,u32 addr,u32 *data)
{
	int err;
	u8 buf[4];
	u32 value = *data;
	buf[3]=(u8)((addr/0x80)>>24);
	buf[2]=(u8)((addr/0x80)>>16);
	buf[1]=(u8)((addr/0x80)>>8);
	buf[0]=(u8)((addr/0x80));
	gsl_write_interface(client,0xf0,buf,4);
	buf[3]=(u8)((value)>>24);
	buf[2]=(u8)((value)>>16);
	buf[1]=(u8)((value)>>8);
	buf[0]=(u8)((value));
	err = gsl_write_interface(client,addr%0x80,buf,4);
	return err;
}
static int gsl_gpio_idt_tp(struct gsl_ts_data *ts)
{
	int i;
	u32 value = 0;
	u32 ru,rd,tu,td;
	u8 rstate,tstate;
	value = 0x1;
	gsl_write_TotalAdr(ts->client,0xff000084,&value);
	for(i=0;i<3;i++){
		gsl_read_TotalAdr(ts->client,0xff020004,&value);
	}
	ru = value & 0x1;
	value = 0x00011112;
	gsl_write_TotalAdr(ts->client,0xff080058,&value);

	for(i=0;i<3;i++){
		gsl_read_TotalAdr(ts->client,0xff020004,&value);
	}
	tu = (value & (0x1 << 1))>>1;

	value = 0x2;
	gsl_write_TotalAdr(ts->client,0xff000084,&value);
	for(i=0;i<3;i++){
		gsl_read_TotalAdr(ts->client,0xff020004,&value);
	}
	rd = value & 0x1;
	value = 0x00011110;
	gsl_write_TotalAdr(ts->client,0xff080058,&value);

	for(i=0;i<3;i++){
		gsl_read_TotalAdr(ts->client,0xff020004,&value);
	}
	td = (value & (0x1 << 1))>>1;
	print_info("[tpd_gsl][%s] [ru,rd]=[%d,%d]\n",__func__,ru,rd);
	print_info("[tpd_gsl][%s] [tu,td]=[%d,%d]\n",__func__,tu,td);
	if(ru == 0 && rd == 0)
		rstate = 0;
	else if(ru == 1 && rd == 1)
		rstate = 1;
	else if(ru == 1 && rd == 0)
		rstate = 2;

	if(tu == 0 && td == 0)
		tstate = 0;
	else if(tu == 1 && td == 1)
		tstate = 1;
	else if(tu == 1 && td == 0)
		tstate = 2;
	if(rstate==0&&tstate==0){
		gsl_cfg_index = 0;
	}
	else if(rstate==0&&tstate==2){
		gsl_cfg_index = 1;
	}
	print_info("[tpd-gsl][%s] [rstate,status]=[%d,%d]\n",__func__,rstate,tstate);
	return 1;
}
#endif
static int gsl_test_i2c(struct i2c_client *client)
{
	int i,err;
	u8 buf[4]={0};
	for(i=0;i<5;i++)
	{
		err=gsl_read_interface(client,0xfc,buf,4);
		if(err>0)
		{
			printk("[tp-gsl] i2c read 0xfc = 0x%02x%02x%02x%02x\n",
				buf[3],buf[2],buf[1],buf[0]);
			break;
		}
	}
	return (err<0?-1:0);
}

static void gsl_io_control(struct i2c_client *client)
{
	u8 buf[4] = {0};
	int i;
#if GSL9XX_VDDIO_1800
	for(i=0;i<5;i++){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0xfe;
		buf[3] = 0x1;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0] = 0x5;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x80;
		gsl_write_interface(client,0x78,buf,4);
		msleep(5);
	}
	msleep(50);
#endif
}
static void gsl_start_core(struct i2c_client *client)
{
	u8 buf[4] = {0};
	buf[0]=0;
	gsl_write_interface(client,0xe0,buf,4);
#ifdef GSL_ALG_ID
	gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
#endif
}

static void gsl_reset_core(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	buf[0] = 0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	msleep(5);
	gsl_io_control(client);
}

static void gsl_clear_reg(struct i2c_client *client)
{
	u8 buf[4]={0};
	//clear reg
	buf[0]=0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	buf[0]=0x3;
	gsl_write_interface(client,0x80,buf,4);
	msleep(5);
	buf[0]=0x4;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	buf[0]=0x0;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	//clear reg
}

#if 0
#define DMA_TRANS_LEN 0x20
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[DMA_TRANS_LEN*4] = {0};
	u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	print_info("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (0xf0 == GSL_DOWNLOAD_DATA[source_line].offset)
		{
			memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);	
			gsl_write_interface(client, 0xf0, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;

			memcpy((buf+send_flag*4 -4),&GSL_DOWNLOAD_DATA[source_line].val,4);	

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    		gsl_write_interface(client, addr, buf, DMA_TRANS_LEN * 4);
				send_flag = 0;
			}

			send_flag++;
		}
	}

	print_info("=============gsl_load_fw end==============\n");

}
#else 
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	print_info("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
    	addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
    	gsl_write_interface(client, addr, buf, 4);	
	}
}
#endif

static void gsl_sw_init(struct i2c_client *client)
{
	int temp;
	static volatile int gsl_sw_flag=0;
	if(1==gsl_sw_flag)
		return;
	gsl_sw_flag=1;
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
	
	temp = gsl_test_i2c(client);
	if(temp<0){
		gsl_sw_flag = 0;
		return;
	}

	gsl_clear_reg(client);
	gsl_reset_core(client);

	gsl_io_control(client);
	gsl_load_fw(client,gsl_cfg_table[gsl_cfg_index].fw,gsl_cfg_table[gsl_cfg_index].fw_size);
	gsl_io_control(client);

	gsl_start_core(client);
	gsl_sw_flag=0;
}

static void check_mem_data(struct i2c_client *client)
{
	char read_buf[4] = {0};
	gsl_read_interface(client, 0xb0, read_buf, 4);

	print_info("[gsl1680][%s] addr = 0xb0; read_buf = %02x%02x%02x%02x\n",
		__func__, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		gsl_sw_init(client);
	}
}

#ifdef TPD_PROC_DEBUG
#define GSL_APPLICATION
#ifdef GSL_APPLICATION
static int gsl_read_MorePage(struct i2c_client *client,u32 addr,u8 *buf,u32 num)
{
	int i;
	u8 tmp_buf[4] = {0};
	u8 tmp_addr;
	for(i=0;i<num/8;i++){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_write_interface(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_read_interface(client,tmp_addr,(buf+i*8),8);
	}
	if(i*8<num){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_write_interface(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_read_interface(client,tmp_addr,(buf+i*8),4);
	}
}
#endif
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	//int i;
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<gsl_cfg_table[gsl_cfg_index].data_size)
				seq_printf(m,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]); 
#endif
		}
		else 
		{
			gsl_write_interface(ddata->client,0xf0,&gsl_data_proc[4],4);
			gsl_read_interface(ddata->client,gsl_data_proc[0],temp_data,4);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
#ifdef GSL_APPLICATION
	else if('a'==gsl_read[0]&&'p'==gsl_read[1]){
		char *buf;
		int temp1;
		tmp = (unsigned int)(((gsl_data_proc[2]<<8)|gsl_data_proc[1])&0xffff);
		buf=kzalloc(tmp,GFP_KERNEL);
		if(buf==NULL)
			return -1;
		if(3==gsl_data_proc[0]){
			gsl_read_interface(ddata->client,gsl_data_proc[3],buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}else if(4==gsl_data_proc[0]){
			temp1=((gsl_data_proc[6]<<24)|(gsl_data_proc[5]<<16)|
				(gsl_data_proc[4]<<8)|gsl_data_proc[3]);
			gsl_read_MorePage(ddata->client,temp1,buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}
		kfree(buf);
	}
#endif
	return 0;
}
static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        	return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}	
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
#ifdef GSL_APPLICATION
	if('a'!=temp_buf[0]||'p'!=temp_buf[1]){
#endif
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
#ifdef GSL_APPLICATION
	}
#endif
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
	#ifdef GSL_TIMER	
		cancel_delayed_work_sync(&gsl_timer_check_work);
	#endif
		gsl_proc_flag = 1;
		gsl_reset_core(ddata->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		gsl_reset_core(ddata->client);
		gsl_start_core(ddata->client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_write_interface(ddata->client,buf[4],buf,4);
	}
	
#ifdef GSL_ALG_ID
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<gsl_cfg_table[gsl_cfg_index].data_size)
		{
			gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
		}
	}
#endif
#ifdef GSL_APPLICATION
	else if('a'==temp_buf[0]&&'p'==temp_buf[1]){
		if(1==path_buf[3]){
			tmp=((path_buf[5]<<8)|path_buf[4]);
			gsl_write_interface(ddata->client,path_buf[6],&path_buf[10],tmp);
		}else if(2==path_buf[3]){
			tmp = ((path_buf[5]<<8)|path_buf[4]);
			tmp1=((path_buf[9]<<24)|(path_buf[8]<<16)|(path_buf[7]<<8)
				|path_buf[6]);
			buf[0]=(char)((tmp1/0x80)&0xff);
			buf[1]=(char)(((tmp1/0x80)>>8)&0xff);
			buf[2]=(char)(((tmp1/0x80)>>16)&0xff);
			buf[3]=(char)(((tmp1/0x80)>>24)&0xff);
			buf[4]=(char)(tmp1%0x80);
			gsl_write_interface(ddata->client,0xf0,buf,4);
			gsl_write_interface(ddata->client,buf[4],&path_buf[10],tmp);
		}else if(3==path_buf[3]||4==path_buf[3]){
			memcpy(gsl_read,temp_buf,4);
			memcpy(gsl_data_proc,&path_buf[3],7);
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
#endif

#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{
	struct gsl_ts_data *ts = ddata;
	struct i2c_client *gsl_client = ts->client;
	static int i2c_lock_flag = 0;
	char read_buf[4]  = {0};
	char init_chip_flag = 0;
	int i,flag;
	print_info("----------------gsl_monitor_worker-----------------\n");	

	if(i2c_lock_flag != 0)
		return;
	else
		i2c_lock_flag = 1;

	gsl_read_interface(gsl_client, 0xb4, read_buf, 4);
	memcpy(int_2nd,int_1st,4);
	memcpy(int_1st,read_buf,4);

	if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&
		int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0])
	{
		printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",
			int_1st[3], int_1st[2], int_1st[1], int_1st[0], 
			int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
	/*check 0xb0 register,check firmware if ok*/
	for(i=0;i<5;i++){
		gsl_read_interface(gsl_client, 0xb0, read_buf, 4);
		if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || 
			read_buf[1] != 0x5a || read_buf[0] != 0x5a){
			printk("gsl_monitor_worker 0xb0 = {0x%02x%02x%02x%02x};\n",
				read_buf[3],read_buf[2],read_buf[1],read_buf[0]);
			flag = 1;
		}else{
			flag = 0;
			break;
		}

	}
	if(flag == 1){
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
	
	/*check 0xbc register,check dac if normal*/
	for(i=0;i<5;i++){
		gsl_read_interface(gsl_client, 0xbc, read_buf, 4);
		if(read_buf[3] != 0 || read_buf[2] != 0 || 
			read_buf[1] != 0 || read_buf[0] != 0){
			flag = 1;
		}else{
			flag = 0;
			break;
		}
	}
	if(flag == 1){
		gsl_reset_core(gsl_client);
		gsl_start_core(gsl_client);
		init_chip_flag = 0;
	}
queue_monitor_work:
	if(init_chip_flag){
		gsl_sw_init(gsl_client);
		memset(int_1st,0xff,sizeof(int_1st));
	}
	
	if(gsl_halt_flag==0){
		queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 200);
	}
	i2c_lock_flag = 0;

}
#endif

#ifdef TPD_PROXIMITY

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}
static int tpd_enable_ps(int enable)
{
	u8 buf[4];
	gsl_ps_enable = enable;
	printk("tpd_enable_ps:gsl_ps_enable = %d\n",gsl_ps_enable);
	if (enable) {
		wake_lock(&ps_lock);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		gsl_write_interface(ddata->client, 0xf0, buf, 4);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x2;
		gsl_write_interface(ddata->client, 0, buf, 4);
		
		tpd_proximity_flag = 1;
		//add alps of function
		printk("tpd-ps function is on\n");
	} else {
		tpd_proximity_flag = 0;
		wake_unlock(&ps_lock);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		gsl_write_interface(ddata->client, 0xf0, buf, 4);
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x00;
		gsl_write_interface(ddata->client, 0, buf, 4);
		printk("tpd-ps function is off\n");
	}
	return 0;
}

static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				err = tpd_enable_ps(value);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;

				sensor_data->values[0] = tpd_get_ps_value();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			printk("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;

}
#endif
#ifdef GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;
	printk("tp-gsl-gesture %s\n",__func__);
	for(i=0;i<len/2;i++){
		buf[0] = ((addr+i*8)/0x80)&0xff;
		buf[1] = (((addr+i*8)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*8)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*8)/0x80)>>24)&0xff;
		gsl_write_interface(ddata->client,0xf0,buf,4);
		gsl_read_interface(ddata->client,(addr+i*8)%0x80,(char *)&data[i*2],8);
	}
	if(len%2){
		buf[0] = ((addr+len*4 - 4)/0x80)&0xff;
		buf[1] = (((addr+len*4 - 4)/0x80)>>8)&0xff;
		buf[2] = (((addr+len*4 - 4)/0x80)>>16)&0xff;
		buf[3] = (((addr+len*4 - 4)/0x80)>>24)&0xff;
		gsl_write_interface(ddata->client,0xf0,buf,4);
		gsl_read_interface(ddata->client,(addr+len*4 - 4)%0x80,(char *)&data[len-1],4);
	}
	return len;
}
static void gsl_enter_doze(struct gsl_ts_data *ts)
{
	u8 buf[4] = {0};
#if 0
	u32 tmp;
	gsl_reset_core(ts->client);
	temp = ARRAY_SIZE(GSLX68X_FW_GESTURE);
	gsl_load_fw(ts->client,GSLX68X_FW_GESTURE,temp);
	gsl_start_core(ts->client);
	msleep(1000);		
#endif

	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_write_interface(ts->client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0x1;
	buf[3] = 0x5a;
	gsl_write_interface(ts->client,0x8,buf,4);
	gsl_gesture_status = GE_NOWORK;
	msleep(50);
	gsl_gesture_status = GE_ENABLE;

}
static void gsl_quit_doze(struct gsl_ts_data *ts)
{
	u8 buf[4] = {0};
	u32 tmp;

	gsl_gesture_status = GE_DISABLE;
		
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(5);
	
	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_write_interface(ts->client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0x5a;
	gsl_write_interface(ts->client,0x8,buf,4);
	msleep(10);

#if 0
	gsl_reset_core(ddata->client);
	temp = ARRAY_SIZE(GSLX68X_FW_CONFIG);
	//gsl_load_fw();
	gsl_load_fw(ddata->client,GSLX68X_FW_CONFIG,temp);
	gsl_start_core(ddata->client);
#endif
}

static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 count = 0;
	count += scnprintf(buf,PAGE_SIZE,"tp gesture is on/off:\n");
	if(gsl_gesture_flag == 1){
		count += scnprintf(buf+count,PAGE_SIZE-count,
				" on \n");
	}else if(gsl_gesture_flag == 0){
		count += scnprintf(buf+count,PAGE_SIZE-count,
				" off \n");
	}
	count += scnprintf(buf+count,PAGE_SIZE-count,"tp gesture:");
	count += scnprintf(buf+count,PAGE_SIZE-count,
			"%c\n",gsl_gesture_c);
    	return count;
}
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#if 1
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;  
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
	}
#endif
	return count;
}
static DEVICE_ATTR(gesture, 0666, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);
#endif

#define GSL_CHIP_NAME	"gslx68x"
static ssize_t gsl_sysfs_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t len=0;
	u32 tmp;
	u8 buf_tmp[4];
	char *ptr = buf;
	ptr += sprintf(ptr,"sileadinc:");
	ptr += sprintf(ptr,GSL_CHIP_NAME);
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();
	ptr += sprintf(ptr,":%08x:",tmp);
	ptr += sprintf(ptr,"%08x:",
		gsl_cfg_table[gsl_cfg_index].data_id[0]);
#endif
	buf_tmp[0]=0x3;buf_tmp[1]=0;buf_tmp[2]=0;buf_tmp[3]=0;
	gsl_write_interface(ddata->client,0xf0,buf_tmp,4);
	gsl_read_interface(ddata->client,0,buf_tmp,4);
	ptr += sprintf(ptr,"%02x%02x%02x%02x\n",buf_tmp[3],buf_tmp[2],buf_tmp[1],buf_tmp[0]);
		
    	return (ptr-buf);
}
static DEVICE_ATTR(version, 0444, gsl_sysfs_version_show, NULL);
static struct attribute *gsl_attrs[] = {
	&dev_attr_version.attr,
#ifdef GSL_GESTURE
	&dev_attr_gesture.attr,
#endif
	NULL
};
static const struct attribute_group gsl_attr_group = {
	.attrs = gsl_attrs,
};//hebufan
static unsigned int gsl_sysfs_init(void)
{
	int ret;
	struct kobject *gsl_debug_kobj;
	gsl_debug_kobj = kobject_create_and_add("gsl_touchscreen", NULL) ;
	if (gsl_debug_kobj == NULL)
	{
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_version.attr);
	if (ret)
	{
		printk("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
	
	#ifdef GSL_GESTURE
	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_gesture.attr);
	if (ret)
	{
		printk("%s: sysfs_create_gesture_file failed\n", __func__);
		return ret;
	}
	#endif
}

static void gsl_report_point(struct gsl_touch_info *ti)
{
	int tmp = 0;
	static int gsl_up_flag = 0; //prevent more up event
	print_info("gsl_report_point %d \n", ti->finger_num);

	if (unlikely(ti->finger_num == 0)) 
	{
		if(gsl_up_flag == 0)
			return;
	    	gsl_up_flag = 0;
        	input_report_key(tpd->dev, BTN_TOUCH, 0);
        	input_mt_sync(tpd->dev);
		if (FACTORY_BOOT == get_boot_mode()|| 
			RECOVERY_BOOT == get_boot_mode())
		{   

			tpd_button(ti->x[tmp], ti->y[tmp], 0);

		}
	} 
	else 
	{
		gsl_up_flag = 1;
		for (tmp = 0; ti->finger_num > tmp; tmp++) 
		{
			print_info("[gsl1680](x[%d],y[%d]) = (%d,%d);\n", 
				ti->id[tmp], ti->id[tmp], ti->x[tmp], ti->y[tmp]);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);

			if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
			{ 
				tpd_button(ti->x[tmp], ti->y[tmp], 1);  
			}
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, ti->id[tmp] - 1);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X,((ti->x[tmp])));
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, ti->y[tmp]);
			printk("!!!x=%d,y=%d\n",((ti->x[tmp])),ti->y[tmp]);//display the x-yyesye
			input_mt_sync(tpd->dev);
		}
	}
	input_sync(tpd->dev);
}
///////////////////////////////////////////////////////////kkkkkk
#ifdef GSL_GESTURE
////////////////\CA\D6\CA\C6e
static ssize_t gt91xxge_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablee);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxge_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablee = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGE_CONFIG_PROC_FILE     "gesture_enablee"
static struct proc_dir_entry *gt91xxge_config_proc = NULL;

static const struct file_operations configge_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxge_config_read_proc,
    .write = gt91xxge_config_write_proc,
};
////////////////////////////////////////////////////////////////////
static ssize_t gt91xxg_enable_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", gesture_power_flage);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxg_enable_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        gesture_power_flage = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXG_ENABLE_CONFIG_PROC_FILE     "gesture_enable"
static struct proc_dir_entry *gt91xxg_enable_config_proc = NULL;

static const struct file_operations config_enable_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxg_enable_config_read_proc,
    .write = gt91xxg_enable_config_write_proc,
};

/////////////////////////////////////////////////////////

////////////////\CA\D6\CA\C6c
static ssize_t gt91xxgc_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablec);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgc_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablec = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGC_CONFIG_PROC_FILE     "gesture_enablec"
static struct proc_dir_entry *gt91xxgc_config_proc = NULL;

static const struct file_operations configgc_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgc_config_read_proc,
    .write = gt91xxgc_config_write_proc,
};
////////////////\CA\D6\CA\C6v
static ssize_t gt91xxgv_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablev);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgv_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablev = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGV_CONFIG_PROC_FILE     "gesture_enablev"
static struct proc_dir_entry *gt91xxgv_config_proc = NULL;

static const struct file_operations configgv_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgv_config_read_proc,
    .write = gt91xxgv_config_write_proc,
};
////////////////\CA\D6\CA\C6m
static ssize_t gt91xxgm_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablem);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgm_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablem = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGM_CONFIG_PROC_FILE     "gesture_enablem"
static struct proc_dir_entry *gt91xxgm_config_proc = NULL;

static const struct file_operations configgm_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgm_config_read_proc,
    .write = gt91xxgm_config_write_proc,
};
////////////////\CA\D6\CA\C6o
static ssize_t gt91xxgo_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enableo);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgo_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enableo = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGO_CONFIG_PROC_FILE     "gesture_enableo"
static struct proc_dir_entry *gt91xxgo_config_proc = NULL;

static const struct file_operations configgo_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgo_config_read_proc,
    .write = gt91xxgo_config_write_proc,
};
////////////////\CA\D6\CA\C6w
static ssize_t gt91xxgw_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablew);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgw_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablew = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGW_CONFIG_PROC_FILE     "gesture_enablew"
static struct proc_dir_entry *gt91xxgw_config_proc = NULL;

static const struct file_operations configgw_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgw_config_read_proc,
    .write = gt91xxgw_config_write_proc,
};
////////////////\CA\D6\CA\C6s
static ssize_t gt91xxgs_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enables);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgs_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enables = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGS_CONFIG_PROC_FILE     "gesture_enables"
static struct proc_dir_entry *gt91xxgs_config_proc = NULL;

static const struct file_operations configgs_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgs_config_read_proc,
    .write = gt91xxgs_config_write_proc,
};
////////////////\CA\D6\CA\C6z
static ssize_t gt91xxgz_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablez);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgz_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablez = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGZ_CONFIG_PROC_FILE     "gesture_enablez"
static struct proc_dir_entry *gt91xxgz_config_proc = NULL;

static const struct file_operations configgz_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgz_config_read_proc,
    .write = gt91xxgz_config_write_proc,
};
////////////////\CA\D6\CA\C6up
static ssize_t gt91xxgup_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enableup);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgup_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enableup = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGUP_CONFIG_PROC_FILE     "gesture_enable_up"
static struct proc_dir_entry *gt91xxgup_config_proc = NULL;

static const struct file_operations configgup_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgup_config_read_proc,
    .write = gt91xxgup_config_write_proc,
};
////////////////\CA\D6\CA\C6down
static ssize_t gt91xxgdo_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enabledo);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgdo_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enabledo = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGDO_CONFIG_PROC_FILE     "gesture_enable_down"
static struct proc_dir_entry *gt91xxgdo_config_proc = NULL;

static const struct file_operations configgdo_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgdo_config_read_proc,
    .write = gt91xxgdo_config_write_proc,
};
////////////////\CA\D6\CA\C6left
static ssize_t gt91xxgle_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enablele);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgle_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enablele = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGLE_CONFIG_PROC_FILE     "gesture_enable_left"
static struct proc_dir_entry *gt91xxgle_config_proc = NULL;

static const struct file_operations configgle_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgle_config_read_proc,
    .write = gt91xxgle_config_write_proc,
};
////////////////\CA\D6\CA\C6right
static ssize_t gt91xxgri_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enableri);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgri_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enableri = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGRI_CONFIG_PROC_FILE     "gesture_enable_right"
static struct proc_dir_entry *gt91xxgri_config_proc = NULL;

static const struct file_operations configgri_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgri_config_read_proc,
    .write = gt91xxgri_config_write_proc,
};
////////////////\CA\D6\CA\C6dobule
static ssize_t gt91xxgdoub_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
       int len = 0;
       char *p = page;
       
    if (*ppos)  // CMD call again
    {
        return 0;
    }

       p += sprintf(p, "%d\n", tpd->gesture_enabledoub);
      *ppos += p - page;

       return (p-page);

}

static ssize_t gt91xxgdoub_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
       int len = 0, bat_tt_enable=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
   if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d", &bat_tt_enable) == 1)
    {

        tpd->gesture_enabledoub = bat_tt_enable;        
        return count;
    }
  
    return -EINVAL;

}
#define GT91XXGDOUB_CONFIG_PROC_FILE     "gesture_enable_double"
static struct proc_dir_entry *gt91xxgdoub_config_proc = NULL;

static const struct file_operations configgdoub_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xxgdoub_config_read_proc,
    .write = gt91xxgdoub_config_write_proc,
};
#endif
//////////////////////////////////////////////////////////////kkkk
static void gsl_report_work(void)
{

	u8 buf[4] = {0};
	u8 i = 0;
	u16 ret = 0;
	u16 tmp = 0;
	struct gsl_touch_info cinfo={0};
	u8 tmp_buf[44] ={0};
	print_info("enter gsl_report_work\n");
#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
    /*added by bernard*/
	if (tpd_proximity_flag == 1)
	{
	
		gsl_read_interface(ddata->client,0xac,buf,4);
		if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
		{
			tpd_proximity_detect = 0;
			//sensor_data.values[0] = 0;
		}
		else
		{
			tpd_proximity_detect = 1;
			//sensor_data.values[0] = 1;
		}
		//get raw data
		print_info(" ps change\n");
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = tpd_get_ps_value();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
			print_info("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	/*end of added*/
#endif

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif	

 	gsl_read_interface(ddata->client, 0x80, tmp_buf, 8);
	if(tmp_buf[0]>=2&&tmp_buf[0]<=10)
		gsl_read_interface(ddata->client, 0x88, &tmp_buf[8], (tmp_buf[0]*4-4));
	cinfo.finger_num = tmp_buf[0] & 0x0f;
	print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(tmp=0;tmp<(cinfo.finger_num>10?10:cinfo.finger_num);tmp++)
	{
		cinfo.id[tmp] = tmp_buf[tmp*4+7] >> 4;
		cinfo.y[tmp] = (tmp_buf[tmp*4+4] | ((tmp_buf[tmp*4+5])<<8));
		cinfo.x[tmp] = (tmp_buf[tmp*4+6] | ((tmp_buf[tmp*4+7] & 0x0f)<<8));
		print_info("tp-gsl  x = %d y = %d \n",cinfo.x[tmp],cinfo.y[tmp]);
	}

#ifdef GSL_ALG_ID
	int tmp1 = 0;
	cinfo.finger_num = (tmp_buf[3]<<24)|(tmp_buf[2]<<16)|(tmp_buf[1]<<8)|(tmp_buf[0]);
	gsl_alg_id_main(&cinfo);
	
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		buf[1]=0;
		buf[2]=0;
		buf[3]=0;
		gsl_write_interface(ddata->client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		printk("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_write_interface(ddata->client,0x8,buf,4);
	}
#endif
#ifdef GSL_GESTURE
	if(GE_ENABLE == gsl_gesture_status && gsl_gesture_flag == 1){
		int tmp_c;
		u8 key_data = 0;
		tmp_c = gsl_obtain_gesture();
		//print_info("tp_gesture %d %c\n",tmp_c,tmp_c&0xff);
		switch(tmp_c){
		case (int)'C':
			if(tpd->gesture_enablec = 1)
					key_data = KEY_SC_CHANGED;
			break;
		case (int)'E':
			if(tpd->gesture_enablee = 1)
				key_data = KEY_S_CHANGED;
			break;
		case (int)'W':
			if(tpd->gesture_enablew = 1)
				key_data = KEY_SW_CHANGED;
			break;
		case (int)'O':
			if(tpd->gesture_enableo = 1)
				key_data = KEY_SO_CHANGED;
			break;
		case (int)'M':
			if(tpd->gesture_enablem = 1)
			key_data = KEY_SM_CHANGED;
			break;
		case (int)'Z':
		if(tpd->gesture_enablez = 1)
			key_data = KEY_SZ_CHANGED;
			break;
		case (int)'V':
			if(tpd->gesture_enablev = 1)
			key_data = KEY_SV_CHANGED;
			break;
		case (int)'S':
			if(tpd->gesture_enables = 1)
			key_data = KEY_SS_CHANGED;
			break;
		case (int)'*':		/* double click */
			if(tpd->gesture_enabledoub = 1)
			key_data = KEY_SCC_CHANGED;
			break;
		/*
		case (int)0xa1fa:	// right 
			if(tpd->gesture_enableri = 1)
			key_data = KEY_SAA_CHANGED;
			break;
		case (int)0xa1fd:	// down 
			if(tpd->gesture_enabledo = 1)
			key_data = KEY_SAB_CHANGED;
			break;
		case (int)0xa1fc:	// up 
			if(tpd->gesture_enableup = 1)
			key_data = KEY_SBA_CHANGED;
			break;
		case (int)0xa1fb:	// left
			if(tpd->gesture_enablele = 1)
			key_data = KEY_SBB_CHANGED;
			break;
		*/
		default:
			break;
		}

		if(key_data != 0){
			gsl_gesture_c = (char)(tmp_c & 0xff);
			//gsl_gesture_status = GE_WAKEUP;
			print_info("tp_gesture %c\n",gsl_gesture_c);
			input_report_key(tpd->dev,key_data,1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev,key_data,0);
			input_sync(tpd->dev);
		}
		// msleep(200);
		return;
	}
#endif


	gsl_report_point(&cinfo);
}


static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		gsl_report_work();
	} while (!kthread_should_stop());	
	return 0;
}
#ifdef GSL_DRV_WIRE_IDT_TP

#define GSL_C		100
#define GSL_CHIP_1	0xffffffff  //sanfengda 
#define GSL_CHIP_2	0xffffffff  //xingzhen
#define GSL_CHIP_3	0xffffffff
#define GSL_CHIP_4	0xffffffff
static unsigned int gsl_count_one(unsigned int flag)
{
	unsigned int tmp=0;	
	int i =0;
	for(i=0;i<32;i++){
		if(flag&(0x1<<i))
			tmp++;
	}
	return tmp;
}
static int gsl_DrvWire_idt_tp(struct gsl_ts_data *ts)
{
	u8 buf[4];
	int i,err=1;
	int flag=0;
	u16 count0,count1;
	unsigned int tmp,tmp0;
	unsigned int tmp1,tmp2,tmp3,tmp4;
	u32 num;
identify_tp_repeat:
	gsl_clear_reg(ts->client);
	gsl_reset_core(ts->client);
	num = ARRAY_SIZE(GSL_IDT_FW);
	gsl_load_fw(ts->client,GSL_IDT_FW,num);
	gsl_start_core(ts->client);
	msleep(200);
	for(i=0;i<3;i++){
		gsl_read_interface(ts->client,0xb4,buf,4);

		print_info("i = %d count0 the test 0xb4 = {0x%02x%02x%02x%02x}\n",i,buf[3],buf[2],buf[1],buf[0]);

		count0 = (buf[3]<<8)|buf[2];
		msleep(5);
		gsl_read_interface(ts->client,0xb4,buf,4);

		print_info("i = %d count1 the test 0xb4 = {0x%02x%02x%02x%02x}\n",i,buf[3],buf[2],buf[1],buf[0]);

		count1 = (buf[3]<<8)|buf[2];
		if((count0 > 1) && (count0 != count1))
			break;
	}
	if((count0 > 1) && count0 != count1){

		print_info("[TP-GSL][%s] is start ok\n",__func__);

		gsl_read_interface(ts->client,0xb8,buf,4);
		tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];

		print_info("the test 0xb8 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);
		tmp1 = gsl_count_one(GSL_CHIP_1^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_1)^GSL_CHIP_1);
		tmp1 += tmp0*GSL_C;
		print_info("[TP-GSL] tmp1 = %d\n",tmp1);
		
		tmp2 = gsl_count_one(GSL_CHIP_2^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_2)^GSL_CHIP_2);
		tmp2 += tmp0*GSL_C;
		print_info("[TP-GSL] tmp2 = %d\n",tmp2);	
	
		tmp3 = gsl_count_one(GSL_CHIP_3^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_3)^GSL_CHIP_3);
		tmp3 += tmp0*GSL_C;
		print_info("[TP-GSL] tmp3 = %d\n",tmp3);	
	
		tmp4 = gsl_count_one(GSL_CHIP_4^tmp);
		tmp0 = gsl_count_one((tmp&GSL_CHIP_4)^GSL_CHIP_4);
		tmp4 += tmp0*GSL_C;
		print_info("[TP-GSL] tmp4 = %d\n",tmp4);

		if(0xffffffff==GSL_CHIP_1)
		{
			tmp1=0xffff;
		}
		if(0xffffffff==GSL_CHIP_2)
		{
			tmp2=0xffff;
		}
		if(0xffffffff==GSL_CHIP_3)
		{
			tmp3=0xffff;
		}
		if(0xffffffff==GSL_CHIP_4)
		{
			tmp4=0xffff;
		}
		print_info("[TP-GSL] tmp1 = %d\n",tmp1);
		print_info("[TP-GSL] tmp2 = %d\n",tmp2);
		print_info("[TP-GSL] tmp3 = %d\n",tmp3);
		print_info("[TP-GSL] tmp4 = %d\n",tmp4);
		tmp = tmp1;
		if(tmp1>tmp2){
			tmp = tmp2;	
		}
		if(tmp > tmp3){
			tmp = tmp3;
		}
		if(tmp>tmp4){
			tmp = tmp4;
		}
	
		if(tmp == tmp1){
			gsl_cfg_index = 0;	
		}else if(tmp == tmp2){
			gsl_cfg_index = 0;
		}else if(tmp == tmp3){
			gsl_cfg_index = 0;
		}else if(tmp == tmp4){
			gsl_cfg_index = 0;
		}
		err = 1;
	}else {
		flag++;
		if(flag < 3)
			goto identify_tp_repeat;
		err = 0;
	}
	return err;	
}
#endif
static int tpd_eint_interrupt_handler(void)
{

	print_info("[gsl1680] TPD interrupt has been triggered\n");
	tpd_flag=1; 
    	wake_up_interruptible(&waiter);
}

static void gsl_hw_init(void)
{
	//power on

	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "TP");
      
	/* reset ctp gsl1680 */
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	/* set interrupt work mode */
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	msleep(100);
}
#ifdef TPD_PROC_DEBUG
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif

static int  gsl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i,err;
	unsigned char tp_data[4];
#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;
#endif

	print_info();

	ddata = kzalloc(sizeof(struct gsl_ts_data), GFP_KERNEL);
	if (!ddata) {
		print_info("alloc ddata memory error\n");
		return -ENOMEM;
	}
	//proc_ddata = ddata;
	mutex_init(&gsl_i2c_lock);
	ddata->client = client;
	print_info("ddata->client->addr = 0x%x \n",ddata->client->addr);
	gsl_hw_init();

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	i2c_set_clientdata(ddata->client, ddata);

	err = gsl_test_i2c(ddata->client);
	if(err<0)
	{
		printk("!!!!!\n");
		goto  err_malloc;
	}
#ifdef GSL_DRV_WIRE_IDT_TP
	gsl_DrvWire_idt_tp(ddata);
#endif
#ifdef GSL_GPIO_IDT_TP
	gsl_gpio_idt_tp(ddata);
#endif

//wu add rawdata test 2015.11.5
#ifdef GSL_TEST_TP
	create_ctp_proc();
#endif



#ifdef GSL_GESTURE
		gsl_FunIICRead(gsl_read_oneframe_data);
		gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);
#endif

	print_info("[tpd-gsl][%s] gsl_cfg_index=%d\n",__func__,gsl_cfg_index);
	input_set_abs_params(tpd->dev,ABS_MT_TRACKING_ID, 0,10, 0, 0);
	gsl_sw_init(ddata->client);
	msleep(20);
	check_mem_data(ddata->client);
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		//err = PTR_ERR(thread);
		//TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", PTR_ERR(thread));
	}
  

mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
	gsl_timer_workqueue = create_workqueue("gsl_timer_check");
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
#ifdef TPD_PROC_DEBUG



	gsl_config_proc = proc_create(GSL_CONFIG_PROC_FILE, 0666, NULL, &gsl_seq_fops);
       if (gsl_config_proc == NULL)
	    {
	        print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	    }
	   gsl_proc_flag = 0;

#endif
//////////////////////////////\CA\D6\CAƻ\BD\D0\D1\CEļ\FE\B4\B4\BD\A8 wangxiaoke 2015/9/18
/////enable gesture
#ifdef GSL_GESTURE
gt91xxg_enable_config_proc = proc_create(GT91XXG_ENABLE_CONFIG_PROC_FILE, 0666, NULL, &config_enable_proc_ops);
   if (gt91xxg_enable_config_proc == NULL)
     print_info("create_proc_entry %s failed\n", GT91XXG_ENABLE_CONFIG_PROC_FILE);
  else
	 print_info("create proc entry %s success", GT91XXG_ENABLE_CONFIG_PROC_FILE);

/////e
gt91xxge_config_proc = proc_create(GT91XXGE_CONFIG_PROC_FILE, 0666, NULL, &configge_proc_ops);
   if (gt91xxge_config_proc == NULL)
     print_info("create_proc_entry %s failed\n", GT91XXGE_CONFIG_PROC_FILE);
  else
	 print_info("create proc entry %s success", GT91XXGE_CONFIG_PROC_FILE);
/////c
gt91xxge_config_proc = proc_create(GT91XXGC_CONFIG_PROC_FILE, 0666, NULL, &configgc_proc_ops);
	 if (gt91xxge_config_proc == NULL)
	   print_info("create_proc_entry %s failed\n", GT91XXGC_CONFIG_PROC_FILE);
	else
	   print_info("create proc entry %s success", GT91XXGC_CONFIG_PROC_FILE);
/////v
	gt91xxge_config_proc = proc_create(GT91XXGV_CONFIG_PROC_FILE, 0666, NULL, &configgv_proc_ops);
	   if (gt91xxge_config_proc == NULL)
		 print_info("create_proc_entry %s failed\n", GT91XXGV_CONFIG_PROC_FILE);
	  else
		 print_info("create proc entry %s success", GT91XXGV_CONFIG_PROC_FILE);
/////m
	gt91xxge_config_proc = proc_create(GT91XXGM_CONFIG_PROC_FILE, 0666, NULL, &configgm_proc_ops);
		 if (gt91xxge_config_proc == NULL)
		   print_info("create_proc_entry %s failed\n", GT91XXGM_CONFIG_PROC_FILE);
		else
		   print_info("create proc entry %s success", GT91XXGM_CONFIG_PROC_FILE);

/////o
gt91xxge_config_proc = proc_create(GT91XXGO_CONFIG_PROC_FILE, 0666, NULL, &configgo_proc_ops);
   if (gt91xxge_config_proc == NULL)
     print_info("create_proc_entry %s failed\n", GT91XXGO_CONFIG_PROC_FILE);
  else
	 print_info("create proc entry %s success", GT91XXGO_CONFIG_PROC_FILE);
/////w
gt91xxge_config_proc = proc_create(GT91XXGW_CONFIG_PROC_FILE, 0666, NULL, &configgw_proc_ops);
	 if (gt91xxge_config_proc == NULL)
	   print_info("create_proc_entry %s failed\n", GT91XXGW_CONFIG_PROC_FILE);
	else
	   print_info("creatprint_infoe proc entry %s success", GT91XXGW_CONFIG_PROC_FILE);
/////s
	gt91xxge_config_proc = proc_create(GT91XXGS_CONFIG_PROC_FILE, 0666, NULL, &configgs_proc_ops);
	   if (gt91xxge_config_proc == NULL)
		 print_info("create_proc_entry %s failed\n", GT91XXGS_CONFIG_PROC_FILE);
	  else
		 print_info("create proc entry %s success", GT91XXGS_CONFIG_PROC_FILE);
/////z
	gt91xxge_config_proc = proc_create(GT91XXGZ_CONFIG_PROC_FILE, 0666, NULL, &configgz_proc_ops);
		 if (gt91xxge_config_proc == NULL)
		   print_info("create_proc_entry %s failed\n", GT91XXGZ_CONFIG_PROC_FILE);
		else
		   print_info("create proc entry %s success", GT91XXGZ_CONFIG_PROC_FILE);
/////up
		gt91xxge_config_proc = proc_create(GT91XXGUP_CONFIG_PROC_FILE, 0666, NULL, &configgup_proc_ops);
		   if (gt91xxge_config_proc == NULL)
			 print_info("create_proc_entry %s failed\n", GT91XXGUP_CONFIG_PROC_FILE);
		  else
			 print_info("create proc entry %s success", GT91XXGUP_CONFIG_PROC_FILE);
/////down
		gt91xxge_config_proc = proc_create(GT91XXGDO_CONFIG_PROC_FILE, 0666, NULL, &configgdo_proc_ops);
			 if (gt91xxge_config_proc == NULL)
			   print_info("create_proc_entry %s failed\n", GT91XXGDO_CONFIG_PROC_FILE);
			else
			   print_info("create proc entry %s success", GT91XXGDO_CONFIG_PROC_FILE);
/////right
	gt91xxge_config_proc = proc_create(GT91XXGRI_CONFIG_PROC_FILE, 0666, NULL, &configgri_proc_ops);
	   if (gt91xxge_config_proc == NULL)
		 print_info("create_proc_entry %s failed\n", GT91XXGRI_CONFIG_PROC_FILE);
	  else
		 print_info("create proc entry %s success", GT91XXGRI_CONFIG_PROC_FILE);
/////left
	gt91xxge_config_proc = proc_create(GT91XXGLE_CONFIG_PROC_FILE, 0666, NULL, &configgle_proc_ops);
		 if (gt91xxge_config_proc == NULL)
		   print_info("create_proc_entry %s failed\n", GT91XXGLE_CONFIG_PROC_FILE);
		else
		   print_info("create proc entry %s success", GT91XXGLE_CONFIG_PROC_FILE);
/////double
			gt91xxge_config_proc = proc_create(GT91XXGDOUB_CONFIG_PROC_FILE, 0666, NULL, &configgdoub_proc_ops);
				 if (gt91xxge_config_proc == NULL)
				   print_info("create_proc_entry %s failed\n", GT91XXGDOUB_CONFIG_PROC_FILE);
				else
				   print_info("create proc entry %s success", GT91XXGDOUB_CONFIG_PROC_FILE);
//һ\B9\B213\B8\F6
#endif
//////////////////////////////////////////////

#ifdef GSL_ALG_ID
	gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
#endif
#ifdef TPD_PROXIMITY
	//obj_ps.self = gsl1680p_obj;
	//	obj_ps.self = cm3623_obj;
	obj_ps.polling = 0;//interrupt mode
	//obj_ps.polling = 1;//need to confirm what mode is!!!
	obj_ps.sensor_operate = tpd_ps_operate;//gsl1680p_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		printk("attach fail = %d\n", err);
	}
	
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
#endif
#ifdef GSL_GESTURE
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_C);
	input_set_capability(tpd->dev, EV_KEY, KEY_S);
	input_set_capability(tpd->dev, EV_KEY, KEY_V);
	input_set_capability(tpd->dev, EV_KEY, KEY_Z);
	input_set_capability(tpd->dev, EV_KEY, KEY_E);
	input_set_capability(tpd->dev, EV_KEY, KEY_M);
	input_set_capability(tpd->dev, EV_KEY, KEY_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_O);
	input_set_capability(tpd->dev, EV_KEY, KEY_U);
	input_set_capability(tpd->dev, EV_KEY, KEY_LEFT);
	input_set_capability(tpd->dev, EV_KEY, KEY_RIGHT);
	input_set_capability(tpd->dev, EV_KEY, KEY_UP);
	input_set_capability(tpd->dev, EV_KEY, KEY_DOWN);
	///////////////////////////////////////
	input_set_capability(tpd->dev, EV_KEY, KEY_SV_CHANGED);//1
	input_set_capability(tpd->dev, EV_KEY, KEY_S_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SC_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SBA_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SM_CHANGED);//5
	input_set_capability(tpd->dev, EV_KEY, KEY_SO_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SW_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SS_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SZ_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SAA_CHANGED);//10
	input_set_capability(tpd->dev, EV_KEY, KEY_SBB_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SAB_CHANGED);
	input_set_capability(tpd->dev, EV_KEY, KEY_SCC_CHANGED);//13
	//__set_bit(EV_KEY, tpd->dev->evbit);
	//__set_bit(KEY_SVCHANGED, tpd->dev->keybit);
	//__set_bit(EV_SYN, tpd->dev->evbit);
#endif

	gsl_sysfs_init();

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
	tpd_load_status = 1;

	return 0;

err_malloc:
	if (ddata)
		kfree(ddata);

	return err;
}

/*****************************************************************************
Prototype    : gsl_remove
Description  : remove gsl1680 driver
Input        : struct i2c_client *client
Output       : int
Return Value : static

 *****************************************************************************/
static int  gsl_remove(struct i2c_client *client)
{
	print_info("[gsl1680] TPD removed\n");
	return 0;
}

/*****************************************************************************
Prototype    : gsl_detect
Description  : gsl1680 driver local setup without board file
Input        : struct i2c_client *client
int kind
struct i2c_board_info *info
Output       : int
Return Value : static

 *****************************************************************************/
static int gsl_detect (struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	print_info("%s, %d\n", __FUNCTION__, __LINE__);
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

static struct i2c_driver gsl_i2c_driver = {
    .driver = {
		.name = TPD_DEVICE,
		.owner = THIS_MODULE,
    },
	.probe = gsl_probe,
	.remove = gsl_remove,
	.id_table = gsl_device_id,
	.detect = gsl_detect,
};

/*****************************************************************************
Prototype    : gsl_local_init
Description  : setup gsl1680 driver
Input        : None
Output       : None
Return Value : static

 *****************************************************************************/
static int gsl_local_init(void)
{
	int ret;
	print_info();
	boot_mode = get_boot_mode();
	print_info("boot_mode == %d \n", boot_mode);

	if (boot_mode == SW_REBOOT)
	boot_mode = NORMAL_BOOT;
#ifdef TPD_HAVE_BUTTON
	print_info("TPD_HAVE_BUTTON\n");
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
#endif

	ret = i2c_add_driver(&gsl_i2c_driver);

	if (ret < 0) {
		print_info("unable to i2c_add_driver\n");
		return -ENODEV;
	}

	if (tpd_load_status == 0) 
	{
		print_info("tpd_load_status == 0, gsl_probe failed\n");
		i2c_del_driver(&gsl_i2c_driver);
		return -ENODEV;
	}

	/* define in tpd_debug.h */
	tpd_type_cap = 1;
	print_info("end %s, %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static void gsl_suspend(struct i2c_client *client)
{
	int tmp;
	print_info();
	//printk("gsl_suspend:gsl_ps_enable = %d\n",gsl_ps_enable);	
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
	    return 0;
	}
#endif
	gsl_halt_flag = 1;
	//version info
	printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
	
	//version info
	
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&gsl_timer_check_work);
#endif
#ifdef GSL_GESTURE
	if(gesture_power_flage == 1){
		gsl_enter_doze(ddata);
		return;
	}
#endif


	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//	gsl_reset_core(ddata->client);
//	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
}

static void gsl_resume(struct i2c_client *client)
{
	print_info();
	//printk("gsl_resume:gsl_ps_enable = %d\n",gsl_ps_enable);
#ifdef TPD_PROXIMITY   
	if (tpd_proximity_flag == 1&&gsl_halt_flag == 0)
	{
		tpd_enable_ps(1);
		return;
	}
#endif   


#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
#ifdef GSL_GESTURE
	if(gsl_gesture_flag == 1){
		gsl_quit_doze(ddata);
	}
#endif

	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
	gsl_reset_core(ddata->client);
	gsl_start_core(ddata->client);
	msleep(20);
	check_mem_data(ddata->client);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
	gsl_halt_flag = 0;
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_enable_ps(1);
		return;
	}
#endif

}


static struct tpd_driver_t gsl_driver = {
	.tpd_device_name = GSL_DEV_NAME,
	.tpd_local_init = gsl_local_init,
	.suspend = gsl_suspend,
	.resume = gsl_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
 	.tpd_have_button = 0,
#endif
};

static int __init gsl_driver_init(void)
{
	int ret;

	print_info();
	printk("!!!!!\n");
	i2c_register_board_info(1, &i2c_tpd, 1);//!!!!!!!!!!!!!!!!!kkk
	if(ret = tpd_driver_add(&gsl_driver) < 0)
		print_info("gsl_driver init error, return num is %d \n", ret);

	return ret;
}

static void __exit gsl_driver_exit(void)
{
	print_info();
	tpd_driver_remove(&gsl_driver);
}

module_init(gsl_driver_init);
module_exit(gsl_driver_exit);
