#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (600)
#define FRAME_HEIGHT (1024)
#define LCM_ID       (0x94)

#define LVDS_LCM_STBY       GPIO118
#define GPIO_LCM1990_RESET      GPIO89
#define GPIO_LCM1990_STBY       GPIO83


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0



static LCM_setting_table_V3 lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
{0x39, 0xB9,  3 ,{0xFF, 0x83, 0x94}},

{0x39, 0xBA,  2 ,{0x73, 0x83}},

{0x39, 0xB1,  15 ,{0x6C, 0x0E, 0x0E, 0x25, 0x04, 0x0F, 0xEF, 0x01, 0x67, 0xDA, 0x23, 0x86, 0xC0, 0x8A, 0x58}},

{0x39, 0xB2,  12 ,{0x85, 0x44, 0x0F, 0x09, 0x24, 0x1C, 0x08, 0x08, 0x1C, 0x4D, 0x00, 0x00}},

{0x39, 0xB4,  12 ,{0x00, 0xFF, 0x59, 0x5A, 0x59, 0x5A, 0x59, 0x5A, 0x01, 0x76, 0x01, 0x76}},

{0x39, 0xBF,  3 ,{0x41, 0x0E, 0x01}},

{0x15, 0xD2,  1 ,{0x00}},

{0x39, 0xD3,  30 ,{0x00, 0x00, 0x00, 0x01, 0x07, 
					0x0C, 0x0C, 0x32, 0x10, 0x07, 
					0x00, 0x05, 0x00, 0x20, 0x0A, 
					0x05, 0x09, 0x00, 0x32, 0x10, 
					0x08, 0x00, 0x21, 0x21, 0x08, 
					0x07, 0x23, 0x0D, 0x07, 0x47}},

{0x39, 0xD5,  44 ,{0x01, 0x00, 0x01, 0x00, 0x03, 
					0x02, 0x03, 0x02, 0x21, 0x20, 
					0x21, 0x20, 0x18, 0x02, 0x18, 
					0x02, 0x18, 0x18, 0x18, 0x18, 
					0x18, 0x18, 0x18, 0x18, 0x18, 
					0x18, 0x18, 0x18, 0x18, 0x18, 
					0x18, 0x18, 0x18, 0x18, 0x18, 
					0x18, 0x18, 0x18, 0x18, 0x18, 
					0x18, 0x18, 0x18, 0x18}},

{0x39, 0xE0,  42 ,{0x00, 0x05, 0x07, 0x28, 0x2D, 
					0x3D, 0x15, 0x33, 0x05, 0x09, 
					0x0B, 0x17, 0x0F, 0x13, 0x15, 
					0x12, 0x15, 0x07, 0x11, 0x12, 
					0x18, 0x00, 0x05, 0x08, 0x28, 
					0x2C, 0x3D, 0x15, 0x33, 0x05, 
					0x09, 0x0A, 0x16, 0x0F, 0x12, 
					0x15, 0x13, 0x14, 0x07, 0x11, 
					0x11, 0x17}},

{0x15, 0xCC,  1 ,{0x09}},

{0x39, 0xC7,  4 ,{0x00, 0xC0, 0x40, 0xC0}},

{0x39, 0xC0,  2 ,{0x30, 0x14}},

{0x15, 0xBC,  1 ,{0x07}},

{0x05, 0x11,0,{}},


{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,200,{}},

{0x39, 0xC6,  2 ,{0x3D, 0x00}},

{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,5,{}},

{0x05, 0x29,0,{}},

{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,50,{}},



	/* FIXME */
	/*
		params->dsi.horizontal_sync_active				= 0x16;// 50  2
		params->dsi.horizontal_backporch				= 0x38;
		params->dsi.horizontal_frontporch				= 0x18;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.horizontal_blanking_pixel =0;    //lenovo:fix flicker issue
	    //params->dsi.LPX=8; 
	*/

};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
#if 1
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   =BURST_VDO_MODE;// BURST_VDO_MODE;

//		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
//params->dsi.mode	 =SYNC_EVENT_VDO_MODE ;
	// DSI
	/* Command mode setting */	
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

   params->dsi.word_count=1024*3;
	// params->dsi.word_count=480*3;

	params->dsi.vertical_sync_active= 2; //10;//10;
	params->dsi.vertical_backporch= 8 ;//==========10;//12;
	params->dsi.vertical_frontporch= 6 ;//=========15;//9;
	params->dsi.vertical_active_line= FRAME_HEIGHT;//hight

	params->dsi.horizontal_sync_active				= 32 ; //110;//100;  //
	params->dsi.horizontal_backporch				= 93 ;//===========110;//80; //
	params->dsi.horizontal_frontporch				=  93 ;//===============100;//100; //
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;//=wight

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.ssc_disable=1;//huangyisong add for RF test 2014.11.25

	params->dsi.pll_select=0;	//0: MIPI_PLL; 1: LVDS_PLL
	params->dsi.PLL_CLOCK = 168;//this value must be in MTK suggested table
	params->dsi.cont_clock = 1;//if not config this para, must config other 7 or 3 paras to gen. PLL

#endif
}

static void lcm_init(void)
{
	unsigned int data_array[16];

#ifdef BUILD_LK
				printf("[LK/LCM] hlcm_init() himax enter\n");
#endif
        #ifdef BUILD_LK
					   printf("---lcm_init--hx8394d- \n");
#else
					   printk("---lcm_init--hx8394d-\n");
#endif


		mt_set_gpio_mode(GPIO_LCM1990_STBY, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM1990_STBY, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ONE);
		MDELAY(60);
		
		mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);   
		mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_STBY
		MDELAY(60);
		mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);    
		mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ZERO); // LCM_RST -h
		MDELAY(30);
		mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);    
		mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_RST -h
		
		MDELAY(30);

      //------------------B9h----------------//
		 data_array[0] = 0x00043902; 						 
		 data_array[1] = 0x9483FFB9; 				
	        dsi_set_cmdq(&data_array, 2, 1); 

	//------------------BAh----------------//
		 data_array[0] = 0x00033902; 						 
		 data_array[1] = 0x008373BA; //			
	        dsi_set_cmdq(&data_array, 2, 1); 
	
	//------------------B1h----------------//
		 data_array[0] = 0x00103902;			   
		 data_array[1] = 0x0E0E6CB1;   
		 data_array[2] = 0xEF0F0425; 
		 data_array[3] = 0x23DA6801; 
		 data_array[4] = 0x588AC086;  
		 dsi_set_cmdq(&data_array, 5, 1);

	//------------------B2h----------------//
               data_array[0] = 0x00103902;					   
		 data_array[1] = 0x0F4485B2; 
		 data_array[2] = 0x081C2409; 	
               data_array[3] = 0x004D1C08;
               data_array[4] = 0x48443000;  			
		 dsi_set_cmdq(&data_array, 5, 1);

	//------------------B4h----------------//
		 data_array[0] = 0x000D3902;			   
		 data_array[1] = 0x59FF00B4;   
		 data_array[2] = 0x595A595A; 
		 data_array[3] = 0x0176015A; 
		 data_array[4] = 0x00000076; 
		 dsi_set_cmdq(&data_array, 5, 1);

	//------------------BFh----------------//
              data_array[0] = 0x00043902;					   
		data_array[1] = 0x010E41BF; 			
		dsi_set_cmdq(&data_array, 2, 1);
	
      //------------------D2h----------------// 
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000000D2; 			
		dsi_set_cmdq(&data_array, 2, 1);	

	//------------------D3h----------------//
		 data_array[0] = 0x001F3902;		   
		 data_array[1] = 0x000100D3;   
		 data_array[2] = 0x0C0C0701; 
		 data_array[3] = 0x00071032; 
		 data_array[4] = 0x0A200005; 
		 data_array[5] = 0x32000905; 
		 data_array[6] = 0x21000810; 
		 data_array[7] = 0x23070821; 
		 data_array[8] = 0x0047070D; 
		 dsi_set_cmdq(&data_array,9, 1);

	//------------------D5h----------------//
		 data_array[0] = 0x002D3902;		   
		 data_array[1] = 0x010001D5;   
		 data_array[2] = 0x03020300; 
		 data_array[3] = 0x21202102; 
		 data_array[4] = 0x18021820; 
		 data_array[5] = 0x18181802; 
		 data_array[6] = 0x18181818; 
		 data_array[7] = 0x18181818; 
		 data_array[8] = 0x18181818; 
	        data_array[9] = 0x18181818; 
		 data_array[10] = 0x18181818; 
		 data_array[11] = 0x18181818; 
		 data_array[12] = 0x00000018; 
		 dsi_set_cmdq(&data_array,13, 1);


         //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000002BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

	 //------------------D8h----------------//
		 data_array[0] = 0x000D3902;		   
		 data_array[1] = 0xAAFFFFD8;   
		 data_array[2] = 0xFFA0AAAA; 
		 data_array[3] = 0xAAAAAAFF; 
               data_array[4] = 0x000000A0; 
		 dsi_set_cmdq(&data_array,5, 1);

         //------------------E0h----------------//
		 data_array[0] = 0x002B3902;		   
		 data_array[1] = 0x0C0600E0;   
		 data_array[2] = 0x1A3D2D28; 
		 data_array[3] = 0x0D0A0737; 
		 data_array[4] = 0x15120E18; 
		 data_array[5] = 0x10061513; 
		 data_array[6] = 0x06001611; 
		 data_array[7] = 0x3D2C280C; 
		 data_array[8] = 0x0A07371A; 
	        data_array[9] = 0x120F180C; 
		 data_array[10] = 0x06141315; 
		 data_array[11] = 0x00161110; 
		 dsi_set_cmdq(&data_array,12, 1);

       //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000000BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

	//------------------C1h----------------//
		 data_array[0] = 0x002C3902;		   
		 data_array[1] = 0x080001C1;    
		 data_array[2] = 0x28201810; 
		 data_array[3] = 0x48403830; 
		 data_array[4] = 0x68605850; 
		 data_array[5] = 0x88807870; 
		 data_array[6] = 0xA8A09890; 
		 data_array[7] = 0xC8C0B8B0; 
		 data_array[8] = 0xE8E0D8D0; 
	        data_array[9] = 0x00FFF8F0; 
		 data_array[10] = 0x00000000; 
		 data_array[11] = 0x00000000; 
		 dsi_set_cmdq(&data_array,12, 1);


       //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000001BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

      //------------------C1h----------------//
		 data_array[0] = 0x002B3902;		   
		 data_array[1] = 0x100800C1;   
		 data_array[2] = 0x30282018; 
		 data_array[3] = 0x50484038; 
		 data_array[4] = 0x70686058; 
		 data_array[5] = 0x90888078; 
		 data_array[6] = 0xB0A8A098; 
		 data_array[7] = 0xD0C8C0B8; 
		 data_array[8] = 0xF0E8E0D8; 
	        data_array[9] = 0x0000FFF8; 
		 data_array[10] = 0x00000000; 
		 data_array[11] = 0x00000000; 
		 dsi_set_cmdq(&data_array,12, 1);


      //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000002BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

      //------------------C1h----------------//
		 data_array[0] = 0x002B3902;		   
		 data_array[1] = 0x0E0700C1;   
		 data_array[2] = 0x2E261E16; 
		 data_array[3] = 0x50483F37; 
		 data_array[4] = 0x6E675F57; 
		 data_array[5] = 0x8C857D76; 
		 data_array[6] = 0xABA49C94; 
		 data_array[7] = 0xC9C1BAB3; 
		 data_array[8] = 0xE6E0D9D1; 
	        data_array[9] = 0x0000F5ED; 
		 data_array[10] = 0x00000000; 
		 data_array[11] = 0x00000000; 
		 dsi_set_cmdq(&data_array,12, 1);



	//------------------CCh----------------// 
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000009CC; 			
		dsi_set_cmdq(&data_array, 2, 1);	

      //------------------C7h----------------//
		data_array[0] = 0x00053902;					   
		data_array[1] = 0x40C000C7; 
		data_array[2] = 0x000000C0; 				
		dsi_set_cmdq(&data_array, 3, 1);

	//------------------C0h----------------//
              data_array[0] = 0x00033902;					   
		data_array[1] = 0x001430C0; 			
		dsi_set_cmdq(&data_array, 2, 1);	

      //------------------BCh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000007BC; 			
		dsi_set_cmdq(&data_array, 2, 1);

	//------------------11h----------------//		
		data_array[0] = 0x00110500;			   
		dsi_set_cmdq(&data_array, 1, 1); 
		MDELAY(1000);	

      //------------------C6h----------------//
              data_array[0] = 0x00033902;					   
		data_array[1] = 0x0000BDC6; 			
		dsi_set_cmdq(&data_array, 2, 1);	
              MDELAY(5);
		 
 	//------------------29h----------------//   
              data_array[0] = 0x00290500;			   
		dsi_set_cmdq(&data_array, 1, 1);
		MDELAY(10);

     //dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

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
	unsigned int data_array[64];

	    //lcm_init();
	#if 0
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(500);
	data_array[0] = 0x00033902;
	data_array[1] = 0x00003DC6;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(100);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);
	#endif

	#if 1
		mt_set_gpio_mode(GPIO_LCM1990_STBY, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM1990_STBY, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ONE);
		MDELAY(60);
		
		mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);   
		mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_STBY
		MDELAY(60);
		mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);    
		mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ZERO); // LCM_RST -h
		MDELAY(30);
		mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);    
		mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_RST -h
		
		MDELAY(30);

      //------------------B9h----------------//
		 data_array[0] = 0x00043902; 						 
		 data_array[1] = 0x9483FFB9; 				
	        dsi_set_cmdq(&data_array, 2, 1); 

	//------------------BAh----------------//
		 data_array[0] = 0x00033902; 						 
		 data_array[1] = 0x008373BA; //			
	        dsi_set_cmdq(&data_array, 2, 1); 
	
	//------------------B1h----------------//
		 data_array[0] = 0x00103902;			   
		 data_array[1] = 0x0E0E6CB1;   
		 data_array[2] = 0xEF0F0425; 
		 data_array[3] = 0x23DA6801; 
		 data_array[4] = 0x588AC086;  
		 dsi_set_cmdq(&data_array, 5, 1);

	//------------------B2h----------------//
               data_array[0] = 0x00103902;					   
		 data_array[1] = 0x0F4485B2; 
		 data_array[2] = 0x081C2409; 	
               data_array[3] = 0x004D1C08;
               data_array[4] = 0x48443000;  			
		 dsi_set_cmdq(&data_array, 5, 1);

	//------------------B4h----------------//
		 data_array[0] = 0x000D3902;			   
		 data_array[1] = 0x59FF00B4;   
		 data_array[2] = 0x595A595A; 
		 data_array[3] = 0x0176015A; 
		 data_array[4] = 0x00000076; 
		 dsi_set_cmdq(&data_array, 5, 1);

	//------------------BFh----------------//
              data_array[0] = 0x00043902;					   
		data_array[1] = 0x010E41BF; 			
		dsi_set_cmdq(&data_array, 2, 1);
	
      //------------------D2h----------------// 
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000000D2; 			
		dsi_set_cmdq(&data_array, 2, 1);	

	//------------------D3h----------------//
		 data_array[0] = 0x001F3902;		   
		 data_array[1] = 0x000100D3;   
		 data_array[2] = 0x0C0C0701; 
		 data_array[3] = 0x00071032; 
		 data_array[4] = 0x0A200005; 
		 data_array[5] = 0x32000905; 
		 data_array[6] = 0x21000810; 
		 data_array[7] = 0x23070821; 
		 data_array[8] = 0x0047070D; 
		 dsi_set_cmdq(&data_array,9, 1);

	//------------------D5h----------------//
		 data_array[0] = 0x002D3902;		   
		 data_array[1] = 0x010001D5;   
		 data_array[2] = 0x03020300; 
		 data_array[3] = 0x21202102; 
		 data_array[4] = 0x18021820; 
		 data_array[5] = 0x18181802; 
		 data_array[6] = 0x18181818; 
		 data_array[7] = 0x18181818; 
		 data_array[8] = 0x18181818; 
	        data_array[9] = 0x18181818; 
		 data_array[10] = 0x18181818; 
		 data_array[11] = 0x18181818; 
		 data_array[12] = 0x00000018; 
		 dsi_set_cmdq(&data_array,13, 1);


         //------------------BDh----------------//
                data_array[0] = 0x00023902;					   
		data_array[1] = 0x000002BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

	 //------------------D8h----------------//
		 data_array[0] = 0x000D3902;		   
		 data_array[1] = 0xAAFFFFD8;   
		 data_array[2] = 0xFFA0AAAA; 
		 data_array[3] = 0xAAAAAAFF; 
               data_array[4] = 0x000000A0; 
		 dsi_set_cmdq(&data_array,5, 1);

         //------------------E0h----------------//
		 data_array[0] = 0x002B3902;		   
		 data_array[1] = 0x0C0600E0;   
		 data_array[2] = 0x1A3D2D28; 
		 data_array[3] = 0x0D0A0737; 
		 data_array[4] = 0x15120E18; 
		 data_array[5] = 0x10061513; 
		 data_array[6] = 0x06001611; 
		 data_array[7] = 0x3D2C280C; 
		 data_array[8] = 0x0A07371A; 
	        data_array[9] = 0x120F180C; 
		 data_array[10] = 0x06141315; 
		 data_array[11] = 0x00161110; 
		 dsi_set_cmdq(&data_array,12, 1);

       //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000000BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

	//------------------C1h----------------//
		 data_array[0] = 0x002C3902;		   
		 data_array[1] = 0x080001C1;   
		 data_array[2] = 0x28201810; 
		 data_array[3] = 0x48403830; 
		 data_array[4] = 0x68605850; 
		 data_array[5] = 0x88807870; 
		 data_array[6] = 0xA8A09890; 
		 data_array[7] = 0xC8C0B8B0; 
		 data_array[8] = 0xE8E0D8D0; 
	        data_array[9] = 0x00FFF8F0; 
		 data_array[10] = 0x00000000; 
		 data_array[11] = 0x00000000; 
		 dsi_set_cmdq(&data_array,12, 1);



       //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000001BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

      //------------------C1h----------------//
		 data_array[0] = 0x002B3902;		   
		 data_array[1] = 0x100800C1;   
		 data_array[2] = 0x30282018; 
		 data_array[3] = 0x50484038; 
		 data_array[4] = 0x70686058; 
		 data_array[5] = 0x90888078; 
		 data_array[6] = 0xB0A8A098; 
		 data_array[7] = 0xD0C8C0B8; 
		 data_array[8] = 0xF0E8E0D8; 
	        data_array[9] = 0x0000FFF8; 
		 data_array[10] = 0x00000000; 
		 data_array[11] = 0x00000000; 
		 dsi_set_cmdq(&data_array,12, 1);

      //------------------BDh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000002BD; 			
		dsi_set_cmdq(&data_array, 2, 1);

      //------------------C1h----------------//
		 data_array[0] = 0x002B3902;		   
		 data_array[1] = 0x0E0700C1;   
		 data_array[2] = 0x2E261E16; 
		 data_array[3] = 0x50483F37; 
		 data_array[4] = 0x6E675F57; 
		 data_array[5] = 0x8C857D76; 
		 data_array[6] = 0xABA49C94; 
		 data_array[7] = 0xC9C1BAB3; 
		 data_array[8] = 0xE6E0D9D1; 
	        data_array[9] = 0x0000F5ED; 
		 data_array[10] = 0x00000000; 
		 data_array[11] = 0x00000000; 
		 dsi_set_cmdq(&data_array,12, 1);


	//------------------CCh----------------// 
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000009CC; 			
		dsi_set_cmdq(&data_array, 2, 1);	

      //------------------C7h----------------//
		data_array[0] = 0x00053902;					   
		data_array[1] = 0x40C000C7; 
		data_array[2] = 0x000000C0; 				
		dsi_set_cmdq(&data_array, 3, 1);

	//------------------C0h----------------//
              data_array[0] = 0x00033902;					   
		data_array[1] = 0x001430C0; 			
		dsi_set_cmdq(&data_array, 2, 1);	

      //------------------BCh----------------//
              data_array[0] = 0x00023902;					   
		data_array[1] = 0x000007BC; 			
		dsi_set_cmdq(&data_array, 2, 1);

	//------------------11h----------------//		
		data_array[0] = 0x00110500;			   
		dsi_set_cmdq(&data_array, 1, 1); 
		MDELAY(200);	

      //------------------C6h----------------//
              data_array[0] = 0x00033902;					   
		data_array[1] = 0x0000BDC6; 			
		dsi_set_cmdq(&data_array, 2, 1);	
              MDELAY(5);
		 
 	//------------------29h----------------//   
              data_array[0] = 0x00290500;			   
		dsi_set_cmdq(&data_array, 1, 1);
		MDELAY(10);
	#endif

}
         
#if 0// (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	
	unsigned char buffer[3];
	unsigned int array[16];
	
	mt_set_gpio_mode(GPIO_LCM1990_STBY, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM1990_STBY, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ONE);

	MDELAY(120);
	mt_set_gpio_mode(GPIO_LCM1990_RESET, GPIO_MODE_00);   
	mt_set_gpio_dir(GPIO_LCM1990_RESET, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_STBY
	MDELAY(60);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ZERO); // LCM_RST -h
	MDELAY(30);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); // LCM_RST -h
	MDELAY(30);


		array[0] = 0x00043902;                          
		array[1] = 0x9483FFB9;                 
		dsi_set_cmdq(array, 2, 1);
		MDELAY(10); 

		array[0] = 0x00033902;                          
		array[1] = 0x008373BA;                 
		dsi_set_cmdq(array, 2, 1);

		array[0] = 0x00013700;// read id return two byte,version and id
		dsi_set_cmdq(array, 1, 1);
		//	id = read_reg(0xF4);
		read_reg_v2(0xF4, buffer, 1);
		id = buffer[0]; //we only need ID
		id1 = buffer[1]; 
		id2 = buffer[2];
#ifdef BUILD_LK
		   printf("%s, LK himax debug: nt35590 id = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, id,id1,id2);
#else
		   printk("%s, kernel himax horse debug: nt35590 id = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, id,id1,id2);
#endif

   return (LCM_ID == id)?1:0;
  

}


static unsigned int lcm_esd_check(void)
{
   unsigned int data_array[64];

#if 0
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
#else
	return FALSE;
#endif
#endif
#if 1
  #ifndef BUILD_LK 
        char  buffer[6]; 
        int   array[4]; 
        //char  b1_buffer[10]; 
        #ifdef BUILD_LK
			   printf("---lcm_esd_check--hx8394d- \n");
#else
			   printk("---lcm_esd_check--hx8394d-\n");
#endif
        if(lcm_esd_test) 
        { 
                lcm_esd_test = FALSE; 
                return TRUE; 
        } 

	//------------------CCh----------------// 
                data_array[0] = 0x00023902;					   
		data_array[1] = 0x000009CC; 			
		dsi_set_cmdq(&data_array, 2, 1);


        array[0] = 0x00043700; 
        dsi_set_cmdq(array, 1, 1); 

        read_reg_v2(0x09, buffer, 4); 
        array[0] = 0x00013700; 
        dsi_set_cmdq(array, 1, 1); 

        read_reg_v2(0xD9, &buffer[4], 1); 

	//char buffer2[6];
	//read_reg_v2(0x45, buffer2, 2);
        

        printk("<%s:%d>buffer[%x][%x][%x][%x][%x]\n", __func__, __LINE__, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	//printk("<%s:%d>buffer[%x][%x][%x][%x]\n", __func__, __LINE__, buffer[0], buffer[1], buffer[2], buffer[3]);  

        if((buffer[0]==0x80) && (buffer[1]==0x73) && (buffer[2]==0x04) && (buffer[3]==0x00) && (buffer[4]==0x80))
	//if((buffer[0]==0x80) && (buffer[1]==0x73) && (buffer[2]==0x04) && (buffer[3]==0x00))  
        { 
                return FALSE; 
        } 
        else 
        {                         
                return TRUE; 
        } 
#else 
        #ifdef BUILD_LK
					   printf("--1-lcm_esd_check--hx8394d- \n");
#else
					   printk("--1-lcm_esd_check--hx8394d-\n");
#endif

        return FALSE; 
#endif 


#endif

}

static unsigned int lcm_esd_recover(void)
{
  
	unsigned int data_array[64];

	#ifdef BUILD_LK
				   printf("---lcm_esd_recover--hx8394d- \n");
	#else
				   printk("---lcm_esd_recover--hx8394d-\n");
	#endif
        

 	//------------------28h----------------//   
        data_array[0] = 0x00280500;			   
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);
		
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ZERO);
	MDELAY(50);
 
	mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ZERO);
	MDELAY(180);
	mt_set_gpio_out(GPIO_LCM1990_STBY, GPIO_OUT_ONE);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCM1990_RESET, GPIO_OUT_ONE); 
	MDELAY(10);

	#ifdef BUILD_LK
			   printf("---lcm_esd_recover-Reset Done- \n");
	#else
			   printk("---lcm_esd_recover-Reset Done-\n");
	#endif

	lcm_init();
//	lcm_resume();

	return TRUE;
}



LCM_DRIVER hx8394d_mipi_dsi_vdo_boe_lcm_drv = 
{
    .name			= "hx8394d_mipi_dsi_vdo_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#if 0//(LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
