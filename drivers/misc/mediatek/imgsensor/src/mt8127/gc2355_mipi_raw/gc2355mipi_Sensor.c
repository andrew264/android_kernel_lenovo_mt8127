/*******************************************************************************************/


/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h> 
#include <asm/system.h>

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc2355mipi_Sensor.h"
#include "gc2355mipi_Camera_Sensor_para.h"
#include "gc2355mipi_CameraCustomized.h"
static DEFINE_SPINLOCK(gc2355mipiraw_drv_lock);

#define GC2355_DEBUG
//#define GC2355_DEBUG_SOFIA
#define GC2355_TEST_PATTERN_CHECKSUM (0x9d1c9dad)//(0x5593c632)

#ifdef GC2355_DEBUG
	#define GC2355DB(fmt, arg...) printk("[GC2355mipiraw][%s] " fmt, __func__, ##arg)
#else
	#define GC2355DB(fmt, arg...)
#endif

#ifdef GC2355_DEBUG_SOFIA
	#define GC2355DBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[GC2355Raw] ",  fmt, ##arg)
#else
	#define GC2355DBSOFIA(fmt, arg...)
#endif

#define mDELAY(ms)  mdelay(ms)

kal_uint32 GC2355_FeatureControl_PERIOD_PixelNum=GC2355_PV_PERIOD_PIXEL_NUMS;
kal_uint32 GC2355_FeatureControl_PERIOD_LineNum=GC2355_PV_PERIOD_LINE_NUMS;
UINT16  gc2355VIDEO_MODE_TARGET_FPS = 30;
MSDK_SENSOR_CONFIG_STRUCT GC2355SensorConfigData;
MSDK_SCENARIO_ID_ENUM GC2355CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT GC2355SensorCCT[]=GC2355MIPI_CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT GC2355SensorReg[ENGINEER_END]=GC2355MIPI_CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static GC2355_PARA_STRUCT gc2355;

#define Sleep(ms) mdelay(ms)
#define GC2355_ORIENTATION IMAGE_NORMAL //IMAGE_NORMAL//IMAGE_NORMAL

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

/*************************************************************************
* FUNCTION
*    GC2355MIPI_write_cmos_sensor
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
static void GC2355_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , 2, GC2355MIPI_WRITE_ID); 
}

/*************************************************************************
* FUNCTION
*    GC2035_read_cmos_sensor
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
static kal_uint8 GC2355_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , 1, (u8*)in_buff, 1, GC2355MIPI_WRITE_ID)) {
        GC2355DB("ERROR: GC2355MIPI_read_cmos_sensor \n");
    }
  return in_buff[0];
}


void GC2355_SetGain(UINT16 iGain)
{

#define ANALOG_GAIN_1 64  // 1.00x
#define ANALOG_GAIN_2 88  //1.37x
#define ANALOG_GAIN_3 121  // 1.89x
#define ANALOG_GAIN_4 168  // 2.59x
#define ANALOG_GAIN_5 239  // 3.70x
#define ANALOG_GAIN_6 330  // 5.06x
#define ANALOG_GAIN_7 470  // 7.15x

	kal_uint16 iReg,temp,temp1;
		
	GC2355DB("GC2355MIPI_SetGain iGain = %d \n",iGain);

	GC2355_write_cmos_sensor(0xb1, 0x01);
	GC2355_write_cmos_sensor(0xb2, 0x00);
	
	iReg = iGain;
	//digital gain
	//GC2355MIPI_write_cmos_sensor(0xb1, iReg>>6);
	//GC2355MIPI_write_cmos_sensor(0xb2, (iReg<<2)&0xfc);
	
	
	if(iReg < 0x40)
		iReg = 0x40;
	if(iReg > 512)
		iReg = 512;
		
	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
			//analog gain
			GC2355_write_cmos_sensor(0xb6,	0x00);// 
			temp = iReg;
			GC2355_write_cmos_sensor(0xb1, temp>>6);
			GC2355_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
			GC2355DB("GC2355 analogic gain 1x , GC2355 add pregain = %d\n",temp);
		
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{
			//analog gain
			GC2355_write_cmos_sensor(0xb6,	0x01);// 
			temp = 64*iReg/ANALOG_GAIN_2;
			GC2355_write_cmos_sensor(0xb1, temp>>6);
			GC2355_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
			GC2355DB("GC2355 analogic gain 1.45x , GC2355 add pregain = %d\n",temp);
		
	}
	
	else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
			//analog gain
			GC2355_write_cmos_sensor(0xb6,	0x02);//
			temp = 64*iReg/ANALOG_GAIN_3;
			GC2355_write_cmos_sensor(0xb1, temp>>6);
			GC2355_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
			GC2355DB("GC2355 analogic gain 2.02x , GC2355 add pregain = %d\n",temp);	
	}
	
//	else if(ANALOG_GAIN_4<= iReg)
    else
	{
			//analog gain
			GC2355_write_cmos_sensor(0xb6,	0x03);//
			temp = 64*iReg/ANALOG_GAIN_4;
			GC2355_write_cmos_sensor(0xb1, temp>>6);
			GC2355_write_cmos_sensor(0xb2, (temp<<2)&0xfc);
			
			GC2355DB("GC2355 analogic gain 2.8x, temp = %d\n", temp);
	}

}   /*  GC2355_SetGain_SetGain  */


void GC2355_camera_para_to_sensor(void)
{}

void GC2355_sensor_to_camera_para(void)
{}

kal_int32  GC2355_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void GC2355_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{}
void GC2355_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{}
kal_bool GC2355_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{    return KAL_TRUE;}

static void GC2355_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
 	kal_uint32 hb = 0;
	kal_uint32 vb = 0;

	if ( SENSOR_MODE_PREVIEW == gc2355.sensorMode )	//SXGA size output
	{
		hb = GC2355_DEFAULT_DUMMY_PIXEL_NUMS + iPixels;
		vb = GC2355_DEFAULT_DUMMY_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== gc2355.sensorMode )
	{
		hb = GC2355_DEFAULT_DUMMY_PIXEL_NUMS + iPixels;
		vb = GC2355_DEFAULT_DUMMY_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		hb = GC2355_DEFAULT_DUMMY_PIXEL_NUMS + iPixels;
		vb = GC2355_DEFAULT_DUMMY_LINE_NUMS + iLines;
	}

	//if(gc2355.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < GC2355_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < GC2355_MAX_FRAME_LENGTH);	//0xFFFF

	//Set HB
	GC2355_write_cmos_sensor(0x05, (hb >> 8)& 0xFF);
	GC2355_write_cmos_sensor(0x06, hb & 0xFF);

	//Set VB
	GC2355_write_cmos_sensor(0x07, (vb >> 8) & 0xFF);
	GC2355_write_cmos_sensor(0x08, vb & 0xFF);
	GC2355DB("GC2355_SetDummy framelength=%d\n",vb+GC2355_VALID_LINE_NUMS);

}   /*  GC2355_SetDummy */

static void GC2355_Sensor_Init(void)
{
	/* SYS */
	GC2355_write_cmos_sensor(0xfe,0x80);
	GC2355_write_cmos_sensor(0xfe,0x80);
	GC2355_write_cmos_sensor(0xfe,0x80);
	GC2355_write_cmos_sensor(0xf2,0x00); //sync_pad_io_ebi
	GC2355_write_cmos_sensor(0xf6,0x00); //up down
	GC2355_write_cmos_sensor(0xf7,0x31); //19 //pll enable
	GC2355_write_cmos_sensor(0xf8,0x06); //Pll mode 2  /////86--������Ƶ
	GC2355_write_cmos_sensor(0xf9,0x0e); //de//[0] pll enable
	GC2355_write_cmos_sensor(0xfa,0x00); //div
	GC2355_write_cmos_sensor(0xfc,0x06); //4e
	//GC2355_write_cmos_sensor(0xfd,0x00);
	GC2355_write_cmos_sensor(0xfe,0x00);
	
	/* ANALOG & CISCTL*/
	// AEC&frame length//
	GC2355_write_cmos_sensor(0x03,0x04);  // 0a -> 04
	GC2355_write_cmos_sensor(0x04,0xb0);  // 41 -> b0
	GC2355_write_cmos_sensor(0x05,0x01); //max 30fps  03
	GC2355_write_cmos_sensor(0x06,0x1c); 
	GC2355_write_cmos_sensor(0x07,0x00);
	GC2355_write_cmos_sensor(0x08,0x0e); //22
	GC2355_write_cmos_sensor(0x0a,0x00); //row start
	GC2355_write_cmos_sensor(0x0c,0x04); //0c//col start
	GC2355_write_cmos_sensor(0x0d,0x04);
	GC2355_write_cmos_sensor(0x0e,0xc0); //c0
	GC2355_write_cmos_sensor(0x0f,0x06); 
	GC2355_write_cmos_sensor(0x10,0x50); //Window setting 1616x1216
	/*GC2355_write_cmos_sensor(0x11,0x00);
	GC2355_write_cmos_sensor(0x12,0x18); //sh_delay
	GC2355_write_cmos_sensor(0x13,0x11);
	GC2355_write_cmos_sensor(0x14,0x01);
	GC2355_write_cmos_sensor(0x15,0x00);
	GC2355_write_cmos_sensor(0x16,0xc1);*/
	GC2355_write_cmos_sensor(0x17,0x14);//14
	//GC2355_write_cmos_sensor(0x18,0x02);
	GC2355_write_cmos_sensor(0x19,0x0b);
	GC2355_write_cmos_sensor(0x1b,0x48);
	GC2355_write_cmos_sensor(0x1c,0x12);
	GC2355_write_cmos_sensor(0x1d,0x10);
	GC2355_write_cmos_sensor(0x1e,0xbc);
	GC2355_write_cmos_sensor(0x1f,0xc9);
	GC2355_write_cmos_sensor(0x20,0x71);
	GC2355_write_cmos_sensor(0x21,0x20);
	GC2355_write_cmos_sensor(0x22,0xa0);
	GC2355_write_cmos_sensor(0x23,0x51);
	GC2355_write_cmos_sensor(0x24,0x19);
	GC2355_write_cmos_sensor(0x27,0x20);
	GC2355_write_cmos_sensor(0x28,0x00);
	GC2355_write_cmos_sensor(0x2b,0x80);// 0x81 20140926
	GC2355_write_cmos_sensor(0x2c,0x38);
	GC2355_write_cmos_sensor(0x2e,0x16);
	GC2355_write_cmos_sensor(0x2f,0x14);
	GC2355_write_cmos_sensor(0x30,0x00);
	GC2355_write_cmos_sensor(0x31,0x01);
	GC2355_write_cmos_sensor(0x32,0x02);
	GC2355_write_cmos_sensor(0x33,0x03);
	GC2355_write_cmos_sensor(0x34,0x07);
	GC2355_write_cmos_sensor(0x35,0x0b);
	GC2355_write_cmos_sensor(0x36,0x0f);
	
    /////////////////////////////////////////////////////
	//////////////////////   MIPI   /////////////////////
	/////////////////////////////////////////////////////
	GC2355_write_cmos_sensor(0xfe, 0x03);

	GC2355_write_cmos_sensor(0x10, 0x81);
	GC2355_write_cmos_sensor(0x01, 0x87);
	GC2355_write_cmos_sensor(0x22, 0x03);  
	GC2355_write_cmos_sensor(0x23, 0x20);
	GC2355_write_cmos_sensor(0x25, 0x10);
	GC2355_write_cmos_sensor(0x29, 0x02);

	GC2355_write_cmos_sensor(0x02, 0x00);
	GC2355_write_cmos_sensor(0x03, 0x90);
	GC2355_write_cmos_sensor(0x04, 0x01);
	GC2355_write_cmos_sensor(0x05, 0x00);
	GC2355_write_cmos_sensor(0x06, 0xa2);
	GC2355_write_cmos_sensor(0x11, 0x2b);
	GC2355_write_cmos_sensor(0x12, 0xd0); 
	GC2355_write_cmos_sensor(0x13, 0x07); 
	GC2355_write_cmos_sensor(0x15, 0x60);

	GC2355_write_cmos_sensor(0x21, 0x10);
	GC2355_write_cmos_sensor(0x24, 0x02);
	GC2355_write_cmos_sensor(0x26, 0x08);
	GC2355_write_cmos_sensor(0x27, 0x06);
	GC2355_write_cmos_sensor(0x2a, 0x0a); 
	GC2355_write_cmos_sensor(0x2b, 0x08);
	
	GC2355_write_cmos_sensor(0x40, 0x00);
	GC2355_write_cmos_sensor(0x41, 0x00);    
	GC2355_write_cmos_sensor(0x42, 0x40);
	GC2355_write_cmos_sensor(0x43, 0x06);  
	GC2355_write_cmos_sensor(0xfe, 0x00);

	/////////////////////////////////////////////////////
	//////////////////////	 gain   /////////////////////
	/////////////////////////////////////////////////////
	GC2355_write_cmos_sensor(0xb0,0x50);
	GC2355_write_cmos_sensor(0xb1,0x01);
	GC2355_write_cmos_sensor(0xb2,0x00);
	GC2355_write_cmos_sensor(0xb3,0x40);
	GC2355_write_cmos_sensor(0xb4,0x40);
	GC2355_write_cmos_sensor(0xb5,0x40);
	GC2355_write_cmos_sensor(0xb6,0x00);
	
	/* crop */
	GC2355_write_cmos_sensor(0x92,0x03);
	GC2355_write_cmos_sensor(0x94,0x01);
	GC2355_write_cmos_sensor(0x95,0x04);
	GC2355_write_cmos_sensor(0x96,0xb0);
	GC2355_write_cmos_sensor(0x97,0x06);
	GC2355_write_cmos_sensor(0x98,0x40); 

	/////////////////////////////////////////////////////
	//////////////////////    BLK   /////////////////////
	/////////////////////////////////////////////////////
	GC2355_write_cmos_sensor(0x18,0x02);
	GC2355_write_cmos_sensor(0x1a,0x01);
	GC2355_write_cmos_sensor(0x40,0x42);
	GC2355_write_cmos_sensor(0x41,0x00); 
	GC2355_write_cmos_sensor(0x44,0x00); 
	GC2355_write_cmos_sensor(0x45,0x00);
	GC2355_write_cmos_sensor(0x46,0x00);	
	GC2355_write_cmos_sensor(0x47,0x00); 
	GC2355_write_cmos_sensor(0x48,0x00); 
	GC2355_write_cmos_sensor(0x49,0x00);
	GC2355_write_cmos_sensor(0x4a,0x00);	
	GC2355_write_cmos_sensor(0x4b,0x00);	
	GC2355_write_cmos_sensor(0x4e,0x3c); 
	GC2355_write_cmos_sensor(0x4f,0x00); 
	GC2355_write_cmos_sensor(0x5e,0x00);
	GC2355_write_cmos_sensor(0x66,0x20);
	GC2355_write_cmos_sensor(0x6a,0x02);
	GC2355_write_cmos_sensor(0x6b,0x02);
	GC2355_write_cmos_sensor(0x6c,0x02);
	GC2355_write_cmos_sensor(0x6d,0x02);
	GC2355_write_cmos_sensor(0x6e,0x02);
	GC2355_write_cmos_sensor(0x6f,0x02);
	GC2355_write_cmos_sensor(0x70,0x02);
	GC2355_write_cmos_sensor(0x71,0x02);

	/////////////////////////////////////////////////////
	////////////////////  dark sun  /////////////////////
	/////////////////////////////////////////////////////
	GC2355_write_cmos_sensor(0x87,0x03); 
	GC2355_write_cmos_sensor(0xe0,0xe7); 
	GC2355_write_cmos_sensor(0xe3,0xc0); 

	/////////////////////////////////////////////////////
	//////////////////////   MIPI   /////////////////////
	/////////////////////////////////////////////////////
	GC2355_write_cmos_sensor(0xfe, 0x03);

	GC2355_write_cmos_sensor(0x10, 0x91);

	GC2355_write_cmos_sensor(0xfe, 0x00);

}   /*  GC2355_Sensor_Init  */
//extern IMM_auxadc_GetOneChannelValue_Cali(int Channel, int *voltage);
UINT32 GC2355Open(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
        int adc_val = 0;
	GC2355DB("GC2355Open enter :\n ");
	sensor_id=((GC2355_read_cmos_sensor(0xf0) << 8) | GC2355_read_cmos_sensor(0xf1)); 
   // IMM_auxadc_GetOneChannelValue_Cali(12, &adc_val);
   // if (sensor_id == GC2355_SENSOR_ID) ) {   /*0.1v = 100000uv , 0v :huaqun ; >0.1:qunhui*/
	//sensor_id = GC2355MIPI_SENSOR_ID;
   //} else {
	//sensor_id = 0xFFFFFFFF;
if (sensor_id != GC2355_SENSOR_ID)
	return ERROR_SENSOR_CONNECT_FAIL;
  // }
        GC2355DB("[%s,%d]:  adc_val = %x  \n", __FUNCTION__, __LINE__, adc_val);
	GC2355DB("GC2355MIPIGetSensorID:%x \n",sensor_id);
	spin_lock(&gc2355mipiraw_drv_lock);
	gc2355.sensorMode = SENSOR_MODE_INIT;
	gc2355.GC2355AutoFlickerMode = KAL_FALSE;
	gc2355.GC2355VideoMode = KAL_FALSE;
	gc2355.shutter = 0xff;
	gc2355.Gain = 64;
	spin_unlock(&gc2355mipiraw_drv_lock);
	GC2355_Sensor_Init();

	GC2355DB("GC2355Open exit :\n ");

    return ERROR_NONE;
}


UINT32 GC2355GetSensorID(UINT32 *sensorID)
{
    int  retry = 1;
    int adc_val = 0;
    GC2355DB("GC2355GetSensorID enter :\n ");

    // check if sensor ID correct
    *sensorID =((GC2355_read_cmos_sensor(0xf0) << 8) | GC2355_read_cmos_sensor(0xf1)); 
   // IMM_auxadc_GetOneChannelValue_Cali(12, &adc_val);
   // if ((*sensorID == GC2355MIPI_SENSOR_ID) && (adc_val < 100000)) {   /*0.1v = 100000uv , 0v :huaqun ; >0.1:qunhui*/
	//*sensorID = GC2355MIPI_SENSOR_ID;
  // } else {
	//*sensorID = 0xFFFFFFFF;
   if (*sensorID != GC2355_SENSOR_ID)
      {
        *sensorID = 0xFFFFFFFF;
	return ERROR_SENSOR_CONNECT_FAIL;
      }
        GC2355DB("[%s,%d]:  adc_val = %d  \n", __FUNCTION__, __LINE__, adc_val);
	GC2355DB("GC2355MIPIGetSensorID:%x \n",*sensorID);
 /*
    if (*sensorID != GC2355MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
  */
    return ERROR_NONE;
}


void GC2355_SetShutter(kal_uint32 iShutter)
{
    //kal_uint8 i=0;
	if(MSDK_SCENARIO_ID_CAMERA_ZSD == GC2355CurrentScenarioId )
	{
		//GC2355DB("always UPDATE SHUTTER when gc2355.sensorMode == SENSOR_MODE_CAPTURE\n");
	}
	else{
		if(gc2355.sensorMode == SENSOR_MODE_CAPTURE)
		{
			//GC2355DB("capture!!DONT UPDATE SHUTTER!!\n");
			//return;
		}
	}
	if(gc2355.shutter == iShutter);
		//return;
	
   spin_lock(&gc2355mipiraw_drv_lock);
   gc2355.shutter= iShutter;
  
   spin_unlock(&gc2355mipiraw_drv_lock);
   if(gc2355.shutter > 16383) gc2355.shutter = 16383;
   if(gc2355.shutter < 12) gc2355.shutter = 12;
   GC2355_write_cmos_sensor(0x03, (gc2355.shutter>>8) & 0x3F);
   GC2355_write_cmos_sensor(0x04, gc2355.shutter & 0xFF);
   	
   
   GC2355DB("GC2355_SetShutter:%x \n",iShutter); 
   	
 //  GC2355DB("GC2355_SetShutter 0x03=%x, 0x04=%x \n",GC2355_read_cmos_sensor(0x03),GC2355_read_cmos_sensor(0x04));
   return;
}   /*  GC2355_SetShutter   */

UINT32 GC2355_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2;
	UINT32 shutter =0;
	temp_reg1 = GC2355_read_cmos_sensor(0x03);    // AEC[b19~b16]
	temp_reg2 = GC2355_read_cmos_sensor(0x04);    // AEC[b15~b8]
	shutter  = ((temp_reg1 << 8)| (temp_reg2));

	return shutter;
}

void GC2355_NightMode(kal_bool bEnable)
{}


UINT32 GC2355Close(void)
{    return ERROR_NONE;}

void GC2355SetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;

    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NORMAL:
			GC2355_write_cmos_sensor(0x17,0x14);//bit[1][0]
//			GC2355_write_cmos_sensor(0x92,0x03);
//			GC2355_write_cmos_sensor(0x94,0x0b);
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            GC2355_write_cmos_sensor(0x17,0x15);
//			GC2355_write_cmos_sensor(0x92,0x03);
//			GC2355_write_cmos_sensor(0x94,0x0b);
            break;
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            GC2355_write_cmos_sensor(0x17,0x16);
//			GC2355_write_cmos_sensor(0x92,0x02);
//			GC2355_write_cmos_sensor(0x94,0x0b);
            break;
        case IMAGE_HV_MIRROR://IMAGE_HV_MIRROR:
			GC2355_write_cmos_sensor(0x17,0x17);
//			GC2355_write_cmos_sensor(0x92,0x02);
//			GC2355_write_cmos_sensor(0x94,0x0b);
            break;
    }
}

UINT32 GC2355Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	GC2355DB("GC2355Preview enter:");

	
	spin_lock(&gc2355mipiraw_drv_lock);
	gc2355.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	gc2355.DummyPixels = 0;//define dummy pixels and lines
	gc2355.DummyLines = 0 ;
	GC2355_FeatureControl_PERIOD_PixelNum=GC2355_PV_PERIOD_PIXEL_NUMS+ gc2355.DummyPixels;
	GC2355_FeatureControl_PERIOD_LineNum=GC2355_PV_PERIOD_LINE_NUMS+gc2355.DummyLines;
	spin_unlock(&gc2355mipiraw_drv_lock);

	//GC2355_write_shutter(gc2355.shutter);
	//write_GC2355_gain(gc2355.pvGain);
	//set mirror & flip
	//GC2355DB("[GC2355Preview] mirror&flip: %d \n",sensor_config_data->SensorImageMirror);
	spin_lock(&gc2355mipiraw_drv_lock);
	gc2355.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&gc2355mipiraw_drv_lock);
	//if(SENSOR_MODE_PREVIEW==gc2355.sensorMode ) return ERROR_NONE;
	
	GC2355DB("GC2355Preview mirror:%d\n",sensor_config_data->SensorImageMirror);
	GC2355SetFlipMirror(GC2355_ORIENTATION);
	GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);
	GC2355DB("GC2355Preview exit: \n");
	mdelay(20);
    return ERROR_NONE;
}	/* GC2355Preview() */


UINT32 GC2355Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	GC2355DB("GC2355Video enter:");
	if(gc2355.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
		//GC2355VideoSetting();
	
	spin_lock(&gc2355mipiraw_drv_lock);
	gc2355.sensorMode = SENSOR_MODE_VIDEO;
	GC2355_FeatureControl_PERIOD_PixelNum=GC2355_VIDEO_PERIOD_PIXEL_NUMS+ gc2355.DummyPixels;
	GC2355_FeatureControl_PERIOD_LineNum=GC2355_VIDEO_PERIOD_LINE_NUMS+gc2355.DummyLines;
/*	
	spin_unlock(&gc2355mipiraw_drv_lock);

	//GC2355_write_shutter(gc2355.shutter);
	//write_GC2355_gain(gc2355.pvGain);

	spin_lock(&gc2355mipiraw_drv_lock);
*/
	gc2355.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&gc2355mipiraw_drv_lock);
	GC2355SetFlipMirror(GC2355_ORIENTATION);
	GC2355DB("GC2355Video mirror:%d\n",sensor_config_data->SensorImageMirror);
	GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);
	//GC2355DBSOFIA("[GC2355Video]frame_len=%x\n", ((GC2355_read_cmos_sensor(0x380e)<<8)+GC2355_read_cmos_sensor(0x380f)));
	mdelay(20);
	GC2355DB("GC2355Video exit:\n");
    return ERROR_NONE;
}


UINT32 GC2355Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 	kal_uint32 shutter = gc2355.shutter;
	kal_uint32 temp_data;
	if( SENSOR_MODE_CAPTURE== gc2355.sensorMode)
	{
		GC2355DB("GC2355Capture BusrtShot!!!\n");
	}else{
		GC2355DB("GC2355Capture enter:\n");
		
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.pvShutter =shutter;
		gc2355.sensorGlobalGain = temp_data;
		gc2355.pvGain =gc2355.sensorGlobalGain;
		spin_unlock(&gc2355mipiraw_drv_lock);

		GC2355DB("[GC2355Capture]gc2355.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",gc2355.shutter, shutter,gc2355.sensorGlobalGain);

		// Full size setting
		//.GC2355CaptureSetting();

		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.sensorMode = SENSOR_MODE_CAPTURE;
		gc2355.imgMirror = sensor_config_data->SensorImageMirror;
		gc2355.DummyPixels = 0;//define dummy pixels and lines                                                                                                         
		gc2355.DummyLines = 0 ;    
		GC2355_FeatureControl_PERIOD_PixelNum = GC2355_FULL_PERIOD_PIXEL_NUMS + gc2355.DummyPixels;
		GC2355_FeatureControl_PERIOD_LineNum = GC2355_FULL_PERIOD_LINE_NUMS + gc2355.DummyLines;
		spin_unlock(&gc2355mipiraw_drv_lock);

		//GC2355DB("[GC2355Capture] mirror&flip: %d\n",sensor_config_data->SensorImageMirror);
		
		GC2355DB("GC2355capture mirror:%d\n",sensor_config_data->SensorImageMirror);
		GC2355SetFlipMirror(GC2355_ORIENTATION);
		GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);

	    if(GC2355CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
	    {
			GC2355DB("GC2355Capture exit ZSD!!\n");
			return ERROR_NONE;
	    }
		GC2355DB("GC2355Capture exit:\n");
	}

    return ERROR_NONE;
}	/* GC2355Capture() */

UINT32 GC2355GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    GC2355DB("GC2355GetResolution!!\n");
	pSensorResolution->SensorPreviewWidth	= GC2355_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= GC2355_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= GC2355_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= GC2355_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth		= GC2355_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = GC2355_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   /* GC2355GetResolution() */

UINT32 GC2355GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorPreviewResolutionX= GC2355_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= GC2355_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX= GC2355_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= GC2355_IMAGE_SENSOR_FULL_HEIGHT;

	spin_lock(&gc2355mipiraw_drv_lock);
	gc2355.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&gc2355mipiraw_drv_lock);

	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
	
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;

    pSensorInfo->CaptureDelayFrame = 4;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 3;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = GC2355_PV_X_START;
            pSensorInfo->SensorGrabStartY = GC2355_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = GC2355_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = GC2355_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = GC2355_FULL_X_START;	//2*GC2355_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = GC2355_FULL_Y_START;	//2*GC2355_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = GC2355_PV_X_START;
            pSensorInfo->SensorGrabStartY = GC2355_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &GC2355SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* GC2355GetInfo() */


UINT32 GC2355Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&gc2355mipiraw_drv_lock);
		GC2355CurrentScenarioId = ScenarioId;
		spin_unlock(&gc2355mipiraw_drv_lock);
		GC2355DB("GC2355CurrentScenarioId=%d\n",GC2355CurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            GC2355Preview(pImageWindow, pSensorConfigData);
            break;
/*        
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC2355Video(pImageWindow, pSensorConfigData);
			break;
	*/		
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            GC2355Capture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
} /* GC2355Control() */


UINT32 GC2355SetVideoMode(UINT16 u2FrameRate)
{
    kal_uint32 MIN_Frame_length =0,extralines=0;
    GC2355DB("[GC2355SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&gc2355mipiraw_drv_lock);
	 gc2355VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&gc2355mipiraw_drv_lock);

	if(u2FrameRate==0)
	{   
		GC2355DB("Disable Video Mode or dynimac fps\n");	
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.DummyPixels = 0;//define dummy pixels and lines
		gc2355.DummyLines = extralines ;
		spin_unlock(&gc2355mipiraw_drv_lock);
		//GC2355_SetDummy(gc2355.DummyPixels ,gc2355.DummyLines);
		return KAL_TRUE;
	}
	
	if(u2FrameRate >30 || u2FrameRate <5)
	    GC2355DB("error frame rate seting %d fps\n",u2FrameRate);

    if(gc2355.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	if(u2FrameRate==30)
    		{
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.DummyPixels = 0;//define dummy pixels and lines
		gc2355.DummyLines = 0 ;
		spin_unlock(&gc2355mipiraw_drv_lock);
		
		GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);
    		}
		else
		{
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.DummyPixels = 0;//define dummy pixels and lines
		MIN_Frame_length = (GC2355MIPI_VIDEO_CLK)/(GC2355_VIDEO_PERIOD_PIXEL_NUMS + gc2355.DummyPixels)/u2FrameRate;
		gc2355.DummyLines = MIN_Frame_length - GC2355_VALID_LINE_NUMS-GC2355_DEFAULT_DUMMY_LINE_NUMS;
		spin_unlock(&gc2355mipiraw_drv_lock);
		GC2355DB("GC2355SetVideoMode MIN_Frame_length %d\n",MIN_Frame_length);
		
		GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);
    		}	
    }
	else if(gc2355.sensorMode == SENSOR_MODE_CAPTURE)
	{
		GC2355DB("-------[GC2355SetVideoMode]ZSD???---------\n");
		if(u2FrameRate==30)
    		{
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.DummyPixels = 0;//define dummy pixels and lines
		gc2355.DummyLines = 0 ;
		spin_unlock(&gc2355mipiraw_drv_lock);
		
		GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);
    		}
		else
			{
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.DummyPixels = 0;//define dummy pixels and lines
		MIN_Frame_length = (GC2355MIPI_VIDEO_CLK)/(GC2355_VIDEO_PERIOD_PIXEL_NUMS + gc2355.DummyPixels)/u2FrameRate;
		gc2355.DummyLines = MIN_Frame_length - GC2355_VALID_LINE_NUMS-GC2355_DEFAULT_DUMMY_LINE_NUMS;
		spin_unlock(&gc2355mipiraw_drv_lock);
		
		GC2355_SetDummy(gc2355.DummyPixels,gc2355.DummyLines);
    		}	
	}

    return KAL_TRUE;
}

UINT32 GC2355SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;
    //GC2355DB("[GC2355SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
	if(bEnable) {   // enable auto flicker
		spin_lock(&gc2355mipiraw_drv_lock);
		gc2355.GC2355AutoFlickerMode = KAL_TRUE;
		spin_unlock(&gc2355mipiraw_drv_lock);
    } else {
    	spin_lock(&gc2355mipiraw_drv_lock);
        gc2355.GC2355AutoFlickerMode = KAL_FALSE;
		spin_unlock(&gc2355mipiraw_drv_lock);
        GC2355DB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

UINT32 GC2355SetTestPatternMode(kal_bool bEnable)
{
    GC2355DB("[GC2355SetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable)
    {
        GC2355_write_cmos_sensor(0x8b,0xb2); //bit[4]: 1 enable test pattern, 0 disable test pattern
    }
	else
	{
	    GC2355_write_cmos_sensor(0x8b,0xa2);//bit[4]: 1 enable test pattern, 0 disable test pattern
	}
    return ERROR_NONE;
}


UINT32 GC2355MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	GC2355DB("GC2355MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = GC2355MIPI_PREVIEW_CLK;
			lineLength = GC2355_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - GC2355_VALID_LINE_NUMS;
			gc2355.sensorMode = SENSOR_MODE_PREVIEW;
			GC2355_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = GC2355MIPI_VIDEO_CLK; 
			lineLength = GC2355_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - GC2355_VALID_LINE_NUMS;
			gc2355.sensorMode = SENSOR_MODE_VIDEO;
			GC2355_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = GC2355MIPI_CAPTURE_CLK;
			lineLength = GC2355_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - GC2355_VALID_LINE_NUMS;
			gc2355.sensorMode = SENSOR_MODE_CAPTURE;
			GC2355_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}


UINT32 GC2355MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}


UINT32 GC2355FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= GC2355_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= GC2355_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= GC2355_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= GC2355_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(GC2355CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = GC2355MIPI_PREVIEW_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = GC2355MIPI_VIDEO_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = GC2355MIPI_CAPTURE_CLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = GC2355MIPI_CAPTURE_CLK;
					*pFeatureParaLen=4;
					break;
			}
		      break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            GC2355_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            GC2355_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            GC2355_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //GC2355_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            GC2355_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = GC2355_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&gc2355mipiraw_drv_lock);
                GC2355SensorCCT[i].Addr=*pFeatureData32++;
                GC2355SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&gc2355mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=GC2355SensorCCT[i].Addr;
                *pFeatureData32++=GC2355SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&gc2355mipiraw_drv_lock);
                GC2355SensorReg[i].Addr=*pFeatureData32++;
                GC2355SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&gc2355mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=GC2355SensorReg[i].Addr;
                *pFeatureData32++=GC2355SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=GC2355_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, GC2355SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, GC2355SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &GC2355SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            GC2355_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            GC2355_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=GC2355_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            GC2355_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            GC2355_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            GC2355_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            GC2355SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            GC2355GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            GC2355SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            GC2355SetTestPatternMode((BOOL)*pFeatureData16);
            break;
	    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=GC2355_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			GC2355MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			GC2355MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* GC2355FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncGC2355=
{
    GC2355Open,
    GC2355GetInfo,
    GC2355GetResolution,
    GC2355FeatureControl,
    GC2355Control,
    GC2355Close
};

UINT32 GC2355_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncGC2355;

    return ERROR_NONE;
}   /* SensorInit() */
