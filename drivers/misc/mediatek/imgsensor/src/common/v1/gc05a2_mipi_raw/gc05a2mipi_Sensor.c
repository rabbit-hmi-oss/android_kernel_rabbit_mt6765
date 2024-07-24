/******************************************************************************
 *
 * Filename:
 * ---------
 *     gc05a2mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 *
 * Version:  V20220830190306 by GC-S-TEAM
 *

 ******************************************************************************/



#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc05a2mipi_Sensor.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "gc05a2_camera_sensor"
#define LOG_1 LOG_INF("GC05A2, MIPI 2LANE\n")
/****************************   Modify end    *******************************************/
#define GC05A2_DEBUG                0
#if GC05A2_DEBUG
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC05A2_SENSOR_ID,
	.checksum_value = 0xe5d32119,
	.pre = {
		.pclk = 224000000,
		.linelength = 3616,
		.framelength = 2064,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 89600000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 224000000,
		.linelength = 3664,
		.framelength = 2032,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 179200000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 224000000,
		.linelength = 3616,
		.framelength = 2064,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 89600000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 224000000,
		.linelength = 3616,
		.framelength = 2064,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 89600000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 224000000,
		.linelength = 3616,
		.framelength = 2064,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 89600000,
		.max_framerate = 300,
	},

	.margin = 16,
	.min_shutter = 4,
	.max_frame_length = 0xfffe,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 5,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
//#if GC05A2_MIRROR_NORMAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
//#elif GC05A2_MIRROR_H
//	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
//#elif GC05A2_MIRROR_V
//	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
//#elif GC05A2_MIRROR_HV
//	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
//#else
//	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
//#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x6e, 0x7e, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x900,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x6e,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972}, /* Preview */
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944}, /* capture */
	{ 2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972}, /* video */
	{ 2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972}, /* hs video */
	{ 2592, 1944, 0, 0, 2592, 1944, 1296, 972, 8, 126, 1280, 720, 0, 0, 1280, 720}  /* slim video */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { 
		(char)((addr >> 8) & 0xff), 
		(char)(addr & 0xff) 
	};

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {
		(char)((addr >> 8) & 0xff),
		(char)(addr & 0xff),
		(char)((para >> 8) & 0xff),
		(char)(para & 0xff)
	};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8bit(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { 
		(char)((addr >> 8) & 0xff), 
		(char)(addr & 0xff), 
		(char)(para & 0xff) 
	};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}



static void table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend = 0, idx = 0;
	kal_uint16 addr = 0, data = 0;

	while (len > idx) {
		addr = para[idx];
		puSendCmd[tosend++] = (char)((addr >> 8) & 0xff);
		puSendCmd[tosend++] = (char)(addr & 0xff);
		data = para[idx + 1];
		puSendCmd[tosend++] = (char)(data & 0xff);
		idx += 2;
#if MULTI_WRITE
		if (tosend >= I2C_BUFFER_LEN || idx == len) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
					3, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 3, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;
#endif
	}
}

static kal_uint32 return_sensor_id(void)
{
	kal_uint32 sensor_id = 0;

	sensor_id = (read_cmos_sensor(0x03f0) << 8) | read_cmos_sensor(0x03f1);
	return sensor_id;
}

static void set_dummy(void)
{
	LOG_INF("frame length = %d\n", imgsensor.frame_length);
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xfffe);
}

static void set_max_framerate(kal_uint16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
		frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	/*kal_uint32 frame_length = 0;*/
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor(0x0340, imgsensor.frame_length & 0xfffe);
	} else
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xfffe);

	/* Update Shutter */
	write_cmos_sensor(0x0202, shutter);

	LOG_INF("shutter = %d, framelength = %d\n",
		shutter, imgsensor.frame_length);
}

static kal_uint16 gain2reg(kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 4;

	reg_gain = (reg_gain < SENSOR_BASE_GAIN) ? SENSOR_BASE_GAIN : reg_gain;
	reg_gain = (reg_gain > SENSOR_MAX_GAIN) ? SENSOR_MAX_GAIN : reg_gain;

	return reg_gain;
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint32 reg_gain = 0;

	reg_gain = gain2reg(gain);
	LOG_INF("gain = %d, reg_gain = %d\n", gain, reg_gain);
	write_cmos_sensor(0x0204, reg_gain & 0xffff);

	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}

kal_uint16 gc05a2_init_addr_data[] = {
/*system*/
	0x0315, 0xd4,
	0x0d06, 0x01,
	0x0a70, 0x80,
	0x031a, 0x00,
	0x0314, 0x00,
	0x0130, 0x08,
	0x0132, 0x01,
	0x0135, 0x01,
	0x0136, 0x38,
	0x0137, 0x03,
	0x0134, 0x5b,
	0x031c, 0xe0,
	0x0d82, 0x14,
	0x0dd1, 0x56,
/*gate_mode*/
	0x0af4, 0x01,
	0x0002, 0x10,
	0x00c3, 0x34,
/*pre_setting*/
	0x0084, 0x21,
	0x0d05, 0xcc,
	0x0218, 0x00,
	0x005e, 0x48,
	0x0d06, 0x01,
	0x0007, 0x16,
	0x0101, GC05A2_MIRROR,
/*analog*/
	0x0342, 0x07,
	0x0343, 0x28,
	0x0220, 0x07,
	0x0221, 0xd0,
	0x0202, 0x07,
	0x0203, 0x32,
	0x0340, 0x07,
	0x0341, 0xf0,
	0x0219, 0x00,
	0x0346, 0x00,
	0x0347, 0x04,
	0x0d14, 0x00,
	0x0d13, 0x05,
	0x0d16, 0x05,
	0x0d15, 0x1d,
	0x00c0, 0x0a,
	0x00c1, 0x30,
	0x034a, 0x07,
	0x034b, 0xa8,
	0x0e0a, 0x00,
	0x0e0b, 0x00,
	0x0e0e, 0x03,
	0x0e0f, 0x00,
	0x0e06, 0x0a,
	0x0e23, 0x15,
	0x0e24, 0x15,
	0x0e2a, 0x10,
	0x0e2b, 0x10,
	0x0e17, 0x49,
	0x0e1b, 0x1c,
	0x0e3a, 0x36,
	0x0d11, 0x84,
	0x0e52, 0x14,
	0x000b, 0x10,
	0x0008, 0x08,
	0x0223, 0x17,
	0x0d27, 0x39,
	0x0d22, 0x00,
	0x03f6, 0x0d,
	0x0d04, 0x07,
	0x03f3, 0x72,
	0x03f4, 0xb8,
	0x03f5, 0xbc,
	0x0d02, 0x73,
/*auto load start*/
	0x00c4, 0x00,
	0x00c5, 0x01,
	0x0af6, 0x00,
	0x0ba0, 0x17,
	0x0ba1, 0x00,
	0x0ba2, 0x00,
	0x0ba3, 0x00,
	0x0ba4, 0x03,
	0x0ba5, 0x00,
	0x0ba6, 0x00,
	0x0ba7, 0x00,
	0x0ba8, 0x40,
	0x0ba9, 0x00,
	0x0baa, 0x00,
	0x0bab, 0x00,
	0x0bac, 0x40,
	0x0bad, 0x00,
	0x0bae, 0x00,
	0x0baf, 0x00,
	0x0bb0, 0x02,
	0x0bb1, 0x00,
	0x0bb2, 0x00,
	0x0bb3, 0x00,
	0x0bb8, 0x02,
	0x0bb9, 0x00,
	0x0bba, 0x00,
	0x0bbb, 0x00,
	0x0a70, 0x80,
	0x0a71, 0x00,
	0x0a72, 0x00,
	0x0a66, 0x00,
	0x0a67, 0x80,
	0x0a4d, 0x4e,
	0x0a50, 0x00,
	0x0a4f, 0x0c,
	0x0a66, 0x00,
	0x00ca, 0x00,
	0x00cb, 0x00,
	0x00cc, 0x00,
	0x00cd, 0x00,
	0x0aa1, 0x00,
	0x0aa2, 0xe0,
	0x0aa3, 0x00,
	0x0aa4, 0x40,
	0x0a90, 0x03,
	0x0a91, 0x0e,
	0x0a94, 0x80,
/*standby*/
	0x0af6, 0x20,
	0x0b00, 0x91,
	0x0b01, 0x17,
	0x0b02, 0x01,
	0x0b03, 0x00,
	0x0b04, 0x01,
	0x0b05, 0x17,
	0x0b06, 0x01,
	0x0b07, 0x00,
	0x0ae9, 0x01,
	0x0aea, 0x02,
	0x0ae8, 0x53,
	0x0ae8, 0x43,
/*gain_partition*/
	0x0af6, 0x30,
	0x0b00, 0x08,
	0x0b01, 0x0f,
	0x0b02, 0x00,
	0x0b04, 0x1c,
	0x0b05, 0x24,
	0x0b06, 0x00,
	0x0b08, 0x30,
	0x0b09, 0x40,
	0x0b0a, 0x00,
	0x0b0c, 0x0e,
	0x0b0d, 0x2a,
	0x0b0e, 0x00,
	0x0b10, 0x0e,
	0x0b11, 0x2b,
	0x0b12, 0x00,
	0x0b14, 0x0e,
	0x0b15, 0x23,
	0x0b16, 0x00,
	0x0b18, 0x0e,
	0x0b19, 0x24,
	0x0b1a, 0x00,
	0x0b1c, 0x0c,
	0x0b1d, 0x0c,
	0x0b1e, 0x00,
	0x0b20, 0x03,
	0x0b21, 0x03,
	0x0b22, 0x00,
	0x0b24, 0x0e,
	0x0b25, 0x0e,
	0x0b26, 0x00,
	0x0b28, 0x03,
	0x0b29, 0x03,
	0x0b2a, 0x00,
	0x0b2c, 0x12,
	0x0b2d, 0x12,
	0x0b2e, 0x00,
	0x0b30, 0x08,
	0x0b31, 0x08,
	0x0b32, 0x00,
	0x0b34, 0x14,
	0x0b35, 0x14,
	0x0b36, 0x00,
	0x0b38, 0x10,
	0x0b39, 0x10,
	0x0b3a, 0x00,
	0x0b3c, 0x16,
	0x0b3d, 0x16,
	0x0b3e, 0x00,
	0x0b40, 0x10,
	0x0b41, 0x10,
	0x0b42, 0x00,
	0x0b44, 0x19,
	0x0b45, 0x19,
	0x0b46, 0x00,
	0x0b48, 0x16,
	0x0b49, 0x16,
	0x0b4a, 0x00,
	0x0b4c, 0x19,
	0x0b4d, 0x19,
	0x0b4e, 0x00,
	0x0b50, 0x16,
	0x0b51, 0x16,
	0x0b52, 0x00,
	0x0b80, 0x01,
	0x0b81, 0x00,
	0x0b82, 0x00,
	0x0b84, 0x00,
	0x0b85, 0x00,
	0x0b86, 0x00,
	0x0b88, 0x01,
	0x0b89, 0x6a,
	0x0b8a, 0x00,
	0x0b8c, 0x00,
	0x0b8d, 0x01,
	0x0b8e, 0x00,
	0x0b90, 0x01,
	0x0b91, 0xf6,
	0x0b92, 0x00,
	0x0b94, 0x00,
	0x0b95, 0x02,
	0x0b96, 0x00,
	0x0b98, 0x02,
	0x0b99, 0xc4,
	0x0b9a, 0x00,
	0x0b9c, 0x00,
	0x0b9d, 0x03,
	0x0b9e, 0x00,
	0x0ba0, 0x03,
	0x0ba1, 0xd8,
	0x0ba2, 0x00,
	0x0ba4, 0x00,
	0x0ba5, 0x04,
	0x0ba6, 0x00,
	0x0ba8, 0x05,
	0x0ba9, 0x4d,
	0x0baa, 0x00,
	0x0bac, 0x00,
	0x0bad, 0x05,
	0x0bae, 0x00,
	0x0bb0, 0x07,
	0x0bb1, 0x3e,
	0x0bb2, 0x00,
	0x0bb4, 0x00,
	0x0bb5, 0x06,
	0x0bb6, 0x00,
	0x0bb8, 0x0a,
	0x0bb9, 0x1a,
	0x0bba, 0x00,
	0x0bbc, 0x09,
	0x0bbd, 0x36,
	0x0bbe, 0x00,
	0x0bc0, 0x0e,
	0x0bc1, 0x66,
	0x0bc2, 0x00,
	0x0bc4, 0x10,
	0x0bc5, 0x06,
	0x0bc6, 0x00,
	0x02c1, 0xe0,
	0x0207, 0x04,
	0x02c2, 0x10,
	0x02c3, 0x74,
	0x02C5, 0x09,
	
	0x02c1, 0xe0,//20221121
	0x0207, 0x04,
	0x02c2, 0x10,
	0x02c5, 0x09,
	0x02c1, 0xe0,
	0x0207, 0x04,
	0x02c2, 0x10,
	0x02c5, 0x09,
	
/*auto load CH_GAIN*/
	0x0aa1, 0x15,
	0x0aa2, 0x50,
	0x0aa3, 0x00,
	0x0aa4, 0x09,
	0x0a90, 0x25,
	0x0a91, 0x0e,
	0x0a94, 0x80,
/*ISP*/
	0x0050, 0x00,
	0x0089, 0x83,
	0x005a, 0x40,
	0x00c3, 0x35,
	0x00c4, 0x80,
	0x0080, 0x10,
	0x0040, 0x12,
	0x0053, 0x0a,
	0x0054, 0x44,
	0x0055, 0x32,
	0x0058, 0x89,
	0x004a, 0x03,
	0x0048, 0xf0,
	0x0049, 0x0f,
	0x0041, 0x20,
	0x0043, 0x0a,
	0x009d, 0x08,
	0x0236, 0x40,
/*gain*/
	0x0204, 0x04,
	0x0205, 0x00,
	0x02b3, 0x00,
	0x02b4, 0x00,
	0x009e, 0x01,
	0x009f, 0x94,
/*OUT 2592x1944*/
	0x0350, 0x01,
	0x0353, 0x00,
	0x0354, 0x08,
	0x034c, 0x0a,
	0x034d, 0x20,
	0x021f, 0x14,
/*auto load REG*/
	0x0aa1, 0x10,
	0x0aa2, 0xf8,
	0x0aa3, 0x00,
	0x0aa4, 0x1f,
	0x0a90, 0x11,
	0x0a91, 0x0e,
	0x0a94, 0x80,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	
	0x03fe, 0x00,//20221121
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,	
	0x0a90, 0x00,
	
	0x0a70, 0x00,
	0x0a67, 0x00,
	0x0af4, 0x29,
/*DPHY */
	0x0d80, 0x07,
	0x0dd3, 0x18,
/*MIPI*/
	0x0107, 0x05,
	0x0117, 0x01,
	0x0d81, 0x00,
/*CISCTL_Reset*/
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
};

static kal_uint16 gc05a2_1296x972_addr_data[] = {
/*system*/
	0x0315, 0xd4,
	0x0d06, 0x01,
	0x0a70, 0x80,
	0x031a, 0x00,
	0x0314, 0x00,
	0x0130, 0x08,
	0x0132, 0x01,
	0x0135, 0x05,
	0x0136, 0x38,
	0x0137, 0x03,
	0x0134, 0x5b,
	0x031c, 0xe0,
	0x0d82, 0x14,
	0x0dd1, 0x56,
/*gate_mode*/
	0x0af4, 0x01,
	0x0002, 0x10,
	0x00c3, 0x34,
/*pre_setting*/
	0x0084, 0x21,
	0x0d05, 0xcc,
	0x0218, 0x80,
	0x005e, 0x49,
	0x0d06, 0x81,
	0x0007, 0x16,
/*analog */
	0x0342, 0x07,
	0x0343, 0x10,
	0x0220, 0x0f,
	0x0221, 0xe0,
	0x0202, 0x03,
	0x0203, 0x32,
	0x0340, 0x08,
	0x0341, 0x10,
	0x0346, 0x00,
	0x0347, 0x04,
	0x0d14, 0x00,
	0x0d13, 0x05,
	0x0d16, 0x05,
	0x0d15, 0x1d,
	0x00c0, 0x0a,
	0x00c1, 0x30,
	0x034a, 0x07,
	0x034b, 0xa8,
	0x000b, 0x0e,
	0x0008, 0x03,
	0x0223, 0x16,
/*auto load DD*/
	0x00ca, 0x00,
	0x00cb, 0x00,
	0x00cc, 0x00,
	0x00cd, 0x00,
/*ISP*/
	0x00c3, 0x35,
	0x0053, 0x0a,
	0x0054, 0x44,
	0x0055, 0x32,
/*OUT 1296x972*/
	0x0350, 0x01,
	0x0353, 0x00,
	0x0354, 0x04,
	0x034c, 0x05,
	0x034d, 0x10,
	0x021f, 0x14,
/*MIPI*/
	0x0d84, 0x06,
	0x0d85, 0x54,
	0x0d86, 0x03,
	0x0d87, 0x2b,
	0x0db3, 0x03,
	0x0db4, 0x04,
	0x0db5, 0x0d,
	0x0db6, 0x01,
	0x0db8, 0x04,
	0x0db9, 0x06,
	0x0d93, 0x03,
	0x0d94, 0x04,
	0x0d95, 0x05,
	0x0d99, 0x06,
	0x0084, 0x01,
/*CISCTL_Reset*/
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
/*OUT*/
	0x0110, 0x01,
};

static kal_uint16 gc05a2_2592x1944_addr_data[] = {
/*system*/
	0x0315, 0xd4,
	0x0d06, 0x01,
	0x0a70, 0x80,
	0x031a, 0x00,
	0x0314, 0x00,
	0x0130, 0x08,
	0x0132, 0x01,
	0x0135, 0x01,
	0x0136, 0x38,
	0x0137, 0x03,
	0x0134, 0x5b,
	0x031c, 0xe0,
	0x0d82, 0x14,
	0x0dd1, 0x56,
/*gate_mode*/
	0x0af4, 0x01,
	0x0002, 0x10,
	0x00c3, 0x34,
/*pre_setting*/
	0x0084, 0x21,
	0x0d05, 0xcc,
	0x0218, 0x00,
	0x005e, 0x48,
	0x0d06, 0x01,
	0x0007, 0x16,
/*analog*/
	0x0342, 0x07,
	0x0343, 0x28,
	0x0220, 0x07,
	0x0221, 0xd0,
	0x0202, 0x07,
	0x0203, 0x32,
	0x0340, 0x07,
	0x0341, 0xf0,
	0x0346, 0x00,
	0x0347, 0x04,
	0x0d14, 0x00,
	0x0d13, 0x05,
	0x0d16, 0x05,
	0x0d15, 0x1d,
	0x00c0, 0x0a,
	0x00c1, 0x30,
	0x034a, 0x07,
	0x034b, 0xa8,
	0x000b, 0x10,
	0x0008, 0x08,
	0x0223, 0x17,
/* auto  load DD*/
	0x00ca, 0x00,
	0x00cb, 0x00,
	0x00cc, 0x00,
	0x00cd, 0x00,
/*ISP*/
	0x00c3, 0x35,
	0x0053, 0x0a,
	0x0054, 0x44,
	0x0055, 0x32,
/*OUT 2592x1944*/
	0x0350, 0x01,
	0x0353, 0x00,
	0x0354, 0x08,
	0x034c, 0x0a,
	0x034d, 0x20,
	0x021f, 0x14,
/*MIPI*/
	0x0d84, 0x0c,
	0x0d85, 0xa8,
	0x0d86, 0x06,
	0x0d87, 0x55,
	0x0db3, 0x06,
	0x0db4, 0x08,
	0x0db5, 0x1e,
	0x0db6, 0x02,
	0x0db8, 0x12,
	0x0db9, 0x0a,
	0x0d93, 0x06,
	0x0d94, 0x09,
	0x0d95, 0x0d,
	0x0d99, 0x0b,
	0x0084, 0x01,
/* CISCTL_Reset*/
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
/*OUT*/
	0x0110, 0x01,
};

static kal_uint16 gc05a2_1280x720_addr_data[] = {
/*system*/
	0x0315, 0xd4,
	0x0d06, 0x01,
	0x0a70, 0x80,
	0x031a, 0x00,
	0x0314, 0x00,
	0x0130, 0x08,
	0x0132, 0x01,
	0x0135, 0x05,
	0x0136, 0x38,
	0x0137, 0x03,
	0x0134, 0x5b,
	0x031c, 0xe0,
	0x0d82, 0x14,
	0x0dd1, 0x56,
/*gate_mode*/
	0x0af4, 0x01,
	0x0002, 0x10,
	0x00c3, 0x34,
/*pre_setting*/
	0x0084, 0x21,
	0x0d05, 0xcc,
	0x0218, 0x80,
	0x005e, 0x49,
	0x0d06, 0x81,
	0x0007, 0x16,
/*analog */
	0x0342, 0x07,
	0x0343, 0x10,
	0x0220, 0x07,
	0x0221, 0xd0,
	0x0202, 0x03,
	0x0203, 0x32,
	0x0340, 0x04,
	0x0341, 0x08,
	0x0346, 0x01,
	0x0347, 0x00,
	0x0d14, 0x00,
	0x0d13, 0x05,
	0x0d16, 0x05,
	0x0d15, 0x1d,
	0x00c0, 0x0a,
	0x00c1, 0x30,
	0x034a, 0x05,
	0x034b, 0xb0,
	0x000b, 0x0e,
	0x0008, 0x03,
	0x0223, 0x16,
/*auto load DD*/
	0x00ca, 0x00,
	0x00cb, 0xfc,
	0x00cc, 0x00,
	0x00cd, 0x00,
/*ISP*/
	0x00c3, 0x35,
	0x0053, 0x0a,
	0x0054, 0x44,
	0x0055, 0x32,
/*OUT 1280x720*/
	0x0350, 0x01,
	0x0353, 0x00,
	0x0354, 0x0c,
	0x034c, 0x05,
	0x034d, 0x00,
	0x021f, 0x14,
/*MIPI*/
	0x0d84, 0x06,
	0x0d85, 0x40,
	0x0d86, 0x03,
	0x0d87, 0x21,
	0x0db3, 0x03,
	0x0db4, 0x04,
	0x0db5, 0x0d,
	0x0db6, 0x01,
	0x0db8, 0x04,
	0x0db9, 0x06,
	0x0d93, 0x03,
	0x0d94, 0x04,
	0x0d95, 0x05,
	0x0d99, 0x06,
	0x0084, 0x01,
/*CISCTL_Reset*/
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x30,
	0x0d17, 0x06,
	0x03fe, 0x00,
	0x0d17, 0x00,
	0x031c, 0x93,
/*OUT*/
	0x0110, 0x01,
};


static void gc05a2_stream_on(void)
{
	write_cmos_sensor_8bit(0x0100, 0x01);
}

static void gc05a2_stream_off(void)
{
	write_cmos_sensor_8bit(0x0100, 0x00);
}


static void sensor_init(void)
{
	table_write_cmos_sensor(gc05a2_init_addr_data,
		sizeof(gc05a2_init_addr_data)/sizeof(kal_uint16));
}

static void preview_setting(void)
{
	gc05a2_stream_off();
	table_write_cmos_sensor(gc05a2_1296x972_addr_data,
		sizeof(gc05a2_1296x972_addr_data)/sizeof(kal_uint16));
	gc05a2_stream_on();
}

static void capture_setting(void)
{
	gc05a2_stream_off();
	table_write_cmos_sensor(gc05a2_2592x1944_addr_data,
		sizeof(gc05a2_2592x1944_addr_data)/sizeof(kal_uint16));
	gc05a2_stream_on();
}

static void normal_video_setting(void)
{
	gc05a2_stream_off();
	table_write_cmos_sensor(gc05a2_1296x972_addr_data,
		sizeof(gc05a2_1296x972_addr_data)/sizeof(kal_uint16));
	gc05a2_stream_on();
}

static void hs_video_setting(void)
{
	gc05a2_stream_off();
	table_write_cmos_sensor(gc05a2_1296x972_addr_data,
		sizeof(gc05a2_1296x972_addr_data)/sizeof(kal_uint16));
	gc05a2_stream_on();
}

static void slim_video_setting(void)
{
	gc05a2_stream_off();
	table_write_cmos_sensor(gc05a2_1280x720_addr_data,
		sizeof(gc05a2_1280x720_addr_data)/sizeof(kal_uint16));
	gc05a2_stream_on();
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor_8bit(0x008c, 0x01);
	else
		write_cmos_sensor_8bit(0x008c, 0x00);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_debug("[gc05a2_camera_sensor]get_imgsensor_id:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			pr_debug("[gc05a2_camera_sensor]get_imgsensor_id:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}


	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	LOG_INF("imgsensor_open\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("[gc05a2_camera_sensor]open:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("[gc05a2_camera_sensor]open:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting();
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/*imgsensor.current_fps = 300*/
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/*This Function not used after ROME*/
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	/***********
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *spin_lock(&imgsensor_drv_lock);
	 *if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 296;
	 *else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 146;
	 *else
	 *	imgsensor.current_fps = framerate;
	 *spin_unlock(&imgsensor_drv_lock);
	 *set_max_framerate(imgsensor.current_fps, 1);
	 ********/
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				framerate, imgsensor_info.cap.max_framerate / 10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);	
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8bit(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps: %d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable: %d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId: %d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 GC05A2_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
