// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for ChipOne och1970 i2c touchscreen controller
 *
 * Copyright (c) 2015-2018 Red Hat Inc.
 *
 * Red Hat authors:
 * Hans de Goede <hdegoede@redhat.com>
 */

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/gpio.h>

#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
//#include <mach/irqs.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
//#include <linux/rtpm_prio.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/file.h>
#include <linux/proc_fs.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

#include <linux/syscore_ops.h>


#include <linux/wait.h>
#include <linux/time.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
//#include <linux/wakelock.h>---kernel  4.14 not support,support wakeup event 
#include <linux/module.h>
#include <linux/sysfs.h>

#include "och1970.h"

/* Normal operation mode defines */
#define och1970_REG_ADDR_WIDTH		8

#define och1970_REG_POWER		0x0004
#define och1970_REG_TOUCHDATA		0x1000
#define och1970_REG_CONFIGDATA		0x8000

/* och1970_REG_POWER commands */
#define och1970_POWER_ACTIVE		0x00
#define och1970_POWER_MONITOR		0x01
#define och1970_POWER_HIBERNATE		0x02
/*
 * The Android driver uses these to turn on/off the charger filter, but the
 * filter is way too aggressive making e.g. onscreen keyboards unusable.
 */
#define och1970_POWER_ENA_CHARGER_MODE	0x55
#define och1970_POWER_DIS_CHARGER_MODE	0x66

#define och1970_MAX_TOUCHES		10

/* Programming mode defines */
#define och1970_PROG_I2C_ADDR		0x0D

#define MAX_FW_UPLOAD_TRIES		3
#define MAX_X_VALUE 7200
#define REPORT_DEGREE_DIFF_THRESHOLD 30
#define MAX_DELAY_THRESHOLD_MS 3000

struct och1970_data {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *wake_gpio;
	struct touchscreen_properties prop;
	char firmware_name[32];
	int degree;

	int reset_gpio;
	u32 reset_gpio_flags;
	u64 pre_time;

	struct delayed_work dump_work;
};

struct och1970_data *och1970;
static bool resume_flag;

/* sin(270°) ~ sin(89°) * 10000 */
static int sin_mul10000[180] = {
	-10000, -9998, -9994, -9986, -9976, -9962, -9945, -9925, -9903, -9877,
	-9848, -9816, -9781, -9744, -9703, -9659, -9613, -9563, -9511, -9455,
	-9397, -9336, -9272, -9205, -9135, -9063, -8988, -8910, -8829, -8746,
	-8660, -8572, -8480, -8387, -8290, -8192, -8090, -7986, -7880, -7771,
	-7660, -7547, -7431, -7314, -7193, -7071, -6947, -6820, -6691, -6561,
	-6428, -6293, -6157, -6018, -5878, -5736, -5592, -5446, -5299, -5150,
	-5000, -4848, -4695, -4540, -4384, -4226, -4067, -3907, -3746, -3584,
	-3420, -3256, -3090, -2924, -2756, -2588, -2419, -2250, -2079, -1908,
	-1736, -1564, -1392, -1219, -1045, -872, -698, -523, -349, -175,
	0, 175, 349, 523, 698, 872, 1045, 1219, 1392, 1564,
	1736, 1908, 2079, 2250, 2419, 2588, 2756, 2924, 3090, 3256,
	3420, 3584, 3746, 3907, 4067, 4226, 4384, 4540, 4695, 4848,
	5000, 5150, 5299, 5446, 5592, 5736, 5878, 6018, 6157, 6293,
	6428, 6561, 6691, 6820, 6947, 7071, 7193, 7314, 7431, 7547,
	7660, 7771, 7880, 7986, 8090, 8192, 8290, 8387, 8480, 8572,
	8660, 8746, 8829, 8910, 8988, 9063, 9135, 9205, 9272, 9336,
	9397, 9455, 9511, 9563, 9613, 9659, 9703, 9744, 9781, 9816,
	9848, 9877, 9903, 9925, 9945, 9962, 9976, 9986, 9994, 9998,
};

static int och1970_read_xfer(struct i2c_client *client, u16 i2c_addr,
			     int reg_addr, int reg_addr_width,
			     void *data, int len, bool silent)
{
	u8 buf[3];
	int i, ret;
	struct i2c_msg msg[2] = {
		{
			.addr = i2c_addr,
			.buf = buf,
			.len = reg_addr_width / 8,
		},
		{
			.addr = i2c_addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = len,
		}
	};

	for (i = 0; i < (reg_addr_width / 8); i++)
		buf[i] = (reg_addr >> (reg_addr_width - (i + 1) * 8)) & 0xff;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != ARRAY_SIZE(msg)) {
		if (ret >= 0)
			ret = -EIO;
		if (!silent)
			dev_err(&client->dev,
				"Error reading addr %#x reg %#x: %d\n",
				i2c_addr, reg_addr, ret);
		return ret;
	}

	return 0;
}

static int och1970_write_xfer(struct i2c_client *client, u16 i2c_addr,
			      int reg_addr, int reg_addr_width,
			      const void *data, int len, bool silent)
{
	u8 buf[3 + 32]; /* 3 bytes for 24 bit reg-addr + 32 bytes max len */
	int i, ret;
	struct i2c_msg msg = {
		.addr = i2c_addr,
		.buf = buf,
		.len = reg_addr_width / 8 + len,
	};

	if (WARN_ON(len > 32))
		return -EINVAL;

	for (i = 0; i < (reg_addr_width / 8); i++)
		buf[i] = (reg_addr >> (reg_addr_width - (i + 1) * 8)) & 0xff;

	memcpy(buf + reg_addr_width / 8, data, len);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		if (ret >= 0)
			ret = -EIO;
		if (!silent)
			dev_err(&client->dev,
				"Error writing addr %#x reg %#x: %d\n",
				i2c_addr, reg_addr, ret);
		return ret;
	}

	return 0;
}

static int och_i2c_read(int sla_addr, int reg, int len, void *readbuf)
{	

	return	och1970_read_xfer(och1970->client, sla_addr, reg,
				 8, readbuf, len, false);
}

static int och_i2c_write(int sla_addr, int reg, int len, u8 value)
{
	return och1970_write_xfer(och1970->client, och1970->client->addr, reg,
				  8, &value, 1, false);
}

void OCH1970_SetOperationMode(enum OCH1970_OPERATION_MODE value)
{
	uint8_t dataTemp,nowValue;
	//enter standby mode first
	och_i2c_read(OCH1970_SLA, OCH1970_REG_CNTL2, 1, &nowValue);
	dataTemp = (nowValue & 0xF0) | OCH1970_VALUE_STANDBY_MODE;
	och_i2c_write(OCH1970_SLA, OCH1970_REG_CNTL2, 1, dataTemp);

	switch(value)
	{
		case OCH1970_OP_STANDBY_MODE:
			return;
		case OCH1970_OP_SINGLE_MODE:
			dataTemp |= OCH1970_VALUE_SINGLE_MODE;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_1_0p5Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_1_0p5Hz;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_2_1Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_2_1Hz;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_3_2Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_3_2Hz;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_4_20Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_4_20Hz;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_5_40Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_5_40Hz;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_6_100Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_6_100Hz;
			break;
		case OCH1970_OP_CONTINUOUS_MODE_7_500Hz:
			dataTemp |= OCH1970_VALUE_CONTINUOUS_MODE_7_500Hz;
			break;
		default:
			return;
	}
	och_i2c_write(OCH1970_SLA, OCH1970_REG_CNTL2, 1, dataTemp);
}

//value: OCH1970_LOW_NOISE_MODE or OCH1970_LOW_POWER_MODE
void OCH1970_SetSDRMode(enum OCH1970_SDR_MODE value)
{
	uint8_t dataTemp,nowValue;
	och_i2c_read(OCH1970_SLA, OCH1970_REG_CNTL2, 1, &nowValue);

	if(value == OCH1970_LOW_NOISE_MODE)
		dataTemp = nowValue & ~bit4;
	else if(value == OCH1970_LOW_POWER_MODE)
		dataTemp = (nowValue & ~bit4) | bit4;
	else;

	och_i2c_write(OCH1970_SLA, OCH1970_REG_CNTL2, 1, dataTemp);
}

//value: OCH1970_HIGH_SENSE_MODE or OCH1970_HIGH_RANGE_MODE
void OCH1970_SetSMRMode(enum OCH1970_SMR_MODE value)
{
	uint8_t dataTemp,nowValue;
	och_i2c_read(OCH1970_SLA, OCH1970_REG_CNTL2, 1, &nowValue);

	if(value == OCH1970_HIGH_SENSE_MODE)
		dataTemp = nowValue & ~bit5;
	else if(value == OCH1970_HIGH_RANGE_MODE)
		dataTemp = (nowValue & ~bit5) | bit5;
	else;

	och_i2c_write(OCH1970_SLA, OCH1970_REG_CNTL2, 1, dataTemp);
}	

void OCH1970_GetXYZ_HLdata(uint8_t *value)
{
	uint8_t dataTemp[8];
	och_i2c_read(OCH1970_SLA, OCH1970_REG_DATAX_Y_Z, 8, dataTemp);
	memcpy(value, dataTemp + 2, 6 * sizeof(uint8_t));
	//6bytes: Xhigh,Xlow,Yhigh,Ylow,Zhigh,Zlow;
}

void OCH1970_Init(enum OCH1970_OPERATION_MODE value1,enum OCH1970_SDR_MODE value2,enum OCH1970_SMR_MODE value3)
{
	//ATTENTION: RST pin should be pulled down first and should at least lasts for 20us
	OCH1970_SetOperationMode(value1);
	OCH1970_SetSDRMode(value2);
	OCH1970_SetSMRMode(value3);
	//add BOP&BRP set function here if needed;
}

static void och1970_dump_work(struct work_struct *data)
{
	uint8_t data_array[6];
	int x_value, y_value, z_value;
	int key=-1;
	bool should_report = false;
	int sinx, index, degree;
	static int pre_degree = 0;
	u64 time_now;
	
	OCH1970_GetXYZ_HLdata(data_array);

	x_value = (int16_t)(((uint16_t)(data_array[0])<<8) | ((uint16_t)(data_array[1])));
	y_value = (int16_t)(((uint16_t)(data_array[2])<<8) | ((uint16_t)(data_array[3])));
	z_value = (int16_t)(((uint16_t)(data_array[4])<<8) | ((uint16_t)(data_array[5])));

	dev_dbg(och1970->dev,"och1970 read data x:%d y:%d z:%d",x_value, y_value, z_value);

	if (x_value > MAX_X_VALUE) {
		x_value = MAX_X_VALUE;
	} else if (x_value < -MAX_X_VALUE) {
		x_value = -MAX_X_VALUE;
	}
	sinx = x_value * 10000 / MAX_X_VALUE;
	for (index=0;index<179&&sinx>sin_mul10000[index];index++);
	if (index < 90) {
		if (z_value < 0) {/* x:180~270 z:270~360 */
			degree = 270 - index;
		} else {/* x:270~360 z:0~90 */
			degree = index + 270;
		}
	} else {
		if (z_value < 0) {/* x:90~180 z:180~270 */
			degree = 270 - index;
		} else {/* x:0~90 z:90~180 */
			degree = index - 90;
		}
	}

	pre_degree = degree;
	time_now = ktime_to_ms(ktime_get());
	if (unlikely(och1970->degree == 360)) { /* init */
		och1970->degree = degree;
		och1970->pre_time = time_now;
		dev_dbg(och1970->dev, "[%s] init degree: %d\n", __func__, och1970->degree);
	} else {
		if (och1970->degree < REPORT_DEGREE_DIFF_THRESHOLD) {
			if (degree >= och1970->degree + REPORT_DEGREE_DIFF_THRESHOLD
					&& degree < och1970->degree + 180) {
				should_report = true;
				key = KEY_DOWN;
			} else if (degree <= och1970->degree + 360 - REPORT_DEGREE_DIFF_THRESHOLD
					&& degree >= och1970->degree + 180) {
				should_report = true;
				key = KEY_UP;
			}
		} else if (och1970->degree + REPORT_DEGREE_DIFF_THRESHOLD >= 360) {
			if (degree >= och1970->degree + REPORT_DEGREE_DIFF_THRESHOLD - 360
					&& degree <= och1970->degree - 180) {
				should_report = true;
				key = KEY_DOWN;
			} else if (degree <= och1970->degree - REPORT_DEGREE_DIFF_THRESHOLD
					&& degree > och1970->degree - 180) {
				should_report = true;
				key = KEY_UP;
			}
		} else {
			if (abs(och1970->degree - degree) >= REPORT_DEGREE_DIFF_THRESHOLD) {
				should_report = true;
				if (degree > och1970->degree) {
					if (degree - och1970->degree <= 180) {
						key = KEY_DOWN;
					} else {
						key = KEY_UP;
					}
				} else {
					if (och1970->degree - degree <= 180) {
						key = KEY_UP;
					} else {
						key = KEY_DOWN;
					}
				}
			}
		}
	}

	dev_dbg(och1970->dev, "[%s] time_now:%lums, pre_time:%lums, key:%d, degree:%d, pre_degree:%d!\n",
			__func__, time_now, och1970->pre_time, key, degree, och1970->degree);
	if (time_now - och1970->pre_time >= MAX_DELAY_THRESHOLD_MS) {
		dev_dbg(och1970->dev, "[%s] timeout, reset!\n", __func__);
		och1970->degree = degree;
		och1970->pre_time = time_now;
	} else if (should_report) {
		input_event(och1970->input, EV_KEY, key, 1);
		input_sync(och1970->input);
		input_event(och1970->input, EV_KEY, key, 0);
		input_sync(och1970->input);
		dev_info(och1970->dev, "[%s] report %d done!\n", __func__, key);
		och1970->degree = degree;
		och1970->pre_time = time_now;
	}
	if (resume_flag)
		schedule_delayed_work(&och1970->dump_work, HZ/20);
}

void och1970_resume(void)
{
	printk(" och1970_resume ");
	if (och1970)
		schedule_delayed_work(&och1970->dump_work, 0);
	resume_flag = true;
}
EXPORT_SYMBOL(och1970_resume);

void och1970_suspend(void)
{
	printk(" och1970_suspend ");
	resume_flag = false;
}
EXPORT_SYMBOL(och1970_suspend);
//static SIMPLE_DEV_PM_OPS(och1970_ops, och1970_suspend, och1970_resume);
/*
static struct dev_pm_ops och1970_ops = {
	.resume_noirq = och1970_resume,
	.suspend_noirq = och1970_suspend,
};
*/
static int och1970_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct input_dev *input;
	int value;
	int ret;

	printk("och1970_probe !!!");
	
	och1970 = kzalloc(sizeof(*och1970), GFP_KERNEL);
	if (!och1970)
		return -ENOMEM;

	och1970->client = client;
	och1970->dev = &client->dev;
	i2c_set_clientdata(client, och1970);

	
	och1970->reset_gpio = of_get_named_gpio_flags(och1970->dev->of_node, "reset-gpio", 0, &och1970->reset_gpio_flags);
	if (och1970->reset_gpio < 0){
		printk("DTS Unable to get reset_gpio");
		kfree(och1970);
		och1970 = NULL;
		return -1;
	}
	
	gpio_request(och1970->reset_gpio, "och1970_reset_gpio");
	
	mdelay(10);
	gpio_direction_output(och1970->reset_gpio, 0);
	mdelay(10);
	gpio_direction_output(och1970->reset_gpio, 1);
	
	ret = och_i2c_read(OCH1970_SLA, 0x00, 1, &value);
	
	if (ret) {
		dev_err(och1970->dev, "och1970 Error reading id: %d\n", ret);
		kfree(och1970);
		och1970 = NULL;
		return ret;
	}
	printk("och1970 read id = 0x%x",value);

	input = devm_input_allocate_device(och1970->dev);
	if (!input) {
		dev_err(och1970->dev, "[%s] failed to allocate input device\n", __func__);
		kfree(och1970);
		och1970 = NULL;
		return -ENOMEM;
	}
	och1970->input = input;
	input->name = "och1970_holl_key";
	__set_bit(EV_KEY, input->evbit);
	__set_bit(KEY_UP, input->keybit);
	__set_bit(KEY_DOWN, input->keybit);
	ret = input_register_device(input);
	if (ret) {
		dev_err(och1970->dev, "[%s] fail to register input device, ret:%d\n", __func__, ret);
		kfree(och1970);
		och1970 = NULL;
		return ret;
	}

	och1970->degree = 360;
	och1970->pre_time = ktime_to_ms(ktime_get());
	OCH1970_Init(OCH1970_OP_CONTINUOUS_MODE_6_100Hz,OCH1970_LOW_NOISE_MODE,OCH1970_HIGH_RANGE_MODE);

	mdelay(10);

	resume_flag = true;
	INIT_DELAYED_WORK(&och1970->dump_work, och1970_dump_work);
	schedule_delayed_work(&och1970->dump_work, 5*HZ);

	//register_syscore_ops(&och1970_ops);

	printk("och1970 probe end");
	return 0;

}
static const struct i2c_device_id och1970_id[] = {{"och1970", 0}, {} };
static const struct of_device_id och1970_match[] =
{
	{.compatible = "och1970",},
    {}
};
static struct i2c_driver och1970_driver = {
	.driver = {
		.name	= "och1970",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(och1970_match),
		//.pm = &och1970_ops,
	},
	.probe = och1970_probe,
	.id_table = och1970_id,
};

static int __init och1970_init(void)
{
	int ret;
	
	printk("och1970_init !!!!");
	
	ret = i2c_add_driver(&och1970_driver);
	if (ret != 0) {
		printk("can't find och1970 compatible");
		return -1;
	}
	return 0;
}

module_init(och1970_init);

MODULE_DESCRIPTION("och1970 I2C Touchscreen Driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
