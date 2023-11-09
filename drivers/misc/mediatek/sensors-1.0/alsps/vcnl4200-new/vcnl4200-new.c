/* drivers/input/misc/vcnl4200.c - vcnl4200 optical sensors driver
 *
 * Copyright (C) 2017 Vishay Capella Microsystems Limited
 * Author: Frank Hsieh <Frank.Hsieh@vishay.com>
 *                                    
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include  "lightsensor.h"
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/types.h>
#include  "vcnl4200-new.h"
#include "capella_cm3602.h"
#include <asm/setup.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/jiffies.h>
#include <linux/math64.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif


#define SUPPORT_MTK_ALSPS_SENSOR_FRAME
#ifdef SUPPORT_MTK_ALSPS_SENSOR_FRAME
#include "cust_alsps.h"
#include "alsps.h"
#endif

#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               pr_debug(APS_TAG"%s\n", __func__)
#define APS_INFO(fmt, args...)   pr_info(APS_TAG fmt, ##args)
#define APS_PR_ERR(fmt, args...)    pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)

#define D(x...) printk(x)
#define SHD_REMOVE_SOME_DEBUG 

#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02

#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY

#define CALIBRATION_FILE_PATH "/efs/cal_data"
#define CHANGE_SENSITIVITY 20 // in percent

struct vcnl4200_info {
	struct class *vcnl4200_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;
	int ps_enable;
	int ps_irq_flag;

#ifdef CONFIG_PM_SLEEP
	int als_enabled_before_suspend;
#endif

	int irq;
	
	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_resolution; // represented using a fixed 10(-5) notation
	uint32_t cal_data; // represented using a fixed 10(-5) notation

#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock ps_wake_lock;
#endif  
	int psensor_opened;
	int lightsensor_opened;
	uint8_t slave_addr;

	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	uint32_t current_lux;
	uint16_t current_adc;
	uint16_t inte_cancel_set;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;

	uint16_t ls_cmd;
};
struct vcnl4200_info *lp_info;

bool cal_data_retrieved = false;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex ps_enable_mutex, ps_disable_mutex, ps_get_adc_mutex;
static struct mutex VCNL4200_control_mutex;

//static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

//static int lightsensor_enable(struct vcnl4200_info *lpi);
//static int lightsensor_disable(struct vcnl4200_info *lpi);

//static void lightsensor_initial_cmd(struct vcnl4200_info *lpi);
//static void psensor_initial_cmd(struct vcnl4200_info *lpi);

static int control_and_report(struct vcnl4200_info *lpi, uint8_t mode, uint16_t param);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct vcnl4200_info *lpi = lp_info;
		
	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = &cmd,
		},
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		},		 
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
	#ifndef SHD_REMOVE_SOME_DEBUG
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][VCNL4200 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);
	#else 
		//record_init_fail = record_init_fail; //shd for record_init_fail define but no used error 
	#endif

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
	#ifndef SHD_REMOVE_SOME_DEBUG
		printk(KERN_ERR "[PS_ERR][VCNL4200 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
	#endif
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct vcnl4200_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
	#ifndef SHD_REMOVE_SOME_DEBUG
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][VCNL4200 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);
	#endif

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
	#ifndef SHD_REMOVE_SOME_DEBUG
		printk(KERN_ERR "[ALS+PS_ERR][VCNL4200 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
	#endif
		return -EIO;
	}

	return 0;
}

static int _vcnl4200_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;
    //printk(" %s %d\n ",__func__,__LINE__);
	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
	#ifndef SHD_REMOVE_SOME_DEBUG
		pr_err(
			"[ALS+PS_ERR][VCNL4200 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
	#endif
		return ret;
	}
    //printk(" %s %d\n ",__func__,__LINE__);

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[VCNL4200] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _vcnl4200_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[VCNL4200] %s: _vcnl4200_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
	#ifndef SHD_REMOVE_SOME_DEBUG
		pr_err("[ALS+PS_ERR][VCNL4200 error]%s: I2C_TxData fail\n", __func__);
	#endif
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *data, bool resume)
{
	struct vcnl4200_info *lpi = lp_info;
	
	int ret = 0;

	if (data == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _vcnl4200_I2C_Read_Word(lpi->slave_addr, ALS_DATA, data);
	if (ret < 0) {
		pr_err(
			"[LS][VCNL4200 error]%s: _vcnl4200_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

  
	D("[LS][VCNL4200] %s: raw adc = 0x%X\n",
		__func__, *data);

	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct vcnl4200_info *lpi = lp_info;

	_vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_THDH, high_thd);
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_THDL, low_thd);

	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct vcnl4200_info *lpi = lp_info;

	if (data == NULL)
		return -EFAULT;	
    printk(" %s %d\n ",__func__,__LINE__);
	ret = _vcnl4200_I2C_Read_Word(lpi->slave_addr, PS_DATA, data);
	if (ret < 0) { // pr_err
		printk(
			"[PS][VCNL4200 error]%s: _vcnl4200_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	} else {
		printk( // pr_err
			"[PS][VCNL4200 OK]%s: _vcnl4200_I2C_Read_Word OK 0x%04x\n",
			__func__, *data);
	}

	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++)
		for (j = (i + 1); j < size; j++)
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	struct vcnl4200_info *lpi = lp_info;

	for (i = 0; i < 3; i++) {
		/*wait interrupt GPIO high*/
		while (gpio_get_value(lpi->intr_pin) == 0) {
			msleep(10);
			wait_count++;
			if (wait_count > 12) {
				pr_err("[PS_ERR][VCNL4200 error]%s: interrupt GPIO low,"
					" get_ps_adc_value\n", __func__);
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0) {
			pr_err("[PS_ERR][VCNL4200 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}

		if (wait_count < 60/10) {/*wait gpio less than 60ms*/
			msleep(60 - (10*wait_count));
		}
		wait_count = 0;
	}

	/*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);*/
	mid_val = mid_value(value, 3);
	D("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);
	
	return 0;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct vcnl4200_info *lpi = lp_info;
  
	uint16_t dummy=0;
  
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, dummy);  
	  
	enable_irq(lpi->irq);
}

static irqreturn_t vcnl4200_irq_handler(int irq, void *data)
{
	struct vcnl4200_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}
static int als_power(int enable)
{
	struct vcnl4200_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int lightsensor_get_cal_data(struct vcnl4200_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open calibration data file\n", __func__);
		set_fs(old_fs);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: cal_data = %d\n",
		__func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static void lightsensor_initial_cmd(struct vcnl4200_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= VCNL4200_ALS_INT_MASK;
	lpi->ls_cmd |= VCNL4200_ALS_SD;
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
}

static void psensor_initial_cmd(struct vcnl4200_info *lpi)
{
	/*must disable p-sensor interrupt befrore IST create*//*disable PS func*/		
	lpi->ps_conf1_val |= VCNL4200_PS_SD;
	lpi->ps_conf1_val &= VCNL4200_PS_INT_MASK;  
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_CONF1_2, lpi->ps_conf1_val);   
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_away_thd_set);
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_THDH,  lpi->ps_close_thd_set);
	D("[PS][VCNL4200] %s, finish\n", __func__);	
}

static int psensor_enable(struct vcnl4200_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_enable_mutex);
	D("[PS][VCNL4200] %s\n", __func__);

	if ( lpi->ps_enable ) {
		D("[PS][VCNL4200] %s: already enabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_PS, 1);
	
	mutex_unlock(&ps_enable_mutex);
	return ret;
}

static int psensor_disable(struct vcnl4200_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_disable_mutex);
	D("[PS][VCNL4200] %s\n", __func__);

	if ( lpi->ps_enable == 0 ) {
		D("[PS][VCNL4200] %s: already disabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_PS,0);
	
	mutex_unlock(&ps_disable_mutex);
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct vcnl4200_info *lpi = lp_info;

	D("[PS][VCNL4200] %s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct vcnl4200_info *lpi = lp_info;

	D("[PS][VCNL4200] %s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct vcnl4200_info *lpi = lp_info;

	D("[PS][VCNL4200] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case PSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val)
			return psensor_enable(lpi);
		else
			return psensor_disable(lpi);
		break;
	case PSENSOR_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[PS][VCNL4200 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "psensor",
	.fops = &psensor_fops
};


static int lightsensor_enable(struct vcnl4200_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	D("[LS][VCNL4200] %s\n", __func__);

	if (lpi->als_enable) {
		D("[LS][VCNL4200] %s: already enabled\n", __func__);
		ret = 0;
	} else
	
	if (!cal_data_retrieved)
	{
		/* get calibration data */		
		ret = lightsensor_get_cal_data(lpi);
		if (ret < 0 && ret != -ENOENT)
		{
			pr_err("%s: lightsensor_get_cal_data() failed\n",
				__func__);
		}
		else
		{
			cal_data_retrieved = true;
		}
	}
	
	ret = control_and_report(lpi, CONTROL_ALS, 1);
	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct vcnl4200_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][VCNL4200] %s\n", __func__);

	if ( lpi->als_enable == 0 ) {
		D("[LS][VCNL4200] %s: already disabled\n", __func__);
		ret = 0;
	} else
    ret = control_and_report(lpi, CONTROL_ALS, 0);
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct vcnl4200_info *lpi = lp_info;
	int rc = 0;

	D("[LS][VCNL4200] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][VCNL4200 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct vcnl4200_info *lpi = lp_info;

	D("[LS][VCNL4200] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct vcnl4200_info *lpi = lp_info;

	/*D("[VCNL4200] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][VCNL4200] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][VCNL4200] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LS][VCNL4200 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};


static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct vcnl4200_info *lpi = lp_info;
	int intr_val;

	intr_val = gpio_get_value(lpi->intr_pin);

	get_ps_adc_value(&value);

	//ret = sprintf(buf, "DEC ADC[%d], ENABLE = %d, intr_pin = %d\n", value, lpi->ps_enable, intr_val);
    // used  for  test
    ret = sprintf(buf, "0x%x\n", value);
	return ret;
}

static ssize_t ps_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct vcnl4200_info *lpi = lp_info;

	ret = sprintf(buf, "Proximity sensor Enable = %d\n",
			lpi->ps_enable);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct vcnl4200_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1)
		return -EINVAL;
	
	D("[PS][VCNL4200] %s: ps_en=%d\n", __func__, ps_en);
	
	if (ps_en)
		psensor_enable(lpi);
	else
		psensor_disable(lpi);

	return count;
}

static ssize_t ps_conf_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vcnl4200_info *lpi = lp_info;
	return sprintf(buf, "PS_CONF1 = 0x%04x, PS_CONF3 = 0x%04x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}
static ssize_t ps_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	struct vcnl4200_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	D("[PS]%s: store value PS conf1 reg = 0x%04x PS conf3 reg = 0x%04x\n", __func__, code1, code2);

	lpi->ps_conf1_val = code1;
	lpi->ps_conf3_val = code2;

	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val );  
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_CONF1_2, lpi->ps_conf1_val );

	return count;
}

static ssize_t ps_thd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct vcnl4200_info *lpi = lp_info;
	ret = sprintf(buf, "[PS][VCNL4200]PS Hi/Low THD ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
	return ret;	
}
static ssize_t ps_thd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	int ps_en;
	struct vcnl4200_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	//away:上门限； close:下门限；值越小，距离越远；
	lpi->ps_close_thd_set = code1;	
	lpi->ps_away_thd_set = code2;
	ps_en = lpi->ps_enable;

	if(ps_en)
		psensor_disable(lpi);
	
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set );
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set );

	if(ps_en)
		psensor_enable(lpi);	
	
	D("[PS][VCNL4200]%s: ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", __func__, lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
    
	return count;
}

static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct vcnl4200_info *lpi = lp_info;

	ret = sprintf(buf, "[PS][VCNL4200]PS_CANC = 0x%04x(%d)\n", lpi->inte_cancel_set,lpi->inte_cancel_set);

	return ret;
}
static ssize_t ps_canc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	int ps_en;
	struct vcnl4200_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS][VCNL4200]PS_CANC: store value = 0x%04x(%d)\n", code,code);
	
	lpi->inte_cancel_set = code;	
	ps_en = lpi->ps_enable;

	if(ps_en)
		psensor_disable(lpi);

	_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_CANC, lpi->inte_cancel_set );

	if(ps_en)
		psensor_enable(lpi);
	
	return count;
}

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct vcnl4200_info *lpi = lp_info;

    uint16_t value;
    ret = get_ps_adc_value(&value);
    
	ret = sprintf(buf, "ADC  =>  0x%04X  ret:%d\n",
		value,ret);

	D("[LS][VCNL4200] %s: ADC = 0x%04X, Lux = %d \n",
		__func__, lpi->current_adc, lpi->current_lux);
#if 0  
	ret = sprintf(buf, "ADC[0x%04X] => Lux %d\n",
		lpi->current_adc, lpi->current_lux);
#endif 
	return ret;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct vcnl4200_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct vcnl4200_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][VCNL4200] %s: lpi->als_enable = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err("[LS][VCNL4200 error]%s: set auto light sensor fail\n",
			__func__);

	return count;
}

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct vcnl4200_info *lpi = lp_info;
	return sprintf(buf, "ALS_CONF = %x\n", lpi->ls_cmd);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct vcnl4200_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	printk(KERN_INFO "[LS]set ALS_CONF = %x\n", lpi->ls_cmd);
	
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
	return count;
}

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct vcnl4200_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct vcnl4200_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = 100000;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}

static struct device_attribute dev_attr_ps_adc =
__ATTR(ps_adc, 0444, ps_adc_show, NULL);

static struct device_attribute dev_attr_ps_enable =
__ATTR(ps_enable, 0664, ps_enable_show, ps_enable_store);	

static struct device_attribute dev_attr_ps_conf =	
__ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store);

static struct device_attribute dev_attr_ps_thd =	
__ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);

static struct device_attribute dev_attr_ps_canc = 	
__ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_ps_adc.attr,
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_conf.attr,
	&dev_attr_ps_thd.attr,
	&dev_attr_ps_canc.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static struct device_attribute dev_attr_ls_adc =
__ATTR(ls_adc, 0444, ls_adc_show, NULL);

static struct device_attribute dev_attr_ls_enable =
__ATTR(ls_enable, 0664, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_ls_conf =
__ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_ls_cali =
__ATTR(ls_cali, 0664, ls_cal_data_show, ls_cal_data_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_ls_adc.attr,
	&dev_attr_ls_enable.attr,
	&dev_attr_ls_conf.attr,
	&dev_attr_ls_cali.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static ssize_t vcnl4200_reg_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct vcnl4200_info *lpi = lp_info;
	int i ;
	uint16_t regs[ID_REG + 1];
	
	for(i = 0; i <= ID_REG; i++)
		_vcnl4200_I2C_Read_Word(lpi->slave_addr, i, (regs + i));
	
	return sprintf(buf, "0x00=0x%04x\n0x01=0x%04x\n0x02=0x%04x\n0x03=0x%04x\n0x04=0x%04x\n0x05=0x%04x\n0x06=0x%04x\n0x07=0x%04x\n0x08=0x%04x\n0x09=0x%04x\n0x0A=0x%04x\n0x0B=0x%04x\n0x0C=0x%04x\n0x0D=0x%04x\n0x0E=0x%04x\n",
		regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], 
		regs[8], regs[9], regs[0xa], regs[0xb], regs[0xc], regs[0xd], regs[0xe]);
}
static ssize_t vcnl4200_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct vcnl4200_info *lpi = lp_info;
	uint8_t cmd;
	uint16_t data = 0;
	
	sscanf(buf, "%x %x", &cmd, &data);
	printk("%s,cmd=%x,data=0x%4x\n", cmd, data);
	
	_vcnl4200_I2C_Write_Word(lpi->slave_addr, cmd, data);
	return count;
}

static struct device_attribute dev_attr_vcnl4200_reg =
__ATTR(vcnl4200_reg, 0664, vcnl4200_reg_show, vcnl4200_reg_store);

static struct attribute *vcnl4200_sysfs_attrs[] = {
	&dev_attr_vcnl4200_reg.attr,
	NULL
};

static struct attribute_group vcnl4200_attribute_group = {
	.attrs = vcnl4200_sysfs_attrs,
};

static int lightsensor_setup(struct vcnl4200_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][VCNL4200 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][VCNL4200 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][VCNL4200 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct vcnl4200_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[PS][VCNL4200 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL4200 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL4200 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

/*
	irq init & initial cmd
*/
static int vcnl4200_setup(struct vcnl4200_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_vcnl4200_intr");
	if (ret < 0) {
		pr_err("[PS][VCNL4200 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL4200 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}
 	
	/*Default disable P sensor and L sensor*/
	lightsensor_initial_cmd(lpi);
	psensor_initial_cmd(lpi);

    ret = request_any_context_irq(lpi->irq,
			vcnl4200_irq_handler,
			IRQF_TRIGGER_LOW,
			"vcnl4200",
			lpi);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL4200 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}
    
    enable_irq(lpi->irq);
    
	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void vcnl4200_early_suspend(struct early_suspend *h)
{
	struct vcnl4200_info *lpi = lp_info;

	D("[LS][VCNL4200] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
	
	if (lpi->ps_enable)
		psensor_disable(lpi);
}

static void vcnl4200_late_resume(struct early_suspend *h)
{
	struct vcnl4200_info *lpi = lp_info;

	D("[LS][VCNL4200] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
		
	if (!lpi->ps_enable)
		psensor_enable(lpi);
}
#endif

#ifdef CONFIG_OF
static int vcnl4200_parse_dt(struct device *dev,
				struct vcnl4200_info *lpi)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

    static struct pinctrl *vcnl4200_pinctrl;
    struct pinctrl_state *irq_pullup;
    
	printk("[LS][VCNL4200] %s\n", __func__);
    
	rc = of_get_named_gpio_flags(np, "capella,intrpin-gpios",
			0, NULL);
	if (rc < 0) 
	{
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} 
	else
	{
		lpi->intr_pin = rc;
		D("[LS][VCNL4200]%s GET INTR PIN \n", __func__);   

		vcnl4200_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(vcnl4200_pinctrl)) {
			pr_err("Failed to get vcnl4200_pinctrl pinctrl.\n");
		} 
		else
		{
			irq_pullup = pinctrl_lookup_state(vcnl4200_pinctrl, "default");
			if (IS_ERR(irq_pullup)) {
			      pr_err("Failed to init \n");
			}
			else {
				pinctrl_select_state(vcnl4200_pinctrl, irq_pullup);
			}
		}  
	}
	rc = of_property_read_u32(np, "capella,slave_address", &temp_val);
	if (rc)
	{
	    //dev_err(dev, "Unable to read slave_address\n");
		printk("Vcnl: Unable to read slave_address\n");
		return rc;
	} 
	else
	{
		lpi->slave_addr = (uint8_t)temp_val;
	}
  
	printk("[PS][VCNL4200]%s PARSE OK \n", __func__);

	return 0;
}
#endif


#ifdef SUPPORT_MTK_ALSPS_SENSOR_FRAME
static int als_open_report_data(int open)
{
	return 0;
}

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_INFO("%s:%d\n", __func__, en);

	//res = vcnl4200_enable_als(lp_info->client, en);
	if(en)
		res = lightsensor_enable(lp_info);
	else
		res = lightsensor_disable(lp_info);
	if (res) {
		APS_PR_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int als_get_data(int *value, int *status)
{
	int err = 0;
	struct vcnl4200_info *obj = NULL;

	if (!lp_info) {
		APS_PR_ERR("lp_info is null!!\n");
		return -1;
	}
	obj = lp_info;
	
	err = get_ls_adc_value((uint16_t *)value, 0);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	printk("%s,err=%d,value=%d,status=%d\n", __func__, err, *value, *status);

	return err;
}

static int ps_open_report_data(int open)
{
	return 0;
}

static int ps_enable_nodata(int en)
{
	int res = 0;

	APS_INFO("%s:%d\n", __func__, en);

	//res = vcnl4200_enable_ps(lp_info->client, en);
	if(en)
		res = psensor_enable(lp_info);
	else
		res = psensor_disable(lp_info);
	if (res) {
		APS_PR_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}
static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;
	struct vcnl4200_info *obj = NULL;
	
	if (!lp_info) {
		APS_PR_ERR("lp_info is null!!\n");
		return -1;
	}
	obj = lp_info;

	err = get_ps_adc_value((uint16_t *)value);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	printk("%s,err=%d,value=%d,status=%d\n", __func__, err, *value, *status);
	return err;
}
#endif


static int vcnl4200_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct vcnl4200_info *lpi;
#ifndef CONFIG_OF  
	struct vcnl4200_platform_data *pdata;
#endif

#ifdef SUPPORT_MTK_ALSPS_SENSOR_FRAME
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
#endif

	printk("[ALS+PS][VCNL4200] %s\n", __func__);

	lpi = kzalloc(sizeof(struct vcnl4200_info), GFP_KERNEL);
	if (!lpi){
        printk("err:[ALS+PS][VCNL4200] vcnl4200_probe\n");
		return -ENOMEM;
    }

	/*D("[VCNL4200] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);

#ifndef CONFIG_OF
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[ALS+PS][VCNL4200 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	
	lpi->intr_pin = pdata->intr;
	
	lpi->power = pdata->power;
	
	lpi->slave_addr = pdata->slave_addr;
	
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	lpi->ls_cmd  = pdata->ls_cmd;
#else
	if( vcnl4200_parse_dt(&client->dev, lpi) < 0 )
	{
		ret = -EBUSY;
		goto err_platform_data_null;  
	}

	//away:上门限； close:下门限；值越小，距离越远；
	//lpi->ps_away_thd_set = 0x90;
	//lpi->ps_close_thd_set = 0x80;
	//lpi->ps_conf1_val = VCNL4200_PS_IT_9T | VCNL4200_PS_PERS_1 | VCNL4200_PS_DR_1_160;
	//lpi->ps_conf3_val = VCNL4200_PS_MP_8 | VCNL4200_PS_SMART_PERS_ENABLE | VCNL4200_LED_I_50;
	lpi->ps_away_thd_set = 0x30;
	lpi->ps_close_thd_set = 0x60;
	lpi->ps_conf1_val = VCNL4200_PS_IT_9T | VCNL4200_PS_PERS_1 | VCNL4200_PS_DR_1_1280;
	lpi->ps_conf3_val = VCNL4200_PS_MP_8 | VCNL4200_PS_SMART_PERS_ENABLE | VCNL4200_LED_I_100;
  
	lpi->ls_cmd = VCNL4200_ALS_PERS_2 | VCNL4200_ALS_IT_100MS;
	lpi->power = NULL;
#endif
	
		
	printk("[PS][VCNL4200] %s: ls_cmd 0x%x\n",
		__func__, lpi->ls_cmd);
	
	if (lpi->ls_cmd == 0) {
		lpi->ls_cmd  = VCNL4200_ALS_PERS_2 | VCNL4200_ALS_IT_100MS;
	}

	lp_info = lpi;

	mutex_init(&VCNL4200_control_mutex);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][VCNL4200 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}
	
	mutex_init(&ps_enable_mutex);
	mutex_init(&ps_disable_mutex);
	mutex_init(&ps_get_adc_mutex);

	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][VCNL4200 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}
	
	ret = lightsensor_get_cal_data(lpi);
	
	//set the default ALS cal data, no lens condition
	lpi->cal_data = 100000;
			
	//set the chip resoultion, please refer to datasheet	
	lpi->als_resolution = 4000;

	lpi->lp_wq = create_singlethread_workqueue("vcnl4200_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][VCNL4200 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

#ifdef CONFIG_HAS_WAKELOCK  
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
#endif

	ret = vcnl4200_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][VCNL4200 error]%s: vcnl4200_setup error!\n", __func__);
		goto err_vcnl4200_setup;
	}
	
	lpi->vcnl4200_class = class_create(THIS_MODULE, "capella_sensors");
	if (IS_ERR(lpi->vcnl4200_class)) {
		ret = PTR_ERR(lpi->vcnl4200_class);
		lpi->vcnl4200_class = NULL;
		goto err_create_class;
	}
    
	lpi->ls_dev = device_create(lpi->vcnl4200_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	lpi->ps_dev = device_create(lpi->vcnl4200_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ps_device;
	}
	
	/* register the attributes */
	//ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj, &light_attribute_group);
	ret = sysfs_create_group(&client->dev.kobj, &light_attribute_group);
	if (ret)
		goto err_sysfs_create_group_light;

	/* register the attributes */
	//ret = sysfs_create_group(&lpi->ps_input_dev->dev.kobj, &proximity_attribute_group);
	ret = sysfs_create_group(&client->dev.kobj, &proximity_attribute_group);
	if (ret)
		goto err_sysfs_create_group_proximity;

	sysfs_create_group(&client->dev.kobj, &vcnl4200_attribute_group);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = vcnl4200_early_suspend;
	lpi->early_suspend.resume = vcnl4200_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif

#ifdef SUPPORT_MTK_ALSPS_SENSOR_FRAME
	ret = 0;
	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;

	
	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;
	//ret = als_register_control_path(&als_ctl);
	if (ret) {
		APS_PR_ERR("als_register_control_path fail = %d\n", ret);
//		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	//ret = als_register_data_path(&als_data);
	if (ret) {
		APS_PR_ERR("als_register_data_path fail = %d\n", ret);
//		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = true;
	ps_ctl.is_support_batch = false;
	ret = ps_register_control_path(&ps_ctl);
	if (ret) {
		APS_PR_ERR("ps_register_control_path fail = %d\n", ret);
//		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	ret = ps_register_data_path(&ps_data);
	if (ret) {
		APS_PR_ERR("ps_register_data_path fail = %d\n", ret);
//		goto exit_sensor_obj_attach_fail;
	}
#endif

	//shd modify: disable by default
    //lightsensor_enable(lpi);
	//psensor_enable(lpi);
	lightsensor_disable(lpi);
	psensor_disable(lpi);
	
	printk("[PS][VCNL4200] %s: Probe success!\n", __func__);
	return ret;

err_sysfs_create_group_proximity:
	device_destroy(lpi->vcnl4200_class, lpi->ps_dev->devt);
	
err_create_ps_device:
	//sysfs_remove_group(&lpi->ls_input_dev->dev.kobj, &proximity_attribute_group);
	sysfs_remove_group(&client->dev.kobj, &proximity_attribute_group);
	
err_sysfs_create_group_light:
	device_destroy(lpi->vcnl4200_class, lpi->ls_dev->devt);
	
err_create_ls_device:
	//sysfs_remove_group(&lpi->ls_input_dev->dev.kobj, &light_attribute_group);
	sysfs_remove_group(&client->dev.kobj, &light_attribute_group);
	
err_create_class:
	class_destroy(lpi->vcnl4200_class);
	
err_vcnl4200_setup:
	gpio_free(lpi->intr_pin); //vcnl4200_setup
	destroy_workqueue(lpi->lp_wq);
#ifdef CONFIG_HAS_WAKELOCK  
	wake_lock_destroy(&(lpi->ps_wake_lock));
#endif  
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
	
err_create_singlethread_workqueue:
err_psensor_setup:
	mutex_destroy(&VCNL4200_control_mutex);
	mutex_destroy(&ps_enable_mutex);
	mutex_destroy(&ps_disable_mutex);
	mutex_destroy(&ps_get_adc_mutex);
	misc_deregister(&lightsensor_misc); //lightsensor_setup
	
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
	
err_platform_data_null:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct vcnl4200_info *lpi, uint8_t mode, uint16_t param ) {
	int ret=0;
	uint16_t adc_value = 0;
	uint16_t ps_data = 0;
	uint32_t lux = 0;
	uint16_t low_thd;
	uint32_t high_thd;	
	int val;
	
	mutex_lock(&VCNL4200_control_mutex);

	if(mode == CONTROL_INT_ISR_REPORT) {
		_vcnl4200_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &param);
		if((param&INT_FLAG_ALS_IF_L)||(param&INT_FLAG_ALS_IF_H)){
			lpi->ls_cmd &= VCNL4200_ALS_INT_MASK;  
			ret = _vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);     
		}
	}
  
	if( mode == CONTROL_ALS ){
		if(param){
			lpi->ls_cmd &= VCNL4200_ALS_SD_MASK;            
		} else {
			lpi->ls_cmd |= VCNL4200_ALS_SD;
			lpi->ls_cmd &= VCNL4200_ALS_INT_MASK;      
		}
		_vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
		lpi->als_enable=param;
	} else if( mode == CONTROL_PS ){
		if(param){ 
			lpi->ps_conf1_val &= VCNL4200_PS_SD_MASK;
			lpi->ps_conf1_val |= VCNL4200_PS_INT_IN_AND_OUT;      
		} else {
			lpi->ps_conf1_val |= VCNL4200_PS_SD;
			lpi->ps_conf1_val &= VCNL4200_PS_INT_MASK;
		}
		_vcnl4200_I2C_Write_Word(lpi->slave_addr, PS_CONF1_2, lpi->ps_conf1_val);    
		lpi->ps_enable=param;  
	}
	if((mode == CONTROL_ALS)||(mode == CONTROL_PS)){  
		if( param==1 ){
			msleep(200);  
		}
	}
     	
	if(lpi->als_enable){
		if( mode == CONTROL_ALS ){
			low_thd = 65535;
			high_thd = 0;
			ret = set_lsensor_range(low_thd, (uint16_t)high_thd);
			lpi->ls_cmd |= VCNL4200_ALS_INT_EN;
			ret = _vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);          	
		}
		else if( mode == CONTROL_INT_ISR_REPORT && 
			((param&INT_FLAG_ALS_IF_L)||(param&INT_FLAG_ALS_IF_H))){
	        
				lpi->ls_cmd &= VCNL4200_ALS_INT_MASK;
				ret = _vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
	      
				get_ls_adc_value(&adc_value, 0);
				lux = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);

				// set interrupt high/low threshold		
				low_thd = (uint16_t)((uint32_t)adc_value * (100 - CHANGE_SENSITIVITY) / 100);
				high_thd = (uint32_t)adc_value * (100 + CHANGE_SENSITIVITY) / 100;
				if (high_thd > 65535){
					high_thd = 65535;
				}

			ret = set_lsensor_range(low_thd, (uint16_t)high_thd);

			lpi->ls_cmd |= VCNL4200_ALS_INT_EN;
			ret = _vcnl4200_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);

			D("[LS][VCNL4200] %s: ADC=0x%03X, Lux =%d, l_thd = 0x%x, h_thd = 0x%x \n",
				__func__, adc_value, lux, low_thd, high_thd);
				lpi->current_lux = lux;
			lpi->current_adc = adc_value;   
		#ifndef SUPPORT_MTK_ALSPS_SENSOR_FRAME
			input_report_abs(lpi->ls_input_dev, ABS_MISC, lux);
			input_sync(lpi->ls_input_dev);
		#else

		#endif
		}
	}

	if(lpi->ps_enable){
		int ps_status = 0;
		if( mode == CONTROL_PS )
			ps_status = PS_CLOSE_AND_AWAY;   
		else if(mode == CONTROL_INT_ISR_REPORT ){  
			if ( param & INT_FLAG_PS_IF_CLOSE )
				ps_status |= PS_CLOSE;      
			if ( param & INT_FLAG_PS_IF_AWAY )
				ps_status |= PS_AWAY;
		}
		  
		if (ps_status!=0){
			switch(ps_status){
				case PS_CLOSE_AND_AWAY:
					get_stable_ps_adc_value(&ps_data);
					val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
					break;
				case PS_AWAY:
					val = 1;
					D("[PS][VCNL4200] proximity detected object away\n");
					break;
				case PS_CLOSE:
					val = 0;
					D("[PS][VCNL4200] proximity detected object close\n");
					break;
			};
		#ifndef SUPPORT_MTK_ALSPS_SENSOR_FRAME
			input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);      
			input_sync(lpi->ps_input_dev);
		#else
			ps_report_interrupt_data(!val);
		#endif
		}
	}

	mutex_unlock(&VCNL4200_control_mutex);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int vcnl4200_suspend(struct device *dev)
{
	struct vcnl4200_info *lpi;
	lpi = dev_get_drvdata(dev);

	/*
	  * Save sensor state and disable them,
	  * this is to ensure internal state flags are set correctly.
	  * device will power off after both sensors are disabled.
	  * P sensor will not be disabled because it  is a wakeup sensor.
	*/

	lpi->als_enabled_before_suspend = lpi->als_enable;  

#ifdef CONFIG_HAS_WAKELOCK
	if (lpi->ps_enable) {
		 wake_lock(&lpi->ps_wake_lock);
	}
#endif
 
#ifdef UNSUPPORT_AUTO_BACKLIGHT
	if (lpi->als_enable == 1)
		lightsensor_disable(lpi);
#endif

	return 0;
}

static int vcnl4200_resume(struct device *dev)
{
	struct vcnl4200_info *lpi;
	lpi = dev_get_drvdata(dev);

#ifdef CONFIG_HAS_WAKELOCK
	if (lpi->ps_enable ) {
		wake_unlock(&(lpi->ps_wake_lock));
	}
#endif

	/* Don't disable light at phone calling
	  * while the automatic backlight is on.
	  */
#ifdef UNSUPPORT_AUTO_BACKLIGHT
	if (lpi->als_enabled_before_suspend)
		lightsensor_enable(lpi);
#endif	

	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(vcnl4200_pm, vcnl4200_suspend, vcnl4200_resume, NULL);


static const struct i2c_device_id vcnl4200_i2c_id[] = {
	{VCNL4200_I2C_NAME, 0},
	{}
};

static struct of_device_id vcnl4200_match_table[] = {
	{ .compatible = "vcnl4200"},
	{ },
};


static struct i2c_driver vcnl4200_driver = {
	.id_table = vcnl4200_i2c_id,
	.probe = vcnl4200_probe,
	.driver = {
		.name = VCNL4200_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &vcnl4200_pm,    
    	.of_match_table = of_match_ptr(vcnl4200_match_table),     
	},
};

#ifndef SUPPORT_MTK_ALSPS_SENSOR_FRAME
static int __init vcnl4200_init(void)
{ 
    printk("vcnl4200_init \n");
	return i2c_add_driver(&vcnl4200_driver);
}

static void __exit vcnl4200_exit(void)
{
	i2c_del_driver(&vcnl4200_driver);
}
#else
static int vcnl4200_local_init(void)
{
	printk("%s\n",__func__);
	if (i2c_add_driver(&vcnl4200_driver)) {
		APS_PR_ERR("add driver error\n");
		return -1;
	}
//	if (-1 == vcnl4200_init_flag)
//		return -1;
	return 0;
}

static int vcnl4200_remove(void)
{
	i2c_del_driver(&vcnl4200_driver);
	return 0;
}

static struct alsps_init_info vcnl4200_init_info = {
	.name = "vcnl4200",
	.init = vcnl4200_local_init,
	.uninit = vcnl4200_remove,
};
static int __init vcnl4200_init(void)
{
	printk("%s\n",__func__);
	alsps_driver_add(&vcnl4200_init_info);
	return 0;
}

static void __exit vcnl4200_exit(void)
{
	APS_FUN();
}
#endif

//module_i2c_driver(vcnl4200_driver);
module_init(vcnl4200_init);
module_exit(vcnl4200_exit);

MODULE_AUTHOR("Frank Hsieh <Frank.Hsieh@vishay.com>");
MODULE_DESCRIPTION("VCNL4200 Optical Sensor Driver");
MODULE_LICENSE("GPL v2");

