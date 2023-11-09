/*
 *  apds9500.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2015 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2015 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

//#define LINUX_KERNEL_2_6_X	1

#define APDS9500_DRV_NAME	"apds9500"
#define DRIVER_VERSION		"1.0.0"

//#define APDS9500_INT		IRQ_EINT(20)

#define APDS9500_PS_DETECTION_THRESHOLD		50
#define APDS9500_PS_HSYTERESIS_THRESHOLD	40
#define APDS9500_PS_PULSE_NUMBER			8

#define APDS9500_PS_CAL_LOOP			1
#define APDS9500_PS_CAL_CROSSTALK_LOW	0
#define	APDS9500_PS_CAL_CROSSTALK_HIGH	30
#define APDS9500_PS_CROSSTALK_DELTA		20
	
/* Change History 
 *
 * 1.0.0	Fundamental Functions of APDS-9500
 *
 */

#define APDS_IOCTL_PS_ENABLE				1
#define APDS_IOCTL_PS_GET_ENABLE			2
#define APDS_IOCTL_PS_POLL_DELAY			3
#define APDS_IOCTL_PS_GET_PDATA				4	// ps_data

#define APDS_DISABLE_PS						0
#define APDS_ENABLE_PS_WITH_INT				1
#define APDS_ENABLE_PS_NO_INT				2
#define	APDS_ENABLE_PS_CALIBRATION			3

/*
 * Defines
 */

enum {
	DIR_NONE,
	DIR_LEFT,
	DIR_RIGHT,
	DIR_UP,
	DIR_DOWN,
	DIR_FORWARD,
	DIR_BACKWARD,
	DIR_ALL
};

/*
 * Structs
 */
 
struct apds9500_data {
	struct i2c_client *client;

	//struct mutex update_lock;
	struct delayed_work	dwork;		/* for interrupt */
	struct delayed_work	ps_dwork;	/* for PS polling */
	struct input_dev *input_dev_ps;

    int irq;
    uint16_t gpio_pin;     // add yang
	int ps_suspended;	
	unsigned int main_ctrl_suspended_value;	/* suspend_resume usage */	

	/* control flag from HAL */
	unsigned int enable_ps_sensor;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold; 	/* always lower than ps_threshold */
	unsigned int ps_detection;				/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;					/* to store PS data */
	unsigned int ps_overflow;				/* to store PS overflow flag */
	unsigned int first_int_enable;			/* to force first interrupt */
	unsigned int ps_poll_delay;				/* needed for proximity sensor polling : ms */
	unsigned int ps_offset;					/* needed if crosstalk under cover glass is big */	
	unsigned int gesture_motion;
	unsigned int gesture_prev_motion;
	unsigned int ae_exposure_ub;
	unsigned int led_current;
};

/*
 * Global data
 */
static struct i2c_client *apds9500_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *apds_workqueue;


/*
 * Management functions
 */

static int apds9500_gesture_processing(struct i2c_client *client)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err;
	int gesture_detection = 0;
	
	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00); // switch to register bank 0
	if (err < 0)
		return err;

	gesture_detection = i2c_smbus_read_word_data(client, 0x43);
	if (gesture_detection < 0)
		return gesture_detection;
		
	gesture_detection &= 0x1FF;
	
	if (gesture_detection == 0x01)
	{
		//data->gesture_motion = DIR_DOWN;
		 //data->gesture_motion = KEY_DOWN;
		 data->gesture_motion = KEY_GESTURE_DIR_UP;
		 
	}
	else if (gesture_detection == 0x02) {
		//data->gesture_motion = DIR_UP;
		  //data->gesture_motion = KEY_UP;
		  data->gesture_motion = KEY_GESTURE_DIR_DOWN;
		  
	}      
	else if (gesture_detection == 0x04) {
		//data->gesture_motion = DIR_RIGHT;
		   //data->gesture_motion = KEY_RIGHT;
		  data->gesture_motion = KEY_GESTURE_DIR_LEFT; 
	}
	else if (gesture_detection == 0x08) {
		//data->gesture_motion = DIR_LEFT;
		// data->gesture_motion = KEY_LEFT;
		data->gesture_motion = KEY_GESTURE_DIR_RIGHT; 
	}
	else if (gesture_detection == 0x10) {
		//data->gesture_motion = DIR_FORWARD;
		  data->gesture_motion = KEY_GESTURE_DIR_FORWARD;
	}
	else if (gesture_detection == 0x20) {
		//data->gesture_motion = DIR_BACKWARD;
		  data->gesture_motion = KEY_GESTURE_DIR_BACK;
	}
	else {
		data->gesture_motion = DIR_NONE;
	}

	 printk("gesture = %d (%d)\n", data->gesture_motion, data->gesture_prev_motion);
	
	//if (data->gesture_motion != DIR_NONE && data->gesture_motion != data->gesture_prev_motion) {
	if (data->gesture_motion != DIR_NONE)
	{
	
		printk("reporting gesture = %d (%d)\n", data->gesture_motion, data->gesture_prev_motion);
		//input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->gesture_motion); /* GESTURE event */	
		 input_report_key(data->input_dev_ps,data->gesture_motion,1); 
		 input_sync(data->input_dev_ps);

		 input_report_key(data->input_dev_ps,data->gesture_motion,0);
		 input_sync(data->input_dev_ps);
		 
		//data->gesture_prev_motion = data->gesture_motion;
	}
	
	return data->gesture_motion;
}

static void apds9500_reschedule_work(struct apds9500_data *data,
					  unsigned long delay)
{
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
    cancel_delayed_work(&data->dwork);
	queue_delayed_work(apds_workqueue, &data->dwork, delay);
}

/* PS polling routine */
static void apds9500_ps_polling_work_handler(struct work_struct *work)
{
	struct apds9500_data *data = container_of(work, struct apds9500_data, ps_dwork.work);
	struct i2c_client *client=data->client;
	
	//printk("==> PS Pollinig\n");	
	apds9500_gesture_processing(client);
	
	queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));	// restart timer
}

/* Interrupt Service Routine */
static void apds9500_work_handler(struct work_struct *work)
{
	struct apds9500_data *data = container_of(work, struct apds9500_data, dwork.work);
	struct i2c_client *client=data->client;

	printk("==> APDS9500 - ISR\n");	

	if (data->enable_ps_sensor) {
		/* PS interrupt */
		apds9500_gesture_processing(client);
	}
}

/* assume this is ISR */
static irqreturn_t apds9500_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds9500_data *data = i2c_get_clientdata(client);

	printk("==> apds9500_interrupt\n");
	apds9500_reschedule_work(data, 0);	

	return IRQ_HANDLED;
}

/*
 * IOCTL support
 */
 #if 0
static int apds9500_load_gesture_setting(struct i2c_client *client, bool interrupt_enable)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err=0;
    
	printk("==> apds9500_load_gesture_setting  init \n");
    
	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00);
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00); // switch to register bank 0
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x41, 0x3F); // R_Int_1_En
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x42, 0x00); // R_Int_2_En
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x46, 0x2D); // R_AELedOff_UB, If OFF Frame average brightness > this x2, AE decrease
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x47, 0x0F); // R_AELedOff_LB, If OFF Frame average brightness < this x2, AE increase
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x48, ((data->ae_exposure_ub)&0xFF));	// 0x3C
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x49, (((data->ae_exposure_ub)>>8)&0xFF));
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x4A, ((data->ae_exposure_ub/2)&0xFF));	// 0x1E
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x4B, (((data->ae_exposure_ub/2)>>8)&0xFF));
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x4C, 0x20); // R_AE_Gain_UB => 1 + (x/16)
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x4D, 0x00); // R_AE_Gain_LB => 1 + (x/16)
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x51, 0x10); // 0x10=auto exposure 0x03 manual exposure
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x5C, 0x02); // default is 0x02, but raw image used 0x06
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x5E, 0x10); // clk manual R_SRAM_CLK manual
	if (err < 0) return err;

	if (interrupt_enable) {
		err = i2c_smbus_write_byte_data(client, 0x80, 0x42); // GPIO0 as output, GPIO1 as input => what are these GPIO0 and GPIO1
		if (err < 0) return err;
	
		err = i2c_smbus_write_byte_data(client, 0x81, 0x44); // GPIO2 as input, GPIO3 as input => what are these GPIO2 and GPIO3
		if (err < 0) return err;
		
		err = i2c_smbus_write_byte_data(client, 0x82, 0x0C);	// Im_INT: INT as output
		if (err < 0) return err;
	}
	else {
		err = i2c_smbus_write_byte_data(client, 0x80, 0x43); // GPIO0 as output, GPIO1 as input => what are these GPIO0 and GPIO1
		if (err < 0) return err;
	
		err = i2c_smbus_write_byte_data(client, 0x81, 0x44); // GPIO2 as input, GPIO3 as input => what are these GPIO2 and GPIO3
		if (err < 0) return err;
		
		err = i2c_smbus_write_byte_data(client, 0x82, 0x0B);	// Im_INT: INT as output => disable interrupt in my platform
		if (err < 0) return err;
	}

	// R_LightThd
	//err = i2c_smbus_write_byte_data(client, 0x83, 0x20);
	//if (err < 0) return err;

	// R_ObjectSizeStartTh
	//err = i2c_smbus_write_byte_data(client, 0x84, 0x20);
	//if (err < 0) return err;

	//err = i2c_smbus_write_byte_data(client, 0x85, 0x00);
	//if (err < 0) return err;

	// R_ObjectSizeEndTh
	err = i2c_smbus_write_byte_data(client, 0x86, 0x10);
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x87, 0x00);
	if (err < 0) return err;

	// R_Cursor_ObjectSizeTh
	//err = i2c_smbus_write_byte_data(client, 0x8B, 0x01);
	//if (err < 0) return err;

	// R_TimeDelayNum
	err = i2c_smbus_write_byte_data(client, 0x8D, 0x00);
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x90, 0x06); // R_NoMotionCountThd No motion counter threshold to quit has motion state
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x91, 0x06); // R_NoObjectCountThd
	if (err < 0) return err;

	// R_XDirectionThd[4:0]
	err = i2c_smbus_write_byte_data(client, 0x93, 0x0D); // R_XDirection default value is 0x0D (0x08)
	if (err < 0) return err;

	//R_YDirectionThd[4:0]
	err = i2c_smbus_write_byte_data(client, 0x94, 0x0A); // R_YDirection default value is 0x0A (0x06)
	if (err < 0) return err;

	//R_ZDirectionThd[4:0]
	err = i2c_smbus_write_byte_data(client, 0x95, 0x08); // Gesture detection z direction threshold
	if (err < 0) return err;

	//R_ZDirectionXYThd[4:0]
	err = i2c_smbus_write_byte_data(client, 0x96, 0x10); // Gesture detection x and y threshold to detect forward or backward
	if (err < 0) return err;

	//R_ZDirectionAngleThd[3:0]
	err = i2c_smbus_write_byte_data(client, 0x97, 0x05); // Gesture detection angle threshold to detect forward or backward
	if (err < 0) return err;

	//R_RotateXYThd[4:0]
	err = i2c_smbus_write_byte_data(client, 0x9A, 0x14); // Gesture detection x and y threshold to detect rotation
	if (err < 0) return err;

	//Filter setting
	err = i2c_smbus_write_byte_data(client, 0x9C, 0x3F); // IIR filter weight between frame position distance, IIR filter frame position distance threshold
	if (err < 0) return err;

	//R_UseBGModel is enable
	err = i2c_smbus_write_byte_data(client, 0x9F, 0xF9); // Background model enable, disable rotation gesture
	if (err < 0) return err;

	//R_BGUpdateMaxIntensity[7:0]
	err = i2c_smbus_write_byte_data(client, 0xA0, 0x48); // R_BGUpdateMaxIntensity - kk 24-Apr-2014
	if (err < 0) return err;

	//R_FilterAverage_Mode
	err = i2c_smbus_write_byte_data(client, 0xA5, 0x19); // Image filter enable [0], Image filter mode: 0: weak average, 1: strong average, 2: 3 out of 9 median average [3:2]
	if (err < 0) return err;

	//R_YtoZSum[5:0]
	err = i2c_smbus_write_byte_data(client, 0xCC, 0x19);
	if (err < 0) return err;

	//R_YtoZFactor[5:0]
	err = i2c_smbus_write_byte_data(client, 0xCD, 0x0B);
	if (err < 0) return err;

	//bit[2:0] = R_PositionFilterLength[2:0]
	err = i2c_smbus_write_byte_data(client, 0xCE, 0x13);
	if (err < 0) return err;

	//bit[3:0] = R_WaveCountThd[3:0]
	err = i2c_smbus_write_byte_data(client, 0xCF, 0x64);	// Wave gesture counter threshold = 4, Wave gesture angle threshold = 6
	if (err < 0) return err;
	
	//R_AbortXYRatio[4:0] & R_AbortLength[6:0]
	err = i2c_smbus_write_byte_data(client, 0xD0, 0x21); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x01);	// switch to register bank 0
	if (err < 0) return err;

	//		2x skip mode, 2x2 average, WOI
	// 0x00		30			30			30
	// 0x01		30			60			30
	// 0x02		0			0			15
	// 0x03		0			0			15
	// 0x04		0x30		0x98		0x00

	err = i2c_smbus_write_byte_data(client, 0x00, 30);	// 30
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x01, 30); // 30
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x02, 15); // 15
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x03, 15); // 15
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x04, 0x01);
	if (err < 0) return err;

	//R_LensShadingComp_EnH
	err = i2c_smbus_write_byte_data(client, 0x25, 0x01);
	if (err < 0) return err;
	
	//R_OffsetX[6:0]
	err = i2c_smbus_write_byte_data(client, 0x26, 0x05);
	if (err < 0) return err;
	
	//R_OffsetY[6:0]
	err = i2c_smbus_write_byte_data(client, 0x27, 0x37);
	if (err < 0) return err;
	
	//R_LSC[6:0]
	err = i2c_smbus_write_byte_data(client, 0x28, 0x7F);
	if (err < 0) return err;
	
	//R_LSFT[3:0]
	err = i2c_smbus_write_byte_data(client, 0x29, 0x0A);
	if (err < 0) return err;
	
	//R_LED_SoftStart_time[7:0]
	err = i2c_smbus_write_byte_data(client, 0x30, 0x03);
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x32, data->led_current);
	if (err < 0) return err;

	//Cmd_DebugPattern[7:0]
	err = i2c_smbus_write_byte_data(client, 0x3E, 0xFF);
	if (err < 0) return err;

	//analog voltage setting
	err = i2c_smbus_write_byte_data(client, 0x5E, 0x3D);
	if (err < 0) return err;

	//246fps//13//240fps//B7//105fps   R_IDLE_TIME[7:0]
	err = i2c_smbus_write_byte_data(client, 0x65, 0xBF); // Idle time for normal operation
	if (err < 0) return err;
	
	//R_IDLE_TIME[15:8]
	err = i2c_smbus_write_byte_data(client, 0x66, 0x00);
	if (err < 0) return err;
	
	//R_IDLE_TIME_SLEEP_1[7:0]
	err = i2c_smbus_write_byte_data(client, 0x67, 0x97); // idle time for weak sleep
	if (err < 0) return err;
	
	//R_IDLE_TIME_SLEEP_1[15:8]
	err = i2c_smbus_write_byte_data(client, 0x68, 0x01);
	if (err < 0) return err;
	
	//R_IDLE_TIME_SLEEP_2[7:0]
	err = i2c_smbus_write_byte_data(client, 0x69, 0xCD); 
	if (err < 0) return err;
	
	//R_IDLE_TIME_SLEEP_2[15:8]
	err = i2c_smbus_write_byte_data(client, 0x6A, 0x01);
	if (err < 0) return err;
	
	//R_Obj_TIME_1[7:0]
	err = i2c_smbus_write_byte_data(client, 0x6B, 0xB0);
	if (err < 0) return err;
		
	//R_Obj_TIME_1[15:8]
	err = i2c_smbus_write_byte_data(client, 0x6C, 0x04);
	if (err < 0) return err;
	
	//R_Obj_TIME_2[7:0]
	err = i2c_smbus_write_byte_data(client, 0x6D, 0x2C); // Deep sleep enter time, unit: two report frame time.
	if (err < 0) return err;
	
	//R_Obj_TIME_2[15:8]
	err = i2c_smbus_write_byte_data(client, 0x6E, 0x01);
	if (err < 0) return err;
	
	//R_TG_EnH
	err = i2c_smbus_write_byte_data(client, 0x72, 0x01);
	if (err < 0) return err;
	
	//Auto Sleep & Wakeup mode
	err = i2c_smbus_write_byte_data(client, 0x73, 0x35);
	if (err < 0) return err;
	
	//R_Control_Mode[2:0]
	err = i2c_smbus_write_byte_data(client, 0x74, 0x00);	// 0=gesture, 3=cursor, 5=ps
	if (err < 0) return err;
	
	//R_SRAM_Read_EnH
	err = i2c_smbus_write_byte_data(client, 0x77, 0x01); // 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00); // switch to register bank 0
	if (err < 0) return err;

    printk("==> apds9500_load_gesture_setting  success \n");
	return 0;
}
#endif 

static int gesture_init(struct i2c_client *client);

static int apds9500_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err;
	
	printk("enable ps senosr ( %d)\n", val);

	
	if ((val != APDS_DISABLE_PS) && 
		(val != APDS_ENABLE_PS_WITH_INT) && 
		(val != APDS_ENABLE_PS_NO_INT)  && 
		(val != APDS_ENABLE_PS_CALIBRATION)) {
		printk("%s:invalid value=%d\n", __func__, val);
		return -1;
	}
	
	if(val == APDS_ENABLE_PS_WITH_INT) {
		//turn on p sensor
		if (data->enable_ps_sensor==APDS_DISABLE_PS) {
		
			//err = apds9500_load_gesture_setting(client, true);
			err = gesture_init(client);
			data->enable_ps_sensor = val;
			data->first_int_enable = 1;
			
			cancel_delayed_work(&data->ps_dwork);
			flush_delayed_work(&data->ps_dwork);			
		}
	}
	else if(val == APDS_ENABLE_PS_NO_INT) {

	//	err = apds9500_load_gesture_setting(client, false);
        err = gesture_init(client);

		data->enable_ps_sensor = val;
		
		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->ps_dwork);
		flush_delayed_work(&data->ps_dwork);
		queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));	
	}
	else if (val == APDS_ENABLE_PS_CALIBRATION) {	// calibrate crosstalk value

		if (data->enable_ps_sensor==APDS_DISABLE_PS) {
		}
		else {
			return -7;
		}
	}
	else {

		data->enable_ps_sensor = APDS_DISABLE_PS;

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->ps_dwork);

		flush_delayed_work(&data->ps_dwork);		
	}
	
	return 0;
}

static int apds9500_ps_open(struct inode *inode, struct file *file)
{
//	printk("apds9500_ps_open\n");
	return 0; 
}

static int apds9500_ps_release(struct inode *inode, struct file *file)
{
//	printk("apds9500_ps_release\n");
	return 0;
}

static long apds9500_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct apds9500_data *data;
    struct i2c_client *client;
    int enable;
    int ret = -1;

    if (arg == 0) return -1;

    if(apds9500_i2c_client == NULL) {
		printk("apds9500_ps_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

    client = apds9500_i2c_client;   
    data = i2c_get_clientdata(apds9500_i2c_client);

    switch (cmd) {
		case APDS_IOCTL_PS_ENABLE:              

			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9500_ps_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}

			ret = apds9500_enable_ps_sensor(client, enable);        
			if(ret < 0) {
				return ret;
			}
		break;

     	case APDS_IOCTL_PS_GET_ENABLE:
			if (copy_to_user((void __user *)arg, &data->enable_ps_sensor, sizeof(data->enable_ps_sensor))) {
				printk("apds9500_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

        case APDS_IOCTL_PS_GET_PDATA:

			data->ps_data =	i2c_smbus_read_word_data(client, 0x43);

			if (copy_to_user((void __user *)arg, &data->ps_data, sizeof(data->ps_data))) {
				printk("apds9500_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

		default:
		break;
    }

	
    return 0;
}

/*
 * SysFS support
 */

static ssize_t apds9500_show_ps_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	int ps_data = 0;

	ps_data = data->gesture_motion;
//add yang
    i2c_smbus_write_byte_data(client, 0xEF, 0x00); // switch to register bank 0

    ps_data = i2c_smbus_read_word_data(client, 0x43);
    
	return sprintf(buf, "0x%2X\n", ps_data);
}

 static ssize_t apds9500_store_ps_data(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct apds9500_data *data = i2c_get_clientdata(client);

	int  ae_exposure_ub  = simple_strtoul(buf, NULL, 0);
	int ae_ub_L    =   ae_exposure_ub & 0xff;
	int ae_ub_H    =   (ae_exposure_ub >> 8) & 0xff;

	int ae_exposure_lb = ae_exposure_ub / 2;

	int ae_lb_L    =   ae_exposure_lb & 0xff;
	int ae_lb_H   =   (ae_exposure_lb >> 8) & 0xff;

	printk("%s: store ps senosr ae_exposure_ub = 0x%x ,ae_ub_L = 0x%x,ae_ub_H = 0x%x \n", __func__, ae_exposure_ub,ae_ub_L,ae_ub_H);

	printk("%s: store ps senosr ae_exposure_lb = 0x%x ,ae_lb_L = 0x%x,ae_lb_H = 0x%x \n", __func__, ae_exposure_lb,ae_lb_L,ae_lb_H);


	i2c_smbus_write_byte_data(client, 0x48, ae_ub_L);

	i2c_smbus_write_byte_data(client, 0x49, ae_ub_H);

	i2c_smbus_write_byte_data(client, 0x4A, ae_lb_L);

	i2c_smbus_write_byte_data(client, 0x4B, ae_lb_H);
	
	
	#if 0

	if(buf[0] == 'A')
	{
		
		i2c_smbus_write_byte_data(client, 0x48, 0x05);
		
		
	}
	else if(buf[0] == 'B')
	{
		
		
		i2c_smbus_write_byte_data(client, 0x49, 0x09);
	}

	#endif
	
	return count;
}

//static DEVICE_ATTR(ps_data, S_IRUGO,apds9500_show_ps_data, NULL);
static DEVICE_ATTR(ps_data, 0664,apds9500_show_ps_data, apds9500_store_ps_data);

static ssize_t apds9500_show_proximity_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9500_store_proximity_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable ps senosr ( %ld)\n", __func__, val);
	
	if ((val != APDS_DISABLE_PS) && 
		(val != APDS_ENABLE_PS_WITH_INT) &&
		(val != APDS_ENABLE_PS_NO_INT) &&
		(val != APDS_ENABLE_PS_CALIBRATION)) {
		printk("**%s:store invalid value=%ld\n", __func__, val);
		return count;
	}

	apds9500_enable_ps_sensor(client, val);	
	
	return count;
}

/* 
static DEVICE_ATTR(proximity_enable, S_IWUGO | S_IRUGO,
		apds9500_show_proximity_enable, apds9500_store_proximity_enable);
*/
static DEVICE_ATTR(proximity_enable, 0644,
                apds9500_show_proximity_enable, apds9500_store_proximity_enable);
        

static struct attribute *apds9500_attributes[] = {
	&dev_attr_ps_data.attr,
	&dev_attr_proximity_enable.attr,
	NULL
};

static const struct attribute_group apds9500_attr_group = {
	.attrs = apds9500_attributes,
};

static struct file_operations apds9500_ps_fops = {
	.owner = THIS_MODULE,
	.open = apds9500_ps_open,
	.release = apds9500_ps_release,
	.unlocked_ioctl = apds9500_ps_ioctl,
};

static struct miscdevice apds9500_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "apds_ps_dev",
	.fops = &apds9500_ps_fops,
};

/*
 * Initialization function
 */

static int apds9500_init_client(struct i2c_client *client)
{
	int err;
	int i=0;

	while (i++ < 10) {
        // wake sensor
		if (i2c_smbus_read_byte_data(client, 0x00) >= 0) {
            err = i2c_smbus_read_byte_data(client, 0x00);
            printk("apds9500_init_client reg00 0x%x\n",err);
			break;
		}
		mdelay(1);
	}
	
	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00);
	if (err < 0) return err;
	
	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds9500_driver;

static struct pinctrl *apds9500_pinctrl;


unsigned int init_reg[][2] = 
{
    {0xEF,0x00},
    {0x32,0x15},
    {0x37,0x07},
    {0x38,0x17},
    {0x39,0x06},
    {0x42,0x01},
    {0x46,0x2D},
    {0x47,0x0F},
    {0x48,0x08},
    {0x49,0x01},
    {0x4A,0x84},
    {0x4B,0x00},
    {0x4C,0x20},
    {0x51,0x10},
    {0x5E,0x10},
    {0x60,0x27},
    {0x80,0x42},
    {0x81,0x44},
    {0x82,0x04},
    {0x8B,0x01},
    {0x90,0x06},
    {0x95,0x0A},
    {0x96,0x0C},
    {0x97,0x05},
    {0x9A,0x14},
    {0x9C,0x3F},
    {0xA5,0x19},
    {0xCC,0x19},
    {0xCD,0x0B},
    {0xCE,0x03},
    {0xCF,0x64},
    {0xD0,0x21},
    {0xEF,0x01},
    {0x02,0x0F},
    {0x03,0x10},
    {0x04,0x02},
    {0x25,0x01},
    {0x27,0x39},
    {0x28,0x7F},
    {0x29,0x08},
    {0x3E,0xFF},
    {0x5E,0x3D},
    {0x65,0x96},
    {0x67,0x97},
    {0x69,0xCD},
    {0x6A,0x01},
    {0x6D,0x2C},
    {0x6E,0x01},
    {0x72,0x01},
    {0x73,0x35},
    {0x74,0x00},
    {0x77,0x01},
};


static int gesture_init(struct i2c_client *client)
{
    int idx = 0;
    for(idx = 0;idx < sizeof(init_reg)/(2*sizeof(unsigned int));idx++)
    {
        if(i2c_smbus_write_byte_data(client, init_reg[idx][0], init_reg[idx][1]) < 0)
        {
            printk("gesture init error\n");
            return -1;
        }   
        else
        {
          //  printk("reg/value: 0x%2X,0x%2X\n",init_reg[idx][0],init_reg[idx][1]);
        }
    }
    return 0;
}

//static int apds9500_resume(struct i2c_client *client);

static int  apds9500_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9500_data *data;

    struct pinctrl_state *irq_pullup;
    
	int err = 0;

	printk("===mlk===start %s\n",__func__);
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9500_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->client = client;
	apds9500_i2c_client = client;

	i2c_set_clientdata(client, data);

	data->ps_threshold = APDS9500_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold = APDS9500_PS_HSYTERESIS_THRESHOLD;
	data->ps_detection = 0;	/* default to no detection */
	data->ps_poll_delay = 25;	// default to 100ms
	data->ps_offset = 0; // default to 0, should load from nv memory if needed
	data->enable_ps_sensor = 0;	// default to 0
	data->ps_suspended = 0;
	data->main_ctrl_suspended_value = 0;	/* suspend_resume usage */
	data->gesture_motion = DIR_NONE;
	data->gesture_prev_motion = DIR_NONE;
	data->ae_exposure_ub = 96*4;	// 96 us
	data->led_current = 7; // ~ 100mA	
	
	//mutex_init(&data->update_lock);

#ifdef CONFIG_SENSOR_APDS9921
	err = gpio_request(platform_data->irq_num, "apds_irq");
	if (err)
    	{
        	printk("Unable to request GPIO.\n");
        	goto exit_kfree;
    	}
    
    	gpio_direction_input(platform_data->irq_num);
    	irq = gpio_to_irq(platform_data->irq_num);
    
    	if (irq < 0)
    	{
        	err = irq;
        	printk("Unable to request gpio irq. err=%d\n", err);
        	gpio_free(platform_data->irq_num);
        
        	goto exit_kfree;
    	}
    
    	data->irq = irq;
	if (request_irq(data->irq, apds9500_interrupt, IRQF_TRIGGER_FALLING,
		APDS9500_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate APDS9950_INT !\n", __func__);
	
		goto exit_kfree;
	}
#else	
    apds9500_pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR(apds9500_pinctrl)) {
		pr_err("Failed to get apds9500 pinctrl.\n");
		//ret = PTR_ERR(apds9500_pinctrl);
	} 
    else
    {
        irq_pullup = pinctrl_lookup_state(apds9500_pinctrl, "default");
        if (IS_ERR(irq_pullup)) {
                pr_err("Failed to init \n");
               // ret = PTR_ERR(irq_pullup);
        }
        else {
            pinctrl_select_state(apds9500_pinctrl, irq_pullup);
        }

    }
    
	data->gpio_pin = of_get_named_gpio(client->dev.of_node, "apds_gpio_int", 0);
    gpio_direction_input(data->gpio_pin);
	data->irq = gpio_to_irq(data->gpio_pin);
	if (request_irq(data->irq, apds9500_interrupt, IRQF_TRIGGER_FALLING,
		APDS9500_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate APDS9500_INT !\n", __func__);
	
		goto exit_kfree;
	}

        
#endif

#ifdef LINUX_KERNEL_2_6_X
	set_irq_wake(client->irq, 1);
#else
//	irq_set_irq_wake(client->irq, 1);
#endif

	// interrupt
	INIT_DELAYED_WORK(&data->dwork, apds9500_work_handler);
	// polling : PS
	INIT_DELAYED_WORK(&data->ps_dwork, apds9500_ps_polling_work_handler); 

	printk("%s interrupt is hooked\n", __func__);

	/* Initialize the APDS9500 chip */
	err = apds9500_init_client(client);
	if (err)
		goto exit_free_dev_ps;

	/* Register to Input Device */
	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		printk("Failed to allocate input device ps\n");
		goto exit_free_dev_ps;
	}
	
	//set_bit(EV_ABS, data->input_dev_ps->evbit);

	//input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 10, 0, 0);

	
	input_set_capability(data->input_dev_ps, EV_KEY, KEY_GESTURE_DIR_UP);
	input_set_capability(data->input_dev_ps, EV_KEY, KEY_GESTURE_DIR_DOWN);
	input_set_capability(data->input_dev_ps, EV_KEY, KEY_GESTURE_DIR_LEFT);
	input_set_capability(data->input_dev_ps, EV_KEY, KEY_GESTURE_DIR_RIGHT);
	input_set_capability(data->input_dev_ps, EV_KEY, KEY_GESTURE_DIR_FORWARD);
	input_set_capability(data->input_dev_ps, EV_KEY, KEY_GESTURE_DIR_BACK);
	


	//data->input_dev_ps->name = "proximity";
	  data->input_dev_ps->name = "gesture";

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("Unable to register input device ps: %s\n",
		       data->input_dev_ps->name);
		goto exit_unregister_dev_ps;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9500_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	/* Register for sensor ioctl */
    err = misc_register(&apds9500_ps_device);
	if (err) {
		printk("Unalbe to register ps ioctl: %d", err);
		goto exit_remove_sysfs_group;
	}
        
	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);

    //enable  interrupt
    apds9500_enable_ps_sensor(client,APDS_ENABLE_PS_WITH_INT);
	
	printk("===mlk===end %s\n",__func__);

	return 0;

	misc_deregister(&apds9500_ps_device);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &apds9500_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);	
exit_free_dev_ps:
	//free_irq(APDS9500_INT, client);	
	free_irq(data->irq, client);	
    
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int  apds9500_remove(struct i2c_client *client)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err;

	/* Power down the device */
	err = i2c_smbus_write_byte_data(client, 0xEF, 0x01); 
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x72, 0x00); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00); 
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x03, 0x01); 
	if (err < 0) return err;

	misc_deregister(&apds9500_ps_device);	

	sysfs_remove_group(&client->dev.kobj, &apds9500_attr_group);

	input_unregister_device(data->input_dev_ps);
	
	//free_irq(APDS9500_INT, client);
	free_irq(data->irq, client);	
	kfree(data);

	return 0;
}

//#define CONFIG_PM


#ifndef CONFIG_PM_SLEEP

static int apds9500_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err = 0;

	printk("apds9500_suspend\n");
	
	// Do nothing as p-sensor is in active
	if(data->enable_ps_sensor)
		return 0;
	
	// suspend
	err = i2c_smbus_write_byte_data(client, 0xEF, 0x01); 
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x72, 0x00); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00); 
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0x03, 0x01); 
	if (err < 0) return err;
	
	data->ps_suspended = 1;	

	if (data->ps_suspended) {
		cancel_delayed_work(&data->dwork);
		flush_delayed_work(&data->dwork);

		flush_workqueue(apds_workqueue);

		disable_irq(data->irq);
	
#ifdef LINUX_KERNEL_2_6_X
		set_irq_wake(client->irq, 0);
#else
		irq_set_irq_wake(client->irq, 0);
#endif

		if(NULL != apds_workqueue){
			destroy_workqueue(apds_workqueue);
			printk(KERN_INFO "%s, Destroy workqueue\n",__func__);
			apds_workqueue = NULL;
		}
	}

	return 0;
}


static int apds9500_resume(struct i2c_client *client)
{
	struct apds9500_data *data = i2c_get_clientdata(client);	
	int err = 0;

	printk("apds9500_resume (main_ctrl=%d)\n", data->main_ctrl_suspended_value);

	if(apds_workqueue == NULL) {
		apds_workqueue = create_workqueue("proximity_als");
		if(NULL == apds_workqueue)
			return -ENOMEM;
	}

	enable_irq(data->irq);

	mdelay(1);

	// wake up command
	i2c_smbus_read_byte_data(client, 0x00);
	mdelay(1);
	i2c_smbus_read_byte_data(client, 0x00);

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x01); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x72, 0x01); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEE, 0x03); // bit[2] = 0, DSP reset 
	if (err < 0) return err;
	
	err = i2c_smbus_write_byte_data(client, 0xEE, 0x07); // bit[2] = 1, DSP not-reset
	if (err < 0) return err;

	data->ps_suspended = 0;

#ifdef LINUX_KERNEL_2_6_X
	set_irq_wake(client->irq, 1);
#else
	irq_set_irq_wake(client->irq, 1);
#endif

	return 0;
}

#else

//#define apds9500_suspend	NULL
//#define apds9500_resume		NULL

#endif /* CONFIG_PM */


#ifndef CONFIG_PM_SLEEP
static const struct dev_pm_ops apds9500_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(apds9500_suspend, apds9500_resume)
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id apds9500_match_table[] = {
	{ .compatible = APDS9500_DRV_NAME },
	{ }
};
#endif


static const struct i2c_device_id apds9500_id[] = {
	{ "apds9500", 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, apds9500_id);

static struct i2c_driver apds9500_driver = {
	.driver = {
		.name	= APDS9500_DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = apds9500_match_table,
#endif
		#ifndef CONFIG_PM_SLEEP
		.pm = &apds9500_pm_ops,
        #endif
	},
	.probe	= apds9500_probe,
	.remove	= apds9500_remove,
	.id_table = apds9500_id,
};

static int __init apds9500_init(void)
{
	apds_workqueue = create_workqueue("proximity_als");
	
	if (!apds_workqueue)
		return -ENOMEM;

	return i2c_add_driver(&apds9500_driver);
}

static void __exit apds9500_exit(void)
{
	if (apds_workqueue)
		destroy_workqueue(apds_workqueue);

	apds_workqueue = NULL;

	i2c_del_driver(&apds9500_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9500 gesture sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9500_init);
module_exit(apds9500_exit);

