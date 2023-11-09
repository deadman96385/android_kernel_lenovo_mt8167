#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/version.h>
#include "alsps.h"
#include "cust_alsps.h"
#include <linux/timer.h>


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/hwmsensor.h>
#include <mach/eint.h>
#else
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#endif
#define APDS9500_POLLING_MODE 0 // 0 = interrupt, 1 = polling


/******************************* define *******************************/
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

#define APDS_IOCTL_GESTURE_ENABLE			3
#define APDS_IOCTL_GESTURE_GET_ENABLE		4

#define APDS_IOCTL_CHANGE2PSMODE            5
#define APDS_IOCTL_CHANGE2GSMODE            6

#define APDS_IOCTL_PS_GET_APPAOACH_DATA     7
#define APDS_IOCTL_GESTURE_GET_DATA			8


#define APDS_DISABLE_PS						0
#define APDS_ENABLE_PS_WITH_INT				1
#define APDS_ENABLE_PS_NO_INT				2
#define	APDS_ENABLE_PS_CALIBRATION			3

#define SWITCH_TO_BANK_0 i2c_smbus_write_byte_data(client, 0xEF, 0x00)
#define SWITCH_TO_BANK_1 i2c_smbus_write_byte_data(client, 0xEF, 0x01)

#define DO_PROCESSING_WHEN_CHANGE2PS_MODE

#define OP_MUTEX_LOCK {mutex_lock(&data->op_mutex); printk("%s -\n", __func__);}

#define OP_MUTEX_UNLOCK {printk("%s +\n", __func__); mutex_unlock(&data->op_mutex);}

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
struct apds9500_ps_mode_data{
    unsigned char approach;
    unsigned char rawdata;
};

#define APDS_DISABLE_GESTURE       0
#define APDS_ENABLE_GESTURE        1

#define APDS_DISABLE_PROXIMITY       0
#define APDS_ENABLE_PROXIMITY        1

//#define APDS9500_GESTURE_MODE 0
//#define APDS9500_PROXIMITY_MODE 1
typedef enum {
	APDS9500_PROXIMITY_MODE = 0,
	APDS9500_GESTURE_MODE
} apds9500_mode;

#define SUPPORT_REMOVE_SLEEP
#define SUPPORT_GESTURE_FAR_MODE // It's far mode if 0x04(bank1)=0x01; The gesture will be opposite, left-right, up - down

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
    int proximity_detection; /* 0 = near-to-far; 1 = far-to-near */
	int proximity_data; /* to store PS data */

	int enable_gesture_sensor;
	int enable_proximity_sensor;

	apds9500_mode mode;
	
	struct timer_list ps_data_timer;
	int ps_data_timer_enable;
	
	struct delayed_work	ps_data_work;
	int ps_data_work_enable;

#ifdef SUPPORT_REMOVE_SLEEP
	struct delayed_work	remove_sleep_timer_work;
	uint8_t remove_sleep_timer_enable;
	uint32_t remove_sleep_timer_count;
#endif

	struct mutex op_mutex;
};

/*
 * Global data
 */
static struct i2c_client *apds9500_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *apds_workqueue;


#define LED_CURRENT 0x15 //0x15
#define PROXIMITY_LED_CURRENT 0x15
#define GESTURE_LED_CURRENT 0x15

//#define PROXIMITY_THRESHOLD_NEAR 0x10
//#define PROXIMITY_THRESHOLD_FAR  0x01
#define PROXIMITY_THRESHOLD_NEAR 0x25
#define PROXIMITY_THRESHOLD_FAR  0x22

#define PROXIMITY_AE_UB 0x5A0
#define PROXIMITY_AE_UB_L (PROXIMITY_AE_UB & 0xff)
#define PROXIMITY_AE_UB_H ((PROXIMITY_AE_UB >> 8) & 0xff)

#define PROXIMITY_AE_LB (PROXIMITY_AE_UB / 2)
#define PROXIMITY_AE_LB_L (PROXIMITY_AE_LB & 0xff)
#define PROXIMITY_AE_LB_H ((PROXIMITY_AE_LB >> 8) & 0xff)


#define GESTURE_AE_UB 0x1E0//0x3C
#define GESTURE_AE_UB_L (GESTURE_AE_UB & 0xff)
#define GESTURE_AE_UB_H ((GESTURE_AE_UB >> 8) & 0xff)

#define GESTURE_AE_LB (GESTURE_AE_UB / 2)
#define GESTURE_AE_LB_L (GESTURE_AE_LB & 0xff)
#define GESTURE_AE_LB_H ((GESTURE_AE_LB >> 8) & 0xff)

#define IDLE_TIME_NORMAL_MODE_UPDATE_RATE_120HZ 183
#define IDLE_TIME_NORMAL_MODE_UPDATE_RATE_120HZ_L (IDLE_TIME_NORMAL_MODE_UPDATE_RATE_120HZ & 0xff)
#define IDLE_TIME_NORMAL_MODE_UPDATE_RATE_120HZ_H ((IDLE_TIME_NORMAL_MODE_UPDATE_RATE_120HZ >> 8) & 0xff)

#define IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ 53
#define IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_L (IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ & 0xff)
#define IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_H ((IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ >> 8) & 0xff)

#if 0
static unsigned char initial_setting_register_array[][2] = 
{
    {0xEF,0x00},
    {0x37,0x07},
    {0x38,0x17},
    {0x39,0x06},
    {0x42,0x01},
    {0x46,0x2D},
    {0x47,0x0F},
    {0x48,0xE0},  // image Exposure
    {0x49,0x01},
    {0x4A,0xF0},
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
    {0x32,LED_CURRENT}, // led current
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

static unsigned char gesture_setting_register_array[][2] =
{
    {0xef,0x00},
    {0x41,0x00},
    {0x42,0x00},
    {0xef,0x00},
    {0x48,GESTURE_AE_UB_L}, //0x1e0
    {0x49,GESTURE_AE_UB_H},
    {0x4A,GESTURE_AE_LB_L},
    {0x4B,GESTURE_AE_LB_H},
    {0x51,0x10},
    {0x83,0x20},
    {0x9f,0xf9},
    {0xef,0x01},
    {0x01,0x1e},
    {0x02,0x0f},
    {0x03,0x10},
    {0x04,0x02},
    {0x32,GESTURE_LED_CURRENT}, // led current
    {0x41,0x40},
    {0x43,0x30},
    //{0x65,0x96},IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_L
    //{0x66,0x00},
    {0x65,IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_L},
    {0x66,IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_H},
    {0x67,0x97},
    {0x68,0x01},
    {0x69,0xcd},
    {0x6a,0x01},
#if 0  
    {0x6b,0xb0},
    {0x6c,0x04},
    {0x6d,0x2c},
    {0x6e,0x01},
#else
    {0x6b,0xff},
	{0x6c,0x30},
	{0x6d,0x2c},
	{0x6e,0x01},
#endif
	
    //{0x72,0x01},
    {0x74,0x00},
    {0xef,0x00},
    {0x41,0xef},
    {0x42,0x01},

};

static unsigned char proximity_setting_register_array[][2] = {
	{0xEF, 0x00},
	{0x41, 0x00},
	{0x42, 0x02},
	{0x46, 0x2D},
	{0x47, 0x0F},
    {0x48, PROXIMITY_AE_UB_L}, //0x5A0
	{0x49, PROXIMITY_AE_UB_H},
	{0x4A, PROXIMITY_AE_LB_L},
	{0x4B, PROXIMITY_AE_LB_H},
	{0x51, 0x13},
	{0x54, 0x14},
	{0x5C, 0x02},
	{0x5E, 0x10},
	{0x69, PROXIMITY_THRESHOLD_NEAR},
	{0x6A, PROXIMITY_THRESHOLD_FAR},
#if (APDS9500_POLLING_MODE)
	{0x80, 0x42},
	{0x81, 0x44},
	{0x82, 0x0B},
#else
	{0x80, 0x42},
	{0x81, 0x44},
	{0x82, 0x04},
#endif
	{0x90, 0x06},
	{0x95, 0x0A},
	{0x96, 0x0C},
	{0x97, 0x05},
	{0x9A, 0x14},
	{0x9C, 0x3F},
	{0x9F, 0xF8},
	{0xA5, 0x19},
	{0xCC, 0x19},
	{0xCD, 0x0B},
	{0xCE, 0x13},
	{0xCF, 0x64},
	{0xD0, 0x21},
	{0xD2, 0x88},
	{0x8C, 0x37},
	{0xEF, 0x01},
	{0x00, 0x1E},
	{0x01, 0x1E},
	{0x02, 0x0F},
	{0x03, 0x0F},
	{0x04, 0x02},
	{0x1E, 0x00},
	{0x25, 0x01},
	{0x26, 0x05},
	{0x27, 0x37},
	{0x28, 0x7F},
	{0x29, 0x0A},
	{0x32, PROXIMITY_LED_CURRENT}, // led current
	{0x3E, 0x00},
	{0x4A, 0x1E},
	{0x4B, 0x1E},
	
	//{0x65, 0xCE},
	//{0x66, 0x0B},
	{0x65,IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_L},
    {0x66,IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_H},
	{0x67, 0xCE},
	{0x68, 0x0B},
#if 0
	//{0x6D, 0x50},
	//{0x6E, 0xC3},
	{0x6b,0xb0},
    {0x6c,0x04},
    {0x6d,0x2c},
    {0x6e,0x01},
#else
	{0x6b,0xff},
	{0x6c,0x30},
	{0x6d,0x2c},
	{0x6e,0x01},
#endif	
	//{0x72, 0x01},
	{0x74, 0x05},
	{0x77, 0x01},
	{0x7C, 0x84},
	{0x7D, 0x03},
	{0x7E, 0x00},
	{0xEF, 0x00},
	{0, 0},
};
#else
static unsigned char initial_setting_register_array[][2] =
{
	{0xEF,0x00},
	{0x37,0x07},
	{0x38,0x17},
	{0x39,0x06},
	{0x42,0x01},
	{0x46,0x2D},
	{0x47,0x0F},
	{0x48,0xE0},  // image Exposure
	{0x49,0x01},
	{0x4A,0xF0},
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
	{0x32,LED_CURRENT}, // led current
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

static unsigned char gesture_setting_register_array[][2] =
{
	{0xef,0x00},
	{0x41,0x00},
	{0x42,0x00},
	{0x48,GESTURE_AE_UB_L}, //0x258
	{0x49,GESTURE_AE_UB_H},
	{0x4A,GESTURE_AE_LB_L},
	{0x4B,GESTURE_AE_LB_H},
	{0x51,0x10},
	{0x83,0x20},
	{0x9f,0xc9}, //0xf9
	{0xd2,0x08}, //0x88 default
	{0xef,0x01},
	//{0x01,0x1e},
	//{0x02,0x0f},
	//{0x03,0x10},
	//{0x04,0x02},
	{0x00,0x1e},
	{0x01,0x1e},
	{0x02,0x0f},
	{0x03,0x0f},
	{0x04,0x01},
	{0x32,GESTURE_LED_CURRENT}, // led current
	{0x41,0x40},
	{0x43,0x30},
	{0x65,IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_L},//shd for sleep
	{0x66,IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_H},
	{0x67,0x97},
	{0x68,0x01},
	{0x69,0xcd},
	{0x6a,0x01},
	{0x6b,0xff}, //shd for sleep 0xb0 0x30
	{0x6c,0x40},
	{0x6d,0x2c}, //0x2c 0x01
	{0x6e,0x01},
	//{0x6b,0xff},
	//{0x6c,0x30},
	//{0x6d,0xff},
	//{0x6e,0xff},
	{0x72,0x01},				 // kk 24-Apr-2020
	{0x74,0x00},
	{0xef,0x00},
	{0x41,0x0f}, //0xef
	{0x42,0x00}, //0x01
};
	 
static unsigned char proximity_setting_register_array[][2] = {
	{0xEF, 0x00},
	{0x41, 0x00},
	{0x42, 0x02},
	{0x46, 0x2D},
	{0x47, 0x0F},
	{0x48, PROXIMITY_AE_UB_L}, //0x3C0
	{0x49, PROXIMITY_AE_UB_H},
	{0x4A, PROXIMITY_AE_LB_L},
	{0x4B, PROXIMITY_AE_LB_H},
	{0x51, 0x13},
	//{0x54, 0x14}, // kk 24-Apr-2020
	//{0x5C, 0x02}, // kk 24-Apr-2020
	//{0x5E, 0x10},   // kk 24-Apr-2020
	{0x69, PROXIMITY_THRESHOLD_NEAR},
	{0x6A, PROXIMITY_THRESHOLD_FAR},
#if (APDS9500_POLLING_MODE)
	{0x80, 0x42},
	{0x81, 0x44},
	{0x82, 0x0B},
#else
	{0x80, 0x42},
	{0x81, 0x44},
	{0x82, 0x04},
#endif
	{0x83, 0x00},	  
	//{0x90, 0x06},  // kk 24-Apr-2020
	//{0x95, 0x0A},  // kk 24-Apr-2020
	//{0x96, 0x0C},  // kk 24-Apr-2020
	//{0x97, 0x05},  // kk 24-Apr-2020
	//{0x9A, 0x14},  // kk 24-Apr-2020
	//{0x9C, 0x3F},  // kk 24-Apr-2020
	{0x9F, 0xF8},	  
	//{0xA5, 0x19},  // kk 24-Apr-2020							  
	//{0xCC, 0x19},  // kk 24-Apr-2020							  
	//{0xCD, 0x0B}, // kk 24-Apr-2020							 
	//{0xCE, 0x13},  // kk 24-Apr-2020							  
	//{0xCF, 0x64},  // kk 24-Apr-2020							  
	//{0xD0, 0x21},  // kk 24-Apr-2020							  
	//{0xD2, 0x88},  // kk 24-Apr-2020							  
	//{0x8C, 0x37},  // kk 24-Apr-2020							  
	{0xEF, 0x01},
	{0x00, 0x1E},
	{0x01, 0x1E},
	{0x02, 0x0F},
	{0x03, 0x0F},
	{0x04, 0x02},
	//{0x1E, 0x00},   // kk 24-Apr-2020
	//{0x25, 0x01},  // kk 24-Apr-2020
	//{0x26, 0x05},  // kk 24-Apr-2020
	//{0x27, 0x37},  // kk 24-Apr-2020
	//{0x28, 0x7F},  // kk 24-Apr-2020
	//{0x29, 0x0A},  // kk 24-Apr-2020
	{0x32, PROXIMITY_LED_CURRENT}, // led current
	//{0x3E, 0x00},   // kk 24-Apr-2020
	{0x41,0x50},   // kk 24-Apr-2020
	{0x43,0x34},   // kk 24-Apr-2020
	//{0x4A, 0x1E},  // kk 24-Apr-2020
	//{0x4B, 0x1E},  // kk 24-Apr-2020
	{0x65, IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_L}, //shd for sleep
	{0x66, IDLE_TIME_GAMING_MODE_UPDATE_RATE_240HZ_H},
	{0x67, 0xCE},
	{0x68, 0x0B},
	
	//{0x6D, 0x50},
	//{0x6E, 0xC3},
	//{0x6b, 0xff}, //shd for sleep
	//{0x6c, 0xff},
	//{0x6d, 0xff},
	//{0x6e, 0xff},
	{0x6b,0xff}, //shd for sleep
	{0x6c,0x40},
	{0x6d,0x2c},
	{0x6e,0x01},

	{0x72, 0x01},  // kk 24-Apr-2020
	{0x74, 0x05},
	//{0x77, 0x01},  // kk 24-Apr-2020
	//{0x7C, 0x84},  // kk 24-Apr-2020
	//{0x7D, 0x03},  // kk 24-Apr-2020
	//{0x7E, 0x00},   // kk 24-Apr-2020
	{0xEF, 0x00},
	{0, 0},
};
#endif

#define SUPPORT_POWER_DOWN_RESET
#ifdef SUPPORT_POWER_DOWN_RESET
#include <linux/regulator/consumer.h>
struct regulator *apds9500_3v3_reg;
static int apds9500_get_3v3_supply(struct device *dev)
{
	int ret;
	struct regulator *apds9500_3v3_ldo;

	printk("%s\n", __func__);

	apds9500_3v3_ldo = devm_regulator_get(dev, "apds9500-3v3");
	if (IS_ERR(apds9500_3v3_ldo)) {
		ret = PTR_ERR(apds9500_3v3_ldo);
		printk("%s,failed to get vmch_3v3 LDO\n", __func__);
		return ret;
	}

	printk("%s,get vmch_3v3 supply ok.\n", __func__);

	/* get current voltage settings */
	ret = regulator_get_voltage(apds9500_3v3_ldo);
	printk("%s,vmch_3v3 voltage = %d\n", __func__, ret);

	apds9500_3v3_reg = apds9500_3v3_ldo;

	return ret;
}

int apds9500_3v3_supply_enable(void)
{
	int ret;
	unsigned int volt_3v3;

	printk("%s\n", __func__);

	if (apds9500_3v3_reg == NULL)
		return 0;

	/* set voltage to 3.3V */
	ret = regulator_set_voltage(apds9500_3v3_reg, 3300000, 3300000);
	if (ret != 0) {
		printk("%s,failed to set vmch_3v3 voltage\n", __func__);
		return ret;
	}

	/* get voltage settings again */
	volt_3v3 = regulator_get_voltage(apds9500_3v3_reg);
	if (volt_3v3 == 3300000)
		printk("%s,check regulator voltage = 3300000 pass!\n", __func__);
	else
		printk("%s,check regulator voltage = 3300000 fail! (voltage: %d)\n", __func__, volt_3v3);

	ret = regulator_enable(apds9500_3v3_reg);
	if (ret != 0) {
		printk("%s,Failed to enable vmch_3v3\n", __func__);
		return ret;
	}

	return ret;
}

int apds9500_3v3_supply_disable(void)
{
	int ret = 0;
	unsigned int volt_3v3;
	unsigned int isenable_3v3;

	if (apds9500_3v3_reg == NULL)
		return 0;

	/* disable regulator */
	isenable_3v3 = regulator_is_enabled(apds9500_3v3_reg);
	printk("%s,query regulator enable status[%d]\n", __func__, isenable_3v3);

	//if (isenable_3v3) 
	{
		regulator_set_voltage(apds9500_3v3_reg, 0, 0);
		msleep(1);
		volt_3v3 = regulator_get_voltage(apds9500_3v3_reg);
		
		ret = regulator_disable(apds9500_3v3_reg);
		if (ret != 0) {
			printk("%s,failed to disable vmch_3v3\n", __func__);
			return ret;
		}
		/* verify */
		isenable_3v3 = regulator_is_enabled(apds9500_3v3_reg);
		if (!isenable_3v3)
			printk("%s,regulator disable pass\n", __func__);
		
		printk("%s,regulator disable,isenable_3v3=%d,volt_3v3=%d\n", __func__, isenable_3v3, volt_3v3);
	}

	return ret;
}

/*
extern void bh1749_reset_and_init(void);
static int apds9500_load_initial_setting(struct i2c_client *client);
static void apds950_wakeup(struct i2c_client *client)
{
	int i = 0;
	while (i++ < 20) {
		if (i2c_smbus_read_byte_data(client, 0x00) >= 0) {
			break;
		}
		mdelay(1);
	}
	printk("%s,i=%d,%d\n", __func__, i, (20 == i) ? "fail" : "success");	
}

static void apds9500_power_down_and_initial(struct i2c_client *client)
{
	apds9500_vmch_supply_3v3_disable();
	msleep(100);
	apds9500_vmch_supply_3v3_enable();
	msleep(1);
	apds950_wakeup(client);
	msleep(1);
	apds9500_load_initial_setting(client);
	msleep(1);

	bh1749_reset_and_init();
}
*/

static ssize_t apds9500_store_3v3_power(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	if(!strncmp(buf, "0", 1))
	{
		apds9500_3v3_supply_disable();
	}
	else
	{
		apds9500_3v3_supply_enable();
	}
	
	return count;
}

static DEVICE_ATTR(power_3v3, 0644, NULL, apds9500_store_3v3_power);

static ssize_t apds9500_store_3v3_power_level(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	
	if(!strncmp(buf, "0", 1))
	{
		ret = regulator_set_voltage(apds9500_3v3_reg, 0, 0);
		if (ret != 0) {
			printk("%s,failed to set lcm_vgp_3v3 voltage\n", __func__);
			return ret;
		}
	}
	else
	{
		ret = regulator_set_voltage(apds9500_3v3_reg, 3300000, 3300000);
		if (ret != 0) {
			printk("%s,failed to set lcm_vgp_3v3 voltage\n", __func__);
			return ret;
		}
	}
	
	return count;
}

static DEVICE_ATTR(power_3v3_level, 0644, NULL, apds9500_store_3v3_power_level);
#endif


#ifdef SUPPORT_REMOVE_SLEEP
#define CHECK_SLEEP_TIME 400
static int apds9500_sensor_suspend(struct i2c_client *client);
static int apds9500_sensor_resume(struct i2c_client *client);
static int apds9500_init_device(struct i2c_client *client, int mode);
static void apds9500_read_and_clear_interrupt(struct i2c_client *client);
static void apds9500_remove_sleep_timer_work_handler(struct work_struct *work)
{
	struct apds9500_data *data = container_of(work, struct apds9500_data, remove_sleep_timer_work.work);
	struct i2c_client *client = data->client;
	u8 op_status;
	u8 reg_b6;

	OP_MUTEX_LOCK;

	if(data->remove_sleep_timer_enable)
	{
		SWITCH_TO_BANK_0;
		op_status = i2c_smbus_read_byte_data(client, 0x45);
		
		reg_b6 = i2c_smbus_read_byte_data(client, 0xb6);

		data->remove_sleep_timer_count += 1;
		
		printk("%s,op_status=%d %d, 0xb6=%02x\n", __func__, op_status, data->remove_sleep_timer_count, reg_b6);
		
		if(op_status != 0)
		{
			printk("9500 enter sleep,suspend-resume\n");
		
			apds9500_sensor_suspend(client);
			msleep(10);
			apds9500_sensor_resume(client);
			/*
			if(data->mode == APDS9500_GESTURE_MODE)
				apds9500_init_device(client, APDS9500_GESTURE_MODE);
			else
				apds9500_init_device(client, APDS9500_PROXIMITY_MODE);
			*/
			apds9500_read_and_clear_interrupt(client);

			data->remove_sleep_timer_count = 0;
		}

		cancel_delayed_work(&data->remove_sleep_timer_work);
		queue_delayed_work(apds_workqueue, &data->remove_sleep_timer_work, CHECK_SLEEP_TIME);
	}
	
	OP_MUTEX_UNLOCK;
}

static ssize_t apds9500_store_remove_sleep_timer(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	if(!strncmp(buf, "0", 1))
	{
		data->remove_sleep_timer_enable = 0;
	}
	else
	{
		data->remove_sleep_timer_enable = 1;
		cancel_delayed_work(&data->remove_sleep_timer_work);
		queue_delayed_work(apds_workqueue, &data->remove_sleep_timer_work, CHECK_SLEEP_TIME);
	}
		
	
	printk("%s,remove_sleep_timer_enable=%d\n", __func__, data->remove_sleep_timer_enable);
	return count;
}

static DEVICE_ATTR(remove_sleep_timer, 0644, NULL, apds9500_store_remove_sleep_timer);
#endif


static int apds9500_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int apds9500_remove(struct i2c_client *client);

static int apds9500_local_init(void);
static int apds9500_local_remove(void);

static void apds9500_read_and_clear_interrupt(struct i2c_client *client)
{
	int reg_43;
	u8 reg_6b;
	
	SWITCH_TO_BANK_0;
	reg_43 = i2c_smbus_read_word_data(client, 0x43);
	reg_6b = i2c_smbus_read_byte_data(client, 0x6B);
	printk("%s,reg_43=%04x,reg_6b=%02x\n", __func__, reg_43, reg_6b);
}

static int apds9500_sensor_suspend(struct i2c_client *client)
{
	int err=0;
	
	err = i2c_smbus_write_byte_data(client, 0xEF, 0x01);
	if (err < 0) return err;

	//err = i2c_smbus_write_byte_data(client, 0x7E, 0x00); 
	//if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x72, 0x00); 
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00);
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x03, 0x01); 
	if (err < 0) return err;
	
	return err;
}

static int apds9500_sensor_resume(struct i2c_client *client)
{
	int err=0;
	int i=0;

	while (i++ < 20) {

		if (i2c_smbus_read_byte_data(client, 0x00) >= 0) {
			break;
		}
		mdelay(1);
	}
	printk("%s,i=%d,%s\n", __func__, i, (i != 20) ? "success" : "failed");

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x01);
	if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0x72, 0x01); 
	if (err < 0) return err;

	//err = i2c_smbus_write_byte_data(client, 0x7E, 0x00); 
	//if (err < 0) return err;

	err = i2c_smbus_write_byte_data(client, 0xEF, 0x00);
	if (err < 0) return err;
	
	/*
		It's important!
		software reset is required for suspend - resume, or sensor will be abnormal
	*/
	err = i2c_smbus_write_byte_data(client, 0xEE, 0x03);
	if (err < 0) return err;
	msleep(10);
	err = i2c_smbus_write_byte_data(client, 0xEE, 0x07);
	if (err < 0) return err;

	return err;	
}

static int apds9500_load_initial_setting(struct i2c_client *client)
{
    int idx = 0;
    for(idx = 0;idx < sizeof( initial_setting_register_array)/(2*sizeof(unsigned int));idx++)
    {
        if(i2c_smbus_write_byte_data(client,  initial_setting_register_array[idx][0],  initial_setting_register_array[idx][1]) < 0)
        {
            printk("gesture init error\n");
            return -1;
        }   
        else
        {
        
        }
    }
    return 0;
}

static int apds9500_load_gesture_setting (struct i2c_client *client)
{
	int idx = 0;
	struct apds9500_data *data = i2c_get_clientdata(client);

	data->mode = APDS9500_GESTURE_MODE;
	data->enable_gesture_sensor = 1;
	
	for(idx = 0;idx < sizeof(gesture_setting_register_array)/2;idx++)
	{
	    if(i2c_smbus_write_byte_data(client, gesture_setting_register_array[idx][0], gesture_setting_register_array[idx][1]) < 0)
	    {
	        printk("psmode init error\n");
	        return -1;
	    }   
	    else
	    {
	    	
	    }
	}

	printk("%s,mode=%d\n", __func__, data->mode);

	return 0;
}

static int apds9500_load_proximity_setting(struct i2c_client *client)
{
	int err = 0, i = 0;
	struct apds9500_data *data = i2c_get_clientdata(client);

	data->mode = APDS9500_PROXIMITY_MODE;
	data->enable_proximity_sensor = 1;
	
	for (i=0;;i++) {
		if (proximity_setting_register_array[i][0]==0 && proximity_setting_register_array[i][1]==0) {
		   break;
		} else {
			err = i2c_smbus_write_byte_data (client, proximity_setting_register_array[i][0], proximity_setting_register_array[i][1]);
			if (err < 0) 
				printk("%s: err = %d, %d [%x, %x]\n", __func__, err, i, proximity_setting_register_array[i][0], proximity_setting_register_array[i][1]); 
	   }
	}

	printk("%s,mode=%d\n", __func__, data->mode);
	
	return err;
}

static int apds9500_init_device(struct i2c_client *client, int mode)
{
	int err = 0;
#if 0
	int i = 0;
	while (i++ < 20) {
		if (i2c_smbus_read_byte_data(client, 0x00) >= 0) {
			break;
		}
		mdelay(1);
	}
	printk("%s,i=%d,%d\n", __func__, i, (20 == i) ? "fail" : "success");
#endif	
	if (mode == APDS9500_GESTURE_MODE) {
		err = apds9500_load_gesture_setting(client);
	}
	else if (mode == APDS9500_PROXIMITY_MODE) {
		err = apds9500_load_proximity_setting(client);
	}

	if (err < 0)
		return err;

	//err = apds9500_sensor_suspend(client);
	//err = apds9500_sensor_resume(client);
	
	return err;
}

static int apds9500_enable_proximity_sensor(struct i2c_client *client, int val)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err;
	
	printk("enable proximity senosr ( %d)\n", val);

	if ((val != APDS_DISABLE_PROXIMITY) && 
		(val != APDS_ENABLE_PROXIMITY)) {
		printk("%s:invalid value=%d\n", __func__, val);
		return -1;
	}
	
#if 0	
	if (data->enable_gesture_sensor == APDS_ENABLE_GESTURE) {
		printk("%s:device busy\n", __func__);
		return -EBUSY;
	}
#else
	if (data->mode == APDS9500_GESTURE_MODE) {
		printk("%s,current is APDS9500_GESTURE_MODE\n", __func__);
		return -EBUSY;
	}
#endif
	
	if(val == APDS_ENABLE_PROXIMITY) {
		//turn on p sensor
		if (data->enable_proximity_sensor==APDS_DISABLE_PROXIMITY) {
		
			data->enable_proximity_sensor = APDS_ENABLE_PROXIMITY;
			
			err = apds9500_init_device(client, APDS9500_PROXIMITY_MODE);
			if (err) {
				printk("Failed to init apds9500\n");
				return err;
			}
	
			err = apds9500_sensor_resume(client);
			
#if (APDS9500_POLLING_MODE)		
		cancel_delayed_work(&data->polling_dwork);
		flush_delayed_work(&data->polling_dwork);
		queue_delayed_work(apds_workqueue, &data->polling_dwork, msecs_to_jiffies(data->proximity_poll_delay));
#else
		//queue_delayed_work(apds_workqueue, &data->monitoring_dwork, msecs_to_jiffies(20));
#endif
		}
	}
	else {

		data->enable_proximity_sensor = APDS_DISABLE_PROXIMITY;

		apds9500_sensor_suspend(client);

#if (APDS9500_POLLING_MODE)			
		/*
		 * If work is already scheduled then subsequent schedules will not

		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->polling_dwork);
		flush_delayed_work(&data->polling_dwork);	
#endif
	}
	
	return 0;
}

static int apds9500_enable_gesture_sensor(struct i2c_client *client, int val)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int err;
	
	printk("enable gesture senosr ( %d)\n", val);

	
	if ((val != APDS_DISABLE_GESTURE) && 
		(val != APDS_ENABLE_GESTURE)) {
		printk("%s:invalid value=%d\n", __func__, val);
		return -1;
	}

#if 0	
	if (data->enable_proximity_sensor == APDS_ENABLE_PROXIMITY) {
		printk("%s:device busy\n", __func__);
		return -EBUSY;
	}
#else
	if (data->mode == APDS9500_PROXIMITY_MODE) {
		printk("%s,current is APDS9500_PROXIMITY_MODE\n", __func__);
		return -EBUSY;
	}
#endif
	
	if(val == APDS_ENABLE_GESTURE) {
		//turn on p sensor
		if (data->enable_gesture_sensor==APDS_DISABLE_GESTURE) {

			data->enable_gesture_sensor = APDS_ENABLE_GESTURE;
		
			err = apds9500_init_device(client, APDS9500_GESTURE_MODE);
			if (err) {
				printk("Failed to init apds9500\n");
				return err;
			}
	
			err = apds9500_sensor_resume(client);
						
#if (APDS9500_POLLING_MODE)			
		cancel_delayed_work(&data->polling_dwork);
		flush_delayed_work(&data->polling_dwork);
		queue_delayed_work(apds_workqueue, &data->polling_dwork, msecs_to_jiffies(data->gesture_poll_delay));
#endif
		}
	}
	else {

		data->enable_gesture_sensor = APDS_DISABLE_GESTURE;

		apds9500_sensor_suspend(client);

#if (APDS9500_POLLING_MODE)			
		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->polling_dwork);
		flush_delayed_work(&data->polling_dwork);	
#endif
	}
	
	return 0;
}

static int apds9500_gesture_processing(struct i2c_client *client)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int ret;
	int gesture_detection = 0;
	u8 reg_b6, reg_44;
	
	ret = i2c_smbus_write_byte_data(client, 0xEF, 0x00); // switch to register bank 0
	if (ret < 0)
		goto exit;
	
	gesture_detection = i2c_smbus_read_word_data(client, 0x43);

	reg_b6 = i2c_smbus_read_byte_data(client, 0xb6);
	reg_44 = i2c_smbus_read_byte_data(client, 0x44);
	
	if (gesture_detection < 0)
	{
		ret = gesture_detection;
		goto exit;
	}
		
	gesture_detection &= 0x1FF;
	
	if (gesture_detection == 0x01)
	{
		//data->gesture_motion = DIR_DOWN;
		//data->gesture_motion = KEY_DOWN;
	#ifndef SUPPORT_GESTURE_FAR_MODE
		data->gesture_motion = KEY_GESTURE_DIR_UP;
	#else
		data->gesture_motion = KEY_GESTURE_DIR_DOWN;
	#endif
	}
	else if (gesture_detection == 0x02) {
		//data->gesture_motion = DIR_UP;
		//data->gesture_motion = KEY_UP;
	#ifndef SUPPORT_GESTURE_FAR_MODE
		data->gesture_motion = KEY_GESTURE_DIR_DOWN;
	#else
		data->gesture_motion = KEY_GESTURE_DIR_UP;
	#endif
	}      
	else if (gesture_detection == 0x04) {
		//data->gesture_motion = DIR_RIGHT;
		//data->gesture_motion = KEY_RIGHT;
	#ifndef SUPPORT_GESTURE_FAR_MODE
		data->gesture_motion = KEY_GESTURE_DIR_LEFT; 
	#else
		data->gesture_motion = KEY_GESTURE_DIR_RIGHT;
	#endif
	}
	else if (gesture_detection == 0x08) {
		//data->gesture_motion = DIR_LEFT;
		// data->gesture_motion = KEY_LEFT;
	#ifndef SUPPORT_GESTURE_FAR_MODE
		data->gesture_motion = KEY_GESTURE_DIR_RIGHT; 
	#else
		data->gesture_motion = KEY_GESTURE_DIR_LEFT;
	#endif
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

	printk("%s,gesture_detection=%d,gesture = %d (%d),0xb6=%02x,0x44=%02x\n", __func__, gesture_detection, data->gesture_motion, data->gesture_prev_motion, reg_b6, reg_44);
	
	//if (data->gesture_motion != DIR_NONE && data->gesture_motion != data->gesture_prev_motion) {
	if (data->gesture_motion != DIR_NONE)
	{
		//printk("reporting gesture = %d (%d)\n", data->gesture_motion, data->gesture_prev_motion);
		//input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->gesture_motion); /* GESTURE event */	
		input_report_key(data->input_dev_ps,data->gesture_motion,1); 
		input_sync(data->input_dev_ps);

		input_report_key(data->input_dev_ps,data->gesture_motion,0);
		input_sync(data->input_dev_ps);

		//data->gesture_prev_motion = data->gesture_motion;
	}
	ret = data->gesture_motion;

exit:	
	
	return ret;
}


static int apds9500_proximity_processing(struct i2c_client *client)
{
	struct apds9500_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned char UB,LB;
	
	ret = i2c_smbus_write_byte_data(client, 0xEF, 0x00); // switch to register bank 0
	if (ret < 0) {
		printk("err 1 = %d\n", ret);
		goto exit;
	}

	data->proximity_detection = i2c_smbus_read_byte_data(client, 0x6B);
	if (data->proximity_detection < 0) {
		printk("err 2 = %d\n", data->proximity_detection);
		ret = data->proximity_detection;
		goto exit;
	}
	data->proximity_data = i2c_smbus_read_byte_data(client, 0x6C);
	if (data->proximity_data < 0) {
		printk("err 3 = %d\n", data->proximity_data);
		ret = data->proximity_data;
		goto exit;
	}
    /* just for debug */
    UB = i2c_smbus_read_byte_data(client, 0x69);
    
    LB = i2c_smbus_read_byte_data(client, 0x6A);
    
    printk("%s,ps_approach_status=0x%02X, ps_raw_data=0x%02X, UB %x LB %x \n", 
		__func__, data->proximity_detection, data->proximity_data, UB, LB);

#if 0	
	data->proximity_detection = 1 - data->proximity_detection;
	if (0 == data->proximity_detection) {
		printk("proximity_data = %d (far-to-near)\n", data->proximity_data);
		return 1;/* FAR-to-NEAR detection */	
	}
	else {
		printk("proximity_data = %d (near-to-far)\n", data->proximity_data);
        return 0;
	}
#else
	if (data->proximity_detection) {
		printk("proximity_data = %d (far-to-near)\n", data->proximity_data);
		ret = 1;/* FAR-to-NEAR detection */	
	}
	else {
		printk("proximity_data = %d (near-to-far)\n", data->proximity_data);
        ret = 0;
	}
#endif

exit:

	return ret;
}

static void apds9500_reschedule_ps_work(struct apds9500_data *data,
					  unsigned long delay)
{
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
    cancel_delayed_work(&data->ps_dwork);
	queue_delayed_work(apds_workqueue, &data->ps_dwork, delay);

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

static void apds9500_ps_work_handler(struct work_struct *work)
{
	struct apds9500_data *data = container_of(work, struct apds9500_data, ps_dwork.work);
	struct i2c_client *client=data->client;
    int value;
    
	if (data->enable_proximity_sensor) {
		/* PS interrupt */
		OP_MUTEX_LOCK;
		
	    value = apds9500_proximity_processing(client);  
	    ps_report_interrupt_data(value);
		
		OP_MUTEX_UNLOCK;
	}
	
	//queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));	// restart timer
}

static void apds9500_work_handler(struct work_struct *work)
{
	struct apds9500_data *data = container_of(work, struct apds9500_data, dwork.work);
	struct i2c_client *client=data->client;

	if (data->enable_gesture_sensor) {
		OP_MUTEX_LOCK;
		
		apds9500_gesture_processing(client);

		OP_MUTEX_UNLOCK;
	}
}

/* assume this is ISR */
static irqreturn_t apds9500_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds9500_data *data = i2c_get_clientdata(client);

    if(data->mode == APDS9500_GESTURE_MODE) {
	    printk("==> gsmode apds9500_interrupt\n");
	    apds9500_reschedule_work(data, 0);	
    }
    else if(data->mode == APDS9500_PROXIMITY_MODE) {       
        printk("==> psmode apds9500_interrupt\n");
        apds9500_reschedule_ps_work(data,0);   
    }
	return IRQ_HANDLED;
}

/*
 * SysFS support
 */
static ssize_t apds9500_show_proximity_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_proximity_sensor);
}

static ssize_t apds9500_store_proximity_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable ps senosr ( %ld)\n", __func__, val);

	OP_MUTEX_LOCK;

	apds9500_enable_proximity_sensor(client, val);	

	OP_MUTEX_UNLOCK;
	
	return count;
}

static ssize_t apds9500_show_gesture_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_gesture_sensor);
}

static ssize_t apds9500_store_gesture_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable ps senosr ( %ld)\n", __func__, val);

	OP_MUTEX_LOCK;
	
	apds9500_enable_gesture_sensor(client, val);	

	OP_MUTEX_UNLOCK;
	
	return count;
}

static ssize_t apds9500_show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	printk("%s,data->mode=%d\n", __func__, data->mode);
	
	return sprintf(buf, "%s\n", (data->mode ? "gs mode" : "ps mode"));
}

static ssize_t apds9500_store_mode(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	//unsigned long val = simple_strtoul(buf, NULL, 10);

	OP_MUTEX_LOCK;

	if(!strncmp(buf, "ps", 2))
	{
		printk("%s,set ps mode\n", __func__);
		//ret = apds9500_load_proximity_setting(client);
		ret = apds9500_init_device(client, APDS9500_PROXIMITY_MODE);
	    if(ret != 0)
		{
			printk(KERN_ERR"in func %s set psmode failed %d\n",__FUNCTION__,ret);
	    }
		//data->mode = APDS9500_PROXIMITY_MODE;

#ifdef DO_PROCESSING_WHEN_CHANGE2PS_MODE
		ret = apds9500_proximity_processing(client);  
	    ps_report_interrupt_data(ret);
#endif
	}
	else if(!strncmp(buf, "gs", 2))
	{
		printk("%s,set gs mode\n", __func__);
		//ret = apds9500_load_gesture_setting(client);
		ret = apds9500_init_device(client, APDS9500_GESTURE_MODE);
		if(ret != 0){
			printk(KERN_ERR"in func %s set gsmode failed %d\n",__FUNCTION__,ret);
		}
		//data->mode = APDS9500_GESTURE_MODE;
	}

	OP_MUTEX_UNLOCK;
	
	printk("%s,apds9500 switch mode=%s,count=%ld,mode=%d\n", __func__, buf, count, data->mode);
	return count;
}

static ssize_t apds9500_store_pswaddr(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int regaddr ,regvalue;
	
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
     
	sscanf(buf, "%x %x", &regaddr, &regvalue);
	printk("apds9500_store_pswaddr,regaddr = 0x%x, regvalue = 0x%x\n",regaddr,regvalue);
	ret = i2c_smbus_write_byte_data(client, regaddr, regvalue);

	OP_MUTEX_UNLOCK;
 
	return count;
}

static ssize_t apds9500_store_psraddr(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int  regaddr ,regvalue;
	
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
	 
	sscanf(buf, "%x", &regaddr);
	//regvalue = i2c_smbus_read_word_data(client, regaddr);
	regvalue = i2c_smbus_read_byte_data(client, regaddr);

	OP_MUTEX_UNLOCK;

	printk("apds9500_store_psraddr,regaddr=0x%x,regvalue=0x%x\n", regaddr, regvalue);

	return count;
}

static ssize_t apds9500_show_vip_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned char reg_buf[64], i = 0;

	OP_MUTEX_LOCK;

	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xEF);

    SWITCH_TO_BANK_0; 
    reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x48);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x49);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x4a);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x4b);

	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6b); //0x6B for PS approach status
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6c); //0x6C for PS raw data

	SWITCH_TO_BANK_1;
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x32);

	SWITCH_TO_BANK_0;

	OP_MUTEX_UNLOCK;

	return sprintf(buf, "bank=0x%02X, exposure=0x%02X 0x%02X 0x%02X 0x%02X, ps_approach_status=0x%02X, ps_raw_data=0x%02X, current=0x%02X\n", 
		reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3], reg_buf[4], reg_buf[5], reg_buf[6], reg_buf[7]);
}

static ssize_t apds9500_show_current(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	u8 led_current;

	OP_MUTEX_LOCK;

	SWITCH_TO_BANK_1;
	led_current = i2c_smbus_read_byte_data(client, 0x32);
	SWITCH_TO_BANK_0;

	OP_MUTEX_UNLOCK;
	
	return sprintf(buf, "current=%x\n", led_current);
}

static ssize_t apds9500_store_current(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int regvalue;
	
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
	 
	sscanf(buf, "%x", &regvalue);
	printk("%s,regvalue = 0x%x\n", __func__, regvalue);
	SWITCH_TO_BANK_1;
	ret = i2c_smbus_write_byte_data(client, 0x32, regvalue);
	SWITCH_TO_BANK_0;

	OP_MUTEX_UNLOCK;
	
	return count;
}

static ssize_t apds9500_show_exposure(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	u8 ae_ub_L, ae_ub_H, ae_lb_L, ae_lb_H;

	OP_MUTEX_LOCK;

	SWITCH_TO_BANK_0;
	ae_ub_L = i2c_smbus_read_byte_data(client, 0x48);
	ae_ub_H = i2c_smbus_read_byte_data(client, 0x49);
	ae_lb_L = i2c_smbus_read_byte_data(client, 0x4A);
	ae_lb_H = i2c_smbus_read_byte_data(client, 0x4B);

	OP_MUTEX_UNLOCK;
	
	return sprintf(buf, "exposur=%02x %02x %02x %02x\n", ae_ub_L, ae_ub_H, ae_lb_L, ae_lb_H);
}

static ssize_t apds9500_store_exposure(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	int  ae_exposure_ub  = simple_strtoul(buf, NULL, 0);
	int ae_ub_L    =   ae_exposure_ub & 0xff;
	int ae_ub_H    =   (ae_exposure_ub >> 8) & 0xff;

	int ae_exposure_lb = ae_exposure_ub / 2;

	int ae_lb_L    =   ae_exposure_lb & 0xff;
	int ae_lb_H   =   (ae_exposure_lb >> 8) & 0xff;

	printk("%s: store ps senosr ae_exposure_ub = 0x%x ,ae_ub_L = 0x%x,ae_ub_H = 0x%x \n", __func__, ae_exposure_ub,ae_ub_L,ae_ub_H);
	printk("%s: store ps senosr ae_exposure_lb = 0x%x ,ae_lb_L = 0x%x,ae_lb_H = 0x%x \n", __func__, ae_exposure_lb,ae_lb_L,ae_lb_H);

	OP_MUTEX_LOCK;

	SWITCH_TO_BANK_0;
	i2c_smbus_write_byte_data(client, 0x48, ae_ub_L);
	i2c_smbus_write_byte_data(client, 0x49, ae_ub_H);
	i2c_smbus_write_byte_data(client, 0x4A, ae_lb_L);
	i2c_smbus_write_byte_data(client, 0x4B, ae_lb_H);

	OP_MUTEX_UNLOCK;
		
	return count;
}

static ssize_t apds9500_show_ps_boundary(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	u8 upper_boundary, low_boundary;

	OP_MUTEX_LOCK;

	SWITCH_TO_BANK_0;
	upper_boundary = i2c_smbus_read_byte_data(client, 0x69);
	low_boundary = i2c_smbus_read_byte_data(client, 0x6a);

	OP_MUTEX_UNLOCK;
	
	return sprintf(buf, "ps_boundary=%02x %02x\n", upper_boundary, low_boundary);
}

static ssize_t apds9500_store_ps_boundary(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	//unsigned long val = simple_strtoul(buf, NULL, 10);
	int upper_boundary, low_boundary;

	OP_MUTEX_LOCK;
	
	sscanf(buf, "%x %x", &upper_boundary, &low_boundary);
	SWITCH_TO_BANK_0;
	i2c_smbus_write_byte_data(client, 0x69, upper_boundary);
	i2c_smbus_write_byte_data(client, 0x6a, low_boundary);
	
	OP_MUTEX_UNLOCK;
	
	printk("%s,upper_boundary=%d,low_boundary=%d\n", __func__, upper_boundary, low_boundary);
	return count;
}

static ssize_t apds9500_show_ps_data_timer(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "ps_data_timer %s\n", (data->ps_data_timer_enable ? "enable" : "disabled"));
}

static ssize_t apds9500_store_ps_data_timer(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	int enable;

	sscanf(buf, "%x", &enable);
	data->ps_data_timer_enable = enable;
	
	if(data->ps_data_timer_enable)
		//mod_timer(&data->ps_data_timer, jiffies + HZ);
		add_timer(&data->ps_data_timer);
	else
		del_timer(&data->ps_data_timer);
	
	printk("%s,ps_data_timer=%d\n", __func__, data->ps_data_timer_enable);
	return count;
}

static ssize_t apds9500_show_ps_data_work(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "ps_data_work %s\n", (data->ps_data_work_enable ? "enable" : "disabled"));
}

static ssize_t apds9500_store_ps_data_work(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	int enable;

	sscanf(buf, "%x", &enable);
	data->ps_data_work_enable = enable;
	
	if(data->mode == APDS9500_PROXIMITY_MODE && data->ps_data_work_enable)
	{
		cancel_delayed_work(&data->ps_data_work);
		queue_delayed_work(apds_workqueue, &data->ps_data_work, 500); //msecs_to_jiffies(1000)
	}
	
	printk("%s,ps_data_work=%d\n", __func__, data->ps_data_work_enable);
	return count;
}

static ssize_t apds9500_store_suspend(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
	apds9500_sensor_suspend(client);
	OP_MUTEX_UNLOCK;
	
	printk("%s\n", __func__);
	return count;
}

static ssize_t apds9500_store_resume(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
	apds9500_sensor_resume(client);
	OP_MUTEX_UNLOCK;
	
	printk("%s\n", __func__);
	return count;
}

static ssize_t apds9500_store_init(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	
	printk("%s\n", __func__);
	
	OP_MUTEX_LOCK;
	apds9500_sensor_suspend(client);
	apds9500_sensor_resume(client);

	if(data->mode == APDS9500_GESTURE_MODE)
		apds9500_init_device(client, APDS9500_GESTURE_MODE);
	else
		apds9500_init_device(client, APDS9500_PROXIMITY_MODE);	
	OP_MUTEX_UNLOCK;
	
	return count;
}

static ssize_t apds9500_show_gesture_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned char reg_buf[64], i = 0;

	OP_MUTEX_LOCK;
	
	SWITCH_TO_BANK_0; 
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x41);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x42);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x48);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x49);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x4a);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x4b);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x51);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x83);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x9f); //9

	SWITCH_TO_BANK_1;
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x00);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x01);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x02);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x03);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x04);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x32);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x41);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x43);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x65);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x66);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x67);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x68);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x69);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6a);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6b);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6c);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6d);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x6e);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x72);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0x74);

	OP_MUTEX_UNLOCK;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
		reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3], reg_buf[4], reg_buf[5], reg_buf[6], reg_buf[7], reg_buf[8], reg_buf[9],
		reg_buf[10], reg_buf[11], reg_buf[12], reg_buf[13], reg_buf[14], reg_buf[15], reg_buf[16], reg_buf[17], reg_buf[18], reg_buf[19],
		reg_buf[20], reg_buf[21], reg_buf[22], reg_buf[23], reg_buf[24], reg_buf[25], reg_buf[26], reg_buf[27], reg_buf[28], reg_buf[29]);
}

static ssize_t apds9500_show_gesture_noise(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned char reg_buf[10], i = 0;

	OP_MUTEX_LOCK;
	
	SWITCH_TO_BANK_0; 
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xac);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xad);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xae);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xaf);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xb0);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xb1);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xb2);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xb3);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xb4);
	reg_buf[i++] = i2c_smbus_read_byte_data(client, 0xb5);

	OP_MUTEX_UNLOCK;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
		reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3], reg_buf[4], reg_buf[5], reg_buf[6], reg_buf[7], reg_buf[8], reg_buf[9]);
}

static ssize_t apds9500_store_wave(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
	
	SWITCH_TO_BANK_0; 
	if(!strncmp(buf, "0", 1))
	{
		i2c_smbus_write_byte_data(client, 0x9f, 0xc9);
		i2c_smbus_write_byte_data(client, 0x41, 0x0f);
		i2c_smbus_write_byte_data(client, 0x42, 0x00);
		i2c_smbus_write_byte_data(client, 0xd2, 0x08);
	}
	else
	{
		i2c_smbus_write_byte_data(client, 0x9f, 0xf9);
		i2c_smbus_write_byte_data(client, 0x41, 0xef);
		i2c_smbus_write_byte_data(client, 0x42, 0x01);
		i2c_smbus_write_byte_data(client, 0xd2, 0x88);
	}

	OP_MUTEX_UNLOCK

	return count;
}

static ssize_t apds9500_show_interrupt_event(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	int reg_43;
	u8 reg_6b, reg_b6;

	OP_MUTEX_LOCK;
	
	SWITCH_TO_BANK_0;
	reg_43 = i2c_smbus_read_word_data(client, 0x43);
	reg_6b = i2c_smbus_read_byte_data(client, 0x6B);
	reg_b6 = i2c_smbus_read_byte_data(client, 0xb6);	

	OP_MUTEX_UNLOCK;

	return sprintf(buf, "%s,reg_43=%04x,reg_6b=%02x,reg_b6=%02x\n", __func__, reg_43, reg_6b, reg_b6);
}

static ssize_t apds9500_store_soft_reset(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);

	OP_MUTEX_LOCK;
	
	SWITCH_TO_BANK_0; 
	i2c_smbus_write_byte_data(client, 0xEE, 0x03);
	i2c_smbus_write_byte_data(client, 0xEE, 0x07);

	OP_MUTEX_UNLOCK

	return count;
}

static ssize_t apds9500_show_bank0(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned char reg_buf_0[0x100];
	int i = 0;

	OP_MUTEX_LOCK;
	
	SWITCH_TO_BANK_0;
	for(i = 0; i <= 0xff; i++)
		reg_buf_0[i] = i2c_smbus_read_byte_data(client, i);

	OP_MUTEX_UNLOCK;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", 
		reg_buf_0[0], reg_buf_0[1], reg_buf_0[2], reg_buf_0[3], reg_buf_0[4], reg_buf_0[5], reg_buf_0[6], reg_buf_0[7], reg_buf_0[8], reg_buf_0[9],
		reg_buf_0[10], reg_buf_0[11], reg_buf_0[12], reg_buf_0[13], reg_buf_0[14], reg_buf_0[15], reg_buf_0[16], reg_buf_0[17], reg_buf_0[18], reg_buf_0[19],
		reg_buf_0[20], reg_buf_0[21], reg_buf_0[22], reg_buf_0[23], reg_buf_0[24], reg_buf_0[25], reg_buf_0[26], reg_buf_0[27], reg_buf_0[28], reg_buf_0[29],
		reg_buf_0[30], reg_buf_0[31], reg_buf_0[32], reg_buf_0[33], reg_buf_0[34], reg_buf_0[35], reg_buf_0[36], reg_buf_0[37], reg_buf_0[38], reg_buf_0[39],
		reg_buf_0[40], reg_buf_0[41], reg_buf_0[42], reg_buf_0[43], reg_buf_0[44], reg_buf_0[45], reg_buf_0[46], reg_buf_0[47], reg_buf_0[48], reg_buf_0[49],
		reg_buf_0[50], reg_buf_0[51], reg_buf_0[52], reg_buf_0[53], reg_buf_0[54], reg_buf_0[55], reg_buf_0[56], reg_buf_0[57], reg_buf_0[58], reg_buf_0[59],
		reg_buf_0[60], reg_buf_0[61], reg_buf_0[62], reg_buf_0[63], reg_buf_0[64], reg_buf_0[65], reg_buf_0[66], reg_buf_0[67], reg_buf_0[68], reg_buf_0[69],
		reg_buf_0[70], reg_buf_0[71], reg_buf_0[72], reg_buf_0[73], reg_buf_0[74], reg_buf_0[75], reg_buf_0[76], reg_buf_0[77], reg_buf_0[78], reg_buf_0[79],
		reg_buf_0[80], reg_buf_0[81], reg_buf_0[82], reg_buf_0[83], reg_buf_0[84], reg_buf_0[85], reg_buf_0[86], reg_buf_0[87], reg_buf_0[88], reg_buf_0[89],
		reg_buf_0[90], reg_buf_0[91], reg_buf_0[92], reg_buf_0[93], reg_buf_0[94], reg_buf_0[95], reg_buf_0[96], reg_buf_0[97], reg_buf_0[98], reg_buf_0[99],
		reg_buf_0[100], reg_buf_0[101], reg_buf_0[102], reg_buf_0[103], reg_buf_0[104], reg_buf_0[105], reg_buf_0[106], reg_buf_0[107], reg_buf_0[108], reg_buf_0[109],
		reg_buf_0[110], reg_buf_0[111], reg_buf_0[112], reg_buf_0[113], reg_buf_0[114], reg_buf_0[115], reg_buf_0[116], reg_buf_0[117], reg_buf_0[118], reg_buf_0[119],
		reg_buf_0[120], reg_buf_0[121], reg_buf_0[122], reg_buf_0[123], reg_buf_0[124], reg_buf_0[125], reg_buf_0[126], reg_buf_0[127], reg_buf_0[128], reg_buf_0[129],
		reg_buf_0[130], reg_buf_0[131], reg_buf_0[132], reg_buf_0[133], reg_buf_0[134], reg_buf_0[135], reg_buf_0[136], reg_buf_0[137], reg_buf_0[138], reg_buf_0[139],
		reg_buf_0[140], reg_buf_0[141], reg_buf_0[142], reg_buf_0[143], reg_buf_0[144], reg_buf_0[145], reg_buf_0[146], reg_buf_0[147], reg_buf_0[148], reg_buf_0[149],
		reg_buf_0[150], reg_buf_0[151], reg_buf_0[152], reg_buf_0[153], reg_buf_0[154], reg_buf_0[155], reg_buf_0[156], reg_buf_0[157], reg_buf_0[158], reg_buf_0[159],
		reg_buf_0[160], reg_buf_0[161], reg_buf_0[162], reg_buf_0[163], reg_buf_0[164], reg_buf_0[165], reg_buf_0[166], reg_buf_0[167], reg_buf_0[168], reg_buf_0[169],
		reg_buf_0[170], reg_buf_0[171], reg_buf_0[172], reg_buf_0[173], reg_buf_0[174], reg_buf_0[175], reg_buf_0[176], reg_buf_0[177], reg_buf_0[178], reg_buf_0[179],
		reg_buf_0[180], reg_buf_0[181], reg_buf_0[182], reg_buf_0[183], reg_buf_0[184], reg_buf_0[185], reg_buf_0[186], reg_buf_0[187], reg_buf_0[188], reg_buf_0[189],
		reg_buf_0[190], reg_buf_0[191], reg_buf_0[192], reg_buf_0[193], reg_buf_0[194], reg_buf_0[195], reg_buf_0[196], reg_buf_0[197], reg_buf_0[198], reg_buf_0[199],
		reg_buf_0[200], reg_buf_0[201], reg_buf_0[202], reg_buf_0[203], reg_buf_0[204], reg_buf_0[205], reg_buf_0[206], reg_buf_0[207], reg_buf_0[208], reg_buf_0[209],
		reg_buf_0[210], reg_buf_0[211], reg_buf_0[212], reg_buf_0[213], reg_buf_0[214], reg_buf_0[215], reg_buf_0[216], reg_buf_0[217], reg_buf_0[218], reg_buf_0[219],
		reg_buf_0[220], reg_buf_0[221], reg_buf_0[222], reg_buf_0[223], reg_buf_0[224], reg_buf_0[225], reg_buf_0[226], reg_buf_0[227], reg_buf_0[228], reg_buf_0[229],
		reg_buf_0[230], reg_buf_0[231], reg_buf_0[232], reg_buf_0[233], reg_buf_0[234], reg_buf_0[235], reg_buf_0[236], reg_buf_0[237], reg_buf_0[238], reg_buf_0[239],
		reg_buf_0[240], reg_buf_0[241], reg_buf_0[242], reg_buf_0[243], reg_buf_0[244], reg_buf_0[245], reg_buf_0[246], reg_buf_0[247], reg_buf_0[248], reg_buf_0[249],
		reg_buf_0[250], reg_buf_0[251], reg_buf_0[252], reg_buf_0[253], reg_buf_0[254], reg_buf_0[255]);
}

static ssize_t apds9500_show_bank1(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9500_data *data = i2c_get_clientdata(client);
	unsigned char reg_buf_1[0x100];
	int i = 0;

	OP_MUTEX_LOCK;

	SWITCH_TO_BANK_1;
	for(i = 0; i <= 0xff; i++)
		reg_buf_1[i] = i2c_smbus_read_byte_data(client, i);

	OP_MUTEX_UNLOCK;

	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", 
		reg_buf_1[0], reg_buf_1[1], reg_buf_1[2], reg_buf_1[3], reg_buf_1[4], reg_buf_1[5], reg_buf_1[6], reg_buf_1[7], reg_buf_1[8], reg_buf_1[9],
		reg_buf_1[10], reg_buf_1[11], reg_buf_1[12], reg_buf_1[13], reg_buf_1[14], reg_buf_1[15], reg_buf_1[16], reg_buf_1[17], reg_buf_1[18], reg_buf_1[19],
		reg_buf_1[20], reg_buf_1[21], reg_buf_1[22], reg_buf_1[23], reg_buf_1[24], reg_buf_1[25], reg_buf_1[26], reg_buf_1[27], reg_buf_1[28], reg_buf_1[29],
		reg_buf_1[30], reg_buf_1[31], reg_buf_1[32], reg_buf_1[33], reg_buf_1[34], reg_buf_1[35], reg_buf_1[36], reg_buf_1[37], reg_buf_1[38], reg_buf_1[39],
		reg_buf_1[40], reg_buf_1[41], reg_buf_1[42], reg_buf_1[43], reg_buf_1[44], reg_buf_1[45], reg_buf_1[46], reg_buf_1[47], reg_buf_1[48], reg_buf_1[49],
		reg_buf_1[50], reg_buf_1[51], reg_buf_1[52], reg_buf_1[53], reg_buf_1[54], reg_buf_1[55], reg_buf_1[56], reg_buf_1[57], reg_buf_1[58], reg_buf_1[59],
		reg_buf_1[60], reg_buf_1[61], reg_buf_1[62], reg_buf_1[63], reg_buf_1[64], reg_buf_1[65], reg_buf_1[66], reg_buf_1[67], reg_buf_1[68], reg_buf_1[69],
		reg_buf_1[70], reg_buf_1[71], reg_buf_1[72], reg_buf_1[73], reg_buf_1[74], reg_buf_1[75], reg_buf_1[76], reg_buf_1[77], reg_buf_1[78], reg_buf_1[79],
		reg_buf_1[80], reg_buf_1[81], reg_buf_1[82], reg_buf_1[83], reg_buf_1[84], reg_buf_1[85], reg_buf_1[86], reg_buf_1[87], reg_buf_1[88], reg_buf_1[89],
		reg_buf_1[90], reg_buf_1[91], reg_buf_1[92], reg_buf_1[93], reg_buf_1[94], reg_buf_1[95], reg_buf_1[96], reg_buf_1[97], reg_buf_1[98], reg_buf_1[99],
		reg_buf_1[100], reg_buf_1[101], reg_buf_1[102], reg_buf_1[103], reg_buf_1[104], reg_buf_1[105], reg_buf_1[106], reg_buf_1[107], reg_buf_1[108], reg_buf_1[109],
		reg_buf_1[110], reg_buf_1[111], reg_buf_1[112], reg_buf_1[113], reg_buf_1[114], reg_buf_1[115], reg_buf_1[116], reg_buf_1[117], reg_buf_1[118], reg_buf_1[119],
		reg_buf_1[120], reg_buf_1[121], reg_buf_1[122], reg_buf_1[123], reg_buf_1[124], reg_buf_1[125], reg_buf_1[126], reg_buf_1[127], reg_buf_1[128], reg_buf_1[129],
		reg_buf_1[130], reg_buf_1[131], reg_buf_1[132], reg_buf_1[133], reg_buf_1[134], reg_buf_1[135], reg_buf_1[136], reg_buf_1[137], reg_buf_1[138], reg_buf_1[139],
		reg_buf_1[140], reg_buf_1[141], reg_buf_1[142], reg_buf_1[143], reg_buf_1[144], reg_buf_1[145], reg_buf_1[146], reg_buf_1[147], reg_buf_1[148], reg_buf_1[149],
		reg_buf_1[150], reg_buf_1[151], reg_buf_1[152], reg_buf_1[153], reg_buf_1[154], reg_buf_1[155], reg_buf_1[156], reg_buf_1[157], reg_buf_1[158], reg_buf_1[159],
		reg_buf_1[160], reg_buf_1[161], reg_buf_1[162], reg_buf_1[163], reg_buf_1[164], reg_buf_1[165], reg_buf_1[166], reg_buf_1[167], reg_buf_1[168], reg_buf_1[169],
		reg_buf_1[170], reg_buf_1[171], reg_buf_1[172], reg_buf_1[173], reg_buf_1[174], reg_buf_1[175], reg_buf_1[176], reg_buf_1[177], reg_buf_1[178], reg_buf_1[179],
		reg_buf_1[180], reg_buf_1[181], reg_buf_1[182], reg_buf_1[183], reg_buf_1[184], reg_buf_1[185], reg_buf_1[186], reg_buf_1[187], reg_buf_1[188], reg_buf_1[189],
		reg_buf_1[190], reg_buf_1[191], reg_buf_1[192], reg_buf_1[193], reg_buf_1[194], reg_buf_1[195], reg_buf_1[196], reg_buf_1[197], reg_buf_1[198], reg_buf_1[199],
		reg_buf_1[200], reg_buf_1[201], reg_buf_1[202], reg_buf_1[203], reg_buf_1[204], reg_buf_1[205], reg_buf_1[206], reg_buf_1[207], reg_buf_1[208], reg_buf_1[209],
		reg_buf_1[210], reg_buf_1[211], reg_buf_1[212], reg_buf_1[213], reg_buf_1[214], reg_buf_1[215], reg_buf_1[216], reg_buf_1[217], reg_buf_1[218], reg_buf_1[219],
		reg_buf_1[220], reg_buf_1[221], reg_buf_1[222], reg_buf_1[223], reg_buf_1[224], reg_buf_1[225], reg_buf_1[226], reg_buf_1[227], reg_buf_1[228], reg_buf_1[229],
		reg_buf_1[230], reg_buf_1[231], reg_buf_1[232], reg_buf_1[233], reg_buf_1[234], reg_buf_1[235], reg_buf_1[236], reg_buf_1[237], reg_buf_1[238], reg_buf_1[239],
		reg_buf_1[240], reg_buf_1[241], reg_buf_1[242], reg_buf_1[243], reg_buf_1[244], reg_buf_1[245], reg_buf_1[246], reg_buf_1[247], reg_buf_1[248], reg_buf_1[249],
		reg_buf_1[250], reg_buf_1[251], reg_buf_1[252], reg_buf_1[253], reg_buf_1[254], reg_buf_1[255]);
}

static DEVICE_ATTR(suspend, 0644, NULL, apds9500_store_suspend);

static DEVICE_ATTR(resume, 0644, NULL, apds9500_store_resume);

static DEVICE_ATTR(init, 0644, NULL, apds9500_store_init);

static DEVICE_ATTR(proximity_enable, 0644,apds9500_show_proximity_enable, apds9500_store_proximity_enable);

static DEVICE_ATTR(gesture_enable, 0644,apds9500_show_gesture_enable, apds9500_store_gesture_enable);

static DEVICE_ATTR(workmode, 0644,apds9500_show_mode, apds9500_store_mode);

static DEVICE_ATTR(pswaddr, 0644, NULL, apds9500_store_pswaddr);

static DEVICE_ATTR(psraddr, 0644, NULL, apds9500_store_psraddr);

static DEVICE_ATTR(showvip, 0644, apds9500_show_vip_reg, NULL);

static DEVICE_ATTR(led_current, 0644, apds9500_show_current, apds9500_store_current);

static DEVICE_ATTR(exposure, 0644, apds9500_show_exposure, apds9500_store_exposure);

static DEVICE_ATTR(ps_boundary, 0644, apds9500_show_ps_boundary, apds9500_store_ps_boundary);

static DEVICE_ATTR(ps_data_timer, 0644, apds9500_show_ps_data_timer, apds9500_store_ps_data_timer);

static DEVICE_ATTR(ps_data_work, 0644, apds9500_show_ps_data_work, apds9500_store_ps_data_work);

static DEVICE_ATTR(showgesturereg, 0644, apds9500_show_gesture_reg, NULL);

static DEVICE_ATTR(showgesturenoise, 0644, apds9500_show_gesture_noise, NULL);

static DEVICE_ATTR(wave, 0644, NULL, apds9500_store_wave);

static DEVICE_ATTR(interrupt_event, 0644, apds9500_show_interrupt_event, NULL);

static DEVICE_ATTR(soft_reset, 0644, NULL, apds9500_store_soft_reset);

static DEVICE_ATTR(showbank0, 0644, apds9500_show_bank0, NULL);

static DEVICE_ATTR(showbank1, 0644, apds9500_show_bank1, NULL);

static struct attribute *apds9500_attributes[] = {
	&dev_attr_proximity_enable.attr,
	&dev_attr_gesture_enable.attr,
	&dev_attr_workmode.attr,
	&dev_attr_pswaddr.attr,
	&dev_attr_psraddr.attr,
	&dev_attr_showvip.attr,
	&dev_attr_led_current.attr,
	&dev_attr_exposure.attr,
	&dev_attr_ps_boundary.attr,
	&dev_attr_ps_data_timer.attr,
	&dev_attr_ps_data_work.attr,

	&dev_attr_suspend.attr,
	&dev_attr_resume.attr,
	&dev_attr_init.attr,
	
#ifdef SUPPORT_REMOVE_SLEEP
	&dev_attr_remove_sleep_timer.attr,
#endif

	&dev_attr_showgesturereg.attr,
	&dev_attr_showgesturenoise.attr,
	&dev_attr_wave.attr,
	&dev_attr_interrupt_event.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_showbank0.attr,
	&dev_attr_showbank1.attr,

#ifdef SUPPORT_POWER_DOWN_RESET
	&dev_attr_power_3v3.attr,
	&dev_attr_power_3v3_level.attr,
#endif

	NULL
};

static const struct attribute_group apds9500_attr_group = {
	.attrs = apds9500_attributes,
};

static void apds9500_ps_data_work_handler(struct work_struct *work)
{
	struct apds9500_data *data = container_of(work, struct apds9500_data, ps_data_work.work);
	struct i2c_client *client = data->client;
	unsigned char reg_buf[4];
	
    printk("%s,data=%p,client=%p,apds9500_i2c_client=%p\n", __func__, data, client, apds9500_i2c_client);
	if(data->mode == APDS9500_PROXIMITY_MODE && data->ps_data_work_enable)
	{
		//SWITCH_TO_BANK_0;
		i2c_smbus_write_byte_data(client, 0xEF, 0x00);
		
		reg_buf[0] = i2c_smbus_read_byte_data(client, 0x6b); //0x6B for PS approach status
		reg_buf[1] = i2c_smbus_read_byte_data(client, 0x6c); //0x6C for PS raw data

		reg_buf[2] = i2c_smbus_read_byte_data(client, 0x69);
		reg_buf[3] = i2c_smbus_read_byte_data(client, 0x6a);

		printk("ps_approach_status=0x%02X, ps_raw_data=0x%02X, 0x%02X 0x%02X\n", reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3]);

		cancel_delayed_work(&data->ps_data_work);
		queue_delayed_work(apds_workqueue, &data->ps_data_work, 500);
	}
}

static void ps_data_timer_function(unsigned long data)
{	
	struct apds9500_data *local_data = (struct apds9500_data *)data;
	struct i2c_client *client = local_data->client;
	unsigned char reg_buf[4];
	printk("%s,data=%p,client=%p,apds9500_i2c_client=%p\n", __func__, local_data, client, apds9500_i2c_client);

	if(local_data->mode == APDS9500_PROXIMITY_MODE && local_data->ps_data_timer_enable)
	{
		//SWITCH_TO_BANK_0;
		i2c_smbus_write_byte_data(client, 0xEF, 0x00);
		
		reg_buf[0] = i2c_smbus_read_byte_data(client, 0x6b); //0x6B for PS approach status
		reg_buf[1] = i2c_smbus_read_byte_data(client, 0x6c); //0x6C for PS raw data

		reg_buf[2] = i2c_smbus_read_byte_data(client, 0x69);
		reg_buf[3] = i2c_smbus_read_byte_data(client, 0x6a);

		printk("ps_approach_status=0x%02X, ps_raw_data=0x%02X, 0x%02X 0x%02X\n", reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3]);

		//mod_timer(&local_data->ps_data_timer, jiffies + HZ / 2);
	}
}

static int apds9500_open(struct inode *inode, struct file *file)
{
//	printk("%s\n", __func__);
	return 0; 
}

static int apds9500_release(struct inode *inode, struct file *file)
{
//	printk("%s\n", __func__);
	return 0;
}

static long apds9500_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct apds9500_data *data;
	struct i2c_client *client;
	struct apds9500_ps_mode_data psdata;
	int enable;
	int ret = -1;
	
	
	if (arg == 0) return -1;

	if(apds9500_i2c_client == NULL) {
		printk("apds9500_ps_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

	client = apds9500_i2c_client;	
	data = i2c_get_clientdata(apds9500_i2c_client);

	OP_MUTEX_LOCK;

	switch (cmd) {
		case APDS_IOCTL_PS_ENABLE:				
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9500_ps_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}

			//ret = apds9500_enable_ps_sensor(client, enable);	
			ret = apds9500_enable_proximity_sensor(client, enable);
			if(ret < 0) {
				return ret;
			}
			break;

		case APDS_IOCTL_PS_GET_ENABLE:
			if (copy_to_user((void __user *)arg, &data->enable_proximity_sensor, sizeof(data->enable_proximity_sensor))) {
				printk("apds9500_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case APDS_IOCTL_GESTURE_ENABLE:				
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9500_ps_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}

			//ret = apds9500_enable_ps_sensor(client, enable);	
			ret = apds9500_enable_gesture_sensor(client, enable);
			if(ret < 0) {
				return ret;
			}
			break;

		case APDS_IOCTL_GESTURE_GET_ENABLE:
			if (copy_to_user((void __user *)arg, &data->enable_gesture_sensor, sizeof(data->enable_gesture_sensor))) {
				printk("apds9500_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;
	
		case APDS_IOCTL_PS_GET_APPAOACH_DATA:
			SWITCH_TO_BANK_0;
			psdata.approach  =	i2c_smbus_read_word_data(client, 0x6b);
			psdata.rawdata	= i2c_smbus_read_word_data(client, 0x6c);
			if (copy_to_user((void __user *)arg, &psdata, sizeof(struct apds9500_ps_mode_data))) {
				printk(KERN_ERR"apds9500_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case APDS_IOCTL_GESTURE_GET_DATA:
			SWITCH_TO_BANK_0;
			data->ps_data = i2c_smbus_read_word_data(client, 0x43);
			if (copy_to_user((void __user *)arg, &data->ps_data, sizeof(data->ps_data))) {
				printk("apds9500_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case APDS_IOCTL_CHANGE2PSMODE:
			if(data->mode != APDS9500_PROXIMITY_MODE)
			{
				//apds9500_sensor_suspend(client);
				//apds9500_sensor_resume(client);
				ret = apds9500_init_device(client, APDS9500_PROXIMITY_MODE);
				if(ret != 0){
					printk("%s init psmode failed %d\n", __FUNCTION__,ret);
				}
				apds9500_read_and_clear_interrupt(client);
			}
			
#ifdef DO_PROCESSING_WHEN_CHANGE2PS_MODE
			ret = apds9500_proximity_processing(client);  
			ps_report_interrupt_data(ret);
#endif
			break;

		case APDS_IOCTL_CHANGE2GSMODE:
			if(data->mode != APDS9500_GESTURE_MODE)
			{
				//apds9500_sensor_suspend(client);
				//apds9500_sensor_resume(client);
				ret = apds9500_init_device(client, APDS9500_GESTURE_MODE);
				if(ret != 0){
					printk("%s init gsmode failed %d\n", __FUNCTION__,ret);
				}
				apds9500_read_and_clear_interrupt(client);
			}
			break;

		default:
			printk("default branch, do nothing \n");
			break;
	}

	OP_MUTEX_UNLOCK;
	
	return 0;
}

static struct file_operations apds9500_fops = {
	.owner = THIS_MODULE,
	.open = apds9500_open,
	.release = apds9500_release,
	.unlocked_ioctl = apds9500_ioctl,
};

static struct miscdevice apds9500_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "apds_ps_dev", //"apds_dev"
	.fops = &apds9500_fops,
};


static int ps_open_report_data(int open)
{
	return 0;
}

static int ps_enable_nodata(int en)
{
    if(en){
        printk("in func %s init ps mode \n",__FUNCTION__);
        //apds9500_init_device(apds9500_i2c_client, APDS9500_PROXIMITY_MODE);
    }
    else
    {   
        
        printk("in func %s init gs mode \n",__FUNCTION__);
        //apds9500_init_device(apds9500_i2c_client, APDS9500_GESTURE_MODE);
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

    if(apds9500_i2c_client == NULL)
    {
        printk(" in func %s i2c client is null \n",__FUNCTION__);
        return -1;
    }
    else{
        *value = apds9500_proximity_processing(apds9500_i2c_client);      
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver apds9500_driver;
static struct pinctrl *apds9500_pinctrl;

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

static int apds9500_init_client(struct i2c_client *client)
{
	//int err;
	int i = 0;

	while (i++ < 20) {
		if (i2c_smbus_read_byte_data(client, 0x00) >= 0) {
			break;
		}
		mdelay(1);
	}
	printk("%s,i=%d,%s\n", __func__, i, (20 == i) ? "fail" : "success");
	
	//err = i2c_smbus_write_byte_data(client, 0xEF, 0x00);
	//if (err < 0) return err;
	
	return 0;
}

static int apds9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);  // get client info
	struct apds9500_data *data;

    struct pinctrl_state *irq_pullup;
	
    /* maybe is alsps need info */
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };
    /* end */    
	int err = 0;
	
	printk("==mlk==enter new %s \n",__func__);

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

	mutex_init(&data->op_mutex);

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

	// for gs 
	INIT_DELAYED_WORK(&data->dwork, apds9500_work_handler);
	//  for ps
	INIT_DELAYED_WORK(&data->ps_dwork, apds9500_ps_work_handler); 

	printk("%s interrupt is hooked\n", __func__);

#ifdef SUPPORT_POWER_DOWN_RESET
	apds9500_get_3v3_supply(&client->dev);
	apds9500_3v3_supply_disable();
	msleep(50);
	apds9500_3v3_supply_enable();
#endif

	/* Initialize the APDS9500 chip */
	msleep(10); // T1 > 700us
	err = apds9500_init_client(client);
	if (err){
		printk("%s,init client fail\n", __func__);
       	goto exit_free_dev_ps;
	}

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
    printk(KERN_ERR"in func %s line %d\n",__FUNCTION__,__LINE__);


	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9500_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	/* Register for sensor ioctl */
    err = misc_register(&apds9500_device);
	if (err) {
		goto exit_remove_sysfs_group;
	}
        
	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
    
	ps_ctl.is_use_common_factory = false;
	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;  // enable or disable ps sensor
	ps_ctl.set_delay = ps_set_delay;
	ps_ctl.batch = ps_batch;   
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = true;
	ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		printk("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;  // return value 
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		printk("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	
	init_timer(&data->ps_data_timer);	
	data->ps_data_timer.data = (unsigned long)data;	
	data->ps_data_timer.expires = jiffies + HZ;  	
	data->ps_data_timer.function = ps_data_timer_function;
	data->ps_data_timer_enable = 0;
	//add_timer(&data->ps_data_timer);
	
	INIT_DELAYED_WORK(&data->ps_data_work, apds9500_ps_data_work_handler);
	data->ps_data_work_enable = 0;
	printk("%s,data=0x%p,client=0x%p,apds9500_i2c_client=0x%p\n", __func__, (void*)data, (void*)client, apds9500_i2c_client);

	msleep(2); // T2 > 400us
	apds9500_load_initial_setting(client);
	msleep(1);
	apds9500_init_device(client, APDS9500_GESTURE_MODE);
	msleep(1);
	apds9500_sensor_resume(client);
	msleep(1);

#ifdef SUPPORT_REMOVE_SLEEP
	data->remove_sleep_timer_enable = 1;
	data->remove_sleep_timer_count = 0;
	INIT_DELAYED_WORK(&data->remove_sleep_timer_work, apds9500_remove_sleep_timer_work_handler);
	cancel_delayed_work(&data->remove_sleep_timer_work);
	queue_delayed_work(apds_workqueue, &data->remove_sleep_timer_work, CHECK_SLEEP_TIME);
#endif

	return 0;

	misc_deregister(&apds9500_device);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &apds9500_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);	
exit_free_dev_ps:
	//free_irq(APDS9500_INT, client);	
	free_irq(data->irq, client);	
exit_sensor_obj_attach_fail:
    
exit_kfree:
	kfree(data);
exit:
	return err;

}

static int apds9500_remove(struct i2c_client *client)
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

	misc_deregister(&apds9500_device);	

	sysfs_remove_group(&client->dev.kobj, &apds9500_attr_group);

	input_unregister_device(data->input_dev_ps);
	
	//free_irq(APDS9500_INT, client);
	free_irq(data->irq, client);	
	kfree(data);

	return 0;
}

static int apds9500_local_init(void)
{
    printk("==mlk==enter %s\n",__func__);
	
    return i2c_add_driver(&apds9500_driver);
}

static int apds9500_local_remove(void)
{
    i2c_del_driver(&apds9500_driver);

    return 0;
}

/* MediaTek alsps information */
static struct alsps_init_info apds9500_init_info = {
    .name = APDS9500_DRV_NAME,                /* Alsps driver name */
    .init     = apds9500_local_init,          /* Initialize alsps driver */
    .uninit = apds9500_local_remove,          /* Uninitialize alsps driver */
};

static int __init apds9500_init(void)
{
    printk("apds95001216_als_init\n");

    apds_workqueue = create_workqueue("proximity_als");
        
    if (!apds_workqueue)
        return -ENOMEM;
    return alsps_driver_add(&apds9500_init_info);
}

static void __exit apds9500_exit(void)
{
    return;
}

MODULE_AUTHOR("Adam Su");
MODULE_DESCRIPTION("Tonly apds9500 Sensor Driver");
MODULE_LICENSE("GPL");

module_init(apds9500_init);
module_exit(apds9500_exit);
