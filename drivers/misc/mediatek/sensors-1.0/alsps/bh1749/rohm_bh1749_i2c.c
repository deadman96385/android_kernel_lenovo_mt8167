/******************************************************************************
 * MODULE       : rohm_bh1749_i2c.c
 * FUNCTION     : Driver source for BH1749, Ambient Light Sensor(RGB) IC
 * AUTHOR       : Aaron Liu
 * MODIFICATION : Modified by Aaron, Mar/15/2017
 * NOTICE       : This software had been verified using MT6797.
 *              : When you use this code and document, Please verify all
 *              : operation in your operating system.
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015-2017 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor,Boston, MA  02110-1301, USA.
 *****************************************************************************/
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/version.h>
#include "alsps.h"
#include "cust_alsps.h"
#include "rohm_bh1749_i2c.h"


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/hwmsensor.h>
#include <mach/eint.h>
#else
#include <linux/gpio.h>
#endif


/******************************* define *******************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
static struct alsps_hw *hw ;
//#else
/* Maintain alsps cust info here */
//static struct alsps_hw alsps_cust;
//static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;

/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
    return &alsps_cust;
}
#endif

const static int judge = 50000;
const static int lux_coef[] = {1011, 37610, 44972, 223, 29288, 12721};
#define BU27006_LUX_SCALE   (100000)


/* structure of peculiarity to use by system */
typedef struct {
    struct i2c_client       *client;            /* structure pointer for i2c bus */
    int                     use_irq;            /* flag of whether to use interrupt or not */
    u32                     tp_module_num;      /* TP module count */
    unsigned int            tp_module_id;       /*  Touch panel manufacture id */
    COLOR_T                 color_type;         /*  Touch panel color type */
    bool                    als_en_status;     
    bool                    als_suspend;     
} RGB_DATA;



/* logical functions */
static int __init           bh1749_als_init(void);
static void __exit          bh1749_als_exit(void);
static int                  bh1749_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  bh1749_remove(struct i2c_client *client);//static void                 bh1749_power(struct alsps_hw *hw, unsigned int on);
//static int                  bh1749_open_report_data(int open);
static int                  bh1749_enable_nodata(int en);
static int                  bh1749_set_delay(u64 delay);
static int                  bh1749_calculate_light(READ_DATA_ARG data);
static int                  bh1749_get_data(int *als_value, int *status);
static int                  bh1749_local_init(void);
static int                  bh1749_local_remove(void);
static COLOR_T              bh1749_get_color_type(void);
static unsigned int         bh1749_get_tp_module_id(void);
static int                  bh1749_register_dump(int addr, RGB_DATA * ptr);

/* access functions */
static int bh1749_driver_init(RGB_DATA *data_ptr);
static int bh1749_driver_shutdown(RGB_DATA *data_ptr);
static int bh1749_driver_reset(RGB_DATA *data_ptr);
static int bh1749_driver_write_power_on_off(RGB_DATA *data_ptr, unsigned char data);
static int bh1749_driver_read_data(RGB_DATA *data_ptr, void *data);
static int bh1749_suspend(struct device *dev);
static int bh1749_resume(struct device *dev);


/**************************** variable declaration ****************************/
static RGB_DATA *obj = NULL;


/**************************** structure declaration ****************************/

static const struct dev_pm_ops bh1749_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(bh1749_suspend, bh1749_resume)
};

/* I2C device IDs supported by this driver */
static const struct i2c_device_id bh1749_id[] = {
    { BH1749_I2C_NAME, 0 }, /* rohm bh1749 driver */
    { }
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,bh1749_i2c"},  /* For bh1749, this string should be the same with dts */
    {},
};
#endif

/* represent an I2C device driver */
static struct i2c_driver bh1749_driver = {
    .driver = {                     /* device driver model driver */
        .owner = THIS_MODULE,
        .name  = BH1749_I2C_NAME,
#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
#endif
        .pm = &bh1749_pm_ops,
    },
    .probe    = bh1749_probe,          /* callback for device binding */
    .remove   = bh1749_remove,         /* callback for device unbinding */
    .shutdown = NULL,
    .id_table = bh1749_id,             /* list of I2C devices supported by this driver */
};

/* MediaTek alsps information */
static struct alsps_init_info bh1749_init_info = {
    .name = BH1749_I2C_NAME,                /* Alsps driver name */
    .init     = bh1749_local_init,          /* Initialize alsps driver */
    .uninit = bh1749_local_remove,          /* Uninitialize alsps driver */
};



/*----------------------------------------------------------------------------*/

static int bh1749_check_reg_valid(unsigned char reg, RGB_DATA * ptr)
{
    unsigned char tmp_reg = reg;

    if(!ptr)
    {
        BH1749_ERR("Para Error!!!\n");
    }

    //check reg valid
    if((tmp_reg  != BH1749_REG_SYSTEMCONTROL  ) &&
            (tmp_reg != BH1749_REG_MODECONTROL1   ) &&
            (tmp_reg != BH1749_REG_MODECONTROL2   ) &&
            (tmp_reg != BH1749_REG_RED_DATA       ) &&
            (tmp_reg != BH1749_REG_GREEN_DATA     ) &&
            (tmp_reg != BH1749_REG_BLUE_DATA      ) &&
            (tmp_reg != BH1749_REG_IR_DATA        ) &&
            (tmp_reg != BH1749_REG_GREEN2_DATA    ) &&
            (tmp_reg != BH1749_REG_INTERRUPT      ) &&
            (tmp_reg != BH1749_REG_INT_PERSISTENCE) &&
            (tmp_reg != BH1749_REG_THRED_HIGH     ) &&
            (tmp_reg != BH1749_REG_THRED_LOW      ) &&
            (tmp_reg != BH1749_REG_MANUFACT_ID    )){
        BH1749_ERR("tmp_reg=0x%x is out of range !!!\n", tmp_reg );
        return -1;
    }

    return 0;
}

static ssize_t bh1749_show_debug(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    if(!obj)
    {
        len += snprintf(buf+len, PAGE_SIZE-len,"obj is null!!\n");
        return len;
    }

    len += snprintf(buf+len, PAGE_SIZE-len,
            "judge=%d, %d, %d, %d, %d, %d, %d\n",
            judge,lux_coef[0],lux_coef[1],lux_coef[2],lux_coef[3],lux_coef[4],lux_coef[5]);

    //For bh1749_store_debug
    len += snprintf(buf+len, PAGE_SIZE-len,
            "You can read/write a register just like the follow:\n        read:  echo \"r 0x40     \" > bh1749_debug\n        write: ehco \"w 0x40 0xFF\" > bh1749_debug\n        (Use dmesg to see kernel log)\n\n");

    len += snprintf(buf+len, PAGE_SIZE-len,
            "All reg value:\n 0x%02x 0x%02x\n", BH1749_REG_RESET, bh1749_register_dump(BH1749_REG_RESET, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_GAIN_TIME, bh1749_register_dump(BH1749_REG_GAIN_TIME, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_ENABLE, bh1749_register_dump(BH1749_REG_ENABLE, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_RED_DATA, bh1749_register_dump(BH1749_REG_RED_DATA, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_RED_DATA + 1, bh1749_register_dump(BH1749_REG_RED_DATA + 1, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_GREEN_DATA, bh1749_register_dump(BH1749_REG_GREEN_DATA, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_GREEN_DATA + 1, bh1749_register_dump(BH1749_REG_GREEN_DATA + 1, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_BLUE_DATA, bh1749_register_dump(BH1749_REG_BLUE_DATA, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_BLUE_DATA + 1, bh1749_register_dump(BH1749_REG_BLUE_DATA + 1, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_IR_DATA, bh1749_register_dump(BH1749_REG_IR_DATA, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_IR_DATA + 1, bh1749_register_dump(BH1749_REG_IR_DATA + 1, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_INTERRUPT, bh1749_register_dump(BH1749_REG_INTERRUPT, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_INT_PERSISTENCE, bh1749_register_dump(BH1749_REG_INT_PERSISTENCE, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_THRED_HIGH, bh1749_register_dump(BH1749_REG_THRED_HIGH, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_THRED_HIGH + 1, bh1749_register_dump(BH1749_REG_THRED_HIGH + 1, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_THRED_LOW, bh1749_register_dump(BH1749_REG_THRED_LOW, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_THRED_LOW + 1, bh1749_register_dump(BH1749_REG_THRED_LOW + 1, obj));

    len += snprintf(buf+len, PAGE_SIZE-len,
            " 0x%02x 0x%02x\n", BH1749_REG_MANUFACT_ID, bh1749_register_dump(BH1749_REG_MANUFACT_ID, obj));

    return len;
}


static ssize_t bh1749_store_debug(struct device_driver *ddri, const char *buf, size_t count)
{
#define MAX_LENGTH (3)

    int reg = 0, i2c_data = 0;
    int i = 0;
    int ret = 0;

    char * str_dest[MAX_LENGTH] = {0};
    char str_src[128];

    char delims[] = " ";
    char *str_result = NULL;
    char *cur_str = str_src;

    if(!obj)
    {
        BH1749_ERR("obj is null !!!\n");
        return 0;
    }

    memcpy(str_src, buf, count);
    BH1749_WARNING("Your input buf is: %s\n", str_src );

    //spilt buf by space(" "), and seperated string are saved in str_src[]
    while(( str_result = strsep( &cur_str, delims ))) {
        if( i < MAX_LENGTH)  //max length should be 3
        {
            str_dest[i++] = str_result;
        }
        else
        {
            //BH1749_WARNING("break\n");
            break;
        }
    }

    if (!strncmp(str_dest[0], "r", 1))
    {
        reg = simple_strtol(str_dest[1], NULL, 16);

        ret = bh1749_check_reg_valid(reg&0xFF, obj);
        if (ret < 0) {
            BH1749_ERR( " reg para error !!!  \n" );
            return -1;
        }
        //read i2c data
        bh1749_register_dump(reg&0xFF, obj);
    }
    else if (!strncmp(str_dest[0], "w",  1))
    {
        reg      = simple_strtol(str_dest[1], NULL, 16);
        i2c_data = simple_strtol(str_dest[2], NULL, 16);

        //check reg valid
        ret = bh1749_check_reg_valid(reg&0xFF, obj);
        if (ret < 0) {
            BH1749_ERR( " reg para error !!!  \n" );
            return -1;
        }

        //write i2c data
        ret = i2c_smbus_write_byte_data(obj->client, reg&0xFF, i2c_data&0xFF);
        if (ret < 0) {
            BH1749_ERR( " I2C read error !!!  \n" );
            return -1;
        }
        BH1749_WARNING("writing...reg=0x%x, i2c_data=0x%x success\n", reg, i2c_data);
    }
    else
    {
        BH1749_ERR("Please input right format: \"r 0x40\", \"w 0x40 0xFF\"  \"para \" \n");
    }

    BH1749_WARNING( "bh1749_store_debug count=%d\n", (int)count);

    return count;
}

static ssize_t bh1749_show_red(struct device_driver *ddri, char *buf)
{	
	u8  raw_data[8] = {0};	
	int      result = 0;	
	unsigned int red;    	
	unsigned int green;          	
	unsigned int blue;    	
		
	result = i2c_smbus_read_i2c_block_data(obj->client, BH1749_REG_RED_DATA, sizeof(raw_data), raw_data);		

	red     = raw_data[0] | (raw_data[1] << 8);	
	green  = raw_data[2] | (raw_data[3] << 8);	
	blue    = raw_data[4] | (raw_data[5] << 8);	
			
	return sprintf(buf,"%d\n",red);
}


static ssize_t bh1749_show_green(struct device_driver *ddri, char *buf)
{	
	u8  raw_data[8] = {0};	
	int      result = 0;		
	unsigned int red;    	
	unsigned int green;          	
	unsigned int blue;     	
			
		
	result = i2c_smbus_read_i2c_block_data(obj->client, BH1749_REG_RED_DATA, sizeof(raw_data), raw_data);		

	red     = raw_data[0] | (raw_data[1] << 8);	
	green  = raw_data[2] | (raw_data[3] << 8);	
	blue    = raw_data[4] | (raw_data[5] << 8);	
			
	return sprintf(buf,"%d\n",green);
}


static ssize_t bh1749_show_blue(struct device_driver *ddri, char *buf)
{	
	u8  raw_data[8] = {0};	
	int      result = 0;	
	unsigned int red;    	
	unsigned int green;          	
	unsigned int blue;          	

	result = i2c_smbus_read_i2c_block_data(obj->client, BH1749_REG_RED_DATA, sizeof(raw_data), raw_data);		

	red     = raw_data[0] | (raw_data[1] << 8);	
	green  = raw_data[2] | (raw_data[3] << 8);	
	blue    = raw_data[4] | (raw_data[5] << 8);	
			
	return sprintf(buf,"%d\n",blue);
}

static ssize_t bh1749_show_rgb(struct device_driver *ddri, char *buf)
{	
	u8  raw_data[8] = {0};	
	u8  ir_data[2] = {0};	
	int      result = 0;	
	unsigned int red;    	
	unsigned int green;          	
	unsigned int blue;   
	unsigned int ir;  	

	result = i2c_smbus_read_i2c_block_data(obj->client, BH1749_REG_RED_DATA, sizeof(raw_data), raw_data);		

	red     = raw_data[0] | (raw_data[1] << 8);	
	green  = raw_data[2] | (raw_data[3] << 8);	
	blue    = raw_data[4] | (raw_data[5] << 8);	    
    
    /* read ir */
    result = i2c_smbus_read_i2c_block_data(obj->client, BH1749_REG_IR_DATA, 2 , ir_data);
    if (result < 0) {
        BH1749_ERR( "bh1749_driver_read_data : transfer error \n");
        return result;
    } else {
        ir = ir_data[0] | (ir_data[1] << 8);
        result = 0;
    }	
	
	return sprintf(buf,"rgbir: %d|%d|%d|%d",red,green,blue,ir);
	

}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(bh1749_debug,        S_IWUSR | S_IRUGO, bh1749_show_debug,   bh1749_store_debug);
static DRIVER_ATTR(bh1749_red,            S_IWUSR | S_IRUGO, bh1749_show_red,  NULL);
static DRIVER_ATTR(bh1749_green,         S_IWUSR | S_IRUGO, bh1749_show_green, NULL);
static DRIVER_ATTR(bh1749_blue,           S_IWUSR | S_IRUGO, bh1749_show_blue,  NULL);
static DRIVER_ATTR(bh1749_rgb,           S_IWUSR | S_IRUGO, bh1749_show_rgb,  NULL);


/*----------------------------------------------------------------------------*/
static struct driver_attribute *bh1749_attr_list[] = {
    &driver_attr_bh1749_debug,
    &driver_attr_bh1749_red,
    &driver_attr_bh1749_green,
    &driver_attr_bh1749_blue,
    &driver_attr_bh1749_rgb,
};

/*----------------------------------------------------------------------------*/
static int bh1749_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(bh1749_attr_list)/sizeof(bh1749_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, bh1749_attr_list[idx])))
        {
            BH1749_WARNING("driver_create_file (%s) = %d\n", bh1749_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int bh1749_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(bh1749_attr_list)/sizeof(bh1749_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, bh1749_attr_list[idx]);
    }

    return err;
}

/**
 * @Brief: bh1749_register_dump print register value for debug
 *
 * @Param: reg_address regsiter address
 *
 * @return: no return
 */
static int bh1749_register_dump(int addr, RGB_DATA * ptr)
{
    int read_data = 0;


    if (!ptr)
    {
        BH1749_ERR(" Parameter error \n");
        return -1 ;
    }

    /* block read */
    read_data = i2c_smbus_read_byte_data(ptr->client, addr);
    if (read_data < 0) {
        BH1749_ERR( "bh1749 driver i2c read : transfer error \n");
        return -1;
    }

    BH1749_WARNING( "reg(0x%x) = 0x%x\n",addr, read_data);
    return read_data;
}




/**
 * @Brief: bh1749_power Power control for bh1749 hardware
 *
 * @Param: hw bh1749 hardware ldo and voltage
 * @Param: on True for power on,flase for power off
 */
#if 0
static void bh1749_power(struct alsps_hw *hw, unsigned int on)
{
    //Todo
}
#endif
/**
 * @Brief: bh1749_open_report_data BH1749 initialization or uninitialization
 *
 * @Param: open 1 for initialize,0 for uninitialize
 *
 * @Returns: 0 for success,other for failed.
 */
static int bh1749_open_report_data(int open)
{
    int      result;

    BH1749_WARNING("open=%d\n", open);

    if (NULL == obj)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    //enable     
    if (open)
    {
        result = bh1749_driver_init(obj);
    }
    else //disable
    {
        result = bh1749_driver_shutdown(obj);
    }

    return result;
}

/**
 * @Brief: bh1749_enable_nodata Enable or disable BH1749
 *
 * @Param: en 1 for enable,0 for disable
 *
 * @Returns: 0 for success,others for failed.
 */
static int bh1749_enable_nodata(int en)
{
    BH1749_WARNING("en=%d\n", en);

    if (NULL == obj)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    obj->als_en_status = en ? true : false;

    return bh1749_driver_write_power_on_off(obj, en);
}

/**
 * @Brief: bh1749_set_delay Set delay,not used for now.
 *
 * @Param: delay Delay time.
 *
 * @Returns: 0 for success,other for failed.
 */
static int bh1749_set_delay(u64 delay)
{
    BH1749_FUN();

    return 0;
}

static int bh1749_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return bh1749_set_delay(samplingPeriodNs);
}

static int bh1749_flush(void)
{
	return als_flush_report();
}


/**
 * @Brief: bh1749_get_tp_module_id Get tp module id,use it to get right parameters.
 *
 * @Returns: Tp module id.
 */
static unsigned int bh1749_get_tp_module_id(void)
{
    /* TODO:Please finish this function. */
    return 2;
}

/**
 * @Brief: bh1749_get_color_type Get color type which use it to get right parameters.
 *
 * @Returns: Color type
 */
static COLOR_T bh1749_get_color_type(void)
{
    /* TODO:Please finish this function. */
    return GOLD;
}


static int bh1749_suspend(struct device *dev)
{
    RGB_DATA * bh1749_data = dev_get_drvdata(dev);

    if(bh1749_data->als_en_status){
        BH1749_WARNING("%s: Enable ALS : 0\n", __func__);
        bh1749_enable_nodata(0);
        bh1749_data->als_suspend = true;
    }

    return 0;
}

static int bh1749_resume(struct device *dev)
{
    RGB_DATA * bh1749_data = dev_get_drvdata(dev);

    if(bh1749_data->als_suspend){
        BH1749_WARNING("%s: Enable ALS : 1\n", __func__);
        bh1749_enable_nodata(1);
        bh1749_data->als_suspend = false;
    }

    return 0;
}


#if AUTO_GAIN
/**
 * @brief   bh1749_rgbc_agc(): change als measure gain dynamic according to the rgb/ir
 * @param[in] client     i2c Handle.
 * @param[in] 
 *
 *
 * @return  ret
 */
static int bh1749_rgbc_agc(struct i2c_client *client, READ_DATA_ARG data, 
        unsigned char rgb_gain,  unsigned char ir_gain )
{

    int ret = 0;
    unsigned char value = 0, target_rgb_gain = 0, target_ir_gain = 0;
    const unsigned char  adc_gain[] = {0, BH1749_1X, 0, BH1749_32X};

    //rgb check
    if(data.red > BH1749_GAIN_CHANGE_MAX ||
            data.green > BH1749_GAIN_CHANGE_MAX ||
            data.blue > BH1749_GAIN_CHANGE_MAX) { //rgb saturation

        if(rgb_gain != BH1749_1X) {
            target_rgb_gain = BH1749_RGB_GAIN_X1;
        }
    } else if (data.red < BH1749_GAIN_CHANGE_MIN ||
            data.green < BH1749_GAIN_CHANGE_MIN ||
            data.blue < BH1749_GAIN_CHANGE_MIN) { //rgb insufficience

        if(rgb_gain != BH1749_32X) {
            target_rgb_gain = BH1749_RGB_GAIN_X32;
        }
    }

    //ir check
    if(data.ir > BH1749_GAIN_CHANGE_MAX) { //ir saturation

        if(ir_gain != BH1749_1X) {
            target_ir_gain = BH1749_IR_GAIN_X1;
        }
    } else if (data.ir < BH1749_GAIN_CHANGE_MIN) { //ir insufficience

        if(ir_gain != BH1749_32X) {
            target_ir_gain = BH1749_IR_GAIN_X32;
        }
    }

    if (target_rgb_gain || target_ir_gain) {

        value = i2c_smbus_read_byte_data(client, BH1749_REG_GAIN_TIME);
        if (value < 0) {
            BH1749_ERR("i2c read error = %d\n", value);
            return (value);
        }

        if(target_rgb_gain){
            value = (value & BH1749_RGB_GAIN_MASK) | target_rgb_gain;
            BH1749_WARNING("bu27006 rgb_gain: %dX -> %dX \n", 
                    rgb_gain,
                    adc_gain[BH1749_RGB_GAIN_VALUE(target_rgb_gain)]);
        }

        if(target_ir_gain){
            value = (value & BH1749_IR_GAIN_MASK) | target_ir_gain;
            BH1749_WARNING("bu27006 ir_gain: %dX -> %dX \n", 
                    ir_gain,
                    adc_gain[BH1749_IR_GAIN_VALUE(target_ir_gain)]);
        }

        ret = i2c_smbus_write_byte_data(client, BH1749_REG_GAIN_TIME, value);
        if (ret < 0) {
            BH1749_ERR(" i2c write error = %d, agc failed !!!\n", ret);
        }
    }

    return ret;
}
#endif



/**
 * @Brief: bh1749_calculate_light Calculate lux base on rgbc
 *
 * @Param: data RGBC value from sensor
 *
 * @Returns: lux value or failed.
 */
static int bh1749_calculate_light(READ_DATA_ARG data)
{
    int lux = 0;
    long long tmp = 0;

    if (data.green == REPORT_MAX_VALUE) {
        lux = LUX_MAX_VALUE;
    }
    else {
        if (data.ir < judge * data.green) {
            if((lux_coef[0] * data.red + lux_coef[1] * data.green) > (lux_coef[2] * data.blue)){
                tmp = lux_coef[0] * data.red + lux_coef[1] * data.green - lux_coef[2] * data.blue;
            }else{
                tmp = 0;
            }
        } else {
            if((lux_coef[3] * data.red + lux_coef[4] * data.green) > (lux_coef[5] * data.blue)){
                tmp = lux_coef[3] * data.red + lux_coef[4] * data.green - lux_coef[5] * data.blue;
            }else{
                tmp = 0;
            }
        }
        //rounding(si she wu ru)
        lux = div_u64(tmp + (BU27006_LUX_SCALE >> 1),BU27006_LUX_SCALE);
    }
    //BH1749_INFO("bu27006 sizeof(char)=%d sizeof(int)=%d sizeof(long)=%d sizeof(long long)=%d\n",
    //        (int)sizeof(char), (int)sizeof(int), (int)sizeof(long), (int)sizeof(long long) );

    //BH1749_INFO("[Hunter] bh1749 tmp=%lld lux=%d\n", tmp, lux );
    return lux;

}



/**
 * @Brief: bh1749_get_data Get data from BH1749 hardware.
 *
 * @Param: als_value Return value including lux and rgbc.
 * @Param: status Return bh1749 status.
 *
 * @Returns: 0 for success,other for failed.
 */
static int bh1749_get_data(int *als_value, int *status)
{
    char result;
    unsigned char       rgb_gain, ir_gain, time;
    READ_DATA_ARG       data = {0};

    const unsigned short measurement_time[] = {0, 0, BH1749_120MS, BH1749_240MS, 0, BH1749_35MS, 0, 0};
    const unsigned char  adc_gain[] = {0, BH1749_1X, 0, BH1749_32X};

    if(als_value == NULL || status == NULL || obj == NULL)
    {
        BH1749_ERR(" Parameter error \n");
        return -EINVAL;
    }

    //BH1749_FUN();

    //set default value to ALSPS_INVALID_VALUE if it can't get an valid data
#if defined(CONFIG_MTK_ROHM_BH1749_COLOR_TEMPERATURE)
    als_value[0] = ALSPS_INVALID_VALUE;
#else
    *als_value = ALSPS_INVALID_VALUE;
#endif

    //get valid from BH1749_REG_ENABLE(0x42)
    result = i2c_smbus_read_byte_data(obj->client, BH1749_REG_ENABLE);
    if (result < 0)
    {
        BH1749_ERR("Read data from IC error.\n");
        return result;
    }
    //BH1749_WARNING("Data valid BH1749_REG_ENABLE(0x%x) = 0x%x\n", BH1749_REG_ENABLE, result);
    if ((result & BH1749_RGBC_VALID_HIGH) == 0)
    {
        BH1749_WARNING("Data Not valid. But it does not matter, please ignore it.\n");
        return -1;
    }

    //get gain from BH1749_REG_GAIN_TIME(0x41)
    result = i2c_smbus_read_byte_data(obj->client, BH1749_REG_GAIN_TIME);
    if (result < 0)
    {
        BH1749_ERR("Read data from IC error.\n");
        return result;
    }

    rgb_gain = adc_gain[BH1749_RGB_GAIN_VALUE(result)];
    ir_gain  = adc_gain[BH1749_IR_GAIN_VALUE(result)];
    time = measurement_time[BH1749_TIME_VALUE(result)];

    //get raw data
    result = bh1749_driver_read_data(obj, (void *)&data);
    if (result)
    {
        BH1749_ERR("Read data from bh1749 failed.\n");
        return result;
    }

    if(!time || !rgb_gain || !ir_gain)
    {
        BH1749_ERR(" parameter error: time=%d, rgb_gain=%d, ir_gain=%d \n", 
                time, rgb_gain, ir_gain);
        return -EINVAL;
    }
    #if 0
    BH1749_INFO("time=%d, rgb_gain=%d, ir_gain=%d \n", time, rgb_gain, ir_gain);
    BH1749_INFO("raw data: red=%d, green=%d, blue=%d, ir=%d \n", 
            data.red, data.green, data.blue, data.ir);
    #endif

#if AUTO_GAIN
    bh1749_rgbc_agc(obj->client, data, rgb_gain, ir_gain);
#endif
    //rohm raw data transform
    data.red   = data.red   * DATA_TRANSFER_COFF / rgb_gain / time;
    data.green = data.green * DATA_TRANSFER_COFF / rgb_gain / time;
    data.blue  = data.blue  * DATA_TRANSFER_COFF / rgb_gain / time;
    data.ir    = data.ir    * DATA_TRANSFER_COFF / ir_gain / time;
    //BH1749_INFO("standardization: red=%d, green=%d, blue=%d, ir=%d \n", 
    //        data.red, data.green, data.blue, data.ir);

#if defined(CONFIG_MTK_ROHM_BH1749_COLOR_TEMPERATURE)
    als_value[0] = bh1749_calculate_light(data);
    als_value[1] = data.red;
    als_value[2] = data.green;
    als_value[3] = data.blue;
    als_value[4] = data.ir;
	
    #if 0
    printk("===mlk===als_value[0] = %d\n", als_value[0] );
    printk("===mlk===als_value[1] = %d\n", als_value[1] );
    printk("===mlk===als_value[2] = %d\n", als_value[2] );
    printk("===mlk===als_value[3] = %d\n", als_value[3] );
    printk("===mlk===als_value[4] = %d\n", als_value[4] );
   #endif
    
    
#else
    *als_value = bh1749_calculate_light(data);
#endif

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;



    return 0;
}



/************************************************************
 *                     access function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : bh1749_driver_init
 * FUNCTION   : initialize BH1749
 * REMARKS    :
 *****************************************************************************/
static int bh1749_driver_init(RGB_DATA * ptr)
{


    int result;
    unsigned char value;

    if (NULL == ptr)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    BH1749_FUN();

    /* execute software reset */
    result = bh1749_driver_reset(ptr);
    if (result != 0) {
        return (result);
    }

    /* BH1749 init */
    value = BH1749_MEASURE_120MS | BH1749_RGB_GAIN_X32 | BH1749_IR_GAIN_X32;
    result = i2c_smbus_write_byte_data(ptr->client, BH1749_REG_GAIN_TIME, value);

    return (result);
}

/******************************************************************************
 * NAME       : bh1749_driver_shutdown
 * FUNCTION   : shutdown BH1749
 * REMARKS    :
 *****************************************************************************/
static int bh1749_driver_shutdown(RGB_DATA * ptr)
{
    int result;

    if (NULL == ptr)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = bh1749_driver_reset(ptr);

    return (result);
}

/******************************************************************************
 * NAME       : bh1749_driver_reset
 * FUNCTION   : reset BH1749 register
 * REMARKS    :
 *****************************************************************************/
static int bh1749_driver_reset(RGB_DATA * ptr)
{
    int result = 0;        

    if (NULL == ptr)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = i2c_smbus_write_byte_data(ptr->client, BH1749_REG_RESET, (BH1749_SW_RESET | BH1749_INT_RESET));

    return (result);
}

/******************************************************************************
 * NAME       : bh1749_driver_write_power_on_off
 * FUNCTION   : power on and off BH1749
 * REMARKS    :
 *****************************************************************************/
static int bh1749_driver_write_power_on_off(RGB_DATA * ptr, unsigned char data)
{
    int           result = 0;
    unsigned char power_set;
    unsigned char mode_ctl2;
    unsigned char write_data;

    if (NULL == ptr)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    BH1749_WARNING(" data=%d\n", data);

    /* read mode control1 register */
    result = i2c_smbus_read_byte_data(ptr->client, BH1749_REG_ENABLE);
    if (result < 0) {
        /* i2c communication error */
        return (result);
    }

    if (data == 0) {
        power_set = BH1749_RGBC_EN_OFF;
    } else {
        power_set = BH1749_RGBC_EN_ON;
    }

    /* read mode control2 and mask RGBC_EN  */
    mode_ctl2  = (unsigned char)(result & ~BH1749_RGBC_EN_ON);
    write_data = mode_ctl2 | power_set;
    result = i2c_smbus_write_byte_data(ptr->client, BH1749_REG_ENABLE, write_data);
    if (result < 0) {
        /* i2c communication error */
        BH1749_ERR( "i2c_smbus_write_byte_data error \n");
        return (result);
    }

    return (result);
}


/******************************************************************************
 * NAME       : bh1749_driver_read_data
 * FUNCTION   : read the value of RGB data and status in BH1749
 * REMARKS    :
 *****************************************************************************/
static int bh1749_driver_read_data(RGB_DATA * ptr, void * reg_data)
{
    int result = 0;
    unsigned char  i2c_data[6] = {0};
    READ_DATA_ARG * data =   (READ_DATA_ARG * )reg_data;

    if (NULL == ptr || NULL == reg_data)
    {
        BH1749_ERR(" Parameter error \n");
        return -EINVAL;
    }


    /* read rgb */
    result = i2c_smbus_read_i2c_block_data(ptr->client, BH1749_REG_RED_DATA, sizeof(i2c_data), i2c_data);
    if (result < 0) {
        BH1749_ERR( "bh1749_driver_read_data : transfer error \n");
        return result;
    } else {
        data->red   = i2c_data[0] | (i2c_data[1] << 8);
        data->green = i2c_data[2] | (i2c_data[3] << 8);
        data->blue  = i2c_data[4] | (i2c_data[5] << 8);
        result = 0;
    }

    /* read ir */
    result = i2c_smbus_read_i2c_block_data(ptr->client, BH1749_REG_IR_DATA, 2 , i2c_data);
    if (result < 0) {
        BH1749_ERR( "bh1749_driver_read_data : transfer error \n");
        return result;
    } else {
        data->ir = i2c_data[0] | (i2c_data[1] << 8);
        result = 0;
    }

    return (result);
}


/**
 * @Brief: bh1749_check_id, check the current IC is bh1749 or not.
 *         If the value of reg(0x92) is equal to 0xE0, it is bh1749, then return 0
 * @Input:  i2c handle
 * @Returns: return 0 if success, otherwise -1 .
 */
static int bh1749_check_id(struct i2c_client *client)
{
    int tmp_id = 0;
    int ret;

    //read manufacture id and check valid
    tmp_id = i2c_smbus_read_byte_data(client, BH1749_REG_MANUFACT_ID);
    if (tmp_id < 0)
    {
        BH1749_ERR("Read data from IC error.\n");
        return tmp_id;
    }
    BH1749_WARNING("MANUFACT_VALUE reg(0x%x)=0x%x\n", BH1749_REG_MANUFACT_ID, tmp_id);

    ret = (BH1749_ID_VALUE == tmp_id )? 0: -1;

    return ret;

}

/******************************************************************************
 * NAME       : bh1749_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int bh1749_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    int tmp_id, result = 0; 
    RGB_DATA  *bh1749_data;

    struct als_control_path als_ctl = {0};
    struct als_data_path    als_data = {0};

    BH1749_WARNING("bh1749_probe for BH1749!!\n");

    if (NULL == client )
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!result) {
        BH1749_ERR( "need I2C_FUNC_I2C\n");
        result = -ENODEV;
        goto err_check_functionality_failed;
    }


    tmp_id = bh1749_check_id(client);
    if (tmp_id < 0)
    {
        BH1749_ERR("Probe failed, driver can not match device correctly!!!\n");
        result = -ENODEV;
        //goto err_check_functionality_failed;
    }

    //begin alloc memory for driver
    bh1749_data = kzalloc(sizeof(*bh1749_data), GFP_KERNEL);
    if (bh1749_data == NULL) {
        result = -ENOMEM;
        goto err_alloc_data_failed;
    }

    memset(bh1749_data, 0, sizeof(*bh1749_data));
    bh1749_data->client = client;
    i2c_set_clientdata(client, bh1749_data);
    obj = bh1749_data;


    als_ctl.open_report_data = bh1749_open_report_data;
    als_ctl.enable_nodata = bh1749_enable_nodata;
    als_ctl.set_delay = bh1749_set_delay;
    als_ctl.batch = bh1749_batch;
    als_ctl.flush = bh1749_flush;
    als_ctl.is_use_common_factory = false;

    result = als_register_control_path(&als_ctl);
    if (result)
    {
        BH1749_ERR("als_register_control_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    als_data.get_data = bh1749_get_data;
    als_data.vender_div = 1;

    result = als_register_data_path(&als_data);
    if (result)
    {
        BH1749_ERR("als_register_data_path failed, error = %d\n", result);
        goto err_power_failed;
    }


    /* Get the touch panel manufacture id and touch panel color type*/
    bh1749_data->tp_module_id = bh1749_get_tp_module_id();
    bh1749_data->color_type = bh1749_get_color_type();

    if((result = bh1749_create_attr(&(bh1749_driver.driver))))
    {
        BH1749_ERR("create attribute err = %d\n", result);
        goto exit_create_attr_failed;
    }

    bh1749_driver_init(bh1749_data);
    bh1749_driver_write_power_on_off(bh1749_data,1);
    

    BH1749_INFO("BH1749 bh1749_probe success!!\n");
    return (result);

exit_create_attr_failed:
err_power_failed:
    kfree(bh1749_data);
    obj = NULL;
err_alloc_data_failed:
err_check_functionality_failed:

    return (result);

}


/**
 * @Brief: bh1749_local_init Initial BH1749 driver.
 *         get hardware resoures from dts, such as als_level, als_value.
 *         then call i2c_add_driver()
 *
 * @Returns: 0 for success,others for failed.
 */
static int bh1749_local_init(void)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0))
    hw = get_cust_alsps_hw();
    const char *name = "mediatek,bh1749_resource";

    BH1749_WARNING("bh1749_local_init\n");

    hw =   get_alsps_dts_func(name, hw);
    if (!hw)
        BH1749_ERR("get dts info fail\n");
#endif
	printk("==mlk==enter %s\n",__func__);
    
    return i2c_add_driver(&bh1749_driver);
}

/******************************************************************************
 * NAME       : bh1749_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int bh1749_remove(struct i2c_client *client)
{
    RGB_DATA *bh1749_data;
    int err;

    if (NULL == client)
    {
        BH1749_ERR(" Parameter error \n");
        return EINVAL;
    }

    if((err = bh1749_delete_attr(&(bh1749_init_info.platform_diver_addr->driver))))
    {
        BH1749_ERR("bh1749_delete_attr fail: %d\n", err);
    }

    bh1749_data = i2c_get_clientdata(client);

    kfree(bh1749_data);
    obj = NULL;

    return err;
}



/**
 * @Brief: bh1749_remove Remove BH1749 driver.
 *
 * @Returns: 0 for success,others for failed.
 */
static int bh1749_local_remove(void)
{

    BH1749_FUN();

   // bh1749_power(hw, 0);
    i2c_del_driver(&bh1749_driver);

    return 0;
}


/******************************************************************************
 * NAME       : bh1749_als_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __init bh1749_als_init(void)
{
    printk("bh1749_als_init\n");
    return alsps_driver_add(&bh1749_init_info);
}

/******************************************************************************
 * NAME       : bh1749_als_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit bh1749_als_exit(void)
{
    return;
}


MODULE_AUTHOR("Aaron Liu@Rohm");
MODULE_DESCRIPTION("ROHM Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");

module_init(bh1749_als_init);
module_exit(bh1749_als_exit);
