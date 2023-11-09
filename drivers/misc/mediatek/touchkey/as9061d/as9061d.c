#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/ioctl.h>



#define REG_SOFTWARE_VERSION 0x25
#define REG_KEY_STATUS 		 0x26
#define REG_LED1             0x27
#define REG_LED2             0x28
#define REG_LED3             0x29

int key_volUp_pin;
int key_volDown_pin;
int key_micMute_pin;



#define AS9061D_LED_IOCTL_MAGIC 'm'
#define AS9061D_LED _IOW(AS9061D_LED_IOCTL_MAGIC, 1, int *)

//#define AS9061d_LED_ON


static struct i2c_client  *as9061d_i2c_client;

struct as9061d_data
{
	struct i2c_client *client;
	struct input_dev *input_dev;	
	struct workqueue_struct *as9061d_workqueue;	
	struct work_struct  as9061d_work;
	struct timer_list  as9061d_timer;
	
	struct timer_list  volUp_timer;
	struct timer_list  volDown_timer;
	struct timer_list  micMute_timer;
	
	int touch_int_pin;
	int touch_volUp_pin;
	int touch_volDown_pin;
	int touch_micMute_pin;
	
	int touch_int_irq;
	int touch_volUp_irq;
	int touch_volDown_irq;
	int touch_micMute_irq;
	
	void *drv_data;
};

static ssize_t as9061d_led_brightness_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	unsigned int   brightness  = simple_strtoul(buf, NULL, 0);

	int ret;
	
	ret = i2c_smbus_write_byte_data(client, REG_LED1, brightness);
				mdelay(10);
			
	ret = i2c_smbus_write_byte_data(client, REG_LED2, brightness);
	mdelay(10);
				
	ret = i2c_smbus_write_byte_data(client, REG_LED3, brightness);
	mdelay(10);
	
	return count;
}


static ssize_t as9061d_led_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client   = to_i2c_client(dev);

	int led1brightness,led2brightness,led3brightness;

	led1brightness = i2c_smbus_read_byte_data(client, REG_LED1);

	led2brightness = i2c_smbus_read_byte_data(client, REG_LED2);

	led3brightness = i2c_smbus_read_byte_data(client, REG_LED3);
	
	return sprintf(buf, "led1brightness = %d\nled2brightness = %d\nled3brightness = %d\n", led1brightness,led2brightness,led3brightness);
}


static ssize_t as9061d_led_volumeup_brightness_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	unsigned int   brightness  = simple_strtoul(buf, NULL, 0);

	int ret;
	
	ret = i2c_smbus_write_byte_data(client, REG_LED3, brightness);
				mdelay(10);
	
	return count;
}


static ssize_t as9061d_led_volumeup_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client   = to_i2c_client(dev);

	int led3brightness;

	led3brightness = i2c_smbus_read_byte_data(client, REG_LED3);
	
	return sprintf(buf, "led3brightness = %d\n", led3brightness);
}


static ssize_t as9061d_led_volumedown_brightness_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	unsigned int   brightness  = simple_strtoul(buf, NULL, 0);

	int ret;
	
	ret = i2c_smbus_write_byte_data(client, REG_LED2, brightness);
				mdelay(10);
	
	return count;
}


static ssize_t as9061d_led_volumedown_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client   = to_i2c_client(dev);

	int led2brightness;

	led2brightness = i2c_smbus_read_byte_data(client, REG_LED2);
	
	return sprintf(buf, "led2brightness = %d\n", led2brightness);
}


static ssize_t as9061d_led_favorite_brightness_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	unsigned int   brightness  = simple_strtoul(buf, NULL, 0);

	int ret;
	
	ret = i2c_smbus_write_byte_data(client, REG_LED1, brightness);
				mdelay(10);
	
	return count;
}


static ssize_t as9061d_led_favorite_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client   = to_i2c_client(dev);

	int led1brightness;

	led1brightness = i2c_smbus_read_byte_data(client, REG_LED1);
	
	return sprintf(buf, "led1brightness = %d\n", led1brightness);
}


static DEVICE_ATTR(led_brightness, 0644,as9061d_led_brightness_show, as9061d_led_brightness_store);
static DEVICE_ATTR(led_volumeup_brightness, 0644,as9061d_led_volumeup_brightness_show, as9061d_led_volumeup_brightness_store);
static DEVICE_ATTR(led_volumedown_brightness, 0644,as9061d_led_volumedown_brightness_show, as9061d_led_volumedown_brightness_store);
static DEVICE_ATTR(led_favorite_brightness, 0644,as9061d_led_favorite_brightness_show, as9061d_led_favorite_brightness_store);


static struct attribute *as9061d_attributes[] = {
	&dev_attr_led_brightness.attr,
	&dev_attr_led_volumeup_brightness.attr,
	&dev_attr_led_volumedown_brightness.attr,
	&dev_attr_led_favorite_brightness.attr,
	NULL
};

static const struct attribute_group as9061d_attr_group = {
	.attrs = as9061d_attributes,
};

static long as9061d_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
   int ret = 0; 
   uint32_t brightness;

    struct i2c_client *client;

    if(as9061d_i2c_client == NULL) {
		printk("as9061d_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

    client = as9061d_i2c_client;

    switch (cmd) {
		case AS9061D_LED:
				#if 0
				if (copy_from_user(&brightness, (void __user *)arg, sizeof(brightness)))
				{
					printk("as9061d_ioctl: copy_from_user failed\n");
					printk("as9061d_ioctl: brightness = %d , arg = %ld ,  &arg =0x%p\n",brightness,arg,(void __user *)arg);
					return -EFAULT;
			      }
			      #endif

			      
				brightness = arg;
			      
				printk("===mlk===%s  brightness = %d \n",__func__,brightness);
				
			       ret = i2c_smbus_write_byte_data(client, REG_LED1, brightness);
				mdelay(10);
			
				ret = i2c_smbus_write_byte_data(client, REG_LED2, brightness);
				mdelay(10);
				
				ret = i2c_smbus_write_byte_data(client, REG_LED3, brightness);
				mdelay(10);
				
		              break;

		default:
		             break;
    }

	
    return 0;
}

static int as9061d_release(struct inode *inode, struct file *file)
{
	return 0;
}



static int as9061d_open(struct inode *inode, struct file *file)
{
	return 0; 
}


static struct file_operations as9061d_fops = {
	.owner = THIS_MODULE,
	.open = as9061d_open,
	.release = as9061d_release,
	.unlocked_ioctl = as9061d_ioctl,
};


static struct miscdevice as9061d_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "as9061d",
	.fops = &as9061d_fops,
};


#ifdef  AS9061d_LED_ON

static void  as9061d_led(struct i2c_client *client)
{
	int ret = 0;

       ret = i2c_smbus_write_byte_data(client, REG_LED1, 100);
	mdelay(100);

	ret = i2c_smbus_write_byte_data(client, REG_LED2, 100);
	mdelay(100);
	
	ret = i2c_smbus_write_byte_data(client, REG_LED3, 100);
	mdelay(100);

};
#endif


static void touch_button_func(struct as9061d_data *as9061d_info)
{
	int keyStatus;
	keyStatus = i2c_smbus_read_byte_data(as9061d_info->client, REG_KEY_STATUS);
	printk("As9061d key  status = 0x%x\n",keyStatus);
	
	switch(keyStatus)
		{
			case 0x1:
			{	
				printk("VOL+\n");

			    input_event(as9061d_info->input_dev, EV_KEY, KEY_VOLUMEUP, 1);
				input_sync(as9061d_info->input_dev);

				input_event(as9061d_info->input_dev, EV_KEY, KEY_VOLUMEUP, 0);
				input_sync(as9061d_info->input_dev);
				
				break;
			}
			case 0x2:
			{	
				printk("VOL-\n");
				input_event(as9061d_info->input_dev, EV_KEY, KEY_VOLUMEDOWN, 1);
				input_sync(as9061d_info->input_dev);

				input_event(as9061d_info->input_dev, EV_KEY, KEY_VOLUMEDOWN, 0);
				input_sync(as9061d_info->input_dev);
				break;
			}
			case 0x4:
			{	
				printk("MIC MUTE\n");
				input_event(as9061d_info->input_dev, EV_KEY, KEY_FAVORITE, 1);
				input_sync(as9061d_info->input_dev);

				input_event(as9061d_info->input_dev, EV_KEY, KEY_FAVORITE, 0);
				input_sync(as9061d_info->input_dev);
				break;
			}
			default:
			   printk("default\n");
			
		}
	
}


static void as9061d_work(struct work_struct *work)
{	
	struct as9061d_data *as9061d_info;
	as9061d_info = container_of(work,struct as9061d_data,as9061d_work);
	printk("===mlk===enter %s\n",__func__);
	touch_button_func(as9061d_info);
	enable_irq(as9061d_info->touch_int_irq);
}

static void micMute_timer_function(unsigned long data)
{	

	int gpio1val;	
	int gpio2val;	
	int gpio3val;	

	
	struct as9061d_data *as9061d_info = (struct as9061d_data *)data;	
	printk("===mlk=== %s\n",__func__);	
	gpio1val = gpio_get_value(key_volUp_pin);
	gpio2val = gpio_get_value(key_volDown_pin);
	gpio3val = gpio_get_value(key_micMute_pin);
	

	/*	*检测按键状态，上报键值	*gpioval = 1 按键松开，gpioval = 0 按键按下	*/	
	printk("gpio1val = %d\n",gpio1val);	
	printk("gpio2val = %d\n",gpio2val);
	printk("gpio3val = %d\n",gpio3val);

	if(!gpio3val)
		{
			input_report_key(as9061d_info->input_dev,KEY_FAVORITE,1); 
			input_sync(as9061d_info->input_dev);
			mod_timer(&as9061d_info->micMute_timer, jiffies + HZ /50);
		}
	else
		{
			input_report_key(as9061d_info->input_dev,KEY_FAVORITE,0); 
			input_sync(as9061d_info->input_dev);
		}

	enable_irq(as9061d_info->touch_micMute_irq);
}


static void volDown_timer_function(unsigned long data)
{	

	int gpio1val;	
	int gpio2val;	
	int gpio3val;	

	
	struct as9061d_data *as9061d_info = (struct as9061d_data *)data;	
	printk("===mlk=== %s\n",__func__);	
	gpio1val = gpio_get_value(key_volUp_pin);
	gpio2val = gpio_get_value(key_volDown_pin);
	gpio3val = gpio_get_value(key_micMute_pin);
	

	/*	*检测按键状态，上报键值	*gpioval = 1 按键松开，gpioval = 0 按键按下	*/	
	printk("gpio1val = %d\n",gpio1val);	
	printk("gpio2val = %d\n",gpio2val);
	printk("gpio3val = %d\n",gpio3val);

	if(!gpio2val)
		{
			input_report_key(as9061d_info->input_dev,KEY_VOLUMEDOWN,1); 
			input_sync(as9061d_info->input_dev);
			mod_timer(&as9061d_info->volDown_timer, jiffies + HZ /50);
		}
	else
		{
			input_report_key(as9061d_info->input_dev,KEY_VOLUMEDOWN,0); 
			input_sync(as9061d_info->input_dev);
		}
		
	enable_irq(as9061d_info->touch_volDown_irq);
}



static void volUp_timer_function(unsigned long data)
{	

	int gpio1val;	
	int gpio2val;	
	int gpio3val;	

	
	struct as9061d_data *as9061d_info = (struct as9061d_data *)data;	
	printk("===mlk=== %s\n",__func__);	
	gpio1val = gpio_get_value(key_volUp_pin);
	gpio2val = gpio_get_value(key_volDown_pin);
	gpio3val = gpio_get_value(key_micMute_pin);
	

	/*	*检测按键状态，上报键值	*gpioval = 1 按键松开，gpioval = 0 按键按下	*/	
	printk("gpio1val = %d\n",gpio1val);	
	printk("gpio2val = %d\n",gpio2val);
	printk("gpio3val = %d\n",gpio3val);

	if(!gpio1val)
		{
			input_report_key(as9061d_info->input_dev,KEY_VOLUMEUP,1); 
			input_sync(as9061d_info->input_dev);
			mod_timer(&as9061d_info->volUp_timer, jiffies + HZ /50);
		}
	else
		{
			input_report_key(as9061d_info->input_dev,KEY_VOLUMEUP,0); 
			input_sync(as9061d_info->input_dev);
		}	
	
	enable_irq(as9061d_info->touch_volUp_irq);
}

static irqreturn_t as9061d_micMute_irq_handler(int irq, void *dev_id)
{	
	struct as9061d_data *as9061d_info = dev_id;
	disable_irq_nosync(as9061d_info->touch_micMute_irq);
	//queue_work(as9061d_info->as9061d_workqueue,&as9061d_info->as9061d_work);
	mod_timer(&as9061d_info->micMute_timer, jiffies + HZ /50);
	
	return IRQ_HANDLED;
}

static irqreturn_t as9061d_volDown_irq_handler(int irq, void *dev_id)
{	
	struct as9061d_data *as9061d_info = dev_id;
	disable_irq_nosync(as9061d_info->touch_volDown_irq);
	//queue_work(as9061d_info->as9061d_workqueue,&as9061d_info->as9061d_work);
	mod_timer(&as9061d_info->volDown_timer, jiffies + HZ /100);
	
	return IRQ_HANDLED;
}

static irqreturn_t as9061d_volUp_irq_handler(int irq, void *dev_id)
{	
	struct as9061d_data *as9061d_info = dev_id;
	disable_irq_nosync(as9061d_info->touch_volUp_irq);
	//queue_work(as9061d_info->as9061d_workqueue,&as9061d_info->as9061d_work);
	mod_timer(&as9061d_info->volUp_timer, jiffies + HZ /100);
	
	return IRQ_HANDLED;
}

static int as9061d_irq_subdevice(struct as9061d_data *as9061d_info)
{
	int ret = 0;
	ret = request_irq(as9061d_info->touch_volUp_irq,   as9061d_volUp_irq_handler,  IRQF_TRIGGER_LOW, "as9061d_volUp_irq",  as9061d_info);
	ret = request_irq(as9061d_info->touch_volDown_irq, as9061d_volDown_irq_handler,IRQF_TRIGGER_LOW, "as9061d_volDwon_irq",as9061d_info);
	ret = request_irq(as9061d_info->touch_micMute_irq, as9061d_micMute_irq_handler,IRQF_TRIGGER_LOW, "as9061d_micMute_irq",as9061d_info);
	return 0;	
}

static void as9061d_parse_dts(struct as9061d_data *as9061d_info,struct device *dev)
{
	struct pinctrl *as9061d_ctrl = NULL;
	struct pinctrl_state *as9061d_int_pullup        = NULL;
	struct pinctrl_state *as9061d_volUp_pullup      = NULL;
	struct pinctrl_state *as9061d_volDown_pullup    = NULL;
	struct pinctrl_state *as9061d_micMute_pullup    = NULL;
	
	as9061d_info->touch_int_pin     = of_get_named_gpio(dev->of_node, "int-gpio", 0);
	as9061d_info->touch_volUp_pin   = of_get_named_gpio(dev->of_node, "volUp-gpio", 0);
	as9061d_info->touch_volDown_pin = of_get_named_gpio(dev->of_node, "volDown-gpio", 0);
	as9061d_info->touch_micMute_pin = of_get_named_gpio(dev->of_node, "micMute-gpio", 0);

	
	as9061d_ctrl 		   = devm_pinctrl_get(dev);
	as9061d_int_pullup     = pinctrl_lookup_state(as9061d_ctrl, "default");
	as9061d_volUp_pullup   = pinctrl_lookup_state(as9061d_ctrl, "key_volUp");
	as9061d_volDown_pullup = pinctrl_lookup_state(as9061d_ctrl, "key_volDown");
	as9061d_micMute_pullup = pinctrl_lookup_state(as9061d_ctrl, "key_micMute");
	
    pinctrl_select_state(as9061d_ctrl, as9061d_int_pullup);
	pinctrl_select_state(as9061d_ctrl, as9061d_volUp_pullup);
	pinctrl_select_state(as9061d_ctrl, as9061d_volDown_pullup);
	pinctrl_select_state(as9061d_ctrl, as9061d_micMute_pullup);


	key_volUp_pin    = as9061d_info->touch_volUp_pin;
	key_volDown_pin  = as9061d_info->touch_volDown_pin;
	key_micMute_pin  = as9061d_info->touch_micMute_pin;
	
	
	gpio_request(as9061d_info->touch_volUp_pin, NULL);
	gpio_request(as9061d_info->touch_volDown_pin, NULL);
	gpio_request(as9061d_info->touch_micMute_pin, NULL);
	
	gpio_direction_input(as9061d_info->touch_volUp_pin); 
	gpio_direction_input(as9061d_info->touch_volDown_pin);
	gpio_direction_input(as9061d_info->touch_micMute_pin);
	

	as9061d_info->touch_volUp_irq   = gpio_to_irq(as9061d_info->touch_volUp_pin);
	as9061d_info->touch_volDown_irq = gpio_to_irq(as9061d_info->touch_volDown_pin);
	as9061d_info->touch_micMute_irq = gpio_to_irq(as9061d_info->touch_micMute_pin);
	
}

static int  as9061d_input_subdevice(struct as9061d_data *as9061d_info,struct device *dev)
{
	int ret;

	as9061d_info->input_dev = devm_input_allocate_device(dev);
	if (!as9061d_info->input_dev) 
	{
		printk("Failed to allocate input device \n");
		return  -ENOMEM;
	}	
	as9061d_info->input_dev->name = "as9061d_key";	

	//__set_bit(EV_REP, as9061d_info->input_dev->evbit);

	input_set_capability(as9061d_info->input_dev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(as9061d_info->input_dev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(as9061d_info->input_dev, EV_KEY, KEY_FAVORITE);
	
	ret = input_register_device(as9061d_info->input_dev);
	if (ret)
	{
		printk("Unable to register input device \n");
		return -ENODEV;
	}

	return 0;

}


static int  as9061d_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	struct device *dev = &client->dev;
	
	
	struct as9061d_data *as9061d_info ;

	printk("===mlk==start enter %s \n",__func__);
	
	as9061d_info = devm_kzalloc(dev,sizeof(struct as9061d_data),GFP_KERNEL);
	if(!as9061d_info)
	{
		return -ENOMEM;
	}

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!ret) 
	{
        printk("need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
     }
	
	as9061d_i2c_client = client;
	
	as9061d_info ->client = client;

	i2c_set_clientdata(client,as9061d_info);
	
 	ret = i2c_smbus_read_byte_data(client, REG_SOFTWARE_VERSION);
	if(ret < 0)
	{
		printk( "read software version  from ic error\n");	
		ret = -EIO;
		goto err_check_functionality_failed;
	}

	printk("As9061d software version = 0x%x\n",ret);

	
	as9061d_parse_dts(as9061d_info,dev);
	
	as9061d_input_subdevice(as9061d_info,dev);
	
	as9061d_info->as9061d_workqueue = create_singlethread_workqueue("as9061d_workqueue");
	
	INIT_WORK(&as9061d_info->as9061d_work, as9061d_work);


	init_timer(&as9061d_info->volUp_timer);	
	as9061d_info->volUp_timer.data = (unsigned long)as9061d_info;	
	as9061d_info->volUp_timer.expires = jiffies + HZ /100;  	
	as9061d_info->volUp_timer.function = volUp_timer_function;
	//add_timer(&as9061d_info->volUp_timer);

	init_timer(&as9061d_info->volDown_timer);	
	as9061d_info->volDown_timer.data = (unsigned long)as9061d_info;	
	as9061d_info->volDown_timer.expires = jiffies + HZ /100;  	
	as9061d_info->volDown_timer.function = volDown_timer_function;
	//add_timer(&as9061d_info->volDown_timer);

	init_timer(&as9061d_info->micMute_timer);	
	as9061d_info->micMute_timer.data = (unsigned long)as9061d_info;	
	as9061d_info->micMute_timer.expires = jiffies + HZ /100;  	
	as9061d_info->micMute_timer.function = micMute_timer_function;
	//add_timer(&as9061d_info->micMute_timer);
	
	
	as9061d_irq_subdevice(as9061d_info);

	
	ret = misc_register(&as9061d_device);
		if (ret) {
			printk("Unalbe to register as9061d ioctl: %d", ret);
			goto err_check_functionality_failed;
		}

	ret = sysfs_create_group(&client->dev.kobj, &as9061d_attr_group);
	
	#ifdef AS9061d_LED_ON
	 as9061d_led(as9061d_info ->client);
	#endif

	printk("===mlk== %s  success\n",__func__);

	return 0;

	err_check_functionality_failed:
		
		 return ret;
}

static int as9061d_remove(struct i2c_client *client)
{
    
    return 0;
}

static const struct i2c_device_id as9061d_id[] = {
    {"as9061d", 0},
    { }
};
MODULE_DEVICE_TABLE(i2c, as9061d_id);

static const struct of_device_id as9061d_of_match[] = {
	{ .compatible = "as9061d" },
	{ }
};
MODULE_DEVICE_TABLE(of, as9061d_of_match);


static struct i2c_driver as9061d_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = "as9061d",
		.of_match_table = of_match_ptr(as9061d_of_match),
    },
    .id_table  = as9061d_id,
    .probe     = as9061d_probe,
    .remove    = as9061d_remove,
};


static int __init as9061d_init(void)
{
	return i2c_add_driver(&as9061d_driver);
}

static void __exit as9061d_exit(void)
{
	i2c_del_driver(&as9061d_driver);
}


module_init(as9061d_init);
module_exit(as9061d_exit);

//module_i2c_driver(as9061d_driver);

MODULE_AUTHOR("lukuan.ma@tcl.com");
MODULE_DESCRIPTION("As9061d Touchkey Driver");
MODULE_LICENSE("GPL");
