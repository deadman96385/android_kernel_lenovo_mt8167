#include <linux/string.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
//#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include "lt9211.h"

#define LT9211_DEBUG
#ifdef LT9211_DEBUG
#define lt9211_printk(x...) printk( "[lt9211 DEBUG]: " x )
#else
#define lt9211_printk(x...)
#endif

#define MIPI_SETTLE_VALUE 0x0f //0x05  0x0a
//#define LT9211_TEST_ON //add by kxz for self_test

static unsigned int BL_EN_PIN;
static unsigned int LCD_PWR_PIN;

u8 VideoFormat=0;
enum VideoFormat Video_Format;
											//hfp, hs, hbp,hact,htotal,vfp, vs, vbp, vact,vtotal,
struct video_timing video_640x480_60Hz     ={ 8, 96,  40, 640,   800, 33,  2,  10, 480,   525,  25000};
struct video_timing video_720x480_60Hz     ={16, 62,  60, 720,   858,  9,  6,  30, 480,   525,  27000};
struct video_timing video_1024x600_60Hz   = {160, 20, 140,1024,  1344,  12,  3,  20,  600, 635  , 55000};
struct video_timing video_1280x720_60Hz    ={110,40, 220,1280,  1650,  5,  5,  20, 720,   750,  74250};
struct video_timing video_1280x720_30Hz    ={110,40, 220,1280,  1650,  5,  5,  20, 720,   750,  37125};
struct video_timing video_1366x768_60Hz    ={26, 110,110,1366,  1592,  13, 6,  13, 768,   800,  81000};
struct video_timing video_1280x1024_60Hz   ={100,100,208,1280,  1688,  5,  5,  32, 1024, 1066, 107960};
struct video_timing video_1920x1080_25Hz   = {88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125,  74250};
struct video_timing video_1920x1080_30Hz   ={88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125,  74250};
//struct video_timing video_1920x1080_60Hz   ={88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125, 148500};
//struct video_timing video_1920x1080_60Hz   ={88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125, 123750}; //50HZ
struct video_timing video_1920x1080_60Hz   ={168, 32, 80,1920,  2200,  5,  4,  36, 1080, 1125, 136125};   //60HZ
struct video_timing video_3840x1080_60Hz   ={176,88, 296,3840,  4400,  4,  5,  36, 1080, 1125, 297000};
struct video_timing video_1920x1200_60Hz   ={48, 32,  80,1920,  2080,  3,  6,  26, 1200, 1235, 154000};
struct video_timing video_3840x2160_30Hz   ={176,88, 296,3840,  4400,  8,  10, 72, 2160, 2250, 297000};
struct video_timing video_3840x2160_60Hz   ={176,88, 296,3840,  4400,  8,  10, 72, 2160, 2250, 594000};
struct video_timing video_1920x720_60Hz    ={148, 44, 88,1920,  2200,  28,  5, 12,  720, 765, 88000};

typedef struct LT9211{
	struct i2c_client *client;
	struct work_struct Lt9211_resume_work;
	struct work_struct Lt9211_suspend_work;
	
	int reset_pin;
	int enable_pin;
	int hact;
	int	vact;
	int hs;
	int vs;
	int hbp;
	int vbp;
	int htotal;
	int vtotal;
	int hfp;
	int vfp;
}LT9211_info_t;

struct i2c_client *g_client;
LT9211_info_t* g_LT9211;

/******add by kxz for Timing******/

//static struct task_struct *timing_task;
struct mutex update_lock;
/******add by kxz for Timing******/

static int LT9211_i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	//printk("%s:i2c_addr:0x%02x--reg:0x%02x--val:0x%02x\n", __FUNCTION__, client->addr, *data, *(data+1));
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;
	//msg.scl_rate=100 * 1000;

	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}

	return ret;
}


static int _LT9211_mipi_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	uint8_t buf[2] = {addr, val};
	int ret;

	ret = LT9211_i2c_write_bytes(client,buf,2);
	if (ret < 0) {
		dev_err(&client->dev, "error %d writing to lvds addr 0x%x\n",ret, addr);
		return -1;
	}

	return 0;
}


#define LT9211_mipi_write(client, addr, val) \
	do { \
		int ret; \
		ret = _LT9211_mipi_write(client, addr, val); \
	} while(0)

static int LT9211_i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags = client->flags;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
	//msgs[0].scl_rate=200 * 1000;

	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];
	//msgs[1].scl_rate=200 * 1000;

	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	return ret;
}


static int LT9211_read(struct i2c_client *client, uint8_t addr)
{
	int ret;
	unsigned char buf[]={addr,0};
	ret = LT9211_i2c_read_bytes(client,buf,2);
	if (ret < 0){
		printk("LT9211_read is fail\n");
		goto fail;
	}

	return buf[1];
fail:
	dev_err(&client->dev, "Error %d reading from subaddress 0x%x\n",ret, addr);
	return -1;
}


void LT9211_ChipID(struct i2c_client *client)
{
	char val=0;
	LT9211_mipi_write( client, 0xff, 0x81);
	val=LT9211_read( client,0x00);
	printk("LT9211 Chip ID:%x,",val);
	val=LT9211_read( client,0x01);
	printk("%x,",val);
	val=LT9211_read( client,0x02);
	printk("%x,\n",val);
}

void LT9211_SystemInt(struct i2c_client *client)
{
	/* system clock init */
	LT9211_mipi_write( client, 0xff, 0x82);
	LT9211_mipi_write( client, 0x01, 0x18);

	LT9211_mipi_write( client, 0xff, 0x86);
	LT9211_mipi_write( client, 0x06, 0x61);
	LT9211_mipi_write( client, 0x07, 0xa8); //fm for sys_clk

	LT9211_mipi_write( client, 0xff, 0x87);
	LT9211_mipi_write( client, 0x14, 0x08); //default value
	LT9211_mipi_write( client, 0x15, 0x00); //default value
	LT9211_mipi_write( client, 0x18, 0x0f);
	LT9211_mipi_write( client, 0x22, 0x08); //default value
	LT9211_mipi_write( client, 0x23, 0x00); //default value
	LT9211_mipi_write( client, 0x26, 0x0f);
}

void LT9211_MipiRxPhy(struct i2c_client *client)
{
#if 1//yelsin for WYZN CUSTOMER 2Lane 9211 mipi2RGB
	LT9211_mipi_write( client, 0xff, 0xd0);
	LT9211_mipi_write( client, 0x00, 0x00); // 0: 4 Lane / 1: 1 Lane / 2 : 2 Lane / 3: 3 Lane
#endif
	/* Mipi rx phy */
	LT9211_mipi_write( client, 0xff, 0x82);
	LT9211_mipi_write( client, 0x02, 0x44); //port A mipi rx enable

	LT9211_mipi_write( client, 0x05, 0x52); //port A CK lane swap 		//32
	LT9211_mipi_write( client, 0x0d, 0x26);
	LT9211_mipi_write( client, 0x17, 0x0c);
	LT9211_mipi_write( client, 0x1d, 0x0c);

	LT9211_mipi_write( client, 0x0a, 0x80);	//0xf7
	LT9211_mipi_write( client, 0x0b, 0x00); //0x77
	#ifdef _Mipi_PortA_
	/*port a*/
	LT9211_mipi_write( client, 0x07, 0x9f); //port clk enable
	LT9211_mipi_write( client, 0x08, 0xfc); //port lprx enable
	#endif
	#ifdef _Mipi_PortB_
	/*port a*/
	LT9211_mipi_write( client, 0x07, 0x9f); //port clk enable
	LT9211_mipi_write( client, 0x08, 0xfc); //port lprx enable
	/*port b*/
	LT9211_mipi_write( client, 0x0f, 0x9F);
	LT9211_mipi_write( client, 0x10, 0xfc);
	LT9211_mipi_write( client, 0x04, 0xa1);
	#endif
	/*port diff swap*/
	LT9211_mipi_write( client, 0x09, 0x01); //port a diff swap
	LT9211_mipi_write( client, 0x11, 0x01); //port b diff swap	

	/*port lane swap*/
	LT9211_mipi_write( client, 0xff, 0x86);
	LT9211_mipi_write( client, 0x33, 0x1b); //port a lane swap	1b:no swap	
	LT9211_mipi_write( client, 0x34, 0x1b); //port b lane swap 1b:no swap

	#ifdef CSI_INPUTDEBUG
	LT9211_mipi_write( client, 0xff, 0xd0);
	LT9211_mipi_write( client, 0x04, 0x10);	//bit4-enable CSI mode
	LT9211_mipi_write( client, 0x21, 0xc6);
	#endif
}

void LT9211_MipiRxDigital(struct i2c_client *client)
{
    LT9211_mipi_write( client,0xff,0x86);
#ifdef _Mipi_PortA_
    LT9211_mipi_write( client,0x30,0x85); //mipirx HL swap
#endif

#ifdef _Mipi_PortB_
	LT9211_mipi_write( client,0x30,0x8f); //mipirx HL swap
#endif

	LT9211_mipi_write( client,0xff,0xD8);
#ifdef _Mipi_PortA_
	LT9211_mipi_write( client,0x16,0x00); //mipirx HL swap
#endif

#ifdef _Mipi_PortB_
	LT9211_mipi_write( client,0x16,0x80); //mipirx HL swap
#endif	

	LT9211_mipi_write( client,0xff,0xd0);
	LT9211_mipi_write( client,0x43,0x12); //rpta mode enable,ensure da_mlrx_lptx_en=0

	LT9211_mipi_write( client,0x02,MIPI_SETTLE_VALUE); //mipi rx controller	//settleֵ		  
}

void LT9211_SetVideoTiming(struct i2c_client *client,struct video_timing *video_format)		//kxz
{
	LT9211_mipi_write( client,0xff,0xd0);
	LT9211_mipi_write( client,0x0d,(u8)(video_format->vtotal>>8)); //vtotal[15:8]
	LT9211_mipi_write( client,0x0e,(u8)(video_format->vtotal)); //vtotal[7:0]
	LT9211_mipi_write( client,0x0f,(u8)(video_format->vact>>8)); //vactive[15:8]
	LT9211_mipi_write( client,0x10,(u8)(video_format->vact)); //vactive[7:0]
	LT9211_mipi_write( client,0x15,(u8)(video_format->vs)); //vs[7:0]
	LT9211_mipi_write( client,0x17,(u8)(video_format->vfp>>8)); //vfp[15:8]
	LT9211_mipi_write( client,0x18,(u8)(video_format->vfp)); //vfp[7:0]	

	LT9211_mipi_write( client,0x11,(u8)(video_format->htotal>>8)); //htotal[15:8]
	LT9211_mipi_write( client,0x12,(u8)(video_format->htotal)); //htotal[7:0]
	LT9211_mipi_write( client,0x13,(u8)(video_format->hact>>8)); //hactive[15:8]
	LT9211_mipi_write( client,0x14,(u8)(video_format->hact)); //hactive[7:0]
	LT9211_mipi_write( client,0x16,(u8)(video_format->hs)); //hs[7:0]
	LT9211_mipi_write( client,0x19,(u8)(video_format->hfp>>8)); //hfp[15:8]
	LT9211_mipi_write( client,0x1a,(u8)(video_format->hfp)); //hfp[7:0]	
}

void LT9211_TimingSet(struct i2c_client *client)
{
	u32 hact = 0;
	u32 vact = 0;
	char fmt = 0;
	u32 pa_lpn = 0;
	char read_val = 0,read_val1 = 0;

	LT9211_mipi_write( client,0xff,0xd0);

	read_val=LT9211_read( client,0x82);
	read_val1=LT9211_read( client,0x83);

	hact = (read_val<<8) + read_val1 ;
	hact = hact/3;

	fmt=LT9211_read( client,0x84);
	fmt = fmt & 0x0f;

	read_val=LT9211_read( client,0x85);
	read_val1=LT9211_read( client,0x86);
	vact = (read_val<<8) + read_val1 ;

	pa_lpn=LT9211_read( client,0x9c);
	lt9211_printk("hact = %d\n",hact);
	lt9211_printk("vact = %d\n",vact);
	lt9211_printk("fmt = %d \n", fmt);
	lt9211_printk("pa_lpn = %d \n", pa_lpn);

	if ((hact == video_1920x1080_60Hz.hact ) &&( vact == video_1920x1080_60Hz.vact ))
   	{
		VideoFormat = video_1920x1080_60Hz_vic;
		LT9211_SetVideoTiming(client,&video_1920x1080_60Hz);
   	}
	else
	{
		VideoFormat = video_none;
		lt9211_printk("video_none \n");
	}
}

void LT9211_MipiRxPll(struct i2c_client *client)
{
	/* dessc pll */
	LT9211_mipi_write( client,0xff,0x82);
	LT9211_mipi_write( client,0x2d,0x48);
	LT9211_mipi_write( client,0x35,0x81);//0x82--720P; 0X81--1080P@60
}

void LT9211_MipiPcr(struct i2c_client *client)
{
	u8 loopx = 0;
	u8 Pcr_M = 0;
	u8 Pcr_overflow = 0;
	u8 Pcr_underflow = 0;
	char val = 0;

	Pcr_M = 0x17; //16、 17、18
	Pcr_overflow = 0x19; //0x1f
	Pcr_underflow = 0x10; //0x0d
	LT9211_mipi_write( client,0xff,0xd0);
	LT9211_mipi_write( client,0x26,Pcr_M);

	LT9211_mipi_write( client,0x2d,Pcr_overflow); //PCR M overflow limit setting.
	LT9211_mipi_write( client,0x31,Pcr_underflow); //PCR M underflow limit setting.
	lt9211_printk("%s \n", __FUNCTION__);
	lt9211_printk("Pcr_M = %d \n", Pcr_M);
	lt9211_printk("overflow limit setting = %d\n", Pcr_overflow);
	lt9211_printk("underflow limit setting = %d\n", Pcr_underflow);

       #if 0            
       LT9211_mipi_write( client,0x23,0x20);
	LT9211_mipi_write( client,0x38,0x02);
	LT9211_mipi_write( client,0x39,0x04);
	LT9211_mipi_write( client,0x3a,0x08);
	LT9211_mipi_write( client,0x3b,0x10);
	LT9211_mipi_write( client,0x3f,0x04);
	LT9211_mipi_write( client,0x40,0x08);
	LT9211_mipi_write( client,0x41,0x10);
	#endif

                     
	LT9211_mipi_write( client,0x0c,0x80);
	LT9211_mipi_write( client,0x1c,0x80);
	LT9211_mipi_write( client,0x24,0x31);
	LT9211_mipi_write( client,0x25,0x80);
	LT9211_mipi_write( client,0x2a,0x03);
	LT9211_mipi_write( client,0x21,0x44);
	LT9211_mipi_write( client,0x22,0x00);
	LT9211_mipi_write( client,0x0a,0x02);

	LT9211_mipi_write( client,0x23,0x80);
	LT9211_mipi_write( client,0x38,0x02);
	LT9211_mipi_write( client,0x39,0x04);
	LT9211_mipi_write( client,0x3a,0x08);
	LT9211_mipi_write( client,0x3b,0x60);
	LT9211_mipi_write( client,0x3f,0x04);
	LT9211_mipi_write( client,0x40,0x08);
	LT9211_mipi_write( client,0x41,0x10);
	LT9211_mipi_write( client,0x42,0x20);
	

	LT9211_mipi_write( client,0xff,0x81);
	LT9211_mipi_write( client,0x0B,0xEF);
	LT9211_mipi_write( client,0x0B,0xFF);
	
	for(loopx = 0; loopx < 10; loopx++) //Check pcr_stable
	{
		mdelay(200);
		LT9211_mipi_write( client,0xff,0xd0);
		mdelay(200);
		val=LT9211_read( client,0x87);
		val=val&0x08;
		if(val)
		{
			lt9211_printk("LT9211 pcr stable\n");
			printk("====Pcr_M:%d===========kxz==\n", Pcr_M);
			break;
		}
		else
		{
			printk("LT9211 pcr unstable!!!!\n");
		}
	}

	LT9211_mipi_write(client,0xff,0xd0);
	printk("stable M = %d\n", (LT9211_read(client,0x94) & 0x7F));
}

void LT9211_stable(struct i2c_client *client)
{
	u8 loopx = 0;
	u8 Pcr_M = 0;
	char val = 0;
	Pcr_M = 0x17; //16、 17、18

	LT9211_mipi_write( client,0xff,0xd0);
	LT9211_mipi_write( client,0x26,Pcr_M);

	lt9211_printk("%s \n", __FUNCTION__);
	lt9211_printk("Pcr_M = %d \n", Pcr_M);

	
	for(loopx = 0; loopx < 10; loopx++) //Check pcr_stable
	{
		mdelay(20);
		LT9211_mipi_write( client,0xff,0xd0);
		mdelay(20);
		val=LT9211_read( client,0x87);
		val=val&0x08;
		if(val)
		{
			lt9211_printk("LT9211 pcr stable\n");
			printk("====Pcr_M:%d===========kxz==\n", Pcr_M);
			break;
		}
		else
		{
			printk("LT9211 pcr unstable!!!!\n");
		}
	}

	LT9211_mipi_write(client,0xff,0xd0);
	printk("stable M = %d\n", (LT9211_read(client,0x94) & 0x7F));
}

void LT9211_TxDigital(struct i2c_client *client)
{
	LT9211_mipi_write( client,0xff,0x85); /* lvds tx controller */
	LT9211_mipi_write( client,0x59,0x50);
	LT9211_mipi_write( client,0x5a,0xaa);
	LT9211_mipi_write( client,0x5b,0xaa);

	LT9211_mipi_write( client,0x5c,0x05); //lvdstx port sel 01:dual;00:single

	LT9211_mipi_write( client,0x88,0x40); //0x50
	LT9211_mipi_write( client,0xa1,0x77);
	LT9211_mipi_write( client,0xff,0x86);
	LT9211_mipi_write( client,0x40,0x40); //tx_src_sel
	/*port src sel*/
	LT9211_mipi_write( client,0x41,0x34);
	LT9211_mipi_write( client,0x42,0x10);
	LT9211_mipi_write( client,0x43,0x23); //pt0_tx_src_sel
	LT9211_mipi_write( client,0x44,0x41);
	LT9211_mipi_write( client,0x45,0x02); //pt1_tx_src_scl

#ifdef lvds_format_JEIDA
	LT9211_mipi_write( client,0xff,0x85);
	LT9211_mipi_write( client,0x59,0xd0);
	LT9211_mipi_write( client,0xff,0xd8);
	LT9211_mipi_write( client,0x11,0x40);
#endif

}

void LT9211_TxPhy(struct i2c_client *client)
{
	LT9211_mipi_write( client,0xff,0x82);
	/* dual-port lvds tx phy */
	LT9211_mipi_write( client,0x62,0x00); //ttl output disable
	LT9211_mipi_write( client,0x3b,0xb8);

	// HDMI_WriteI2C_Byte(0x3b,0xb8); //dual port lvds enable	
	LT9211_mipi_write( client,0x3e,0x92);
	LT9211_mipi_write( client,0x3f,0x48);
	LT9211_mipi_write( client,0x40,0x00); //modify by kxz for emc //old data 0x31
	LT9211_mipi_write( client,0x43,0x80);
	LT9211_mipi_write( client,0x44,0x00);
	LT9211_mipi_write( client,0x45,0x00);
	LT9211_mipi_write( client,0x49,0x00);
	LT9211_mipi_write( client,0x4a,0x01);
	LT9211_mipi_write( client,0x4e,0x00);
	LT9211_mipi_write( client,0x4f,0x00);
	LT9211_mipi_write( client,0x50,0x00);
	LT9211_mipi_write( client,0x53,0x00);
	LT9211_mipi_write( client,0x54,0x01);
	LT9211_mipi_write( client,0xff,0x81);
	LT9211_mipi_write( client,0x20,0x7b);
	LT9211_mipi_write( client,0x20,0xff); //mlrx mltx calib reset
}

void LT9211_Txpll(struct i2c_client *client)
{
	u8 loopx = 0;
	char val = 0;

	LT9211_mipi_write( client,0xff,0x82);
	LT9211_mipi_write( client,0x36,0x01); //b7:txpll_pd

	LT9211_mipi_write( client,0x37,0x2a);

	LT9211_mipi_write( client,0x38,0x06);
	LT9211_mipi_write( client,0x39,0x30);
	LT9211_mipi_write( client,0x3a,0x0e); //0x8e
	LT9211_mipi_write( client,0xff,0x87);
	LT9211_mipi_write( client,0x37,0x0e); //0x14

	//modify by kxz
	//LT9211_mipi_write( client,0x2f,0x16); //modify by kxz for Spread Spectrum //06 close 16 open
	LT9211_mipi_write( client,0x30,0x03);
	LT9211_mipi_write( client,0x31,0x41);
	LT9211_mipi_write( client,0x32,0x00);
	LT9211_mipi_write( client,0x33,0x0b);
	LT9211_mipi_write( client,0x34,0x00);
	LT9211_mipi_write( client,0x35,0x16);
	LT9211_mipi_write( client,0x2f,0x16); //modify by mlk for Spread Spectrum //06 close 16 open


	LT9211_mipi_write( client,0x13,0x00);
	LT9211_mipi_write( client,0x13,0x80);
	//modify by kxz

	mdelay(100);
	for(loopx = 0; loopx < 10; loopx++) //Check Tx PLL cal
	{
		LT9211_mipi_write( client,0xff,0x87);
		val=LT9211_read( client,0x1f);
		if(val & 0x80)
		{
			val=LT9211_read( client,0x20);
			if(val & 0x80)
			{
				lt9211_printk("LT9211 tx pll lock\n");
			}
			else
			{
				lt9211_printk("LT9211 tx pll unlocked\n");
			}
			lt9211_printk("LT9211 tx pll cal done\n");
			break;
		}
		else
		{
			lt9211_printk("LT9211 tx pll unlocked\n");
		}
	}

	lt9211_printk("system success\n");
}

void LT9211_RXCSC(struct i2c_client *client)
{
	LT9211_mipi_write( client,0xff,0xf9);
	LT9211_mipi_write( client,0x86,0x00);
	LT9211_mipi_write( client,0x87,0x00);
}

void LT9211_ClockCheckDebug(struct i2c_client *client)
{
	int fm_value = 0;
	char val = 0;
	LT9211_mipi_write( client,0xff,0x86);
	LT9211_mipi_write( client,0x00,0x0a);
    fm_value = 0;
	val=LT9211_read( client,0x08);
	fm_value = (val & 0x0f);
	fm_value = (fm_value<<8) ;
	val=LT9211_read( client,0x09);
	fm_value = fm_value + val;
	fm_value = (fm_value<<8) ;
	val=LT9211_read( client,0x0a);
	fm_value = fm_value + val;
	lt9211_printk("dessc pixel clock: %d \n",fm_value);
}

void LT9211_VideoCheckDebug(struct i2c_client *client)
{
	LT9211_info_t *lt9211 = i2c_get_clientdata(client);
	
	char sync_polarity = 0;
	char val = 0,val1 = 0;
	lt9211_printk(" @@@@@@@@@@@@@@ _____ %s \n",__FUNCTION__);
	LT9211_mipi_write( client,0xff,0x86);

	sync_polarity=LT9211_read( client,0x70);

	lt9211 -> vs=LT9211_read( client,0x71);

	val=LT9211_read( client,0x72);
	val1=LT9211_read( client,0x73);
	lt9211 -> hs = (val<<8) + val1;

	lt9211 -> vbp=LT9211_read( client,0x74);
	lt9211 -> vfp=LT9211_read( client,0x75);

	val=LT9211_read( client,0x76);
	val1=LT9211_read( client,0x77);
	lt9211 -> hbp = (val<<8) + val1;

	val=LT9211_read( client,0x78);
	val1=LT9211_read( client,0x79);
	lt9211 -> hfp = (val<<8) + val1;

	val=LT9211_read( client,0x7A);
	val1=LT9211_read( client,0x7B);
	lt9211 -> vtotal = (val<<8) + val1;

	val=LT9211_read( client,0x7C);
	val1=LT9211_read( client,0x7D);
	lt9211 -> htotal = (val<<8) + val1;

	val=LT9211_read( client,0x7E);
	val1=LT9211_read( client,0x7F);
	lt9211 -> vact = (val<<8) + val1;

	val=LT9211_read( client,0x80);
	val1=LT9211_read( client,0x81);
	lt9211 -> hact = (val<<8) + val1;

	lt9211_printk("sync_polarity = %d\n", sync_polarity);
	lt9211_printk("hfp %d hs %d , hbp %d , hact %d, htotal =%d\n", lt9211 -> hfp, lt9211 -> hs, lt9211 -> hbp, lt9211 -> hact, lt9211 -> htotal);
	lt9211_printk("vfp %d, vs %d, vbp %d, vact %d, vtotal = %d\n", lt9211 -> vfp, lt9211 -> vs, lt9211 -> vbp, lt9211 -> vact, lt9211 -> vtotal);

}

void LT9211_BT_Set(struct i2c_client *client)
{
	LT9211_info_t *lt9211 = i2c_get_clientdata(client);
	u16 tmp_data = 0;
	if( (LT9211_OutPutModde == OUTPUT_BT1120_16BIT) || (LT9211_OutPutModde == OUTPUT_BT656_8BIT) )
	{
		tmp_data = lt9211 -> hs + lt9211 -> hbp;
		LT9211_mipi_write( client,0xff,0x85);
		LT9211_mipi_write( client,0x61,(u8)(tmp_data>>8));
		LT9211_mipi_write( client,0x62,(u8)tmp_data);
		LT9211_mipi_write( client,0x63,(u8)(lt9211 -> hact>>8));
		LT9211_mipi_write( client,0x64,(u8)lt9211 -> hact);
		LT9211_mipi_write( client,0x65,(u8)(lt9211 -> htotal>>8));
		LT9211_mipi_write( client,0x66,(u8)lt9211 -> htotal);
		tmp_data = lt9211 -> vs + lt9211 -> vbp;
		LT9211_mipi_write( client,0x67,(u8)tmp_data);
		LT9211_mipi_write( client,0x68,0x00);
		LT9211_mipi_write( client,0x69,(u8)(lt9211 -> vact>>8));
		LT9211_mipi_write( client,0x6a,(u8)lt9211 -> vact);
		LT9211_mipi_write( client,0x6b,(u8)(lt9211 -> vtotal>>8));
		LT9211_mipi_write( client,0x6c,(u8)lt9211 -> vtotal);
	}
}

void LT9211_Pattern(struct i2c_client *client, struct video_timing *video_format)
{
    u32 pclk_khz = 0;
	u8 dessc_pll_post_div = 0;
	u32 pcr_m = 0;
	u32 pcr_k = 0;
	printk("============LT9211_Pattern============kxz=====\n");
    pclk_khz = video_format->pclk_khz;

    LT9211_mipi_write( client,0xff,0xf9);
	LT9211_mipi_write( client,0x3e,0x80);

    LT9211_mipi_write( client,0xff,0x85);
	LT9211_mipi_write( client,0x88,0xc0); //0x90:TTL RX-->Mipi TX  ; 0xd0:lvds RX->MipiTX  0xc0:Chip Video pattern gen->Lvd TX

    LT9211_mipi_write( client,0xa1,0x64);
    LT9211_mipi_write( client,0xa2,0xff);

	LT9211_mipi_write( client,0xa3,(u8)((video_format->hs+video_format->hbp)/256));
	LT9211_mipi_write( client,0xa4,(u8)((video_format->hs+video_format->hbp)%256)); //h_start
	
	LT9211_mipi_write( client,0xa5,(u8)((video_format->vs+video_format->vbp)%256)); //v_start

	LT9211_mipi_write( client,0xa6,(u8)(video_format->hact/256));
	LT9211_mipi_write( client,0xa7,(u8)(video_format->hact%256)); //hactive

	LT9211_mipi_write( client,0xa8,(u8)(video_format->vact/256));
	LT9211_mipi_write( client,0xa9,(u8)(video_format->vact%256)); //vactive

	LT9211_mipi_write( client,0xaa,(u8)(video_format->htotal/256));
	LT9211_mipi_write( client,0xab,(u8)(video_format->htotal%256)); //htotal

	LT9211_mipi_write( client,0xac,(u8)(video_format->vtotal/256));
	LT9211_mipi_write( client,0xad,(u8)(video_format->vtotal%256)); //vtotal

	LT9211_mipi_write( client,0xae,(u8)(video_format->hs/256)); 
	LT9211_mipi_write( client,0xaf,(u8)(video_format->hs%256)); //hsa

	LT9211_mipi_write( client,0xb0,(u8)(video_format->vs%256)); //vsa

	//dessc pll to generate pixel clk
	LT9211_mipi_write( client,0xff,0x82); //dessc pll
	LT9211_mipi_write( client,0x2d,0x48); //pll ref select xtal 

	if(pclk_khz < 44000)
	{
		LT9211_mipi_write( client,0x35,0x83);
		dessc_pll_post_div = 16;
	}

	else if(pclk_khz < 88000)
	{
		LT9211_mipi_write( client,0x35,0x82);
		dessc_pll_post_div = 8;
	}

	else if(pclk_khz < 176000)
	{
		LT9211_mipi_write( client,0x35,0x81);
		dessc_pll_post_div = 4;
	}

	else if(pclk_khz < 352000)
	{
		LT9211_mipi_write( client,0x35,0x80);
		dessc_pll_post_div = 0;
	}

	pcr_m = (pclk_khz * dessc_pll_post_div) / 25;
	pcr_k = pcr_m % 1000;
	pcr_m = pcr_m / 1000;
	pcr_k <<= 14;

	//pixel clk
	LT9211_mipi_write( client,0xff,0xd0); //pcr
	LT9211_mipi_write( client,0x2d,0x7f);
	LT9211_mipi_write( client,0x31,0x00);

	LT9211_mipi_write( client,0x26,0x80 | ((u8)pcr_m));
	LT9211_mipi_write( client,0x27,(u8)((pcr_k >> 16) & 0xff)); //K
	LT9211_mipi_write( client,0x28,(u8)((pcr_k >> 8) & 0xff)); //K
	LT9211_mipi_write( client,0x29,(u8)(pcr_k & 0xff)); //K
}


static void lt9211_init(struct i2c_client *client)
{
	LT9211_ChipID(client);
	LT9211_SystemInt(client);
	LT9211_MipiRxPhy(client);
	LT9211_MipiRxDigital(client);
	LT9211_MipiRxPll(client);
	LT9211_MipiPcr(client);
	LT9211_TimingSet(client);
	LT9211_TxDigital(client);
	LT9211_TxPhy(client);
#ifdef LT9211_TEST_ON
	LT9211_Pattern(client, &video_1920x1080_60Hz);
#endif
	mdelay(10);
	LT9211_Txpll(client);
	LT9211_RXCSC(client);
	LT9211_ClockCheckDebug(client);
	LT9211_VideoCheckDebug(client);
	LT9211_BT_Set(client);

}

static void lt9211_power_on(LT9211_info_t* lt9211)
{
	gpio_direction_output(lt9211->enable_pin, 1);
	msleep(10);
	gpio_direction_output(lt9211->reset_pin, 0);
	msleep(10);
	gpio_direction_output(lt9211->reset_pin, 1);
}

static void lt9211_power_off(LT9211_info_t* lt9211)
{
	gpio_direction_output(lt9211->reset_pin, 0);
	msleep(20);
	gpio_direction_output(lt9211->enable_pin, 0);
}


static void LT9211_resume_work(struct work_struct *work)
{
	LT9211_info_t* lt9211 = container_of(work, struct LT9211, Lt9211_resume_work);
	struct i2c_client *client = lt9211 -> client;

	printk("################## %s =====kxz======\n", __FUNCTION__);
	client = g_client;

	mutex_lock(&update_lock);
	lt9211_power_on(lt9211);

	gpio_direction_output(LCD_PWR_PIN, 1);
	msleep(10);
	lt9211_init(client);
	gpio_direction_output(BL_EN_PIN, 1);
	mutex_unlock(&update_lock);
}

static void LT9211_suspend_work(struct work_struct *work)
{
	LT9211_info_t* lt9211;
	lt9211 = container_of(work, struct LT9211, Lt9211_suspend_work);

	printk("################## %s =====kxz======\n", __FUNCTION__);
	mutex_lock(&update_lock);
	gpio_direction_output(BL_EN_PIN, 0);
	//msleep(200);
	lt9211_power_off(lt9211);

	gpio_direction_output(LCD_PWR_PIN, 0);
	mutex_unlock(&update_lock);
	msleep(1200);
}
/*
static int timing_thread(void *data)
{
	while(1)
	{
		while(lt9211_suspend())	//lcm_suspend
		{
			printk("======mipi to lvds=======enter==lt9211_suspend============kxz=====\n");
			mutex_lock(&update_lock);
			gpio_direction_output(BL_EN_PIN, 0);
			//msleep(200);
			lt9211_power_off(g_LT9211);

			gpio_direction_output(LCD_PWR_PIN, 0);
			msleep(1200);
			mutex_unlock(&update_lock);
			break;
		}
		while(lt9211_resume())	//lcm_resume
		{
			printk("======mipi to lvds=======enter==lt9211_resume============kxz=====\n");
			mutex_lock(&update_lock);
			lt9211_power_on(g_LT9211);

			gpio_direction_output(LCD_PWR_PIN, 1);
			msleep(10);
			lt9211_init(g_client);

			gpio_direction_output(BL_EN_PIN, 1);
			mutex_unlock(&update_lock);
			break;
		}

	}
	
	return 0;
}
*/

static ssize_t lt9211_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	LT9211_info_t *lt9211 = i2c_get_clientdata(client);

	char val = 0,val1 = 0;

	LT9211_mipi_write( client,0xff,0x86);

	lt9211 -> vs=LT9211_read( client,0x71);

	val=LT9211_read( client,0x72);
	val1=LT9211_read( client,0x73);
	lt9211 -> hs = (val<<8) + val1;

	lt9211 -> vbp=LT9211_read( client,0x74);
	lt9211 -> vfp=LT9211_read( client,0x75);

	val=LT9211_read( client,0x76);
	val1=LT9211_read( client,0x77);
	lt9211 -> hbp = (val<<8) + val1;

	val=LT9211_read( client,0x78);
	val1=LT9211_read( client,0x79);
	lt9211 -> hfp = (val<<8) + val1;

	val=LT9211_read( client,0x7A);
	val1=LT9211_read( client,0x7B);
	lt9211 -> vtotal = (val<<8) + val1;

	val=LT9211_read( client,0x7C);
	val1=LT9211_read( client,0x7D);
	lt9211 -> htotal = (val<<8) + val1;

	val=LT9211_read( client,0x7E);
	val1=LT9211_read( client,0x7F);
	lt9211 -> vact = (val<<8) + val1;

	val=LT9211_read( client,0x80);
	val1=LT9211_read( client,0x81);
	lt9211 -> hact = (val<<8) + val1;

	LT9211_mipi_write( client,0xff,0xd0);
	val=LT9211_read( client,0x87);
	val1=val&0x08;
	if(val1)
	{
		return sprintf(buf, "LT9211 pcr stable d087 = %d\nhfp %d hs %d , hbp %d , hact %d, htotal =%d\nvfp %d, vs %d, vbp %d, vact %d, vtotal = %d\n",
			val, lt9211 -> hfp, lt9211 -> hs, lt9211 -> hbp, lt9211 -> hact, lt9211 -> htotal, lt9211 -> vfp, lt9211 -> vs, lt9211 -> vbp, lt9211 -> vact, lt9211 -> vtotal);
	}
	else
	{
		return sprintf(buf, "LT9211 pcr unstable!!!!!! d087 = %d\nhfp %d hs %d , hbp %d , hact %d, htotal =%d\nvfp %d, vs %d, vbp %d, vact %d, vtotal = %d\n",
			val, lt9211 -> hfp, lt9211 -> hs, lt9211 -> hbp, lt9211 -> hact, lt9211 -> htotal, lt9211 -> vfp, lt9211 -> vs, lt9211 -> vbp, lt9211 -> vact, lt9211 -> vtotal);
	}

}


static ssize_t lt9211_reg_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int regaddr ,regvalue;
	
	struct i2c_client *client = to_i2c_client(dev);
     
	sscanf(buf, "0x%x 0x%x", &regaddr, &regvalue);

	printk("==mlk===regaddr = 0x%x, regvalue = 0x%x\n",regaddr,regvalue);

	LT9211_mipi_write( client, regaddr, regvalue);
 
	return count;
}


static DEVICE_ATTR(lt9211_reg, 0644, lt9211_reg_show, lt9211_reg_store);

//tqq add start 
int brightness = -1;
int flag_suspend = 0;

static ssize_t lt9211_lcd_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	printk("%s,tqq test 0506 brightness=%d\n", __func__, brightness);
	return sprintf(buf, "brightness %d\n", brightness);

}


static ssize_t lt9211_lcd_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	     
	sscanf(buf, "%d", &brightness);

	if (brightness == 0 && flag_suspend == 0){
		flag_suspend = 1;
		schedule_work(&g_LT9211->Lt9211_suspend_work);
	}
	
	if (flag_suspend == 1 && brightness != 0){
			lt9211_power_on(g_LT9211);
			gpio_direction_output(LCD_PWR_PIN, 1);
			msleep(10);
			lt9211_init(g_client);
			gpio_direction_output(BL_EN_PIN, 1);
			
		flag_suspend = 0;
	}
 
	return count;
}

static DEVICE_ATTR(brightness, 0644, lt9211_lcd_show, lt9211_lcd_store);
//tqq add end

static struct attribute *lt9211_attributes[] = {
	&dev_attr_lt9211_reg.attr,
	&dev_attr_brightness.attr,
	NULL
};

static const struct attribute_group lt9211_attr_group = {
	.attrs = lt9211_attributes,
};

static int lt9211_parse_dts(LT9211_info_t* lt9211, struct device *dev)
{
	int ret = -1;

	lt9211->reset_pin = of_get_named_gpio(dev->of_node, "reset_gpio", 0);
	lt9211->enable_pin = of_get_named_gpio(dev->of_node, "enable_gpio", 0);
	BL_EN_PIN = of_get_named_gpio(dev->of_node, "bl_enable", 0);
	LCD_PWR_PIN = of_get_named_gpio(dev->of_node, "lcd_power", 0);

	ret = gpio_request(lt9211->enable_pin, "lt9211_enable_gpio");
	if (ret < 0)
	{
		printk("%s(): lt9211_enable_gpio request failed %d ====kxz===\n", __func__, ret);
		return ret;
	}

	ret = gpio_request(lt9211->reset_pin, "lt9211_rst_gpio");
	if (ret < 0)
	{
		printk("%s(): lt9211_rst_gpio request failed %d ====kxz===\n", __func__, ret);
		return ret;
	}

	ret = gpio_request(BL_EN_PIN, "BL_EN_PIN");
	if (ret < 0) 
	{
		printk("%s(): lcd_en_gpio request failed %d ====kxz===\n",__func__, ret);
		return ret;
	}

	ret = gpio_request(LCD_PWR_PIN, "LCD_PWR_PIN");
	if (ret < 0)
	{
		printk("%s(): gpio_lcd_power request failed %d ====kxz===\n",__func__, ret);
		return ret;
	}

	return 0;
}

#ifdef LT9211_TEST_ON
static void LT9211_self_test(LT9211_info_t* lt9211)
{
	lt9211_power_on(lt9211);
	gpio_direction_output(LCD_PWR_PIN, 1);

	msleep(10);
	lt9211_init(client);

	gpio_direction_output(BL_EN_PIN, 1);
}
#endif


int LT9211_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
	LT9211_info_t* lt9211;
	g_client = client;

	printk("======mipi to lvds=======enter==LT9211_probe============kxz=====\n");

	lt9211 = devm_kzalloc(&client->dev, sizeof(*lt9211), GFP_KERNEL);
	if (!lt9211)
		return -ENOMEM;

	g_LT9211 = lt9211;
	i2c_set_clientdata(client, lt9211);
	
	mutex_init(&update_lock);
//	timing_task = kthread_create(timing_thread, NULL, "timing_task");

	lt9211_parse_dts(lt9211, &client->dev);

#ifdef LT9211_TEST_ON
	LT9211_self_test(lt9211);
#endif
	LT9211_stable(client);
	ret = sysfs_create_group(&client->dev.kobj, &lt9211_attr_group);
	if(ret < 0){
		return -ENOMEM;
	}

	INIT_WORK(&lt9211->Lt9211_resume_work, LT9211_resume_work);
	INIT_WORK(&lt9211->Lt9211_suspend_work, LT9211_suspend_work);

//	wake_up_process(timing_task);
	printk("======mipi to lvds=======LT9211_probe===end=========kxz=====\n");
	
	return 0;
}

static struct of_device_id LT9211_dt_ids[] = {
	{.compatible = "LT9211"},
	{},
};

static struct i2c_device_id LT9211_id[] = {
	{"LT9211", 0},
	{},
};

int LT9211_suspend(struct device *dev)
{
	printk("################## %s =============================kxz======\n", __FUNCTION__);
	schedule_work(&g_LT9211->Lt9211_suspend_work);
	return 0;
}

int LT9211_resume(struct device *dev)
{
	printk("################## %s =============================kxz======\n", __FUNCTION__);
	schedule_work(&g_LT9211->Lt9211_resume_work);
	return 0;
}

static const struct dev_pm_ops lt9211_pm_ops = {
#ifdef CONFIG_PM_SLEEP
//	.suspend = LT9211_suspend,
//	.resume = LT9211_resume,
	.poweroff = LT9211_suspend,
	.restore = LT9211_resume,
#endif
};

struct i2c_driver LT9211_driver  = {
	.driver = {
		.owner	= THIS_MODULE,
		.pm		= &lt9211_pm_ops,
		.name	= "LT9211",
		.of_match_table = (LT9211_dt_ids),
	},
	.id_table	= LT9211_id,
	.probe      = LT9211_probe,
	.remove 	= NULL,
};


int __init LT9211_init(void)
{
	return i2c_add_driver(&LT9211_driver);
}

void __exit LT9211_exit(void)
{
	i2c_del_driver(&LT9211_driver);
}

MODULE_AUTHOR("xiangzhe.kong@tcl.com");
//late_initcall(LT9211_init);
module_init(LT9211_init);
module_exit(LT9211_exit);
MODULE_LICENSE("GPL");
