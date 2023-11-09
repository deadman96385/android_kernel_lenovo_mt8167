/******************************************************************************
 * MODULE       : rohm_bh1749_i2c.h
 * FUNCTION     : Driver header for BH1749 Ambient Light Sensor(RGB) IC
 * AUTHOR       : Aaron Liu
 * MODIFICATION : Modified by Aaron Liu, Sep/28/2016
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2017 - ROHM CO.,LTD.
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
#ifndef _ROHM_BH1749_I2C_H_
#define _ROHM_BH1749_I2C_H_


/*-----------------------------------------------------*/

#define BH1749_DGB_SWITCH         // debug switch
#define  BH1749_TAG             "[ALS/PS]BH1749"

#ifdef BH1749_DGB_SWITCH
#define BH1749_DEBUG   1
#else
#define BH1749_DEBUG   0
#endif

#define  BH1749_ERR(f, a...)        do {printk(KERN_ERR BH1749_TAG "ERROR (%s(), %d):"   f, __func__,  __LINE__, ## a);} while (0)
#define  BH1749_WARNING(f, a...)    do {printk(KERN_ERR BH1749_TAG "(%s(), %d):"     f, __func__,  __LINE__, ## a);} while (0)
#define  BH1749_INFO(f, a...)       do {printk(KERN_ERR BH1749_TAG "INFO (%s(), %d):"   f, __func__,  __LINE__, ## a);} while (0)


#if BH1749_DEBUG
#define  BH1749_FUN()               do {printk(KERN_ERR BH1749_TAG "(%s(), %d)\n",         __func__,  __LINE__);} while (0)
#define  BH1749_DBG(f, a...)        do {printk(KERN_ERR BH1749_TAG "DEBUG (%s(), %d):" f, __func__,  __LINE__, ## a);} while (0)
#else
#define  BH1749_FUN()   do {} while (0)
#define  BH1749_DBG(f, a...)   do {} while (0)
#endif
/*-----------------------------------------------------*/

/************ define register for IC ************/
/************ BH1749 REGSTER  *******************/

#define BH1749_REG_SYSTEMCONTROL      (0x40)
#define BH1749_REG_MODECONTROL1       (0x41)
#define BH1749_REG_MODECONTROL2       (0x42)
#define BH1749_REG_RED_DATA           (0x50)
#define BH1749_REG_GREEN_DATA         (0x52)
#define BH1749_REG_BLUE_DATA          (0x54)
#define BH1749_REG_IR_DATA            (0x58)
#define BH1749_REG_GREEN2_DATA        (0x5A)
#define BH1749_REG_INTERRUPT          (0x60)
#define BH1749_REG_INT_PERSISTENCE    (0x61)
#define BH1749_REG_THRED_HIGH         (0x62)
#define BH1749_REG_THRED_LOW          (0x64)
#define BH1749_REG_MANUFACT_ID        (0x92)
#define BH1749_ID_VALUE               (0xE0)

#define BH1749_REG_RESET              (BH1749_REG_SYSTEMCONTROL)
#define BH1749_REG_GAIN_TIME          (BH1749_REG_MODECONTROL1)
#define BH1749_REG_ENABLE             (BH1749_REG_MODECONTROL2)



#define BH1749_INT_SOURCE_GREEN          (1 << 2)  /* Green channel */
#define BH1749_INT_ENABLE                (1)       /* INT pin enable */
#define BH1749_INT_DISABLE               (0)       /* INT pin disable */
#define BH1749_PERSISTENCE_VALUE         (0b10)    /* Interrupt status is updated if 4 consecutive threshold judgments are the same */
#define BH1749_INT_STATUS_ACTIVE         (1 << 7)  /* active status */


/************ define parameter for register ************/
#define BH1749_SW_RESET               (1 << 7)
#define BH1749_INT_RESET              (1 << 6)

#define BH1749_POWER_ON               (1 << 0)
#define BH1749_POWER_OFF              (0 << 0)

/* REG_MODECONTROL1(0x41) */
#define BH1749_MEASURE_120MS          (0b010)  //0x2
#define BH1749_MEASURE_240MS          (0b011)  //0x3
#define BH1749_MEASURE_35MS           (0b101)  //0x5

#define BH1749_120MS                  (120)
#define BH1749_240MS                  (240)
#define BH1749_35MS                   (30)

#define BH1749_1X                     (1)
#define BH1749_32X                    (32)


#define BH1749_RGB_GAIN_SHIFT_VALUE   (3)
#define BH1749_RGB_GAIN_X1            (0b01 << BH1749_RGB_GAIN_SHIFT_VALUE)   //0x8
#define BH1749_RGB_GAIN_X32           (0b11 << BH1749_RGB_GAIN_SHIFT_VALUE)   //0x18

#define BH1749_IR_GAIN_SHIFT_VALUE   (5)
#define BH1749_IR_GAIN_X1             (0b01 << BH1749_IR_GAIN_SHIFT_VALUE)   //0x20
#define BH1749_IR_GAIN_X32            (0b11 << BH1749_IR_GAIN_SHIFT_VALUE)   //0x60

/* REG_MODECONTROL2(0x42) */
#define BH1749_RGBC_EN_ON             (1 << 4)  /* RGBC measurement is active */
#define BH1749_RGBC_EN_OFF            (0 << 4)  /* RGBC measurement is inactive and becomes power down */

#define BH1749_RGBC_VALID_HIGH        (1 << 7)


// GAIN change automaticly according to the current rgb ir raw data
#define AUTO_GAIN  (1)
#define BH1749_GAIN_CHANGE_MAX          (60000)
#define BH1749_GAIN_CHANGE_MIN          (100)

#define BH1749_TIME_GAIN_MASK           (0xF8)           //reg41 bit0-2 is 0
#define BH1749_RGB_GAIN_MASK            (0xE7)           //reg41 bit3-4 is 0
#define BH1749_IR_GAIN_MASK             (0x9F)           //reg41 bit5-6 is 0

#define BH1749_TIME_VALUE(a)                 (a & 0x07)   //get bit0-2      
#define BH1749_RGB_GAIN_VALUE(a)            ((a & 0x18) >> BH1749_RGB_GAIN_SHIFT_VALUE)      //get bit3-4   
#define BH1749_IR_GAIN_VALUE(a)             ((a & 0x60) >> BH1749_IR_GAIN_SHIFT_VALUE)       //get bit5-6

#define REPORT_MAX_VALUE                    (2097120) //65535*32, the max value during 1X 
#define LUX_MAX_VALUE                       (65535)  
#define DATA_TRANSFER_COFF                  (32 * 120) //32X gain, 120 atime


/************ definition to dependent on sensor IC ************/
#define BH1749_I2C_NAME        ("bh1749_i2c")


/************ typedef struct ************/
/* structure to read data value from sensor */
typedef struct {
    unsigned int red;         /* data value of red data from sensor */
    unsigned int green;       /* data value of green data from sensor */
    unsigned int blue;        /* data value of blue data from sensor */
    unsigned int ir ;         /* data value of clear data from sensor */
} READ_DATA_ARG;


/* Color type */
typedef enum {
    GOLD = 0,
    WHITE,
    BLACK,
} COLOR_T;



#endif /* _ROHM_BH1749_I2C_H_ */
