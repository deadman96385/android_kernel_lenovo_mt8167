/* include/linux/VCNL4200.h
 *
 * Copyright (C) 2015 Vishay Capella Microsystems Limited
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

#ifndef __LINUX_VCNL4200_H
#define __LINUX_VCNL4200_H

#define VCNL4200_I2C_NAME "vcnl4200"

/* Define Slave Address*/
#define	VCNL4200_slave_add	0xA2>>1

/*Define Command Code*/
#define		ALS_CONF		  0x00
#define		ALS_THDH  	  0x01
#define		ALS_THDL	    0x02
#define		PS_CONF1_2    0x03
#define		PS_CONF3      0x04
#define		PS_CANC       0x05
#define		PS_THDL       0x06
#define		PS_THDH       0x07
#define		PS_DATA       0x08
#define		ALS_DATA      0x09
#define		INT_FLAG      0x0D
#define		ID_REG        0x0E

/*vcnl4200*/
/*for ALS CONF command*/
#define VCNL4200_ALS_IT_50MS 	  (0 << 6)
#define VCNL4200_ALS_IT_100MS 	(1 << 6)
#define VCNL4200_ALS_IT_200MS 	(2 << 6)
#define VCNL4200_ALS_IT_400MS 	(3 << 6)
#define VCNL4200_ALS_PERS_1 		(0 << 2)
#define VCNL4200_ALS_PERS_2 		(1 << 2)
#define VCNL4200_ALS_PERS_4 		(2 << 2)
#define VCNL4200_ALS_PERS_8 		(3 << 2)
#define VCNL4200_ALS_INT_EN	 	  (1 << 1) /*enable/disable Interrupt*/
#define VCNL4200_ALS_INT_MASK	  0xFFFD
#define VCNL4200_ALS_SD			    (1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/
#define VCNL4200_ALS_SD_MASK		0xFFFE

/*for PS CONF1_2 command*/
#define VCNL4200_PS_INT_OFF	        (0 << 8) /*enable/disable Interrupt*/
#define VCNL4200_PS_INT_IN          (1 << 8)
#define VCNL4200_PS_INT_OUT         (2 << 8)
#define VCNL4200_PS_INT_IN_AND_OUT  (3 << 8)

#define VCNL4200_PS_INT_MASK        0xFCFF

#define VCNL4200_PS_DR_1_160  (0 << 6)
#define VCNL4200_PS_DR_1_320  (1 << 6)
#define VCNL4200_PS_DR_1_640  (2 << 6)
#define VCNL4200_PS_DR_1_1280 (3 << 6)
#define VCNL4200_PS_PERS_1 	  (0 << 4)
#define VCNL4200_PS_PERS_2 	  (1 << 4)
#define VCNL4200_PS_PERS_3 	  (2 << 4)
#define VCNL4200_PS_PERS_4 	  (3 << 4)
#define VCNL4200_PS_IT_1T 	  (0 << 1)
#define VCNL4200_PS_IT_1_5T   (1 << 1)
#define VCNL4200_PS_IT_2T 	  (2 << 1)
#define VCNL4200_PS_IT_4T 		(3 << 1)
#define VCNL4200_PS_IT_8T 		(4 << 1)
#define VCNL4200_PS_IT_9T 		(5 << 1)
#define VCNL4200_PS_SD	      (1 << 0)/*enable/disable PS func, 1:disable , 0: enable*/
#define VCNL4200_PS_SD_MASK   0xFFFE

/*for PS CONF3 command*/
#define VCNL4200_PS_MS_NORMAL          (0 << 13)
#define VCNL4200_PS_MS_LOGIC_ENABLE    (1 << 13)
#define VCNL4200_LED_I_50              (0 << 8)
#define VCNL4200_LED_I_75              (1 << 8)
#define VCNL4200_LED_I_100             (2 << 8)
#define VCNL4200_LED_I_120             (3 << 8)
#define VCNL4200_LED_I_140             (4 << 8)
#define VCNL4200_LED_I_160             (5 << 8)
#define VCNL4200_LED_I_180             (6 << 8)
#define VCNL4200_LED_I_200             (7 << 8)

#define VCNL4200_PS_MP_1               (0 << 6)
#define VCNL4200_PS_MP_2               (1 << 6)
#define VCNL4200_PS_MP_4               (2 << 6)
#define VCNL4200_PS_MP_8               (3 << 6)
#define VCNL4200_PS_SMART_PERS_ENABLE  (1 << 4)
#define VCNL4200_PS_ACTIVE_FORCE_MODE  (1 << 3)
#define VCNL4200_PS_ACTIVE_FORCE_TRIG  (1 << 2)

/*for INT FLAG*/
#define INT_FLAG_PS_SAFLAG           (1<<15)
#define INT_FLAG_PS_SPFLAG           (1<<14)
#define INT_FLAG_ALS_IF_L            (1<<13)
#define INT_FLAG_ALS_IF_H            (1<<12)
#define INT_FLAG_PS_IF_CLOSE         (1<<9)
#define INT_FLAG_PS_IF_AWAY          (1<<8)  

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

struct vcnl4200_platform_data {
	int intr;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t slave_addr;
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;	
};

#endif