/*
 * m41st85w.h
 *
 * Copyright (C) 2002 Intrinsyc Software Inc.
 * Copyright (C) 2003 Intel Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This driver is adapted from the DS1307 driver
 *
 */
#ifndef M41ST85W_H
#define M41ST85W_H

#define M41ST85W_I2C_SLAVE_ADDR	0x68

#define M41ST85W_RAM_ADDR_START	0x00
#define M41ST85W_RAM_ADDR_END	0x13
#define M41ST85W_RAM_SIZE 0x14

#define PROC_M41ST85W_NAME	"driver/m41st85w"

struct rtc_mem {
	unsigned int	loc;
	unsigned int	nr;
	unsigned char	*data;
};

#define M41ST85W_GETDATETIME	0
#define M41ST85W_SETTIME		1
#define M41ST85W_SETDATETIME	2
#define M41ST85W_GETCTRL		3
#define M41ST85W_SETCTRL		4
#define M41ST85W_MEM_READ		5
#define M41ST85W_MEM_WRITE	    6

/* Buffer descriptors */
#define M41ST85W_OFF_SEC		1
#define M41ST85W_OFF_MIN		2
#define M41ST85W_OFF_HOUR		3
#define M41ST85W_OFF_WDAY		4
#define M41ST85W_OFF_MDAY		5
#define M41ST85W_OFF_MON		6
#define M41ST85W_OFF_YEAR		7

#define M41ST85W_MASK_SEC		0x7f
#define M41ST85W_MASK_MIN		0x7f
#define M41ST85W_MASK_HOUR		0x3f
#define M41ST85W_MASK_WDAY		0x07
#define M41ST85W_MASK_MDAY		0x7f
#define M41ST85W_MASK_MON		0x3f
#define M41ST85W_MASK_YEAR		0xff

#define m41ST85w_parse_param(val, param, buf) \
{ val = (buf[ M41ST85W_OFF_##param ] & M41ST85W_MASK_##param ); val = bcd2bin(val); }

#define m41ST85w_set_param(buf, param, val) \
{ buf[ M41ST85W_OFF_##param ] = (val); buf[ M41ST85W_OFF_##param ] = bin2bcd(buf[ M41ST85W_OFF_##param ]); }

#define SQW_ENABLE	0x40	/* Square Wave Enable */
#define SQW_DISABLE	0x00	/* Square Wave disable */

#define RATE_32768HZ	0x10	/* Rate Select 32.768KHz */
#define RATE_8192HZ	    0x20	/* Rate Select 8.192KHz */
#define RATE_4096HZ	    0x30	/* Rate Select 4.096KHz */
#define RATE_2048HZ     0x40    /* Rate Select 2.048Khz */
#define RATE_1024HZ     0x50    /* Rate Select 1.024Khz */
#define RATE_512HZ      0x60    /* Rate Select 0.512Khz */
#define RATE_256HZ      0x70    /* Rate Select 0.256Khz */
#define RATE_128HZ      0x80    /* Rate Select 0.128Khz */
#define RATE_64HZ       0x90    /* Rate Select 0.064Khz */
#define RATE_32HZ       0xa0    /* Rate Select 0.032Khz */
#define RATE_16HZ       0xb0    /* Rate Select 0.016Khz */
#define RATE_8HZ        0xc0    /* Rate Select 0.008Khz */
#define RATE_4HZ        0xd0    /* Rate Select 0.004Khz */
#define RATE_2HZ        0xe0    /* Rate Select 0.0001Khz */
#define RATE_1HZ	    0xf0	/* Rate Select 1Hz */
#define RATE_0HZ        0x00    /* Rate Select None */

#define CLOCK_STOP	0x80	/* Clock Stop */
#define CLOCK_HALT  0x40    /* Clock Halt */

#endif
