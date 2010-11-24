/*
 * m41st85w.c
 *
 * Device driver for ST's Real Time Controller M41ST85W.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/bcd.h>

#define M41ST85W_REG_SECONDS	0x01
#define M41ST85W_REG_MINUTES	0x02
#define M41ST85W_REG_HOURS	0x03
#define M41ST85W_REG_DAY	0x04
#define M41ST85W_REG_DATE	0x05
#define M41ST85W_REG_MONTH	0x06
#define M41ST85W_REG_YEAR	0x07
#define M41ST85W_REG_ALMONTH	0x0A	/* Alarm month */
#define M41ST85W_REG_ALHOUR	0x0C	/* Alarm month */
#define M41ST85W_REG_SQW	0x13

#define SQW_ENABLE		0x40	/* Square Wave Enable */
#define SQW_DISABLE		0x00	/* Square Wave disable */

#define RATE_32768HZ		0x10	/* Rate Select 32.768KHz */
#define RATE_8192HZ		0x20	/* Rate Select 8.192KHz */
#define RATE_4096HZ		0x30	/* Rate Select 4.096KHz */
#define RATE_2048HZ		0x40    /* Rate Select 2.048Khz */
#define RATE_1024HZ		0x50    /* Rate Select 1.024Khz */
#define RATE_512HZ		0x60    /* Rate Select 0.512Khz */
#define RATE_256HZ		0x70    /* Rate Select 0.256Khz */
#define RATE_128HZ		0x80    /* Rate Select 0.128Khz */
#define RATE_64HZ		0x90    /* Rate Select 0.064Khz */
#define RATE_32HZ		0xa0    /* Rate Select 0.032Khz */
#define RATE_16HZ		0xb0    /* Rate Select 0.016Khz */
#define RATE_8HZ		0xc0    /* Rate Select 0.008Khz */
#define RATE_4HZ		0xd0    /* Rate Select 0.004Khz */
#define RATE_2HZ		0xe0    /* Rate Select 0.0001Khz */
#define RATE_1HZ		0xf0	/* Rate Select 1Hz */
#define RATE_0HZ		0x00    /* Rate Select None */

#define CLOCK_STOP		0x80	/* Clock Stop */
#define CLOCK_HALT		0x40    /* Clock Halt */



struct m41st85w {
	struct i2c_client *client;
	struct rtc_device *rtc;
};

static struct i2c_driver m41st85w_driver;

static int m41st85w_set_ctrl(struct i2c_client *client, int *cinfo)
{
	int ret;

	/* need to set square wave first */
	ret = i2c_smbus_write_byte_data(client, M41ST85W_REG_SQW, cinfo[1]);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(client, M41ST85W_REG_ALMONTH, cinfo[0]);
}

static void m41st85w_enable_clock(struct i2c_client *client, int enable)
{
	int buf1, buf2;
	int ctrl_info[2];
	int ret;

	if (enable) {
		ctrl_info[0] = SQW_ENABLE;
		ctrl_info[1] = RATE_32768HZ;
	} else {
		ctrl_info[0] = SQW_DISABLE;
		ctrl_info[1] = RATE_0HZ;
	}
	ret = m41st85w_set_ctrl(client, ctrl_info);
	if (ret < 0)
		printk(KERN_ERR "%s %d: m41st85w_set_ctrl failed\n",
				__func__, __LINE__);

	/* read Clock-Halt bit and second counter */
	buf1 = i2c_smbus_read_byte_data(client, M41ST85W_REG_ALHOUR);
	if (buf1 < 0)
		printk(KERN_ERR "%s %d: i2c_smbus_read_byte_data failed\n",
				__func__, __LINE__);

	buf2 = i2c_smbus_read_byte_data(client, M41ST85W_REG_SECONDS);
	if (buf2 < 0)
		printk(KERN_ERR "%s %d: i2c_smbus_read_byte_data failed\n",
				__func__, __LINE__);


	if (enable) {
		buf1 &= ~CLOCK_HALT; /* clear Clock-Halt bit */
		buf2 &= ~CLOCK_STOP;
	} else {
		buf1 |= CLOCK_HALT; /* set Clock-Halt bit */
		buf2 |= CLOCK_STOP;
	}

	ret = i2c_smbus_write_byte_data(client, M41ST85W_REG_ALHOUR, buf1);
	if (ret < 0)
		printk(KERN_ERR "%s %d: i2c_smbus_write_byte_data failed\n",
				__func__, __LINE__);

	ret = i2c_smbus_write_byte_data(client, M41ST85W_REG_SECONDS, buf2);
	if (ret < 0)
		printk(KERN_ERR "%s %d: i2c_smbus_write_byte_data failed\n",
				__func__, __LINE__);
}

static int m41st85w_read_time(struct device *dev, struct rtc_time *dt)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 buf[7];
	int ret;
	unsigned int year, month, day, week, hour, minute, second;

	ret = i2c_smbus_read_i2c_block_data(client, 0x01, 7, buf);

	if (ret < 0)
		return ret;
	if (ret < 7)
		return -EIO;

	second = buf[0];
	minute = buf[1];
	hour = buf[2];
	week = buf[3];
	day = buf[4];
	month = buf[5];
	year = buf[6];

	dt->tm_sec = bcd2bin(second & 0x7f);
	dt->tm_min = bcd2bin(minute & 0x7f);
	dt->tm_hour = bcd2bin(hour & 0x3f);
	dt->tm_wday = bcd2bin(week & 0x07);
	dt->tm_mday = bcd2bin(day & 0x7f);
	dt->tm_mon = bcd2bin(month & 0x3f);
	dt->tm_year = bcd2bin(year);

	/* fix up values */
	/* dt->tm_mon is zero-based */
	dt->tm_mon--;
	/* year is 1900 + dt->tm_year */
	if (dt->tm_year < 95)
		dt->tm_year += 100;

	dt->tm_isdst = 0;

	return rtc_valid_tm(dt);
}

static int
		m41st85w_set_time(struct device *dev, struct rtc_time *dt)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 buf[7];

	buf[0] = bin2bcd(dt->tm_sec);
	buf[1] = bin2bcd(dt->tm_min);
	buf[2] = bin2bcd(dt->tm_hour);
	buf[3] = bin2bcd(dt->tm_wday);
	buf[4] = bin2bcd(dt->tm_mday);
	buf[5] = bin2bcd(dt->tm_mon + 1);
	/* The year only ranges from 0-99, we are being passed an offset
	* from 1900, and the chip calulates leap years based on 2000,
	* thus we adjust by 100.
	*/

	buf[6] = bin2bcd(dt->tm_year - 100);

	return i2c_smbus_write_i2c_block_data(client,
					      0x01, 7, buf);
}

static struct rtc_class_ops m41st85w_rtc_ops = {
	.read_time = m41st85w_read_time,
	.set_time = m41st85w_set_time,
};

static int __devinit m41st85w_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct m41st85w *m41st85w;
	int ret;

	m41st85w = kzalloc(sizeof(struct m41st85w), GFP_KERNEL);
	if (!m41st85w)
		return -ENOMEM;

	m41st85w->client = client;
	i2c_set_clientdata(client, m41st85w);

	m41st85w_enable_clock(client, 1);

	m41st85w->rtc = rtc_device_register(client->name, &client->dev,
					  &m41st85w_rtc_ops, THIS_MODULE);
	if (IS_ERR(m41st85w->rtc)) {
		ret = PTR_ERR(m41st85w->rtc);
		dev_err(&client->dev, "unable to register the class device\n");
		goto out_free;
	}

	return 0;

out_free:
	kfree(m41st85w);
	return ret;
}

static int __devexit m41st85w_remove(struct i2c_client *client)
{
	struct m41st85w *m41st85w = i2c_get_clientdata(client);

	m41st85w_enable_clock(client, 0);
	rtc_device_unregister(m41st85w->rtc);
	kfree(m41st85w);
	return 0;
}

static struct i2c_device_id m41st85w_id[] = {
	{ "m41st85w", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, m41st85w_id);

static struct i2c_driver m41st85w_driver = {
	.driver = {
		.name = "rtc-m41st85w",
		.owner = THIS_MODULE,
	},
	.probe = m41st85w_probe,
	.remove = __devexit_p(m41st85w_remove),
	.id_table = m41st85w_id,
};

static int __init m41st85w_init(void)
{
	return i2c_add_driver(&m41st85w_driver);
}

static void __exit m41st85w_exit(void)
{
	i2c_del_driver(&m41st85w_driver);
}

module_init(m41st85w_init);
module_exit(m41st85w_exit);

MODULE_AUTHOR("Artem Kukhta");
MODULE_DESCRIPTION("ST M41ST85W RTC Driver");
MODULE_LICENSE("GPL");
