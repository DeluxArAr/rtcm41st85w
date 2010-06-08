/*
 * m41st85w.c
 *
 * Device driver for ST's Real Time Controller M41ST85W.
 *
 * Copyright (C) 2002 Intrinsyc Software Inc.
 * Copyright (C) 2003 Intel Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This driver is adapted from the DS1307 driver
 * Kernel time sync borrowed from RMK's acorn RTC driver
 *
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/clocksource.h>
#include <linux/bcd.h>

#include "m41st85w.h"

/* add from rtc-m41t80.c */
#define M41T80_FEATURE_HT	(1 << 0)	/* Halt feature */
#define M41T80_FEATURE_BL	(1 << 1)	/* Battery low indicator */
#define M41T80_FEATURE_SQ	(1 << 2)	/* Squarewave feature */


static int m41st85w_attached;
static unsigned short slave_address = M41ST85W_I2C_SLAVE_ADDR;

//*struct i2c_driver m41st85w_driver;	//old
struct i2c_client *m41st85w_i2c_client = NULL;
//extern int (*set_rtc) (void);

//*static unsigned short ignore[] = { I2C_CLIENT_END };	//old
//*static unsigned short normal_addr[] =	//old
//*    { M41ST85W_I2C_SLAVE_ADDR, I2C_CLIENT_END };	//old

/*static struct i2c_client_address_data addr_data = {
      //normal_i2c:normal_addr,
      //probe:ignore,
      //ignore:ignore,
};*/

static int m41st85w_rtc_ioctl(struct inode *, struct file *, unsigned int,
			      unsigned long);
static int m41st85w_rtc_open(struct inode *inode, struct file *file);
static int m41st85w_rtc_release(struct inode *inode, struct file *file);
static int m41st85w_k_set_rtc_time(void);
void m41st85w_k_set_tlet(struct work_struct *work);

static int m41st85w_get_datetime(struct i2c_client *, struct rtc_time *);
static int m41st85w_set_datetime(struct i2c_client *, struct rtc_time *);
static int m41st85w_rtc_ioctl(struct inode *, struct file *,
		   unsigned int, unsigned long);

static struct file_operations rtc_fops = {
      owner:THIS_MODULE,
      ioctl:m41st85w_rtc_ioctl,
      open:m41st85w_rtc_open,
      release:m41st85w_rtc_release,
};

static struct miscdevice m41st85w_rtc_miscdev = {
	RTC_MINOR,
	"rtc",
	&rtc_fops
};

//*static int m41st85w_probe(struct i2c_adapter *adap);
/*		+change detach->remove		*/
static int m41st85w_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int m41st85w_remove(struct i2c_client *client);
static int m41st85w_command(struct i2c_client *client, unsigned int cmd,
			    void *arg);

/*		+		*/
static struct i2c_device_id m41st85w_idtable[] = {
	//{ "m41st85w", 0 },
	{ "m41st85w", M41T80_FEATURE_HT | M41T80_FEATURE_BL | M41T80_FEATURE_SQ },
	{}
};

MODULE_DEVICE_TABLE(i2c, m41st85w_idtable);
/*		+		*/

static struct i2c_driver m41st85w_driver = {
      //id:I2C_DRIVERID_M41ST85W,
      //*attach_adapter:m41st85w_probe,
      //*detach_client:m41st85w_detach,
	.driver = {
		.name	= "m41st85w",
	},	
	.probe = m41st85w_probe,	//+
	.remove = __devexit_p(m41st85w_remove),	//+
	.command = m41st85w_command,
	.id_table = m41st85w_idtable,	//+
};

#define DAT(x) ((unsigned int)((x)->data))	/* keep the control register info */

static int m41st85w_readram(char *buf, int len)
{
	unsigned char ad[1] = { 0 };
	int ret;
	struct i2c_msg msgs[2] = {
		{m41st85w_i2c_client->addr, 0, 1, ad},
		{m41st85w_i2c_client->addr, I2C_M_RD, len, buf}
	};

	ret = i2c_transfer(m41st85w_i2c_client->adapter, msgs, 2);

	return ret;
}

static void m41st85w_enable_clock(int enable)
{		
    unsigned char buf[2], buf2[2], ad[1] = { 1};
    struct i2c_msg msgs[2] = {
        { m41st85w_i2c_client->addr , 0,        1, ad},
        { m41st85w_i2c_client->addr , I2C_M_RD, 1, buf}
    };
    unsigned char ctrl_info[2];
    int ret;

	printk("M41ST85W: m41st85w_enable_clock() successfully called\n");
    if(enable)
    {
        ctrl_info[0] = SQW_ENABLE;
        ctrl_info[1] = RATE_32768HZ;
    }
    else
    {
        ctrl_info[0] = SQW_DISABLE;
        ctrl_info[1] = RATE_0HZ;
    }
    ret = m41st85w_command(m41st85w_i2c_client, M41ST85W_SETCTRL, &ctrl_info);
    if(ret < 0)
    {
        printk ("%s %d: m41st85w_command failed\n", __func__,__LINE__);
    }
    /* read addr 1 (Clock-Halt bit and second counter */
    ad[0] = 0xC;
    ret = i2c_transfer(m41st85w_i2c_client->adapter, msgs, 2);
    if(ret < 0)
    {
        printk ("%s %d: i2c_transfer failed\n", __func__, __LINE__);
    }

    if(enable)
    {
        buf2[1] = buf[1] & ~CLOCK_HALT; /* clear Clock-Halt bit */
        ad[0] = 0x1;
        ret = i2c_transfer(m41st85w_i2c_client->adapter, msgs, 2);
        buf[1] = buf[1] & ~CLOCK_STOP;
    }
    else
    {
        buf2[1] = buf2[1] | CLOCK_HALT; /* set Clock-Halt bit */
        buf[1] = buf[1] | CLOCK_STOP;
    }

    /* addr 12, halt bit */
    buf2[0] = 0xC;
    ret = i2c_master_send(m41st85w_i2c_client, (char *)buf2, 2);
    if(ret < 0)
    {
        printk ("%s %d: i2c_master_send failed\n", __func__, __LINE__);
    }

    buf[0] = 0x1;
    ret = i2c_master_send(m41st85w_i2c_client, (char *)buf, 2);
    if(ret < 0)
    {
        printk ("%s %d: i2c_master_send failed\n", __func__, __LINE__);
    }
	printk("M41ST85W: m41st85w_enable_clock() successfully exit\n");
}


/*
 *регистрация RTC
*/

static int m41st85w_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return m41st85w_get_datetime(to_i2c_client(dev), tm);
}

static int m41st85w_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return m41st85w_set_datetime(to_i2c_client(dev), tm);
}

static struct rtc_class_ops m41st85w_rtc_ops = {
	.read_time = m41st85w_rtc_read_time,	//
	.set_time = m41st85w_rtc_set_time,	//
};
/**/




/*		+change name attach->probe with new parameters		*/
/*static int m41st85w_probe(struct i2c_adapter *adap, int addr, int kind)*/
static int __devinit m41st85w_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	int ret = 0;
	struct i2c_client *c;
	struct rtc_time rtctm;
	struct rtc_device *rtc = NULL;
	unsigned char data[10];
	unsigned char ad[1] = {0};	

	printk("M41ST85W: m41st85w_probe() successfully called\n");		
	/*struct i2c_msg ctrl_wr[1] = {
		{addr, 0, 2, data},
	};

	struct i2c_msg ctrl_rd[2] = {
		{addr, 0, 1, ad},
		{addr, I2C_M_RD, 2, data},
	};
	if (!(c = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		return -ENOMEM;
	}

	memset(c, 0, sizeof(struct i2c_client));*/

	//strncpy(c->name, "M41ST85W", I2C_NAME_SIZE);

	//*c->addr		= addr;
	//*c->adapter	= adap;
	//c->driver	= &m41st85w_driver;

	data[0] = 0;
	data[1] = 0;

	/*if ((ret = i2c_transfer(c->adapter, ctrl_wr, 1)) != 1) {
                printk(KERN_ERR "%s: %s: unable to init ctrl1\n",
			__func__, c->name);
                ret = -ENODEV;
                goto out;
        }

	if ((ret = i2c_transfer(c->adapter, ctrl_rd, 2)) != 2) {
		printk(KERN_ERR "%s: %s: uanble to read ctrl2\n",
			__func__, c->name);
		ret = -ENODEV;
		goto out;
	}*/

	/* регистрация RTC */
	rtc = rtc_device_register(client->name, &client->dev,
				  &m41st85w_rtc_ops, THIS_MODULE);  
	if (IS_ERR(rtc)) {
		printk("Error in registration RTC\n");
		rtc = NULL;
		goto exit;
	}
	/* i.e. skip the error catching code in m41st85w_command for now */
	m41st85w_attached = 1;
	printk("M41ST85W: m41st85w_probe() m41st85w_attached = 1\n");
	

	//*m41st85w_i2c_client = c;
	m41st85w_i2c_client = client;
	m41st85w_enable_clock(1);
	m41st85w_command(m41st85w_i2c_client, M41ST85W_GETDATETIME,
		(void *)&rtctm);

	xtime.tv_nsec = 0;
	xtime.tv_sec = mktime(1900 + rtctm.tm_year, rtctm.tm_mon + 1,
		rtctm.tm_mday, rtctm.tm_hour, rtctm.tm_min, rtctm.tm_sec);
	
	/* Fix for Uptime */
        wall_to_monotonic.tv_sec = -xtime.tv_sec;
        wall_to_monotonic.tv_nsec = -xtime.tv_nsec;

	//set_rtc = m41st85w_k_set_rtc_time;

	//*ret = i2c_attach_client(c);

//out:
	/**if (ret < 0) {
		m41st85w_attached = 0;
		kfree(c);
	}*/
exit:
	return 0;
}

/*static int m41st85w_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, m41st85w_attach);
}*/

static int __devexit m41st85w_remove(struct i2c_client *client)
{
	printk("M41ST85W: m41st85w_remove() successfully called\n");
	if (client) {
		//*i2c_detach_client(client);
		kfree(client);
		client = m41st85w_i2c_client = NULL;
	}
	m41st85w_enable_clock(0);
	printk("M41ST85W: m41st85w_remove() successfully exit\n");

	return 0;
}

static void m41st85w_convert_to_time(struct rtc_time *dt, char *buf)
{
	m41ST85w_parse_param(dt->tm_sec, SEC, buf);
	m41ST85w_parse_param(dt->tm_min, MIN, buf);
	m41ST85w_parse_param(dt->tm_hour, HOUR, buf);
	m41ST85w_parse_param(dt->tm_wday, WDAY, buf);
	m41ST85w_parse_param(dt->tm_mday, MDAY, buf);
	m41ST85w_parse_param(dt->tm_mon, MON, buf);
	m41ST85w_parse_param(dt->tm_year, YEAR, buf);

	/* fix up values */
	/* dt->tm_mon is zero-based */
	dt->tm_mon--;
	/* year is 1900 + dt->tm_year */
	if (dt->tm_year < 95)
		dt->tm_year += 100;

	dt->tm_isdst = 0;

    printk("m41st85w_get_datetime: year = %d\n", dt->tm_year);
    printk("m41st85w_get_datetime: mon  = %d\n", dt->tm_mon);
    printk("m41st85w_get_datetime: mday = %d\n", dt->tm_mday);
    printk("m41st85w_get_datetime: hour = %d\n", dt->tm_hour);
    printk("m41st85w_get_datetime: min  = %d\n", dt->tm_min);
    printk("m41st85w_get_datetime: sec  = %d\n", dt->tm_sec);

}

static int m41st85w_get_datetime(struct i2c_client *client, struct rtc_time *dt)
{
	unsigned char buf[8], addr[1] = { 0 };
	struct i2c_msg msgs[2] = {
		{client->addr, 0, 1, addr},
		{client->addr, I2C_M_RD, 8, buf}
	};
	int ret = -EIO;

	memset(buf, 0, sizeof(buf));
	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret >= 0) {
		m41st85w_convert_to_time(dt, buf);
		ret = 0;
	} else
		printk("m41st85w_get_datetime(), i2c_transfer() returned %d\n",
		       ret);

	return ret;
}

static int
m41st85w_set_datetime(struct i2c_client *client, struct rtc_time *dt)
//-		      int datetoo)
{
	unsigned char buf[8];
	int ret, len = 4;

	printk("m41st85w_set_datetime: tm_year = %d\n", dt->tm_year);
	printk("m41st85w_set_datetime: tm_mon  = %d\n", dt->tm_mon);
	printk("m41st85w_set_datetime: tm_mday = %d\n", dt->tm_mday);
	printk("m41st85w_set_datetime: tm_hour = %d\n", dt->tm_hour);
	printk("m41st85w_set_datetime: tm_min  = %d\n", dt->tm_min);
	printk("m41st85w_set_datetime: tm_sec  = %d\n", dt->tm_sec);

	buf[0] = 1;		/* register address on m41st85w */
	m41ST85w_set_param(buf, SEC, dt->tm_sec);
	m41ST85w_set_param(buf, MIN, dt->tm_min);
	m41ST85w_set_param(buf, HOUR, dt->tm_hour);

	//-if (datetoo) {
		len = 8;
		m41ST85w_set_param(buf, WDAY, dt->tm_wday);
		m41ST85w_set_param(buf, MDAY, dt->tm_mday);
		m41ST85w_set_param(buf, MON, dt->tm_mon + 1);
/* The year only ranges from 0-99, we are being passed an offset from 1900,
 * and the chip calulates leap years based on 2000, thus we adjust by 100.
 */
		m41ST85w_set_param(buf, YEAR, dt->tm_year - 100);
	//-}
	ret = i2c_master_send(client, (char *)buf, len);
	if (ret >= 0)
		ret = 0;
	else
		printk
		    ("m41st85w_set_datetime(), i2c_master_send() returned %d\n",
		     ret);
	printk("m41st85w_set_datetime new: tm_year = %d\n", dt->tm_year);
	printk("m41st85w_set_datetime new: tm_mon  = %d\n", dt->tm_mon);
	printk("m41st85w_set_datetime new: tm_mday = %d\n", dt->tm_mday);
	printk("m41st85w_set_datetime new: tm_hour = %d\n", dt->tm_hour);
	printk("m41st85w_set_datetime new: tm_min  = %d\n", dt->tm_min);
	printk("m41st85w_set_datetime new: tm_sec  = %d\n", dt->tm_sec);

	return ret;
}

static int m41st85w_get_ctrl(struct i2c_client *client, unsigned char *ctrl)
{
//    *ctrl = DAT(client);

	return 0;
}

static int m41st85w_set_ctrl(struct i2c_client *client, unsigned char *cinfo)
{	
	unsigned char buf[2];
	int ret;

	printk("M41ST85W: m41st85w_set_ctrl() successfully called\n");
	/* need to set square wave first */
	buf[0] = 13;
	buf[1] = cinfo[1];
	ret = i2c_master_send(client, (char *)buf, 2);
	if (ret < 0) {
		return ret;
	}

	buf[0] = 10;
	buf[1] = cinfo[0];
	/* save the control reg info in the client data field so that get_ctrl
	 * function doesn't have to do an I2C transfer to get it.
	 */
	// Need to save frequency also
//    DAT(client) = buf[1];

	ret = i2c_master_send(client, (char *)buf, 2);
	printk("M41ST85W: m41st85w_set_ctrl() successfully exit\n");

	return ret;
}

static int m41st85w_read_mem(struct i2c_client *client, struct rtc_mem *mem)
{	
	unsigned char addr[1] = { 0 };
	struct i2c_msg msgs[2] = {
		{client->addr, 0, 1, addr},
		{client->addr, I2C_M_RD, mem->nr, mem->data}
	};

	printk("M41ST85W: m41st85w_read_mem() successfully called\n");

	if ((mem->loc < M41ST85W_RAM_ADDR_START) ||
	    ((mem->loc + mem->nr - 1) > M41ST85W_RAM_ADDR_END))
		return -EINVAL;

	addr[0] = mem->loc;
	printk("M41ST85W: m41st85w_read_mem() successfully exit\n");

	return i2c_transfer(client->adapter, msgs, 2) >= 0 ? 0 : -EIO;
}

static int m41st85w_write_mem(struct i2c_client *client, struct rtc_mem *mem)
{	
	unsigned char addr[1] = { 0 };
	struct i2c_msg msgs[2] = {
		{client->addr, 0, 1, addr},
		{client->addr, 0, mem->nr, mem->data}
	};

	printk("M41ST85W: m41st85w_write_mem() successfully called\n");

	if ((mem->loc < M41ST85W_RAM_ADDR_START) ||
	    ((mem->loc + mem->nr - 1) > M41ST85W_RAM_ADDR_END))
		return -EINVAL;

	addr[0] = mem->loc;
	printk("M41ST85W: m41st85w_write_mem() successfully exit\n");

	return i2c_transfer(client->adapter, msgs, 2) >= 0 ? 0 : -EIO;
}

static int
m41st85w_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	printk("M41ST85W: m41st85w_command() successfully called\n");	
	if(!m41st85w_attached)
	{
		printk("m41st85w: user space accessed rtc, but no chip attached... unloading driver\n");
		remove_proc_entry(PROC_M41ST85W_NAME, NULL);
		misc_deregister(&m41st85w_rtc_miscdev);
		i2c_del_driver(&m41st85w_driver);
		return -ENXIO;
	}

	switch (cmd) {
	case M41ST85W_GETDATETIME:
		printk("M41ST85W: m41st85w_command(M41ST85W_GETDATETIME) successfully exit\n");
		return m41st85w_get_datetime(client, arg);

	case M41ST85W_SETTIME:
		printk("M41ST85W: m41st85w_command(M41ST85W_SETTIME) successfully exit\n");
		return m41st85w_set_datetime(client, arg);//-

	case M41ST85W_SETDATETIME:
		printk("M41ST85W: m41st85w_command(M41ST85W_SETDATETIME) successfully exit\n");
		return m41st85w_set_datetime(client, arg);//-

	case M41ST85W_GETCTRL:
		printk("M41ST85W: m41st85w_command(M41ST85W_GETCTRL) successfully exit\n");
		return m41st85w_get_ctrl(client, arg);

	case M41ST85W_SETCTRL:
		printk("M41ST85W: m41st85w_command(M41ST85W_SETCTRL) successfully exit\n");
		return m41st85w_set_ctrl(client, arg);

	case M41ST85W_MEM_READ:
		printk("M41ST85W: m41st85w_command(M41ST85W_MEM_READ) successfully exit\n");
		return m41st85w_read_mem(client, arg);

	case M41ST85W_MEM_WRITE:
		printk("M41ST85W: m41st85w_command(M41ST85W_MEM_WRITE) successfully exit\n");
		return m41st85w_write_mem(client, arg);

	default:
		printk("M41ST85W: m41st85w_command() successfully exit by default\n");
		return -EINVAL;
	}
}

static int m41st85w_rtc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int m41st85w_rtc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int
m41st85w_rtc_ioctl(struct inode *inode, struct file *file,
		   unsigned int cmd, unsigned long arg)
{	
	struct rtc_time wtime;
	int status = 0;
	
	printk("M41ST85W: m41st85w_rtc_ioctl() successfully called\n");	

	if(!m41st85w_attached)
	{
		printk("m41st85w m41st85w_rtc_ioctl(): user space accessed rtc, but no chip attached... unloading driver\n");
		remove_proc_entry(PROC_M41ST85W_NAME, NULL);
		misc_deregister(&m41st85w_rtc_miscdev);
		i2c_del_driver(&m41st85w_driver);
		return -ENXIO;
	}

	switch (cmd) {
	default:
	case RTC_UIE_ON:
	case RTC_UIE_OFF:
	case RTC_PIE_ON:
	case RTC_PIE_OFF:
	case RTC_AIE_ON:
	case RTC_AIE_OFF:
	case RTC_ALM_SET:
	case RTC_ALM_READ:
	case RTC_IRQP_READ:
	case RTC_IRQP_SET:
	case RTC_EPOCH_SET:
	case RTC_WKALM_SET:
	case RTC_WKALM_RD:
		status = -EINVAL;
		break;
	case RTC_EPOCH_READ:
		return put_user(1970, (unsigned long *)arg);
	case RTC_RD_TIME:
		m41st85w_command(m41st85w_i2c_client, M41ST85W_GETDATETIME,
				 &wtime);
		if (copy_to_user((void *)arg, &wtime, sizeof(struct rtc_time)))
			status = -EFAULT;
		break;

	case RTC_SET_TIME:
		if (!capable(CAP_SYS_TIME)) {
			status = -EACCES;
			break;
		}

		if (copy_from_user
		    (&wtime, (struct rtc_time *)arg, sizeof(struct rtc_time))) {
			status = -EFAULT;
			break;
		}

		m41st85w_command(m41st85w_i2c_client, M41ST85W_SETDATETIME,
				 &wtime);
		break;
	}
	printk("M41ST85W: m41st85w_rtc_ioctl() successfully exit\n");	
	
	return status;
}

static char *m41st85w_mon2str(unsigned int mon)
{
	char *mon2str[12] = {
		"Jan", "Feb", "Mar", "Apr", "May", "Jun",
		"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
	};
	if (mon > 11)
		return "error";
	else
		return mon2str[mon];
}

static int m41st85w_rtc_proc_output(char *buf)
{
#define CHECK(ctrl,bit) ((ctrl & bit) ? "yes" : "no")
	unsigned char ram[M41ST85W_RAM_SIZE];
	int ret;

	char *p = buf;

	printk("M41ST85W: m41st85w_rtc_proc_output() successfully called\n");

	ret = m41st85w_readram(ram, M41ST85W_RAM_SIZE);
	if (ret > 0) {
		int i;
		struct rtc_time dt;
		char text[9];

		p += sprintf(p, "ST M41ST85W (64x8 Serial Real Time Clock)\n");

		m41st85w_convert_to_time(&dt, ram);
		p += sprintf(p,
		     "Date/Time	     : %02d-%s-%04d %02d:%02d:%02d\n",
			     dt.tm_mday, m41st85w_mon2str(dt.tm_mon),
			     dt.tm_year + 1900, dt.tm_hour, dt.tm_min,
			     dt.tm_sec);

		p += sprintf(p, "Clock halted	     : %s\n",
			     CHECK(ram[1], 0x80));
		p += sprintf(p, "24h mode	     : %s\n", "yes");
		p += sprintf(p, "Square wave enabled  : %s\n",
			     CHECK(ram[10], 0x40));
		p += sprintf(p, "Freq		     : ");

		switch (ram[19] & 0x03) {
		case RATE_1HZ:
			p += sprintf(p, "1Hz\n");
			break;
		case RATE_4096HZ:
			p += sprintf(p, "4.096kHz\n");
			break;
		case RATE_8192HZ:
			p += sprintf(p, "8.192kHz\n");
			break;
		case RATE_32768HZ:
		default:
			p += sprintf(p, "32.768kHz\n");
			break;

		}

		p += sprintf(p, "RAM dump:\n");
		text[8] = '\0';
		for (i = 0; i < M41ST85W_RAM_SIZE; i++) {
			p += sprintf(p, "%02X ", ram[i]);

			if ((ram[i] < 32) || (ram[i] > 126))
				ram[i] = '.';
			text[i % 8] = ram[i];
			if ((i % 8) == 7)
				p += sprintf(p, "%s\n", text);
		}
		p += sprintf(p, "\n");
	} else {
		p += sprintf(p, "Failed to read RTC memory!\n");
	}
	
	printk("M41ST85W: m41st85w_rtc_proc_output() successfully exit\n");
	return p - buf;
}

static int m41st85w_rtc_read_proc(char *page, char **start, off_t off,
				  int count, int *eof, void *data)
{	
	int len;

	printk("M41ST85W: m41st85w_rtc_read_proc() successfully called\n");

	if(!m41st85w_attached)
	{
		printk("m41st85w m41st85w_rtc_read_proc(): user space accessed rtc, but no chip attached... unloading driver\n");
		remove_proc_entry(PROC_M41ST85W_NAME, NULL);
		misc_deregister(&m41st85w_rtc_miscdev);
		i2c_del_driver(&m41st85w_driver);
		return 0;
	}

	len = m41st85w_rtc_proc_output(page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	printk("M41ST85W: m41st85w_rtc_read_proc() successfully exit\n");
	return len;
}

static struct work_struct short_wq;

static int m41st85w_k_set_rtc_time(void)
{
	if (in_interrupt())
		schedule_work(&short_wq);
	else
		m41st85w_k_set_tlet(NULL);
	return 0;
}



void m41st85w_k_set_tlet(struct work_struct *work)
{
	struct rtc_time new_rtctm, old_rtctm;
	unsigned long nowtime = xtime.tv_sec;
	//unsigned long nowtime;

	if (m41st85w_command(m41st85w_i2c_client, M41ST85W_GETDATETIME, &old_rtctm))
		return;

	new_rtctm.tm_sec = nowtime % 60;
	nowtime /= 60;
	new_rtctm.tm_min = nowtime % 60;
	nowtime /= 60;
	new_rtctm.tm_hour = nowtime % 24;

	/*
	 * avoid writing when we're going to change the day
	 * of the month.  We will retry in the next minute.
	 * This basically means that if the RTC must not drift
	 * by more than 1 minute in 11 minutes.
	 *
	 * [ rtc: 1/1/2000 23:58:00, real 2/1/2000 00:01:00,
	 *   rtc gets set to 1/1/2000 00:01:00 ]
	 */
	if ((old_rtctm.tm_hour == 23 && old_rtctm.tm_min == 59) ||
	    (new_rtctm.tm_hour == 23 && new_rtctm.tm_min == 59))
		return;

	m41st85w_command(m41st85w_i2c_client, M41ST85W_SETTIME,&new_rtctm);
}

/**/
static struct i2c_board_info __initdata m41st85w_i2c_board_info[] = {
  {               
      I2C_BOARD_INFO("m41st85w", 0x68),
  }
};

/**/

static int __init m41st85w_init(void)
{	
	int retval = 0;	

	printk("M41ST85W: m41st85w_init called\n");
	
	/*first argument - bus number*/
	i2c_register_board_info(1, m41st85w_i2c_board_info, ARRAY_SIZE(m41st85w_i2c_board_info));


//	normal_addr[0] = slave_address;

	retval = i2c_add_driver(&m41st85w_driver);
	printk("M41ST85W m41st85w_init: i2c_add_driver(&m41st85w_driver) retval=%d\n", retval);	

	if (retval == 0) {
		misc_register(&m41st85w_rtc_miscdev);
		create_proc_read_entry(PROC_M41ST85W_NAME, 0, 0,
				       m41st85w_rtc_read_proc, NULL);
		printk("I2C: M41ST85W RTC driver successfully loaded\n");
	}

	INIT_WORK(&short_wq, m41st85w_k_set_tlet);
	printk("M41ST85W: m41st85w_init successfully exit\n");
	return retval;
}

static void __exit m41st85w_exit(void)
{
	printk("M41ST85W: m41st85w_exit successfully called\n");
	remove_proc_entry(PROC_M41ST85W_NAME, NULL);
	misc_deregister(&m41st85w_rtc_miscdev);
	i2c_del_driver(&m41st85w_driver);
	printk("M41ST85W: m41st85w_exit successfully exit\n");
}

module_init(m41st85w_init);
module_exit(m41st85w_exit);

MODULE_AUTHOR("Intel Corp.");
MODULE_LICENSE("GPL");
