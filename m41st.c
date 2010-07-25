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

static int m41st85w_attached;
struct i2c_client *m41st85w_i2c_client = NULL;

static int m41st85w_rtc_ioctl(struct inode *, struct file *, unsigned int,
			      unsigned long);
static int m41st85w_rtc_open(struct inode *inode, struct file *file);
static int m41st85w_rtc_release(struct inode *inode, struct file *file);
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

static int m41st85w_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int m41st85w_remove(struct i2c_client *client);
static int m41st85w_command(struct i2c_client *client, unsigned int cmd, void *arg);

static struct i2c_device_id m41st85w_idtable[] = {
	{ "m41st85w", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, m41st85w_idtable);

struct i2c_driver m41st85w_driver = {     
	.driver = {
		.name	= "m41st85w",
	},	
	.probe = m41st85w_probe,
	.remove = m41st85w_remove,
	.command = m41st85w_command,
	.id_table = m41st85w_idtable,
};



static void m41st85w_enable_clock(int enable)
{		

	printk("M41ST85W: m41st85w_enable_clock() successfully called\n");

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
	.read_time = m41st85w_rtc_read_time,
	.set_time = m41st85w_rtc_set_time,
};
/**/


static int __devinit m41st85w_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rtc_device *rtc = NULL;

	printk("M41ST85W: m41st85w_probe() successfully called\n");

	/* i.e. skip the error catching code in m41st85w_command for now */
	m41st85w_attached = 1;
	printk("M41ST85W: m41st85w_probe() m41st85w_attached = 1\n");
	

	//*m41st85w_i2c_client = c;
	m41st85w_i2c_client = client;
	i2c_set_clientdata(client, m41st85w_i2c_client);

	/* регистрация RTC */
	rtc = rtc_device_register(client->name, &client->dev,
				  &m41st85w_rtc_ops, THIS_MODULE);  
	if (IS_ERR(rtc)) {
		printk("Error in registration RTC\n");
		rtc = NULL;
		goto exit;
	}
exit:
	return 0;
}

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
{

	return 1;
}

static int
m41st85w_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	printk("M41ST85W: m41st85w_command() successfully called\n");	
	if(!m41st85w_attached)
	{
		printk("m41st85w: user space accessed rtc, but no chip attached... unloading driver\n");
		return 0;
	}
	return 1;
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
	int status = 0;
	
	printk("M41ST85W: m41st85w_rtc_ioctl() successfully called\n");	

	if(!m41st85w_attached)
	{
		printk("m41st85w m41st85w_rtc_ioctl(): user space accessed rtc, but no chip attached... unloading driver\n");
		return 0;
	}

	printk("M41ST85W: m41st85w_rtc_ioctl() successfully exit\n");	
	
	return status;
}

static int m41st85w_rtc_read_proc(char *page, char **start, off_t off,
				  int count, int *eof, void *data)
{	

	printk("M41ST85W: m41st85w_rtc_read_proc() successfully called\n");

	if(!m41st85w_attached)
	{
		printk("m41st85w m41st85w_rtc_read_proc(): user space accessed rtc, but no chip attached... unloading driver\n");
		return 0;
	}

	
	printk("M41ST85W: m41st85w_rtc_read_proc() successfully exit\n");
	return 1;
}



void m41st85w_k_set_tlet(struct work_struct *work)
{
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
	int retval = 0, ret;	

	printk("M41ST85W: m41st85w_init called\n");
	ret = i2c_register_board_info(1, m41st85w_i2c_board_info, ARRAY_SIZE(m41st85w_i2c_board_info));
	printk("M41ST85W m41st85w_init: i2c_register_board_info retval=%d\n", ret);

//	normal_addr[0] = slave_address;

	retval = i2c_add_driver(&m41st85w_driver);
	printk("M41ST85W m41st85w_init: i2c_add_driver(&m41st85w_driver) retval=%d\n", retval);	

	if (retval == 0) {
		misc_register(&m41st85w_rtc_miscdev);
		create_proc_read_entry(PROC_M41ST85W_NAME, 0, 0,
				       m41st85w_rtc_read_proc, NULL);
		printk("I2C: M41ST85W RTC driver successfully loaded\n");
	}

	//INIT_WORK(&short_wq, m41st85w_k_set_tlet);
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
