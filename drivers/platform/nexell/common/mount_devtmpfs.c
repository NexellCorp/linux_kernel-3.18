#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>

static int __init devtmpfs_init(void)
{
	char *rdsrc = CONFIG_INITRAMFS_SOURCE;
	if (!strcmp(rdsrc, ""))
		return 0;

	printk("Mount   : devtmpfs\n");
	devtmpfs_mount("dev");

	return 0;
}
late_initcall(devtmpfs_init);