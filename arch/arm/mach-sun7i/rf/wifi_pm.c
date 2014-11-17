#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <plat/sys_config.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include "wifi_pm.h"

#define wifi_pm_msg(...)    do {printk("[wifi_pm]: "__VA_ARGS__);} while(0)


struct wifi_pm_ops wifi_select_pm_ops;
static char* wifi_mod[] = {" ",
			   "bcm40181",   /* 1 - BCM40181(BCM4330)*/
			   "bcm40183",   /* 2 - BCM40183(BCM4330)*/
			   "rtl8723as",  /* 3 - RTL8723AS(RF-SM02B) */
			   "rtl8189es",  /* 4 - RTL8189ES(SM89E00) */
			   "rtl8192cu",  /* 5 - RTL8192CU*/
			   "rtl8188eu",  /* 6 - RTL8188EU*/
			   "ap6210",     /* 7 - AP6210*/
			   "ap6330",     /* 8 - AP6330*/
			   "ap6181",     /* 9 - AP6181*/
			   "rtl8723au",  /* 10 - RTL8723AU */
};

int wifi_pm_get_mod_type(void)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;
	if (ops->wifi_used)
		return ops->module_sel;
	else {
		wifi_pm_msg("No select wifi, please check your config !!\n");
		return 0;
	}
}
EXPORT_SYMBOL(wifi_pm_get_mod_type);

int wifi_pm_gpio_ctrl(char* name, int level)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;	
	if (ops->wifi_used && ops->gpio_ctrl)		
		return ops->gpio_ctrl(name, level);	
	else {		
		wifi_pm_msg("No select wifi, please check your config !!\n");		
		return -1;	
	}
}
EXPORT_SYMBOL(wifi_pm_gpio_ctrl);

void wifi_pm_power(int on)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;
	int power = on;

	if (ops->wifi_used && ops->power)
		return ops->power(1, &power);
	else {
		wifi_pm_msg("No select wifi, please check your config !!\n");
		return;
	}
}
EXPORT_SYMBOL(wifi_pm_power);

#ifdef CONFIG_PROC_FS
static int wifi_pm_power_stat(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct wifi_pm_ops *ops = (struct wifi_pm_ops *)data;
	char *p = page;
	int power = 0;

	if (ops->power)
		ops->power(0, &power);

	p += sprintf(p, "%s : power state %s\n", ops->mod_name, power ? "on" : "off");
	return p - page;
}

static int wifi_pm_power_ctrl(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	struct wifi_pm_ops *ops = (struct wifi_pm_ops *)data;
	int power = simple_strtoul(buffer, NULL, 10);
    
	power = power ? 1 : 0;
	if (ops->power)
		ops->power(1, &power);
	else
		wifi_pm_msg("No power control for %s\n", ops->mod_name);
	return sizeof(power);	
}

static inline void awwifi_procfs_attach(void)
{
	char proc_rootname[] = "driver/wifi-pm";
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;

	ops->proc_root = proc_mkdir(proc_rootname, NULL);
	if (IS_ERR(ops->proc_root))
	{
		wifi_pm_msg("failed to create procfs \"driver/wifi-pm\".\n");
	}

	ops->proc_power = create_proc_entry("power", 0644, ops->proc_root);
	if (IS_ERR(ops->proc_power))
	{
		wifi_pm_msg("failed to create procfs \"power\".\n");
	}
	ops->proc_power->data = ops;
	ops->proc_power->read_proc = wifi_pm_power_stat;
	ops->proc_power->write_proc = wifi_pm_power_ctrl;
}

static inline void awwifi_procfs_remove(void)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;
	char proc_rootname[] = "driver/wifi-pm";

	remove_proc_entry("power", ops->proc_root);
	remove_proc_entry(proc_rootname, NULL);
}
#else
static inline void awwifi_procfs_attach(void) {}
static inline void awwifi_procfs_remove(void) {}
#endif

static int wifi_pm_get_res(void)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;

	if (SCRIPT_PARSER_OK != script_parser_fetch(wifi_para, "wifi_used", &ops->wifi_used, 1)) {
		pr_warning("pm: parse wifi_used failed\n");
		return -1;
	}
	if (!ops->wifi_used) {
		pr_info("pm: wifi pm is disable in config\n");
		return -1;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch(wifi_para, "wifi_sdc_id", &ops->sdio_id, 1)) {
		pr_warning("pm: parse wifi_sdc_id failed\n");
		return -1;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch(wifi_para, "wifi_usbc_id", &ops->usb_id, 1)) {
		pr_warning("pm: parse wifi_sdc_id failed\n");
		return -1;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch(wifi_para, "wifi_mod_sel", &ops->module_sel, 1)) {
		pr_warning("pm: parse wifi_sdc_id failed\n");
		return -1;
	}

	ops->mod_name = wifi_mod[ops->module_sel];
	
	printk("pm: select wifi: %s\n", wifi_mod[ops->module_sel]);

	return 0;
}

static int __devinit wifi_pm_probe(struct platform_device *pdev)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;

	switch (ops->module_sel) {
	case 1: /* BCM40181 */
	case 2: /* BCM40183 */
	case 3: /* RTL8723AS */
	case 4: /* RTL8189ES */
	case 5: /* RTL8192CU */
	case 6: /* RTL8188EU */
		pr_warning("wifi_pm: Unsupported wifi type!\n");
		break;
	case 7: /* AP6210 */
	case 8: /* AP6330 */
	case 9: /* AP6181 */
		ap6xxx_gpio_init();
		break;
	case 10: /* RTL8723AU */
		pr_warning("wifi_pm: Unsupported wifi type!\n");
		break;
	default:
		pr_warning("wifi_pm: Unsupported wifi type!\n");
	}

	awwifi_procfs_attach();
	wifi_pm_msg("wifi gpio init is OK !!\n");
	return 0;
}

static int __devexit wifi_pm_remove(struct platform_device *pdev)
{
	awwifi_procfs_remove();
	wifi_pm_msg("wifi gpio is released !!\n");
	return 0;
}

#ifdef CONFIG_PM
static int wifi_pm_suspend(struct device *dev)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;

	if (ops->standby)
		ops->standby(1);
	return 0;
}

static int wifi_pm_resume(struct device *dev)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;

	if (ops->standby)
		ops->standby(0);
	return 0;
}

static struct dev_pm_ops wifi_dev_pm_ops = {
	.suspend	= wifi_pm_suspend,
	.resume		= wifi_pm_resume,
};
#endif

static struct platform_device wifi_pm_dev = {
	.name           = "wifi_pm",
};

static struct platform_driver wifi_pm_driver = {
	.driver.name    = "wifi_pm",
	.driver.owner   = THIS_MODULE,
#ifdef CONFIG_PM
	.driver.pm      = &wifi_dev_pm_ops,
#endif
	.probe          = wifi_pm_probe,
	.remove         = __devexit_p(wifi_pm_remove),
};

static int __init wifi_pm_init(void)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;

	memset(ops, 0, sizeof(struct wifi_pm_ops));
	wifi_pm_get_res();
	if (!ops->wifi_used)
		return 0;

	platform_device_register(&wifi_pm_dev);
	return platform_driver_register(&wifi_pm_driver);
}

static void __exit wifi_pm_exit(void)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;
	if (!ops->wifi_used)
		return;

	platform_driver_unregister(&wifi_pm_driver);
	memset(ops, 0, sizeof(struct wifi_pm_ops));
}

module_init(wifi_pm_init);
module_exit(wifi_pm_exit);

MODULE_LICENSE("GPL");


