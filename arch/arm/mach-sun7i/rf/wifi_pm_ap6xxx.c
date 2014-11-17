/*
 * ap6xxx sdio wifi power management API
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <plat/sys_config.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <linux/regulator/consumer.h>
#include "wifi_pm.h"

#define ap6xxx_msg(...)    do {printk("[ap6xxx]: "__VA_ARGS__);} while(0)

static int ap6xxx_wl_regon = 0;
static int ap6xxx_bt_regon = 0;

static int ap6xxx_gpio_ctrl(char* name, int level)
{
	int i = 0;	
	int ret = 0;
	int gpio = 0;
	char * gpio_name[2] = {"ap6xxx_wl_regon", "ap6xxx_bt_regon"};

	for (i = 0; i < 2; i++) {
		if (strcmp(name, gpio_name[i]) == 0) {
			switch (i)
			{
			case 0: /*ap6xxx_wl_regon*/
				gpio = ap6xxx_wl_regon;
				break;
			case 1: /*ap6xxx_bt_regon*/
				gpio = ap6xxx_bt_regon;
				break;
			default:
				ap6xxx_msg("no matched gpio.\n");
			}
			break;
		}
	}

	ret = gpio_write_one_pin_value(gpio, level, name);
	
	return 0;
}

static int ap6xxx_gpio_read(char* name)
{
	int i = 0;	
	int gpio = 0;
	int val = 0;
	char * gpio_name[2] = {"ap6xxx_wl_regon", "ap6xxx_bt_regon"};

	for (i = 0; i < 2; i++) {
		if (strcmp(name, gpio_name[i]) == 0) {
			switch (i)
			{
			case 0: /*ap6xxx_wl_regon*/
				gpio = ap6xxx_wl_regon;
				break;
			case 1: /*ap6xxx_bt_regon*/
				gpio = ap6xxx_bt_regon;
				break;
			default:
				ap6xxx_msg("no matched gpio.\n");
			}
			break;
		}
	}

	val = gpio_read_one_pin_value(gpio, name);
	
	return val;
}


void ap6xxx_power(int mode, int *updown)
{
	if (mode) {
		if (*updown) {
			ap6xxx_gpio_ctrl("ap6xxx_wl_regon", 1);
			mdelay(100);
		} else {
			ap6xxx_gpio_ctrl("ap6xxx_wl_regon", 0);
			mdelay(100);
		}
		pr_info("sdio wifi power state: %s\n", *updown ? "on" : "off");
	} else {
		*updown = ap6xxx_gpio_read("ap6xxx_wl_regon");
	}

	return;	
}

static void cfg_gpio_32k_clkout(int gpio_index)
{
	int ret;    
	struct clk *clk_32k, *parent;    
    
	parent = clk_get(NULL, CLK_SYS_LOSC);	
	clk_32k = clk_get(NULL, CLK_MOD_OUTA);
	ret = clk_set_parent(clk_32k, parent);

	if(ret){
		pr_err("ap6xxx: 32k clk_set_parent fail\n");
		return;
	}

	ret = clk_set_rate(clk_32k, 32768);
	if(ret){
		pr_err("ap6xxx: 32k clk_set_rate fail\n");
		return;
	}

	clk_enable(clk_32k);
}
void ap6xxx_gpio_init(void)
{
	struct wifi_pm_ops *ops = &wifi_select_pm_ops;
	int ap6xxx_lpo = 0;

/* CT expected ap6xxx_lpo as a GPIO */
	ap6xxx_lpo = gpio_request_ex(wifi_para, "ap6xxx_lpo");
	if (!ap6xxx_lpo) {
		pr_err("ap6xxx: request lpo gpio failed\n");
		return;
	}

	if(ap6xxx_lpo) {
		pr_info("ap6xxx: config 32k clock\n");
		cfg_gpio_32k_clkout(ap6xxx_lpo);
	}

	ap6xxx_wl_regon = gpio_request_ex(wifi_para, "ap6xxx_wl_regon");
	if (!ap6xxx_wl_regon) {
		pr_err("ap6xxx: request wl_regon gpio failed\n");
		return;
	}

	ap6xxx_bt_regon = gpio_request_ex(wifi_para, "ap6xxx_bt_regon");
	if (!ap6xxx_bt_regon) {
		pr_err("ap6xxx: request ap6xxx_bt_regon gpio failed\n");
		return;
	}

	ops->gpio_ctrl	= ap6xxx_gpio_ctrl;
	ops->power = ap6xxx_power;
}
