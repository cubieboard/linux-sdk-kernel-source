#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/gpio.h>
#include <linux/gpio.h>

//GPIO标号,非常重要，通过script.fex中的选项gpio_para对应配置顺序。从上往下对应的gpio标号依次是1，2，3......

int num =  5;


static int __init power_init(void)
{

	/*4.申请GPIO资源*/
        if (gpio_request(num,NULL)){
		printk("reqest gpio error\n");
	}

	/*5.配置为输出口*/
	gpio_direction_output(num,1);

	return 0;
}

static void __exit power_exit(void)
{
	/*释放GPIO资源*/
	gpio_free(num);
}
module_init(power_init);
module_exit(power_exit);

MODULE_AUTHOR("sam");
MODULE_DESCRIPTION("Poll up gpio to power on 3G/4G for CubieAIO board");
MODULE_LICENSE("GPL");

