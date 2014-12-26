
#include "lcd_panel_cfg.h"

//delete this line if you want to use the lcd para define in sys_config1.fex
//#define LCD_PARA_USE_CONFIG

static void LCD_power_on(__u32 sel);
static void LCD_power_off(__u32 sel);
static void LCD_bl_open(__u32 sel);
static void LCD_bl_close(__u32 sel);


static void LCD_panel_init(__u32 sel);
static void LCD_panel_exit(__u32 sel);
void lp079x01_init(void);
void lp079x01_exit(void);

void lcd_cmd_init(void);
void lcd_cmd_exit(void);

#define spi_csx_set(v)	(LCD_GPIO_write(0, 0, v))       //PI10,gpio0
#define spi_sck_set(v)  (LCD_GPIO_write(0, 1, v))	//PI11,gpio1
#define spi_sdi_set(v)  (LCD_GPIO_write(0, 2, v))	//PI12,gpio2
#define lcd_rst_set(v)  (LCD_GPIO_write(0, 3, v))	//PH14,gpio3

#define lcd_panel_rst(v)
#define lcd_2828_rst(v) 
#define lcd_2828_pd(v)  


#ifdef LCD_PARA_USE_CONFIG
static __u8 g_gamma_tbl[][2] = 
{
        //{input value, corrected value}
        {0, 0},
        {15, 15},
        {30, 30},
        {45, 45},
        {60, 60},
        {75, 75},
        {90, 90},
        {105, 105},
        {120, 120},
        {135, 135},
        {150, 150},
        {165, 165},
        {180, 180},
        {195, 195},
        {210, 210},
        {225, 225},
        {240, 240},
        {255, 255},
};

static void LCD_cfg_panel_info(__panel_para_t * info)
{
        __u32 i = 0, j=0;

        memset(info,0,sizeof(__panel_para_t));

        info->lcd_x             = 800;
        info->lcd_y             = 480;
        info->lcd_dclk_freq     = 33;       //MHz

        info->lcd_pwm_not_used  = 0;
        info->lcd_pwm_ch        = 0;
        info->lcd_pwm_freq      = 10000;     //Hz
        info->lcd_pwm_pol       = 0;

        info->lcd_if            = 0;        //0:hv(sync+de); 1:8080; 2:ttl; 3:lvds

        info->lcd_hbp           = 215;      //hsync back porch
        info->lcd_ht            = 1055;     //hsync total cycle
        info->lcd_hspw          = 0;        //hsync plus width
        info->lcd_vbp           = 34;       //vsync back porch
        info->lcd_vt            = 2 * 525;  //vysnc total cycle *2
        info->lcd_vspw          = 0;        //vysnc plus width

        info->lcd_hv_if         = 0;        //0:hv parallel 1:hv serial 
        info->lcd_hv_smode      = 0;        //0:RGB888 1:CCIR656
        info->lcd_hv_s888_if    = 0;        //serial RGB format
        info->lcd_hv_syuv_if    = 0;        //serial YUV format

        info->lcd_cpu_if        = 0;        //0:18bit 4:16bit
        info->lcd_frm           = 0;        //0: disable; 1: enable rgb666 dither; 2:enable rgb656 dither

        info->lcd_lvds_ch       = 0;        //0:single channel; 1:dual channel
        info->lcd_lvds_mode     = 0;        //0:NS mode; 1:JEIDA mode
        info->lcd_lvds_bitwidth = 0;        //0:24bit; 1:18bit
        info->lcd_lvds_io_cross = 0;        //0:normal; 1:pn cross

        info->lcd_io_cfg0       = 0x10000000;

        info->lcd_gamma_correction_en = 0;
        if(info->lcd_gamma_correction_en)
        {
                __u32 items = sizeof(g_gamma_tbl)/2;

        for(i=0; i<items-1; i++)
        {
                __u32 num = g_gamma_tbl[i+1][0] - g_gamma_tbl[i][0];

                //__inf("handling{%d,%d}\n", g_gamma_tbl[i][0], g_gamma_tbl[i][1]);
                for(j=0; j<num; j++)
                {
                        __u32 value = 0;

                        value = g_gamma_tbl[i][1] + ((g_gamma_tbl[i+1][1] - g_gamma_tbl[i][1]) * j)/num;
                        info->lcd_gamma_tbl[g_gamma_tbl[i][0] + j] = (value<<16) + (value<<8) + value;
                        //__inf("----gamma %d, %d\n", g_gamma_tbl[i][0] + j, value);
                }
        }
        info->lcd_gamma_tbl[255] = (g_gamma_tbl[items-1][1]<<16) + (g_gamma_tbl[items-1][1]<<8) + g_gamma_tbl[items-1][1];
        //__inf("----gamma 255, %d\n", g_gamma_tbl[items-1][1]);
        }
}
#endif

void spi_9bit_wire(__u32 tx)
{
	__u8 i;

	spi_csx_set(0);

	for(i=0;i<9;i++)
	{
		LCD_delay_us(1);
		spi_sck_set(0);
		LCD_delay_us(1);
		if(tx & (1 << 8))
			spi_sdi_set(1);
		else
			spi_sdi_set(0);
		LCD_delay_us(1);
		spi_sck_set(1);
		LCD_delay_us(1);
		tx <<= 1;
	}
	spi_sck_set(0);
	LCD_delay_us(1);
	spi_csx_set(1);
	LCD_delay_us(1);
}

static __s32 LCD_open_flow(__u32 sel)
{
	LCD_OPEN_FUNC(sel, LCD_power_on, 10);           //open lcd power, and delay 50ms

	LCD_OPEN_FUNC(sel, LCD_panel_init, 120);         //open lcd controller, and delay 100ms

	LCD_OPEN_FUNC(sel, TCON_open,	120);           //open lcd power, than delay 200ms

	LCD_OPEN_FUNC(sel, LCD_bl_open, 0);             //open lcd backlight, and delay 0ms

	printk(KERN_WARNING"******************* LCD_open_flow*************************");
	return 0;
}

static __s32 LCD_close_flow(__u32 sel)
{	
	LCD_CLOSE_FUNC(sel, LCD_bl_close, 0);           //close lcd backlight, and delay 0ms
	LCD_CLOSE_FUNC(sel, LCD_panel_exit, 0);         //close lcd controller, and delay 0ms
	LCD_CLOSE_FUNC(sel, TCON_close,	50);            //open lcd power, than delay 200ms
	LCD_CLOSE_FUNC(sel, LCD_power_off, 50);         //close lcd power, and delay 500ms
	return 0;
}

static void LCD_power_on(__u32 sel)
{
        LCD_POWER_EN(sel, 1);//config lcd_power pin to open lcd power
}

static void LCD_power_off(__u32 sel)
{
        LCD_POWER_EN(sel, 0);//config lcd_power pin to close lcd power
}

static void LCD_bl_open(__u32 sel)
{
        LCD_PWM_EN(sel, 1);//open pwm module
        LCD_BL_EN(sel, 1);//config lcd_bl_en pin to open lcd backlight
}

static void LCD_bl_close(__u32 sel)
{
        LCD_BL_EN(sel, 0);//config lcd_bl_en pin to close lcd backlight
        LCD_PWM_EN(sel, 0);//close pwm module
}

static void LCD_panel_init(__u32 sel)
{
	printk(KERN_WARNING"******************* lcd_panel_init*************************");
       	lcd_cmd_init();
        return;
}

static void LCD_panel_exit(__u32 sel)
{
        lcd_cmd_exit();
        return;
}

//sel: 0:lcd0; 1:lcd1
static __s32 LCD_user_defined_func(__u32 sel, __u32 para1, __u32 para2, __u32 para3)
{
        return 0;
}

void LCD_get_panel_funs_0(__lcd_panel_fun_t * fun)
{
#ifdef LCD_PARA_USE_CONFIG
        fun->cfg_panel_info = LCD_cfg_panel_info;//delete this line if you want to use the lcd para define in sys_config1.fex
#endif
        fun->cfg_open_flow = LCD_open_flow;
        fun->cfg_close_flow = LCD_close_flow;
        fun->lcd_user_defined_func = LCD_user_defined_func;
}

void lcd_cmd_init(void)
{
	//Hardware pin reset 
        lcd_rst_set(0);
        LCD_delay_ms(1);
        lcd_rst_set(1);
        LCD_delay_ms(1);
		

	//software reset
	spi_9bit_wire(0x001);
        LCD_delay_ms(120);

	//VGH/VGL Setting
	spi_9bit_wire(0x0C1);
	spi_9bit_wire(0x1A8);
	spi_9bit_wire(0x1B1);
	spi_9bit_wire(0x145);
	spi_9bit_wire(0x104);

	//VCOMDC
	spi_9bit_wire(0x0C5);
	spi_9bit_wire(0x180);
	spi_9bit_wire(0x168);

	//GVDD/GVSS
	spi_9bit_wire(0x0C6);
	spi_9bit_wire(0x1BD);
	spi_9bit_wire(0x184);

	//NGVDD/NGVSS
	spi_9bit_wire(0x0C7);
	spi_9bit_wire(0x1BD);
	spi_9bit_wire(0x184);
	//Sleep out
	spi_9bit_wire(0x011);
        LCD_delay_ms(120);

	//Gamma Setting
	spi_9bit_wire(0x0F2);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x182);

	//Gamma enable
	spi_9bit_wire(0x026);
	spi_9bit_wire(0x108);

	//Positive gamma setting
	spi_9bit_wire(0x0E0);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x106);
	spi_9bit_wire(0x10B);
	spi_9bit_wire(0x10D);
	spi_9bit_wire(0x10F);
	spi_9bit_wire(0x113);
	spi_9bit_wire(0x10D);
	spi_9bit_wire(0x10D);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x104);
	spi_9bit_wire(0x109);
	spi_9bit_wire(0x113);
	spi_9bit_wire(0x114);
	spi_9bit_wire(0x12B);
	spi_9bit_wire(0x126);
	spi_9bit_wire(0x123);

	//Negative gamma setting
	spi_9bit_wire(0x0E1);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x106);
	spi_9bit_wire(0x10B);
	spi_9bit_wire(0x10D);
	spi_9bit_wire(0x10F);
	spi_9bit_wire(0x113);
	spi_9bit_wire(0x10D);
	spi_9bit_wire(0x10D);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x104);
	spi_9bit_wire(0x109);
	spi_9bit_wire(0x113);
	spi_9bit_wire(0x114);
	spi_9bit_wire(0x12B);
	spi_9bit_wire(0x126);
	spi_9bit_wire(0x123);

	//Enable gamma setting
	spi_9bit_wire(0x026);
	spi_9bit_wire(0x108);

	//Enable 2-dot func
	spi_9bit_wire(0x0FD);
	spi_9bit_wire(0x100);
	spi_9bit_wire(0x108);

	//Display on
	spi_9bit_wire(0x029);
}

void lcd_cmd_exit(void)
{
	//power off
	spi_9bit_wire(0x010);
}

void lp079x01_exit(void)
{
}

//EXPORT_SYMBOL(LCD_get_panel_funs_0);

