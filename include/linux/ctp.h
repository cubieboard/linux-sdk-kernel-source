#ifndef _CTP_H
#define _CTP_H
//#define TOUCH_KEY_LIGHT_SUPPORT
struct ctp_config_info{
        int ctp_used;
        __u32 twi_id;
        int screen_max_x;
        int screen_max_y;
        int revert_x_flag;
        int revert_y_flag;
        int exchange_x_y_flag;
        u32 irq_gpio_number;
        u32 wakeup_gpio_number; 
#ifdef TOUCH_KEY_LIGHT_SUPPORT 
        u32 key_light_gpio_number;
#endif             
};

enum{
        DEBUG_INIT = 1U << 0,
        DEBUG_SUSPEND = 1U << 1,
        DEBUG_INT_INFO = 1U << 2,
        DEBUG_X_Y_INFO = 1U << 3,
        DEBUG_KEY_INFO = 1U << 4,
        DEBUG_WAKEUP_INFO = 1U << 5,
        DEBUG_OTHERS_INFO = 1U << 6, 
                     
}; 
extern bool ctp_get_int_enable(u32 *enable);
extern bool ctp_set_int_enable(u32 enable);
extern bool ctp_get_int_port_rate(u32 *clk);
extern bool ctp_set_int_port_rate(u32 clk);
extern bool ctp_get_int_port_deb(u32 *clk_pre_scl);
extern bool ctp_set_int_port_deb(u32 clk_pre_scl);
extern void ctp_free_platform_resource(void);
extern int  ctp_init_platform_resource(void);
extern void ctp_print_info(struct ctp_config_info info,int debug_level);
extern int  ctp_wakeup(int status,int ms);
extern int ctp_i2c_write_bytes(struct i2c_client *client, uint8_t *data, uint16_t len);
extern int ctp_i2c_read_bytes_addr16(struct i2c_client *client, uint8_t *buf, uint16_t len);
extern bool ctp_i2c_test(struct i2c_client * client);
#ifdef TOUCH_KEY_LIGHT_SUPPORT 
extern int ctp_key_light(int status,int ms);
#endif
#endif