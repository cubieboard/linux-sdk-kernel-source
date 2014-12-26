/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include "ft5x_ts.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/pm.h>
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/gpio.h> 
#include <linux/ctp.h>


//#include <mach/sys_config.h> 


#define FOR_TSLIB_TEST
//#define TOUCH_KEY_SUPPORT
#ifdef TOUCH_KEY_SUPPORT
#define TOUCH_KEY_FOR_EVB13
//#define TOUCH_KEY_FOR_ANGDA
#ifdef TOUCH_KEY_FOR_ANGDA
#define TOUCH_KEY_X_LIMIT	        (60000)
#define TOUCH_KEY_NUMBER	        (4)
#endif
#ifdef TOUCH_KEY_FOR_EVB13
#define TOUCH_KEY_LOWER_X_LIMIT	        (848)
#define TOUCH_KEY_HIGHER_X_LIMIT	(852)
#define TOUCH_KEY_NUMBER	        (5)
#endif
#endif

//#define CONFIG_SUPPORT_FTS_CTP_UPG
        
struct i2c_dev{
        struct list_head list;	
        struct i2c_adapter *adap;
        struct device *dev;
};

static struct class *i2c_dev_class;
static LIST_HEAD (i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

#define FT5X_NAME	"ft5x_ts"

static struct i2c_client *this_client;

#ifdef TOUCH_KEY_SUPPORT
static int key_tp  = 0;
static int key_val = 0;
#endif



#define CTP_CHECK_DELAY 1
#define _DOUBLE_CLICK_CHECK_DEBUG_
#ifdef _DOUBLE_CLICK_CHECK_DEBUG_
static int double_click_check = 0;
static int touch_querry_cnt   = 0;
static int double_click_check_last_x = 0;
static int double_click_check_last_y = 0;
#endif

#define _FT_DATA_FILTER_
#ifdef _FT_DATA_FILTER_
	static int mis_touch_chcek = 0;
	static int mis_touch_check_double_click_check_last_x = 0;
	static int mis_touch_check_double_click_check_last_y = 0;
#endif

/*********************************************************************************************/
#define CTP_IRQ_NUMBER                  (config_info.irq_gpio_number)
#define CTP_IRQ_MODE			(TRIG_EDGE_NEGATIVE)
#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define CTP_NAME			 FT5X_NAME
#define PRESS_MAX			(255)

static int screen_max_x = 0;
static int screen_max_y = 0;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static u32 int_handle = 0;
static __u32 twi_id = 0;
static bool is_suspend = false;

struct ft5x_ts_data *s_ft5x_ts;

//extern struct ctp_config_info config_info;

static u32 debug_mask = 0xff;
#define dprintk(level_mask,fmt,arg...)    printk("***CTP***"fmt, ## arg)//if(unlikely(debug_mask & level_mask)) \
       // printk("***CTP***"fmt, ## arg)
module_param_named(debug_mask,debug_mask,int,S_IRUGO | S_IWUSR | S_IWGRP);
/*********************************************************************************************/
/*------------------------------------------------------------------------------------------*/        
/* Addresses to scan */
static const unsigned short normal_i2c[2] = {0x5c,I2C_CLIENT_END};
static const int chip_id_value[] = {0x55,0x06,0x08,0x02,0xa3};

static void ft5x_resume_events(struct work_struct *work);
struct workqueue_struct *ft5x_resume_wq;
static DECLARE_WORK(ft5x_resume_work, ft5x_resume_events);
/*------------------------------------------------------------------------------------------*/ 

static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
        int ret = 0, i = 0;
        dprintk(DEBUG_INIT,"#################ctp_detect\n");
        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
                return -ENODEV;
    
	if(twi_id == adapter->nr){
	       // ret = i2c_smbus_read_byte_data(client,0xA3);
                dprintk(DEBUG_INIT,"addr:0x%x,chip_id_value:0x%x\n",client->addr,ret);
            //    while(chip_id_value[i++]){
                       // if(ret == chip_id_value[i - 1]){
            	                strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
    		                return 0;
                       // }                   
               // }
        	printk("%s:I2C connection might be something wrong ! \n",__func__);
        	//return -ENODEV;
	}else{
		printk("%s:I2C adapter->nr might be something wrong ! \n",__func__);
		return -ENODEV;
	}
}

int fts_ctpm_fw_upgrade_with_i_file(void);

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);
	
	list_for_each_entry(i2c_dev,&i2c_dev_list,list){
		dprintk(DEBUG_OTHERS_INFO,"--line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",\
		        __LINE__,i2c_dev->adap->nr,index);
		if(i2c_dev->adap->nr == index){
		     goto found;
		}
	}
	i2c_dev = NULL;
	
found: 
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev ;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap) 
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS){
		dprintk(DEBUG_OTHERS_INFO,"i2c-dev:out of device minors (%d) \n",adap->nr);
		return ERR_PTR (-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev){
		return ERR_PTR(-ENOMEM);
	}
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	
	return i2c_dev;
}


static int ft5x_i2c_rxdata(char *rxdata, int length);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
	s16     touch_ID1;
	s16     touch_ID2;
	s16     touch_ID3;
	s16     touch_ID4;
	s16     touch_ID5;
        u8      touch_point;
};

struct ft5x_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct delayed_work 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
    int prev_touch_num;
};


/* ---------------------------------------------------------------------
*
*   Focal Touch panel upgrade related driver
*
*
----------------------------------------------------------------------*/

typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit 

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        (0x70>>1)

static void delay_ms(FTS_WORD  w_ms)
{
	//platform related, please implement this function
	msleep( w_ms );
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_read_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;

	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret != dw_lenth){
		printk("ret = %d. \n", ret);
		printk("i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_write_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret != dw_lenth){
		printk("i2c_write_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}


/***************************************************************************************/

/*
[function]: 
    read out the register value.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[out]    :the returned register value;
    bt_len[in]        :length of pbt_buf, should be set to 2;        
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
u8 fts_register_read(u8 e_reg_name, u8* pbt_buf, u8 bt_len)
{
	u8 read_cmd[3]= {0};
	u8 cmd_len     = 0;

	read_cmd[0] = e_reg_name;
	cmd_len = 1;    

	/*call the write callback function*/
	//    if(!i2c_write_interface(I2C_CTPM_ADDRESS, &read_cmd, cmd_len))
	//    {
	//        return FTS_FALSE;
	//    }


	if(!i2c_write_interface(I2C_CTPM_ADDRESS, read_cmd, cmd_len))	{//change by zhengdixu
		return FTS_FALSE;
	}

	/*call the read callback function to get the register value*/        
	if(!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len)){
		return FTS_FALSE;
	}
	return FTS_TRUE;
}

/*
[function]: 
    write a value to register.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[in]        :the returned register value;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int fts_register_write(u8 e_reg_name, u8 bt_value)
{
	FTS_BYTE write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = bt_value;

	/*call the write callback function*/
	//return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, 2);
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2); //change by zhengdixu
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	//return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, num);
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);//change by zhengdixu
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_write(u8* pbt_buf, u16 dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_read(u8* pbt_buf, u8 bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
	//ft5x_i2c_rxdata
}

#if 0
static void ft5x_touch_up(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
        input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
        input_mt_sync(data->input_dev);
}
#endif

/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH       128 //2//4//8//16//32//64//128//256

static unsigned char CTPM_FW[]=
{
        #include "ft_app.i"
};
unsigned char fts_ctpm_get_i_file_ver(void)
{
        unsigned int ui_sz;
        ui_sz = sizeof(CTPM_FW);
        if (ui_sz > 2){
                return CTPM_FW[ui_sz - 2];
        }else{
                //TBD, error handling?
                return 0xff; //default value
        }
}

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, u16 dw_lenth)
{
        u8 reg_val[2] = {0};
        FTS_BOOL i_ret = 0;
        u16 i = 0;
        
        
        u16  packet_number;
        u16  j;
        u16  temp;
        u16  lenght;
        u8  packet_buf[FTS_PACKET_LENGTH + 6];
        u8  auc_i2c_write_buf[10];
        u8 bt_ecc;

        /*********Step 1:Reset  CTPM *****/
        /*write 0xaa to register 0xfc*/
        delay_ms(100);
        fts_register_write(0xfc,0xaa);
        delay_ms(50);
        /*write 0x55 to register 0xfc*/
        fts_register_write(0xfc,0x55);
        printk("Step 1: Reset CTPM test\n");
        
        delay_ms(30);

        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = 0x55;
        auc_i2c_write_buf[1] = 0xaa;
        i = 0;
        do{
                i++;
                i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
                printk("Step 2: Enter update mode. \n");
                delay_ms(5);
        }while((FTS_FALSE == i_ret) && i<5);

        /*********Step 3:check READ-ID***********************/
        /*send the opration head*/
        i = 0;
        do{
                if(i > 3){
                        cmd_write(0x07,0x00,0x00,0x00,1);
		        return ERR_READID; 
                }
                /*read out the CTPM ID*/
                printk("====Step 3:check READ-ID====");
                cmd_write(0x90,0x00,0x00,0x00,4);
                byte_read(reg_val,2);
                i++;
                delay_ms(5);
                printk("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
         }while((reg_val[1] != 0x03)&&(reg_val[1] != 0x06));//while(reg_val[0] != 0x79 || reg_val[1] != 0x03);

        /*********Step 4:erase app*******************************/
        cmd_write(0x61,0x00,0x00,0x00,1);
        delay_ms(1500);
        printk("Step 4: erase. \n");
        
        /*********Step 5:write firmware(FW) to ctpm flash*********/
        bt_ecc = 0;
        printk("Step 5: start upgrade. \n");
        dw_lenth = dw_lenth - 8;
        packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
        packet_buf[0] = 0xbf;
        packet_buf[1] = 0x00;
        for (j=0;j<packet_number;j++){
                temp = j * FTS_PACKET_LENGTH;
                packet_buf[2] = (FTS_BYTE)(temp>>8);
                packet_buf[3] = (FTS_BYTE)temp;
                lenght = FTS_PACKET_LENGTH;
                packet_buf[4] = (FTS_BYTE)(lenght>>8);
                packet_buf[5] = (FTS_BYTE)lenght;
        
                for (i=0;i<FTS_PACKET_LENGTH;i++){
                        packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
                        bt_ecc ^= packet_buf[6+i];
                }
        
                byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
                delay_ms(FTS_PACKET_LENGTH/6 + 1);
                if ((j * FTS_PACKET_LENGTH % 1024) == 0){
                        printk("upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
                }
        }

        if ((dw_lenth) % FTS_PACKET_LENGTH > 0){
                temp = packet_number * FTS_PACKET_LENGTH;
                packet_buf[2] = (FTS_BYTE)(temp>>8);
                packet_buf[3] = (FTS_BYTE)temp;
        
                temp = (dw_lenth) % FTS_PACKET_LENGTH;
                packet_buf[4] = (FTS_BYTE)(temp>>8);
                packet_buf[5] = (FTS_BYTE)temp;
        
                for (i=0;i<temp;i++){
                        packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
                        bt_ecc ^= packet_buf[6+i];
                }
        
                byte_write(&packet_buf[0],temp+6);    
                delay_ms(20);
        }

        //send the last six byte
        for (i = 0; i<6; i++){
                temp = 0x6ffa + i;
                packet_buf[2] = (FTS_BYTE)(temp>>8);
                packet_buf[3] = (FTS_BYTE)temp;
                temp =1;
                packet_buf[4] = (FTS_BYTE)(temp>>8);
                packet_buf[5] = (FTS_BYTE)temp;
                packet_buf[6] = pbt_buf[ dw_lenth + i]; 
                bt_ecc ^= packet_buf[6];
        
                byte_write(&packet_buf[0],7);  
                delay_ms(20);
        }

        /*********Step 6: read out checksum***********************/
        /*send the opration head*/
        //cmd_write(0xcc,0x00,0x00,0x00,1);//把0xcc当作寄存器地址，去读出一个字节
        // byte_read(reg_val,1);//change by zhengdixu

	fts_register_read(0xcc, reg_val,1);
	
        printk("Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
        if(reg_val[0] != bt_ecc){
                cmd_write(0x07,0x00,0x00,0x00,1);
		return ERR_ECC;
        }

        /*********Step 7: reset the new FW***********************/
        cmd_write(0x07,0x00,0x00,0x00,1);
        msleep(30);
        return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
        unsigned char uc_temp;
        unsigned char i ;
        
        printk("[FTS] start auto CLB.\n");
        msleep(200);
        fts_register_write(0, 0x40);  
        delay_ms(100);                       //make sure already enter factory mode
        fts_register_write(2, 0x4);               //write command to start calibration
        delay_ms(300);
        for(i=0;i<100;i++){
                fts_register_read(0,&uc_temp,1);
                if (((uc_temp&0x70)>>4) == 0x0){    //return to normal mode, calibration finish
                        break;
                }
                delay_ms(200);
                printk("[FTS] waiting calibration %d\n",i);
        }
        
        printk("[FTS] calibration OK.\n");
        
        msleep(300);
        fts_register_write(0, 0x40);          //goto factory mode
        delay_ms(100);                       //make sure already enter factory mode
        fts_register_write(2, 0x5);          //store CLB result
        delay_ms(300);
        fts_register_write(0, 0x0);          //return to normal mode 
        msleep(300);
        printk("[FTS] store CLB result OK.\n");
        return 0;
}
void getVerNo(u8* buf, int len)
{
	u8 start_reg=0x0;
	int ret = -1;
	//int status = 0;
	int i = 0;
	start_reg = 0xa6;

#if 0
	printk("read 0xa6 one time. \n");
	if(FTS_FALSE == fts_register_read(0xa6, buf, len)){
                return ;
	}
	
	for (i=0; i< len; i++) {
		printk("=========buf[%d] = 0x%x \n", i, buf[i]);
	}
	
	printk("read 0xa8. \n");
	if(FTS_FALSE == fts_register_read(0xa8, buf, len)){
                return ;
	}
	for (i=0; i< len; i++) {
		printk("=========buf[%d] = 0x%x \n", i, buf[i]);
	}

	ft5x_i2c_rxdata(buf, len);
	
        for (i=0; i< len; i++) {
                printk("=========buf[%d] = 0x%x \n", i, buf[i]);
        }

        byte_read(buf, len);
        for (i=0; i< len; i++) {
                printk("=========buf[%d] = 0x%x \n", i, buf[i]);
        }
          
#endif

	ret =fts_register_read(0xa6, buf, len);
	//et = ft5406_read_regs(ft5x0x_ts_data_test->client,start_reg, buf, 2);
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return;
	}
	for (i=0; i<2; i++) {
		printk("=========buf[%d] = 0x%x \n", i, buf[i]);
	}
	return;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret = 0;
	unsigned char a;
	unsigned char b;
#define BUFFER_LEN (2)            //len == 2 
	unsigned char buf[BUFFER_LEN] = {0};
   
	//=========FW upgrade========================*/
	printk("%s. \n", __func__);

	pbt_buf = CTPM_FW;
	//msleep(200);
        // cmd_write(0x07,0x00,0x00,0x00,1);
	msleep(100);
	getVerNo(buf, BUFFER_LEN);
	a = buf[0];
	b = fts_ctpm_get_i_file_ver();
	printk("a == %hu,  b== %hu \n",a, b);

	/*
	  * when the firmware in touch panel maybe corrupted,
	  * or the firmware in host flash is new, need upgrade
	  */
	if ( 0xa6 == a ||a < b ){
		/*call the upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(&pbt_buf[0],sizeof(CTPM_FW));
		if (i_ret != 0){
			printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		} else {
			printk("[FTS] upgrade successfully.\n");
			fts_ctpm_auto_clb();  //start auto CLB
		}
	}	
	return i_ret;
	
}

unsigned char fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2){
		return CTPM_FW[0];
	}
	else{
		return 0xff; //default value
	}
}

static int ft5x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		printk("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static void ft5x_ts_release(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
#ifdef CONFIG_FT5X0X_MULTITOUCH	
#ifdef TOUCH_KEY_SUPPORT
	if(1 == key_tp){
		input_report_key(data->input_dev, key_val, 0);
		dprintk(DEBUG_KEY_INFO,"Release Key = %d\n",key_val);		
	} else{
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	}
#else
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#endif

#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	
	input_sync(data->input_dev);
	return;

}


static void dump_data(unsigned char *buf,int len)
{
	int i;
	printk("\n======================DUMP DATA=====================");
	for(i = 0;i < len; i ++){
		if(i%8 == 0)
			printk("\n");
			
		printk("0x%X  ",buf[i]);
		
	}
	printk("\n======================DUMP DATA=====================\n");
}
static int ft5x_read_data(void)
{
#define GET_ABS(x,y) (x>y?(x-y):(y-x))
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	unsigned char buf[256 /*44 */]={0};
	unsigned char two_piont_buf[8]={64};
	unsigned char config_reg_buf[5]={0x67};
	int dis_x = 0,		dis_y = 0;
	int ret = -1;

#ifdef _DOUBLE_CLICK_CHECK_DEBUG_
	if(double_click_check){
		touch_querry_cnt++;
		//dprintk(DEBUG_X_Y_INFO,"\t  timeout++ =%d \n",touch_querry_cnt);
	}
#endif

//#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x_i2c_rxdata(buf, 110/*256 */ );
	if (ret < 0) {
		dprintk(DEBUG_X_Y_INFO,"%s:%d read_data i2c_rxdata failed: %d\n", __func__, __LINE__,ret);
		#ifdef  _FT_DATA_FILTER_
		mis_touch_chcek = 0;
		#endif
		return ret;
	}
	//dprintk(DEBUG_X_Y_INFO," xy_map :\n");
	//dump_data(buf,110/*256 *//*44 */ );

	//printk("\n############# (x1,y1)=(%d,%d) (x2,y2)=(%d,%d)  ###############\n",
	//	buf[64] + (buf[68]<<8),
	//	buf[65] + (buf[69]<<8),
	//	buf[66] + (buf[70]<<8),
	//	buf[67] + (buf[71]<<8));


/*
    ret = ft5x_i2c_rxdata(two_piont_buf, 8);
	if (ret < 0) {
		dprintk(DEBUG_X_Y_INFO,"%s:%d read_data i2c_rxdata failed: %d\n", __func__, __LINE__,ret);
		return ret;
	}
	dprintk(DEBUG_X_Y_INFO," two point :\n");
	dump_data(two_piont_buf,8);

	
	ret = ft5x_i2c_rxdata(config_reg_buf, 5);
//#else
//	ret = ft5x_i2c_rxdata(buf, 31);
//#endif
	if (ret < 0) {
		dprintk(DEBUG_X_Y_INFO,"%s:%d read_data i2c_rxdata failed: %d\n", __func__, __LINE__,ret);
		return ret;
	}
	dprintk(DEBUG_X_Y_INFO," config reg :\n");
	dump_data(config_reg_buf,5);

	return -1;
*/
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = buf[109];

	if (event->touch_point == 0 ) {
		if(data->prev_touch_num != 0){
			ft5x_ts_release();
		}
		data->prev_touch_num = 0;
		#ifdef  _FT_DATA_FILTER_
		mis_touch_chcek = 0;
		//dprintk(DEBUG_X_Y_INFO,"\t 22 \n");
		#endif
		return 1;
	}

	data->prev_touch_num = event->touch_point;

	switch (event->touch_point) {
	case 5:
		event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
		event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
		dprintk(DEBUG_X_Y_INFO,"source data:event->x5 = %d, event->y5 = %d. \n", event->x5, event->y5);
		if(1 == exchange_x_y_flag){
			swap(event->x5, event->y5);
		}
		if(1 == revert_x_flag){
			event->x5 = SCREEN_MAX_X - event->x5;
		}
		if(1 == revert_y_flag){
			event->y5 = SCREEN_MAX_Y - event->y5;
		}
		event->touch_ID5=(s16)(buf[0x1d] & 0xF0)>>4;
		
		dprintk(DEBUG_X_Y_INFO,"touch id : %d. \n",event->touch_ID5);
	case 4:
		event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
		event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
		dprintk(DEBUG_X_Y_INFO,"source data:event->x4 = %d, event->y4 = %d. \n", event->x4, event->y4);
		if(1 == exchange_x_y_flag){
			swap(event->x4, event->y4);
		}
		if(1 == revert_x_flag){
			event->x4 = SCREEN_MAX_X - event->x4;
		}
		if(1 == revert_y_flag){
			event->y4 = SCREEN_MAX_Y - event->y4;
		}	
		event->touch_ID4=(s16)(buf[0x17] & 0xF0)>>4;
		
		dprintk(DEBUG_X_Y_INFO,"touch id : %d. \n",event->touch_ID4);
	case 3:
		event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
		event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
		dprintk(DEBUG_X_Y_INFO,"source data:event->x3 = %d, event->y3 = %d. \n", event->x3, event->y3);
		if(1 == exchange_x_y_flag){
			swap(event->x3, event->y3);
		}
		if(1 == revert_x_flag){
			event->x3 = SCREEN_MAX_X - event->x3;
		}
		if(1 == revert_y_flag){
			event->y3 = SCREEN_MAX_Y - event->y3;
		}
		event->touch_ID3=(s16)(buf[0x11] & 0xF0)>>4;
		dprintk(DEBUG_X_Y_INFO,"touch id : %d. \n",event->touch_ID3);
	case 2:
		event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
		event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
		dprintk(DEBUG_X_Y_INFO,"source data:event->x2 = %d, event->y2 = %d. \n", event->x2, event->y2);
		if(1 == exchange_x_y_flag){
			swap(event->x2, event->y2);
		}
		if(1 == revert_x_flag){
			event->x2 = SCREEN_MAX_X - event->x2;
		}
		if(1 == revert_y_flag){
			event->y2 = SCREEN_MAX_Y - event->y2;
		}
		event->touch_ID2=(s16)(buf[0x0b] & 0xF0)>>4;
		
		dprintk(DEBUG_X_Y_INFO,"touch id : %d. \n",event->touch_ID2);
	case 1:
		event->x1 = (s16)(buf[68] & 0x0F)<<8 | (s16)buf[64];
		event->y1 = (s16)(buf[69] & 0x0F)<<8 | (s16)buf[65];
		//dprintk(DEBUG_X_Y_INFO,"source data:event->x1 = %d, event->y1 = %d. x_s=%d y_s=%d t_p=%d\n",
		//		event->x1, event->y1, buf[103], buf[104], buf[109]);
		if(1 == exchange_x_y_flag){
			swap(event->x1, event->y1);
		}

		if(1 == revert_x_flag){
			event->x1 = SCREEN_MAX_X - event->x1;
		}
		if(1 == revert_y_flag){
			event->y1 = SCREEN_MAX_Y - event->y1;
		}
		event->touch_ID1=(s16)(buf[0x05] & 0xF0)>>4;
		//dprintk(DEBUG_X_Y_INFO,"touch id : %d. \n",event->touch_ID1);
		//dprintk(DEBUG_X_Y_INFO,"touch x=%d y=%d \n",event->x1,event->y1);
		if(event->x1==0 && event->y1==0)
			return -1;

		//dprintk(DEBUG_X_Y_INFO,"\t start to check eouble_click timeout=%d \n",touch_querry_cnt);
		///< check double click
		#ifdef  _FT_DATA_FILTER_
		if(mis_touch_chcek){
			if((GET_ABS(mis_touch_check_double_click_check_last_x,event->x1)<15) && (GET_ABS(mis_touch_check_double_click_check_last_y,event->y1)<10)) {///< same touch
				return 1; //< same touch,return error;
			} else {
				;//dprintk(DEBUG_X_Y_INFO,"\t 33 %d %d\n",
				//		GET_ABS(mis_touch_check_double_click_check_last_x,event->x1),
				//		GET_ABS(mis_touch_check_double_click_check_last_y,event->y1)
				//		);
			}
		}

		mis_touch_chcek = 1;
		mis_touch_check_double_click_check_last_x = event->x1;
		mis_touch_check_double_click_check_last_y = event->y1;
		#endif

		#ifdef _DOUBLE_CLICK_CHECK_DEBUG_
		//if(touch_querry_cnt > (20/CTP_CHECK_DELAY)){  ///< 距离上次click时间太长
		if(touch_querry_cnt<=2 && touch_querry_cnt>12){  ///< 距离上次click时间太长
			touch_querry_cnt = 0;
			double_click_check = 1;
			//dprintk(DEBUG_X_Y_INFO,"\t\t restart to check eouble_click timeout=%d \n",touch_querry_cnt);
		}else{
			if(double_click_check){  ///< double_click命中
				touch_querry_cnt   = 0;
				dis_x = GET_ABS(double_click_check_last_x, event->x1);
				dis_y = GET_ABS(double_click_check_last_y, event->y1);
				//dprintk(DEBUG_X_Y_INFO,"\t\t verify double_click x=%d y=%d %d %d dis_x=%d, dis_y=%d\n",
				//		event->x1,event->y1, double_click_check_last_x, double_click_check_last_y,dis_x, dis_y);
				if(dis_x<30 && dis_y<15) {
					double_click_check = 0;
					event->x1 = double_click_check_last_x;
					event->y1 = double_click_check_last_y;
					//dprintk(DEBUG_X_Y_INFO,"\t\t double_click x=%d y=%d \n",event->x1,event->y1);
				}
			}else{
				double_click_check = 1;
			}
		}

		double_click_check_last_x = event->x1;
		double_click_check_last_y = event->y1;
		#endif
		break;
	default:
		return -1;
	}
	event->pressure = 200;
        return 0;
}

#ifdef TOUCH_KEY_LIGHT_SUPPORT
static void ft5x_lighting(void)
{
        ctp_key_light(1,15);
	return;
}
#endif



static void ft5x_report_multitouch(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#ifdef TOUCH_KEY_SUPPORT
	if(1 == key_tp){
		return;
	}
#endif

	switch(event->touch_point) {
	case 5:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID5);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		dprintk(DEBUG_X_Y_INFO,"report data:===x5 = %d,y5 = %d ====\n",event->x5,event->y5);
	case 4:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID4);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		dprintk(DEBUG_X_Y_INFO,"report data:===x4 = %d,y4 = %d ====\n",event->x4,event->y4);
	case 3:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID3);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		dprintk(DEBUG_X_Y_INFO,"report data:===x3 = %d,y3 = %d ====\n",event->x3,event->y3);
	case 2:
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID2);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		dprintk(DEBUG_X_Y_INFO,"report data:===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
	case 1:
	
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID1);	
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);


		
		dprintk(DEBUG_X_Y_INFO,"report data:===x1 = %d,y1 = %d ====\n",event->x1,event->y1);
		break;
	/*case 0:
		ft5x_touch_up();
		break;*/
	default:
		dprintk(DEBUG_X_Y_INFO,"report data:==touch_point default =\n");
		break;
	}
	
	input_sync(data->input_dev);
	return;
}

#ifndef CONFIG_FT5X0X_MULTITOUCH
static void ft5x_report_singletouch(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	dprintk(DEBUG_X_Y_INFO,"report:===x1 = %d,y1 = %d ====\n",event->x1,event->y1);
	input_report_key(data->input_dev, BTN_TOUCH, 1);
	input_sync(data->input_dev);
	return;
}
#endif

#ifdef TOUCH_KEY_SUPPORT
static void ft5x_report_touchkey(void)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#ifdef TOUCH_KEY_FOR_ANGDA
	if((1==event->touch_point)&&(event->x1 > TOUCH_KEY_X_LIMIT)){
		key_tp = 1;
		if(event->y1 < 40){
			key_val = 1;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);  
			dprintk(DEBUG_KEY_INFO,"===KEY 1====\n");
		}else if(event->y1 < 90){
			key_val = 2;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			dprintk(DEBUG_KEY_INFO,"===KEY 2 ====\n");
		}else{
			key_val = 3;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			dprintk(DEBUG_KEY_INFO,"===KEY 3====\n");	
		}
	} else{
		key_tp = 0;
	}
#endif
#ifdef TOUCH_KEY_FOR_EVB13
	if((1==event->touch_point)&&((event->x1 > TOUCH_KEY_LOWER_X_LIMIT)&&(event->x1<TOUCH_KEY_HIGHER_X_LIMIT))){
		key_tp = 1;
		if(event->y1 < 5){
			key_val = 1;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);  
			dprintk(DEBUG_KEY_INFO,"===KEY 1====\n");     
		}else if((event->y1 < 45)&&(event->y1>35)){
			key_val = 2;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			dprintk(DEBUG_KEY_INFO,"===KEY 2 ====\n");
		}else if((event->y1 < 75)&&(event->y1>65)){
			key_val = 3;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			dprintk(DEBUG_KEY_INFO,"===KEY 3====\n");
		}else if ((event->y1 < 105)&&(event->y1>95))	{
			key_val = 4;
			input_report_key(data->input_dev, key_val, 1);
			input_sync(data->input_dev);     
			dprintk(DEBUG_KEY_INFO,"===KEY 4====\n");	
		}
	}else{
		key_tp = 0;
	}
#endif

#ifdef TOUCH_KEY_LIGHT_SUPPORT
	ft5x_lighting();
#endif
	return;
}
#endif

static void ft5x_report_value(void)
{

#ifdef TOUCH_KEY_SUPPORT
	ft5x_report_touchkey();
#endif

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ft5x_report_multitouch();
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	ft5x_report_singletouch();
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	return;
}	

static int tst_cnt =0 ; ///for debug sensity
static void ft5x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	tst_cnt++;
	ret = ft5x_read_data();
	if (ret == 0) {
		ft5x_report_value();
		//dprintk(DEBUG_INT_INFO,"\t yy %d\n",tst_cnt);
		tst_cnt = 0;
	}
	//dprintk(DEBUG_INT_INFO,"%s:ret:%d\n",__func__,ret);
	queue_delayed_work(s_ft5x_ts->ts_workqueue, &s_ft5x_ts->pen_event_work, CTP_CHECK_DELAY);
}

#if 0
static u32 ft5x_ts_interrupt(struct ft5x_ts_data *ft5x_ts)
{

	/*enum gpio_eint_trigtype trigtype;

	sw_gpio_eint_get_trigtype(CTP_IRQ_NUMBER,&sw_gpio_eint_get_trigtype);
	if(sw_gpio_eint_get_trigtype == TRIG_EDGE_NEGATIVE){
		sw_gpio_eint_set_trigtype(CTP_IRQ_NUMBER,TRIG_EDGE_POSITIVE);
		ft5x_ts->touch_down = 1;
	}else {
		sw_gpio_eint_set_trigtype(CTP_IRQ_NUMBER,TRIG_EDGE_NEGATIVE);
		ft5x_ts->touch_down = 0;
	}
	*/
	dprintk(DEBUG_INT_INFO,"==========ft5x_ts TS Interrupt============\n"); 
	queue_work(ft5x_ts->ts_workqueue, &ft5x_ts->pen_event_work);
	
	return 0;
}
#endif 

static void ft5x_resume_events (struct work_struct *work)
{
	//ctp_wakeup(0,20);
#ifdef CONFIG_HAS_EARLYSUSPEND	
	if(STANDBY_WITH_POWER_OFF == standby_level){
	        msleep(100);
	}
#endif	
//	sw_gpio_eint_set_enable(CTP_IRQ_NUMBER,1);
}



static int ft5x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	dprintk(DEBUG_SUSPEND,"==ft5x_ts_suspend=\n");
	dprintk(DEBUG_SUSPEND,"CONFIG_PM: write FT5X0X_REG_PMODE .\n");
	
	is_suspend = false; 
	
	flush_workqueue(ft5x_resume_wq);
	//sw_gpio_eint_set_enable(CTP_IRQ_NUMBER,0);
	cancel_delayed_work_sync(&data->pen_event_work);
	flush_workqueue(data->ts_workqueue);
	//ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	
	return 0;
}
static int ft5x_ts_resume(struct i2c_client *client)
{
	dprintk(DEBUG_SUSPEND,"==CONFIG_PM:ft5x_ts_resume== \n");
	if(is_suspend == false)
	        queue_work(ft5x_resume_wq, &ft5x_resume_work);
	        
	return 0;		
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x_ts_data *data = i2c_get_clientdata(this_client);
	dprintk(DEBUG_SUSPEND,"==ft5x_ts_suspend=\n");
	dprintk(DEBUG_SUSPEND,"CONFIG_HAS_EARLYSUSPEND: write FT5X0X_REG_PMODE .\n");
	
#ifndef CONFIG_HAS_EARLYSUSPEND
        is_suspend = true;
#endif  
        if(is_suspend == true){ 
	        flush_workqueue(ft5x_resume_wq);
	        //sw_gpio_eint_set_enable(CTP_IRQ_NUMBER,0);
	        cancel_work_sync(&data->pen_event_work);
	        flush_workqueue(data->ts_workqueue);
	        //ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	}
	
	is_suspend = true;
}

static void ft5x_ts_late_resume(struct early_suspend *handler)
{
	dprintk(DEBUG_SUSPEND,"==CONFIG_HAS_EARLYSUSPEND:ft5x_ts_resume== \n");
	
	queue_work(ft5x_resume_wq, &ft5x_resume_work);	
	is_suspend = true;
}
#endif

static int ft5x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x_ts_data *ft5x_ts;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int err = 0;
        

#ifdef TOUCH_KEY_SUPPORT
	int i = 0;
#endif

	dprintk(DEBUG_INIT,"====%s begin=====.  \n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk("check_functionality_failed\n");
		goto exit_check_functionality_failed;
	}


	ft5x_ts = kzalloc(sizeof(*ft5x_ts), GFP_KERNEL);
	if (!ft5x_ts)	{
		err = -ENOMEM;
		printk("alloc_data_failed\n");
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(client, ft5x_ts);


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	fts_ctpm_fw_upgrade_with_i_file();
#endif

	INIT_DELAYED_WORK(&ft5x_ts->pen_event_work, ft5x_ts_pen_irq_work);
	ft5x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x_ts->ts_workqueue) {
		err = -ESRCH;
		printk("ts_workqueue fail!\n");
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x_ts->input_dev = input_dev;


#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);	
#ifdef FOR_TSLIB_TEST
	set_bit(BTN_TOUCH, input_dev->keybit);
#endif
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, 4, 0, 0);
#ifdef TOUCH_KEY_SUPPORT
	key_tp = 0;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 1; i < TOUCH_KEY_NUMBER; i++)
		set_bit(i, input_dev->keybit);
#endif
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	//input_dev->id.bustype = BUS_I2C;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.product = 2;
	input_dev->id.vendor  = 3;
	input_dev->id.version = 10;
	input_dev->phys  = "input3";
	
	input_dev->name	= CTP_NAME;		//dev_name(&client->dev)
	//input_dev->dev.parent = &client->dev;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,"ft5x_ts_probe: failed to register input device: %s\n",
		        dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	ft5x_resume_wq = create_singlethread_workqueue("ft5x_resume");
	if (ft5x_resume_wq == NULL) {
		printk("create ft5x_resume_wq fail!\n");
		return -ENOMEM;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x_ts->early_suspend.suspend = ft5x_ts_early_suspend;
	ft5x_ts->early_suspend.resume	= ft5x_ts_late_resume;
	register_early_suspend(&ft5x_ts->early_suspend);
#endif

#ifdef CONFIG_FT5X0X_MULTITOUCH
	dprintk(DEBUG_INIT,"CONFIG_FT5X0X_MULTITOUCH is defined. \n");
#endif


	
	ft5x_set_reg(110,0x08);//int enable,low,indicate mode
	ft5x_set_reg(103,100);//int enable,low,indicate mode
	ft5x_set_reg(104,100);//int enable,low,indicate mode

	ft5x_ts->prev_touch_num = -1;

	#if 0
        int_handle = sw_gpio_irq_request(CTP_IRQ_NUMBER,CTP_IRQ_MODE,(peint_handle)ft5x_ts_interrupt,ft5x_ts);
	if (!int_handle) {
		printk("ft5x_ts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	
	
	ctp_set_int_port_rate(1);
	ctp_set_int_port_deb(0x07);
	#else
		    s_ft5x_ts = ft5x_ts;
	   queue_delayed_work(ft5x_ts->ts_workqueue, &ft5x_ts->pen_event_work, CTP_CHECK_DELAY);
	#endif
	dprintk(DEBUG_INIT,"reg clk: 0x%08x\n", readl(0xf1c20a18));

	
    	i2c_dev = get_free_i2c_dev(client->adapter);	
	if (IS_ERR(i2c_dev)){	
		err = PTR_ERR(i2c_dev);	
		printk("i2c_dev fail!");	
		return err;	
	}
	
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,client->adapter->nr),
	         NULL, "aw_i2c_ts%d", client->adapter->nr);	
	if (IS_ERR(dev))	{		
			err = PTR_ERR(dev);
			printk("dev fail!\n");		
			return err;	
	}

	dprintk(DEBUG_INIT,"==%s over =\n", __func__);
	return 0;

exit_irq_request_failed:
//        sw_gpio_irq_free(int_handle);
        cancel_work_sync(&ft5x_resume_work);
	destroy_workqueue(ft5x_resume_wq);	
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
        i2c_set_clientdata(client, NULL);
        cancel_delayed_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
exit_create_singlethread:
	kfree(ft5x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
        
	return err;
}

static int __devexit ft5x_ts_remove(struct i2c_client *client)
{

	struct ft5x_ts_data *ft5x_ts = i2c_get_clientdata(client);
	ft5x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	
	printk("==ft5x_ts_remove=\n");
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR,client->adapter->nr));
//	sw_gpio_irq_free(int_handle);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x_ts->early_suspend);
#endif
	cancel_work_sync(&ft5x_resume_work);
	destroy_workqueue(ft5x_resume_wq);
	input_unregister_device(ft5x_ts->input_dev);
	input_free_device(ft5x_ts->input_dev);
	cancel_delayed_work_sync(&ft5x_ts->pen_event_work);
	destroy_workqueue(ft5x_ts->ts_workqueue);
	kfree(ft5x_ts);
    
	i2c_set_clientdata(this_client, NULL);
	//ctp_free_platform_resource();

	return 0;

}

static const struct i2c_device_id ft5x_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ft5x_ts_id);

static struct i2c_driver ft5x_ts_driver = {
	.class          = I2C_CLASS_HWMON,
	.probe		= ft5x_ts_probe,
	.remove		= __devexit_p(ft5x_ts_remove),
	.id_table	= ft5x_ts_id,
	.suspend        = ft5x_ts_suspend,
	.resume         = ft5x_ts_resume,
	.driver	= {
		.name	= CTP_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= normal_i2c,

};

static int aw_open(struct inode *inode, struct file *file)
{
	int subminor;
	int ret = 0;	
	struct i2c_client *client;
	struct i2c_adapter *adapter;	
	struct i2c_dev *i2c_dev;	

	printk("====%s======.\n", __func__);
	dprintk(DEBUG_OTHERS_INFO,"enter aw_open function\n");
	subminor = iminor(inode);
	dprintk(DEBUG_OTHERS_INFO,"subminor=%d\n",subminor);
	
	//lock_kernel();	
	i2c_dev = i2c_dev_get_by_minor(2);	
	if (!i2c_dev)	{	
		printk("error i2c_dev\n");		
		return -ENODEV;	
	}	
	adapter = i2c_get_adapter(i2c_dev->adap->nr);	
	if (!adapter)	{		
		return -ENODEV;	
	}	
	
	client = kzalloc(sizeof(*client), GFP_KERNEL);	
	
	if (!client)	{		
		i2c_put_adapter(adapter);		
		ret = -ENOMEM;	
	}	
	snprintf(client->name, I2C_NAME_SIZE, "pctp_i2c_ts%d", adapter->nr);
	client->driver = &ft5x_ts_driver;
	client->adapter = adapter;		
	file->private_data = client;
		
	return 0;
}

static long aw_ioctl(struct file *file, unsigned int cmd,unsigned long arg ) 
{
	//struct i2c_client *client = (struct i2c_client *) file->private_data;

	dprintk(DEBUG_OTHERS_INFO,"====%s====\n",__func__);
	dprintk(DEBUG_OTHERS_INFO,"line :%d,cmd = %d,arg = %ld.\n",__LINE__,cmd,arg);
	
	switch (cmd) {
	case UPGRADE:
	        dprintk(DEBUG_OTHERS_INFO,"==UPGRADE_WORK=\n");
		//fts_ctpm_fw_upgrade_with_i_file();
		// calibrate();
		break;
	default:
		break;			 
	}	
	return 0;
}

static int aw_release (struct inode *inode, struct file *file) 
{
	struct i2c_client *client = file->private_data;
	dprintk(DEBUG_OTHERS_INFO,"enter aw_release function.\n");		
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;
	return 0;	  
}

static const struct file_operations aw_i2c_ts_fops ={	
	.owner = THIS_MODULE, 		
	.open = aw_open, 	
	.unlocked_ioctl = aw_ioctl,	
	.release = aw_release, 
};
static int ctp_get_system_config(void)
{   
        //ctp_print_info(config_info,DEBUG_INIT);
        twi_id = 1;//config_info.twi_id;
        screen_max_x = 800;//config_info.screen_max_x;
        screen_max_y = 480;//config_info.screen_max_y;

        revert_x_flag = 0;//config_info.revert_x_flag;
        revert_y_flag = 0;//config_info.revert_y_flag;
        exchange_x_y_flag = 0;//config_info.exchange_x_y_flag;
        if((twi_id == 0) || (screen_max_x == 0) || (screen_max_y == 0)){
                printk("%s:read config error!\n",__func__);
                return 0;
        }
        return 1;
}
static int __init ft5x_ts_init(void)
{ 
	int ret = -1;      
	dprintk(DEBUG_INIT,"****************************************************************\n");


        if(!ctp_get_system_config()){
                printk("%s:read config fail!\n",__func__);
                return ret;
        }
        
	//ctp_wakeup(0,20);  
		
	ft5x_ts_driver.detect = ctp_detect;

	ret= register_chrdev(I2C_MAJOR,"aw_i2c_ts",&aw_i2c_ts_fops );	
	if(ret) {	
		printk("%s:register chrdev failed\n",__FILE__);	
		return ret;
	}
	
	i2c_dev_class = class_create(THIS_MODULE,"aw_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {		
		ret = PTR_ERR(i2c_dev_class);		
		class_destroy(i2c_dev_class);	
	}
        ret = i2c_add_driver(&ft5x_ts_driver);
        
        dprintk(DEBUG_INIT,"****************************************************************\n");
	return ret;
}

static void __exit ft5x_ts_exit(void)
{
	printk("==ft5x_ts_exit==\n");
	i2c_del_driver(&ft5x_ts_driver);
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR, "aw_i2c_ts");
}

late_initcall(ft5x_ts_init);
module_exit(ft5x_ts_exit);
MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x TouchScreen driver");
MODULE_LICENSE("GPL");

