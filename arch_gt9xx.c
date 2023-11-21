/*
* 电容触控屏驱动 GT911
*/
#include <stdio.h>
#include "string.h"

#include "hw_control_AM335x.h"
#include "soc_AM335x.h"

#include "cp15.h"
#include "cpu.h"
#include "hw_cm_per.h"
#include "hw_types.h"

#include "hw_gpmc.h"
#include "hw_gpio_v2.h"

#include "arch_sys.h"
#include "arch_gpio.h"
#include "arch_i2c.h"
#include "arch_gt9xx.h"

#define GTP_DRIVER_SEND_CFG   0       // driver send config to TP in intilization

#define CTP_CFG_GROUP1 {\
    0x41,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x8C,\
    0x1E,0x0C,0x50,0x3C,0x03,0x07,0x01,0x01,0x00,0x00,\
    0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x8B,0x2B,0x0C,\
    0x50,0x52,0xD6,0x09,0x00,0x00,0x00,0x9C,0x02,0x1D,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x4A,0x64,0x9E,0xE5,0x01,0x14,0x00,0x00,0x04,\
    0x74,0x4C,0x00,0x70,0x50,0x00,0x69,0x55,0x00,0x63,\
    0x5B,0x00,0x5E,0x61,0x00,0x5E,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x01,0x1B,0x14,0x0D,0x14,0x00,0x00,0x01,0x00,\
    0x61,0x03,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
    0x85,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,\
    0x12,0x14,0x16,0x18,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
    0xFF,0xFF,0x00,0x01,0x02,0x04,0x06,0x07,0x08,0x09,\
    0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x25,\
    0x26,0x28,0x29,0x2A,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
    0xFF,0xFF,0xFF,0xFF,0x7A,0x01,\
    }

#define GTP_MAX_HEIGHT   480
#define GTP_MAX_WIDTH    800
#define GTP_INT_TRIGGER  0  //0:Rising 1:Falling

#define GTP_MAX_TOUCH      1

#define GTP_POLL_TIME               10
#define GTP_ADDR_LENGTH             2
#define GTP_CONFIG_MIN_LENGTH       186
#define GTP_CONFIG_MAX_LENGTH       240
#define FAIL                        0
#define SUCCESS                     1
#define SWITCH_OFF                  0
#define SWITCH_ON                   1

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

//Register define
#define GTP_READ_COOR_ADDR          0x814E
#define GTP_REG_SLEEP               0x8040
#define GTP_REG_SENSOR_ID           0x814A
#define GTP_REG_CONFIG_DATA         0x8047
#define GTP_REG_VERSION             0x8140
//#define GTP_REG_HW_INFO             0x4220

#define RESOLUTION_LOC              3
#define TRIGGER_LOC                 8

#define TPD_WARP_X
#define TPD_WARP_Y

#ifdef TPD_WARP_X
#undef TPD_WARP_X
#define TPD_WARP_X(x_max, x) ( x_max - 1 - x )
#else
#define TPD_WARP_X(x_max, x) x
#endif

#ifdef TPD_WARP_Y
#undef TPD_WARP_Y
#define TPD_WARP_Y(y_max, y) ( y_max - 1 - y )
#else
#define TPD_WARP_Y(y_max, y) y
#endif

/*************************************************************/

//#define GT9XX_DEBUG

#ifdef GT9XX_DEBUG
#define GTP_DEBUG(...) printf(__VA_ARGS__)
#else
#define GTP_DEBUG(...)
#endif

#define GTP_INFO(...) printf(__VA_ARGS__)
#define GTP_ERROR(...) printf(__VA_ARGS__)


#define GTP_GPIO_INT       GPIO(1, 28)
#define GTP_GRIO_RST       GPIO(3, 20)

/**************************************************************/

#define TS_QUEUE_SIZE   30

typedef struct {
    volatile int idx_in;
    volatile int idx_out;
    volatile int err;
    TouchEvent buff[TS_QUEUE_SIZE];
} TsEventQueue;

static TsEventQueue gtp_ts_queue = {0};

static int gtp_ts_queue_push(TouchEvent * ev) 
{
	int idx_in = gtp_ts_queue.idx_in;
    int idx_out = gtp_ts_queue.idx_out;
	int count = idx_in - idx_out;
	if (count < 0) {
		count += TS_QUEUE_SIZE;
	}
	if (count >= (TS_QUEUE_SIZE - 1)) {
		gtp_ts_queue.err += 1;  //缓存满导致数据丢失
		return -1;
	}
    if (ev == NULL) return count;
	
	int index = idx_in;
    memcpy(&gtp_ts_queue.buff[index], ev, sizeof(TouchEvent));
	index++;
	if (index >= TS_QUEUE_SIZE) {  //环状缓冲
		index = 0;
	}
	gtp_ts_queue.idx_in = index;
	return (count + 1);
}

static int gtp_ts_queue_pop(TouchEvent * ev) 
{
    int idx_in = gtp_ts_queue.idx_in;
    int idx_out = gtp_ts_queue.idx_out;
	int count = idx_in - idx_out;
	if (count < 0) {
		count += TS_QUEUE_SIZE;
	}
	if (count <= 0) {  //缓冲区空
		return 0;  
	}
    if (ev == NULL) return count;
	
	int index = idx_out;
    memcpy(ev, &gtp_ts_queue.buff[index], sizeof(TouchEvent));
    index++;
	if (index >= TS_QUEUE_SIZE) {
		index = 0;
	}
    gtp_ts_queue.idx_out = index;
    return 1;
}

static void gtp_ts_queue_clear()
{
    gtp_ts_queue.idx_in = 0;
    gtp_ts_queue.idx_out = 0;
}

/***************************************************/

static i2c_slave gt911 =
{
  0,      //bus number
  0x5D,   //i2c address
  400000
};

static unsigned char int_type = 0;
static int abs_x_max = 0;
static int abs_y_max = 0;

static unsigned char gtp_txrx_buff[256] = {0};

static int gtp_i2c_read(unsigned short reg, unsigned char * buf, int len)
{
    unsigned char data[2] = {0};
    data[1] = reg & 0xFF;
    data[0] = (reg >> 8) & 0xFF;
    return arch_i2c_txrx(&gt911, data, 2, buf, len);
}

static int gtp_i2c_write(unsigned short reg, unsigned char * buf, int len)
{
    unsigned char * data = gtp_txrx_buff;
    memset(gtp_txrx_buff, 0, 256);
    data[1] = reg & 0xFF;
    data[0] = (reg >> 8) & 0xFF;
    if (len > 252) len = 252;
    memcpy(&data[2], buf, len);
    return arch_i2c_txrx(&gt911, data, len + 2, NULL, 0);
}

//static unsigned char config[GTP_CONFIG_MAX_LENGTH] = {0};
//static unsigned char cfg_info_group1[] = CTP_CFG_GROUP1;

#if GTP_DRIVER_SEND_CFG

int gtp_send_cfg(unsigned short reg)
{
    int ret = 1;
    int retry = 0;
    GTP_INFO("Driver Send Config\n");
    for (retry = 0; retry < 5; retry++) {
        ret = gtp_i2c_write(reg, config, GTP_CONFIG_MAX_LENGTH);
        if (ret > 0) {
            break;
        }
    }
    return ret;
}

#endif

static void gtp_event_isr()
{
    int ret;
    unsigned char end_cmd[3] = {0};
    unsigned char point_data[1 + (8 * GTP_MAX_TOUCH) + 1] = {0};
    unsigned char touch_num = 0;
    unsigned char finger = 0;
    int i = 0;
    unsigned char *coor_data = NULL;
    int input_x = 0;
    int input_y = 0;
    int input_w = 0;
    int id = 0;

    static unsigned char pre_touch = 0;
    static TouchEvent touch_ev = {0};

    ret = gtp_i2c_read(GTP_READ_COOR_ADDR, point_data, 10);
    if (ret < 0) {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        return;
    }
    finger = point_data[0];
    if (finger == 0x00) return;
    if ((finger & 0x80) == 0) {
        goto exit_work_func;
    }
    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH) {
        goto exit_work_func;
    }
    if (touch_num > 1) {
        unsigned char buf[8 * GTP_MAX_TOUCH] = {0};
        ret = gtp_i2c_read(GTP_READ_COOR_ADDR, buf, 8 * (touch_num - 1));
        memcpy(&point_data[10], &buf[0], 8 * (touch_num - 1));
    }

    //GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);
    if (touch_num) {
        for (i = 0; i < touch_num; i++) {
            coor_data = &point_data[i * 8 + 1];

            id = coor_data[0] & 0x0F;      
            input_x  = coor_data[1] | coor_data[2] << 8;
            input_y  = coor_data[3] | coor_data[4] << 8;
            input_w  = coor_data[5] | coor_data[6] << 8;

            if (input_x >= 0 && input_x <= abs_x_max &&
                input_y >= 0 && input_y <= abs_y_max)
            {
                input_x = TPD_WARP_X(abs_x_max, input_x);
                input_y = TPD_WARP_Y(abs_y_max, input_y);

                GTP_DEBUG("DN (%d)(%d, %d)[%d] \n", id, input_x, input_y, input_w);
                touch_ev.id = id;
                touch_ev.x = input_x;
                touch_ev.y = input_y;
                touch_ev.w = input_w;
                touch_ev.touch = 1;
                gtp_ts_queue_push(&touch_ev);
            }
        }
    }
    else {
        if (pre_touch) {
            touch_ev.id = 0;
            //touch_ev.x = input_x;
            //touch_ev.y = input_y;
            touch_ev.w = 0;
            touch_ev.touch = 0;
            GTP_DEBUG("UP (%d)(%d, %d) \n", id, touch_ev.x, touch_ev.y);
            gtp_ts_queue_push(&touch_ev);
        }
    }
    pre_touch = touch_num;

exit_work_func:
    ret = gtp_i2c_write(GTP_READ_COOR_ADDR, end_cmd, 13);
    if (ret < 0) {
        GTP_INFO("I2C write end_cmd  error!\n");
    }
}

static void gtp_reset_guitar()
{
    GTP_DEBUG("GTP RESET!\n");
    GpioSetConfig(GTP_GRIO_RST, PINMUX_IEN, PINMUX_PUP, PINMUX_MODE7);
    GpioSetOutput(GTP_GRIO_RST);
    GpioSetConfig(GTP_GPIO_INT, PINMUX_IEN, PINMUX_PUP, PINMUX_MODE7);
    GpioSetOutput(GTP_GPIO_INT);
    arch_sys_delay(5);

    GpioClr(GTP_GRIO_RST);
    arch_sys_delay(20);
    GpioClr(GTP_GPIO_INT);
    arch_sys_delay(3);
    GpioSet(GTP_GRIO_RST);
    arch_sys_delay(10);
}

static int gtp_i2c_test()
{
    unsigned char test[2] = {0};
    int retry = 0;
    int ret = -1;
    
    GTP_DEBUG("gtp_i2c_test\n");
    while(retry++ < 5) {
        ret = gtp_i2c_read(GTP_REG_CONFIG_DATA, test, 1);
        if (ret >= 0) {
            GTP_DEBUG("gtp_i2c_test(0x8047): 0x%02X\n", test[0]);
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.\n",retry);
        arch_sys_delay(10);
    }
    return ret;
}

static int gtp_read_version(unsigned short * version)
{
    int ret = -1;
    unsigned char buf[8] = {0};

    GTP_DEBUG("gtp_read_version\n");
    ret = gtp_i2c_read(GTP_REG_VERSION, buf, 6);
    if (ret < 0) {
        GTP_ERROR("GTP read version failed\n");
        return ret;
    }

    if (version) {
        *version = (buf[5] << 8) | buf[4];
    }

    if (buf[3] == 0x00) {
        GTP_INFO("IC Version: %c%c%c_%02x%02x \n", buf[0], buf[1], buf[2], buf[5], buf[4]);
    }
    else {
        GTP_INFO("IC Version: %c%c%c%c_%02x%02x \n", buf[0], buf[1], buf[2], buf[3], buf[5], buf[4]);
    }
    return ret;
}

static int gtp_i2c_read_dbl_check(unsigned short reg, unsigned char * rxbuf, int len)
{
    unsigned char buf[16] = {0};
    unsigned char confirm_buf[16] = {0};
    unsigned char retry = 0;
    
    while (retry++ < 3) {
        memset(buf, 0xAA, 16);
        gtp_i2c_read(reg, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        gtp_i2c_read(reg, confirm_buf, len + 2);
        
        if (!memcmp(buf, confirm_buf, len+2)) {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!\n", reg, len);
    return FAIL;
}

static int gtp_init_panel()
{
    /*int ret = 0;
    int cfg_len = CFG_GROUP_LEN(cfg_info_group1);
    unsigned char sensor_id = 0;

    cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(GTP_REG_CONFIG_DATA, config, cfg_len);
    if (ret < 0) {
        GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!\n");
        abs_x_max = GTP_MAX_WIDTH;
        abs_y_max = GTP_MAX_HEIGHT;
        int_type = GTP_INT_TRIGGER;
    }
    GTP_INFO("gtp_init_panel\n");
    if ((abs_x_max == 0) && (abs_y_max == 0))
    {
        abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        int_type = (config[TRIGGER_LOC]) & 0x03; 
    }
    GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x\n", abs_x_max, abs_y_max, int_type);
    arch_sys_delay(10);
*/
    GTP_DEBUG("gtp_init_panel\n");
    abs_x_max = GTP_MAX_WIDTH;
    abs_y_max = GTP_MAX_HEIGHT;
    return 0;
}

/**********************************************************/

int arch_gt911_init()
{
    int ret;
    unsigned short version_info = 0;
    
    arch_i2c_init(&gt911);
    gtp_reset_guitar();
    
    ret = gtp_i2c_test();
    if (ret < 0) {
        GTP_ERROR("I2C communication ERROR!\n");
        return ret;
    }

    ret = gtp_read_version(&version_info);
    if (ret < 0) {
        GTP_ERROR("Read version failed.\n");
        return ret;
    }
    gtp_init_panel();
    gtp_ts_queue_clear();

    GpioSetConfig(GTP_GPIO_INT, PINMUX_IEN, PINMUX_POFF, PINMUX_MODE7);
	GpioSetInput(GTP_GPIO_INT);
    GpioEnableInt(GTP_GPIO_INT, GPIO_INT_TYPE_RISING, gtp_event_isr);
    return 0;
}

int arch_gt9xx_get(TouchEvent * ev)
{
    return gtp_ts_queue_pop(ev);
}
