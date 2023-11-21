#ifndef __GT9147_H
#define __GT9147_H	

/* LCD面板类型 */
enum
{
	LCD_35_480X320 = 0,	/* 3.5寸 480 * 320 */	
	LCD_43_480X272,		/* 4.3寸 480 * 272 */
	LCD_50_480X272,		/* 5.0寸 480 * 272 */
	LCD_50_800X480,		/* 5.0寸 480 * 272 */
	LCD_70_800X480,		/* 7.0寸 800 * 480 */	
	LCD_70_1024X600,	/* 7.0寸 1024 * 600 */		
};
/* 触摸板类型 */
enum
{
	CT_FT5X06 = 0,		/* FT系列电容触摸IC */
	CT_GT811,			/* 7寸电容触摸GT811 800 * 480 */
	CT_GT911,			/* 7寸电容触摸GT911 800 * 480 */
	
	CT_STMPE811			/* 电阻触摸 */
};
typedef struct
{
	unsigned char Enable;
	unsigned char TimerCount;
	unsigned char i2c_addr;
	
	unsigned char TouchpointFlag;
	unsigned char Touchkeystate;

	unsigned short X0;
	unsigned short Y0;
	unsigned short P0;

	unsigned short X1;
	unsigned short Y1;
	unsigned short P1;

	unsigned short X2;
	unsigned short Y2;
	unsigned short P2;

	unsigned short X3;
	unsigned short Y3;
	unsigned short P3;

	unsigned short X4;
	unsigned short Y4;
	unsigned short P4;
}GT911_T;

/* 定义触笔中断INT的GPIO端口 */
#define TP_INT_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOD_CLK_ENABLE()
#define TP_INT_GPIO_PORT            GPIOD
#define TP_INT_PIN  	            GPIO_PIN_11
 
//I2C读写命令	
#define GT911_CMD_WR 		0X28     	//写命令
#define GT911_CMD_RD 		0X29		//读命令
  
//GT911 部分寄存器定义 

#define GT911_READ_XY_REG          0x814E /* 坐标寄存器 */ 
#define GT911_CLEARBUF_REG         0x814E /* 清除坐标寄存器 */ 
#define GT911_CONFIG_REG           0x8047 /* 配置参数寄存器 */ 
#define GT911_COMMAND_REG          0x8040 /* 实时命令 */ 
#define GT911_PRODUCT_ID_REG       0x8140 /* 芯片ID */ 
#define GT911_VENDOR_ID_REG        0x814A /* 当前模组选项信息 */ 
#define GT911_CONFIG_VERSION_REG   0x8047 /* 配置文件版本号 */ 
#define GT911_CONFIG_CHECKSUM_REG  0x80FF /* 配置文件校验码 */ 
#define GT911_FIRMWARE_VERSION_REG 0x8144 /* 固件版本号 */ 

  
unsigned char GT911_Init(void);
unsigned char GT911_Scan(unsigned char mode); 

#endif



