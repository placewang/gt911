#ifndef __GT9147_H
#define __GT9147_H	

/* LCD������� */
enum
{
	LCD_35_480X320 = 0,	/* 3.5�� 480 * 320 */	
	LCD_43_480X272,		/* 4.3�� 480 * 272 */
	LCD_50_480X272,		/* 5.0�� 480 * 272 */
	LCD_50_800X480,		/* 5.0�� 480 * 272 */
	LCD_70_800X480,		/* 7.0�� 800 * 480 */	
	LCD_70_1024X600,	/* 7.0�� 1024 * 600 */		
};
/* ���������� */
enum
{
	CT_FT5X06 = 0,		/* FTϵ�е��ݴ���IC */
	CT_GT811,			/* 7����ݴ���GT811 800 * 480 */
	CT_GT911,			/* 7����ݴ���GT911 800 * 480 */
	
	CT_STMPE811			/* ���败�� */
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

/* ���崥���ж�INT��GPIO�˿� */
#define TP_INT_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOD_CLK_ENABLE()
#define TP_INT_GPIO_PORT            GPIOD
#define TP_INT_PIN  	            GPIO_PIN_11
 
//I2C��д����	
#define GT911_CMD_WR 		0X28     	//д����
#define GT911_CMD_RD 		0X29		//������
  
//GT911 ���ּĴ������� 

#define GT911_READ_XY_REG          0x814E /* ����Ĵ��� */ 
#define GT911_CLEARBUF_REG         0x814E /* �������Ĵ��� */ 
#define GT911_CONFIG_REG           0x8047 /* ���ò����Ĵ��� */ 
#define GT911_COMMAND_REG          0x8040 /* ʵʱ���� */ 
#define GT911_PRODUCT_ID_REG       0x8140 /* оƬID */ 
#define GT911_VENDOR_ID_REG        0x814A /* ��ǰģ��ѡ����Ϣ */ 
#define GT911_CONFIG_VERSION_REG   0x8047 /* �����ļ��汾�� */ 
#define GT911_CONFIG_CHECKSUM_REG  0x80FF /* �����ļ�У���� */ 
#define GT911_FIRMWARE_VERSION_REG 0x8144 /* �̼��汾�� */ 

  
unsigned char GT911_Init(void);
unsigned char GT911_Scan(unsigned char mode); 

#endif



