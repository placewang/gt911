#include "main.h"
#include "string.h" 
#include "gt911.h"
#include "bsp_iic.h"

//GT911配置参数表
//flash原有版本号,才会更新配置.
const unsigned char GT911_CFG_TBL[]=
{ 
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
}; 

GT911_T g_GT911;

/*
*	函 数 名: GT911_WriteReg
*	功能说明: 写1个或连续的多个寄存器
*	形    参: _usRegAddr : 寄存器地址
*			  _pRegBuf : 寄存器数据缓冲区
*			 _ucLen : 数据长度
*	返 回 值: 无
*/
static void GT911_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;

    i2c_Start();					/* 总线开始信号 */

    i2c_SendByte(g_GT911.i2c_addr);	/* 发送设备地址+写信号 */
	i2c_WaitAck();

    i2c_SendByte(_usRegAddr >> 8);	/* 地址高8位 */
	i2c_WaitAck();

    i2c_SendByte(_usRegAddr);		/* 地址低8位 */
	i2c_WaitAck();

	for (i = 0; i < _ucLen; i++)
	{
	    i2c_SendByte(_pRegBuf[i]);	/* 寄存器数据 */
		i2c_WaitAck();
	}
    i2c_Stop();                   	/* 总线停止信号 */
}
/*
*	函 数 名: GT911_ReadReg
*	功能说明: 读1个或连续的多个寄存器
*	形    参: _usRegAddr : 寄存器地址
*			  _pRegBuf : 寄存器数据缓冲区
*			 _ucLen : 数据长度
*	返 回 值: 无
*/
static void GT911_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;
	
	{
		i2c_Start();					/* 总线开始信号 */

		i2c_SendByte(g_GT911.i2c_addr);	/* 发送设备地址+写信号 */
		i2c_WaitAck();

		i2c_SendByte(_usRegAddr >> 8);	/* 地址高8位 */
		i2c_WaitAck();

		i2c_SendByte(_usRegAddr);		/* 地址低8位 */
		i2c_WaitAck();

		i2c_Start();
		i2c_SendByte(g_GT911.i2c_addr + 0x01);	/* 发送设备地址+读信号 */

		i2c_WaitAck();
	}
	
	for (i = 0; i < 30; i++);

	for (i = 0; i < _ucLen - 1; i++)
	{
	    _pRegBuf[i] = i2c_ReadByte();	/* 读寄存器数据 */
		i2c_Ack();
	}

	/* 最后一个数据 */
	 _pRegBuf[i] = i2c_ReadByte();		/* 读寄存器数据 */
	i2c_NAck();

    i2c_Stop();							/* 总线停止信号 */
}

/*
*	函 数 名: GT911_ReadMaxXY
*	功能说明: 获得GT911触摸板的分辨率，X、Y最大值+1.
*	形    参: *_X : 水平分辨率
*			  *_Y : 垂直分辨率
*	返 回 值: 无
*/
void GT911_ReadMaxXY(uint16_t *_X, uint16_t *_Y)
{
	uint8_t buf[4];
	
	GT911_ReadReg(0x8048, buf, 4);
	
	*_X = buf[0] + buf[1] * 256;
	*_Y = buf[2] + buf[3] * 256;
}

/*
*	函 数 名: GT911_ReadID
*	功能说明: 获得GT911的芯片ID
*	形    参: 无
*	返 回 值: 16位版本
*/
uint32_t GT911_ReadID(void)
{
	uint8_t buf[4]; 

	GT911_ReadReg(GT911_PRODUCT_ID_REG, buf, 4); 

	return ((uint32_t)buf[3] << 24) + ((uint32_t)buf[2] << 16) + ((uint32_t)buf[1] <<8) + buf[0]; 
}

/*
*	函 数 名: GT911_ReadVersion
*	功能说明: 获得GT911的芯片版本
*	形    参: 无
*	返 回 值: 16位版本
*/
uint16_t GT911_ReadVersion(void)
{
	uint8_t buf[2]; 

	GT911_ReadReg(GT911_FIRMWARE_VERSION_REG, buf, 2); 

	return ((uint16_t)buf[1] << 8) + buf[0]; 
}
/*
*	函 数 名: GT911_Timer1ms
*	功能说明: 每隔1ms调用1次
*	形    参: 无
*	返 回 值: 无
*/
void GT911_Timer1ms(void)
{
	g_GT911.TimerCount++;
}
/*
*	函 数 名: bsp_DetectLcdType
*	功能说明: 通过I2C触摸芯片，识别LCD模组类型。结果存放在全局变量 g_LcdType 和 g_TouchType
*	形    参: 无
*	返 回 值: 无
*/
uint8_t g_TouchType;
uint8_t g_LcdType;
void bsp_DetectLcdType(void)
{	
	g_TouchType = 0xFF;
	g_LcdType =   0xFF;		
    /*
		GT911的I2C地址0x28,0x29 两种。
			通过读取触摸IC的芯片ID来识别。
	*/
	if (i2c_CheckDevice(0x28) == 0)
	{
		uint32_t id;
		g_GT911.i2c_addr = 0x28;
		id = GT911_ReadID();			
		if (id == 0x00313139)
		{			
			uint16_t MaxX, MaxY;
			
			GT911_ReadMaxXY(&MaxX, &MaxY);
			if (MaxX == 800 && MaxY == 480)
			{
				g_TouchType = CT_GT911;
				g_LcdType = LCD_70_800X480;		
				printf("检测到7寸电容屏GT911(0x28) 800*480\n");
			 }				
		 }
	 }
	 else if (i2c_CheckDevice(0xBA) == 0)
	 {
	  	uint32_t id;
	
		delay_us(500);
		g_GT911.i2c_addr = 0xBA;
		id = GT911_ReadID();			
		if (id == 0x00313139)
		{			
			uint16_t MaxX, MaxY;
				
			GT911_ReadMaxXY(&MaxX, &MaxY);
			if (MaxX == 800 && MaxY == 480)
			{
				g_GT911.i2c_addr = 0xBA;
				g_TouchType = CT_GT911;
				g_LcdType = LCD_70_800X480;
				printf("检测到7.0寸电容屏GT911(0xBA)800x480\n");
			}
		 }
	  }						
	  delay_us(10);
}

/*
*	函 数 名: TOUCH_IntWakeUpForGT
*	功能说明: GT928触摸，GT911触摸。INT唤醒函数。复位后INT给个高电平脉冲，可以唤醒处于休眠的芯片
*	形    参: 无
*	返 回 值: 无
*/
void TOUCH_IntWakeUpForGT(void)
{
	/* 配置TP_INT触摸中断引脚 */
	GPIO_InitTypeDef gpio_init;

	/* 第1步：打开GPIO时钟 */
	TP_INT_GPIO_CLK_ENABLE();
	
	/* 第2步：配置所有的按键GPIO为浮动输入模式 */
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;   	  /* 设置输出 */
	gpio_init.Pull = GPIO_NOPULL;                 /* 上下拉电阻不使能 */
	
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* GPIO速度等级 */
	gpio_init.Pin = TP_INT_PIN;
	HAL_GPIO_Init(TP_INT_GPIO_PORT, &gpio_init);	

	TP_INT_GPIO_PORT->BSRR = ((uint32_t)TP_INT_PIN << 16U);	/* INT = 0 */
	delay_us(200);
	
	TP_INT_GPIO_PORT->BSRR = TP_INT_PIN;	/* INT = 1 */
	delay_us(2000);
	TP_INT_GPIO_PORT->BSRR = ((uint32_t)TP_INT_PIN << 16U);	/* INT = 0 */
	delay_us(200);
	
	gpio_init.Mode = GPIO_MODE_INPUT;   		/* 设置输入 */
	gpio_init.Pull = GPIO_NOPULL;                /* 上下拉电阻不使能 */
	HAL_GPIO_Init(TP_INT_GPIO_PORT, &gpio_init);	
	
}
/*
初始化GT9271触摸屏
返回值:0,初始化成功;1,初始化失败 
*/
unsigned char GT911_Init(void)
{   
	__HAL_RCC_GPIOD_CLK_ENABLE();			//开启GPIOD时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();			//开启GPIOE时钟   

	GPIO_InitTypeDef gpio_init;
		
    /*配置所有的按键GPIO为浮动输入模式 */
    gpio_init.Mode = GPIO_MODE_INPUT;   		/* 设置输入 */
    gpio_init.Pull = GPIO_NOPULL;               /* 上下拉电阻不使能 */
    
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* GPIO速度等级 */
    gpio_init.Pin = TP_INT_PIN;
    HAL_GPIO_Init(TP_INT_GPIO_PORT, &gpio_init);	    
		
     bsp_DetectLcdType();	/* 自动识别触摸芯片型号 */
    #if 0	/* 调试代码 */
        printf("GT911 ID :%08X, Version :%04X \n", GT911_ReadID(), GT911_ReadVersion());
    #endif    
	g_GT911.TimerCount = 0;
	g_GT911.TimerCount = 0;
	g_GT911.Enable = 1;

	return 1;  
}


