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
/*
*	函 数 名: TOUCH_PenInt
*	功能说明: 判断触摸按下
*	形    参: 无
*	返 回 值: 0表示无触笔按下，1表示有触笔按下
*/
uint8_t TOUCH_PenInt(void)
{
	if ((TP_INT_GPIO_PORT->IDR & TP_INT_PIN) == 0)
	{
		return 1;
	}
	return 0;
}
/*
*	函 数 名: GT911_Scan
*	功能说明: 读取GT911触摸数据。读取全部的数据，需要 720us左右
*	形    参: 无
*	返 回 值: 无
*/
unsigned char GT911_Scan(void)
{
	uint8_t buf[48];
	uint8_t clear_flag = 0;
	g_GT911.TimerCount = 0;

	if (TOUCH_PenInt() == 1)
	{		
        //	/* 不用INT引脚，读状态寄存器 */
        //	GT911_ReadReg(GT911_READ_XY_REG, buf, 1);
        //	if (buf[0] == 0)
        //	{

        #if 1		/* 只读1点 */
            GT911_ReadReg(GT911_READ_XY_REG, buf, 8);
        #else		/* 读5个触摸点 */
            GT911_ReadReg(GT911_READ_XY_REG, buf, 40);
        #endif
            
            GT911_WriteReg(GT911_READ_XY_REG, &clear_flag,	1);		/* 读完坐标后必须写0清除 */
	
	/*
		0x814E R/W Bufferstatus Large_Detect number of touch points 
			Bit7: Buffer status，1表示坐标（或按键）已经准备好，主控可以读取；0表示未就绪，数据无效。当主控读取完坐标后，必须通过I2C将此标志（或整个字节）写为0。
			Bit4: HaveKey, 1表示有按键，0表示无按键（已经松键）。
			Bit3~0: Number of touch points, 屏上的坐标点个数
	
		0x814F R Point1 track id                    0x8157 R Point2 track id 
		0x8150 R Point1Xl 触摸点 1，X 坐标低 8 位   0x8158 R Point2Xl 触摸点 2，X 坐标低 8 位 
		0x8151 R Point1Xh 触摸点 1，X 坐标高 8 位   0x8159 R Point2Xh 触摸点 2，X 坐标高 8 位 
		0x8152 R Point1Yl 触摸点 1，Y 坐标低 8 位   0x815A R Point2Yl 触摸点 2，Y 坐标低 8 位 
		0x8153 R Point1Yh 触摸点 1，Y 坐标高 8 位   0x815B R Point2Yh 触摸点 2，Y 坐标高 8 位 
		0x8154 R Point1 触摸点 1，触摸面积低 8 位   0x815C R Point2 触摸点 2，触摸面积低 8 位 
		0x8155 R Point1 触摸点 1，触摸面积高 8 位   0x815D R Point2 触摸点 2，触摸面积高 8 位 
		0x8156 ----                                 0x815E ---- 

		0x815F R Point3 track id         		  0x8167 R Point4 track id          
		0x8160 R Point3Xl 触摸点 3，X 坐标低 8 位 0x8168 R Point4Xl 触摸点 4，X 坐标低 8 位 
		0x8161 R Point3Xh 触摸点 3，X 坐标高 8 位 0x8169 R Point4Xh 触摸点 4，X 坐标高 8 位 
		0x8162 R Point3Yl 触摸点 3，Y 坐标低 8 位 0x816A R Point4Yl 触摸点 4，Y 坐标低 8 位 
		0x8163 R Point3Yh 触摸点 3，Y 坐标高 8 位 0x816B R Point4Yh 触摸点 4，Y 坐标高 8 位
		0x8164 R Point3 触摸点 3，触摸面积低 8 位 0x816C R Point4 触摸点 4，触摸面积低 8 位 
		0x8165 R Point3 触摸点 3，触摸面积高 8 位 0x816D R Point4 触摸点 4，触摸面积高 8 位 
		0x8166 ----                               0x816E ----

		0x816F R Point5 track id 
		0x8170 R Point5Xl 触摸点 5，X 坐标低 8 位 
		0x8171 R Point5Xh 触摸点 5，X 坐标高 8 位 
		0x8172 R Point5Yl 触摸点 5，Y 坐标低 8 位 
		0x8173 R Point5Yh 触摸点 5，Y 坐标高 8 位 
		0x8174 R Point5 触摸点 5，触摸面积低 8 位 
		0x8175 R Point5 触摸点 5，触摸面积高 8 位 
		0x8176 --
		
	*/
	g_GT911.TouchpointFlag = buf[0];
	g_GT911.Touchkeystate = buf[1];

	g_GT911.X0 = ((uint16_t)buf[3] << 8) + buf[2];
	g_GT911.Y0 = ((uint16_t)buf[5] << 8) + buf[4];
	g_GT911.P0 = ((uint16_t)buf[7] << 8) + buf[6];

	#if 0	/* 其余4点一般不用 */
		g_GT911.X1 = ((uint16_t)buf[9] << 8) + buf[10];
		g_GT911.Y1 = ((uint16_t)buf[11] << 8) + buf[12];
		g_GT911.P1 = ((uint16_t)buf[13] << 8) + buf[14];

		g_GT911.X2 = ((uint16_t)buf[17] << 8) + buf[16];
		g_GT911.Y2 = ((uint16_t)buf[19] << 8) + buf[18];
		g_GT911.P2 = ((uint16_t)buf[21] << 8) + buf[20];

		g_GT911.X3 = ((uint16_t)buf[24] << 8) + buf[23];
		g_GT911.Y3 = ((uint16_t)buf[26] << 8) + buf[25];
		g_GT911.P3 = ((uint16_t)buf[28] << 8) + buf[27];

		g_GT911.X4 = ((uint16_t)buf[31] << 8) + buf[30];
		g_GT911.Y4 = ((uint16_t)buf[33] << 8) + buf[32];
		g_GT911.P4 = ((uint16_t)buf[35] << 8) + buf[34];
	#endif

		/* 坐标转换 :
			电容触摸板左下角是 (0，0);  右上角是 (479，799)
			需要转到LCD的像素坐标 (左上角是 (0，0), 右下角是 (799，479)
		*/
			
	if (g_GT911.X0 > 799)
		g_GT911.X0 = 799;
			
	if (g_GT911.Y0 > 479)
		g_GT911.Y0 = 479;		
 
//            for ( uint8_t i = 0; i < 34; i++)
//            {
//                printf("%02X ", buf[i]);
//            }
//            printf("\r\n");
            printf("(%5d,%5d,%3d) ",  g_GT911.X0, g_GT911.Y0, g_GT911.P0);
//            printf("(%5d,%5d,%3d) ",  g_GT911.X1, g_GT911.Y1, g_GT911.P1);
//            printf("(%5d,%5d,%3d) ",  g_GT911.X2, g_GT911.Y2, g_GT911.P2);
//            printf("(%5d,%5d,%3d) ",  g_GT911.X3, g_GT911.Y3, g_GT911.P3);
//            printf("(%5d,%5d,%3d) ",  x, y, g_GT911.P4);

   }
}

