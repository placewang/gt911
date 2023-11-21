#include "main.h"
#include "bsp_iic.h"

						    
//延时nus
//nus为要延时的us数.	
//注意:nus的值不要大于1000us
void delay_us(unsigned int nus)
{		
	unsigned int ticks;
	unsigned int told,tnow,tcnt=0;
	unsigned int reload=SysTick->LOAD;	    //LOAD的值	    	 
	ticks=nus*(SystemCoreClock / 1000000);  //需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	}
}
//控制I2C速度的延时
void i2c_Delay(void)
{
    delay_us(4);
}


/*
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参:  无
*	返 回 值: 无
*/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	
	I2C_SCL_0();
	i2c_Delay();
}

/*
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参:  无
*	返 回 值: 无
*/
void i2c_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/*
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参:  _ucByte ： 等待发送的字节
*	返 回 值: 无
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */	
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参:  无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/*
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参:  无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/*
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参:  无
*	返 回 值: 无
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	
	i2c_Delay();
	I2C_SCL_1();	
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	
	
	i2c_Delay();
}

/*
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参:  无
*	返 回 值: 无
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	
	i2c_Delay();
	I2C_SCL_1();	
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参:  _Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ()&&I2C_SCL_READ())
	{
		i2c_Start();		/* 发送启动信号 */

		/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
		i2c_SendByte(_Address | I2C_WR);
		ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

		i2c_Stop();			    /* 发送停止信号 */

		return ucAck;
	}
	return 1;	                /* I2C总线异常 */
}

//电容触摸芯片IIC接口初始化
void CT_IIC_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	/* 打开GPIO时钟 */
	ALL_I2C_GPIO_CLK_ENABLE();
	
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;	// 设置开漏输出 */
	gpio_init.Pull = GPIO_NOPULL;			// 上下拉电阻不使能 
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;	// GPIO_SPEED_FREQ_HIGH
	
	gpio_init.Pin = I2C_SCL_PIN;	
	HAL_GPIO_Init(I2C_SCL_GPIO, &gpio_init);	
	
	gpio_init.Pin = I2C_SDA_PIN;	
	HAL_GPIO_Init(I2C_SDA_GPIO, &gpio_init);
//	delay_us(500);
    i2c_Stop();  
}

