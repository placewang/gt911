#ifndef __MYCT_IIC_H
#define __MYCT_IIC_H

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

#define I2C_SCL_GPIO	GPIOE			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_GPIO	GPIOE			/* 连接到SDA数据线的GPIO */

#define I2C_SCL_PIN		GPIO_PIN_4		/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_PIN_5		/* 连接到SDA数据线的GPIO */

#define ALL_I2C_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOE_CLK_ENABLE()

/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  I2C_SCL_GPIO->BSRR = I2C_SCL_PIN				       /* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_GPIO->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)   /* SCL = 0 */

#define I2C_SDA_1()  I2C_SDA_GPIO->BSRR = I2C_SDA_PIN				       /* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_GPIO->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)   /* SDA = 0 */


#define I2C_SDA_READ()  ((I2C_SDA_GPIO->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#define I2C_SCL_READ()  ((I2C_SCL_GPIO->IDR & I2C_SCL_PIN) != 0)	/* 读SCL口线状态 */

//IIC所有操作函数
void delay_us(unsigned int nus);
void CT_IIC_Init(void);                	        //初始化IIC的IO口				 
void i2c_Start(void);				            //发送IIC开始信号
void i2c_Stop(void);	  				        //发送IIC停止信号
void i2c_SendByte(unsigned char txd);			//IIC发送一个字节
unsigned char i2c_ReadByte(void);	            //IIC读取一个字节
unsigned char i2c_WaitAck(void); 				//IIC等待ACK信号
void i2c_Ack(void);					            //IIC发送ACK信号
void i2c_NAck(void);					        //IIC不发送ACK信号
unsigned char i2c_CheckDevice(unsigned char);   //检测I2C总线设备，CPU向发送设备地址
#endif







