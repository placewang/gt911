#ifndef __MYCT_IIC_H
#define __MYCT_IIC_H

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

#define I2C_SCL_GPIO	GPIOE			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_GPIO	GPIOE			/* ���ӵ�SDA�����ߵ�GPIO */

#define I2C_SCL_PIN		GPIO_PIN_4		/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		GPIO_PIN_5		/* ���ӵ�SDA�����ߵ�GPIO */

#define ALL_I2C_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOE_CLK_ENABLE()

/* �����дSCL��SDA�ĺ� */
#define I2C_SCL_1()  I2C_SCL_GPIO->BSRR = I2C_SCL_PIN				       /* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_GPIO->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)   /* SCL = 0 */

#define I2C_SDA_1()  I2C_SDA_GPIO->BSRR = I2C_SDA_PIN				       /* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_GPIO->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)   /* SDA = 0 */


#define I2C_SDA_READ()  ((I2C_SDA_GPIO->IDR & I2C_SDA_PIN) != 0)	/* ��SDA����״̬ */
#define I2C_SCL_READ()  ((I2C_SCL_GPIO->IDR & I2C_SCL_PIN) != 0)	/* ��SCL����״̬ */

//IIC���в�������
void delay_us(unsigned int nus);
void CT_IIC_Init(void);                	        //��ʼ��IIC��IO��				 
void i2c_Start(void);				            //����IIC��ʼ�ź�
void i2c_Stop(void);	  				        //����IICֹͣ�ź�
void i2c_SendByte(unsigned char txd);			//IIC����һ���ֽ�
unsigned char i2c_ReadByte(void);	            //IIC��ȡһ���ֽ�
unsigned char i2c_WaitAck(void); 				//IIC�ȴ�ACK�ź�
void i2c_Ack(void);					            //IIC����ACK�ź�
void i2c_NAck(void);					        //IIC������ACK�ź�
unsigned char i2c_CheckDevice(unsigned char);   //���I2C�����豸��CPU�����豸��ַ
#endif







