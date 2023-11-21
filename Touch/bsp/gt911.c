#include "main.h"
#include "string.h" 
#include "gt911.h"
#include "bsp_iic.h"

//GT911���ò�����
//flashԭ�а汾��,�Ż��������.
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
*	�� �� ��: GT911_WriteReg
*	����˵��: д1���������Ķ���Ĵ���
*	��    ��: _usRegAddr : �Ĵ�����ַ
*			  _pRegBuf : �Ĵ������ݻ�����
*			 _ucLen : ���ݳ���
*	�� �� ֵ: ��
*/
static void GT911_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;

    i2c_Start();					/* ���߿�ʼ�ź� */

    i2c_SendByte(g_GT911.i2c_addr);	/* �����豸��ַ+д�ź� */
	i2c_WaitAck();

    i2c_SendByte(_usRegAddr >> 8);	/* ��ַ��8λ */
	i2c_WaitAck();

    i2c_SendByte(_usRegAddr);		/* ��ַ��8λ */
	i2c_WaitAck();

	for (i = 0; i < _ucLen; i++)
	{
	    i2c_SendByte(_pRegBuf[i]);	/* �Ĵ������� */
		i2c_WaitAck();
	}
    i2c_Stop();                   	/* ����ֹͣ�ź� */
}
/*
*	�� �� ��: GT911_ReadReg
*	����˵��: ��1���������Ķ���Ĵ���
*	��    ��: _usRegAddr : �Ĵ�����ַ
*			  _pRegBuf : �Ĵ������ݻ�����
*			 _ucLen : ���ݳ���
*	�� �� ֵ: ��
*/
static void GT911_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;
	
	{
		i2c_Start();					/* ���߿�ʼ�ź� */

		i2c_SendByte(g_GT911.i2c_addr);	/* �����豸��ַ+д�ź� */
		i2c_WaitAck();

		i2c_SendByte(_usRegAddr >> 8);	/* ��ַ��8λ */
		i2c_WaitAck();

		i2c_SendByte(_usRegAddr);		/* ��ַ��8λ */
		i2c_WaitAck();

		i2c_Start();
		i2c_SendByte(g_GT911.i2c_addr + 0x01);	/* �����豸��ַ+���ź� */

		i2c_WaitAck();
	}
	
	for (i = 0; i < 30; i++);

	for (i = 0; i < _ucLen - 1; i++)
	{
	    _pRegBuf[i] = i2c_ReadByte();	/* ���Ĵ������� */
		i2c_Ack();
	}

	/* ���һ������ */
	 _pRegBuf[i] = i2c_ReadByte();		/* ���Ĵ������� */
	i2c_NAck();

    i2c_Stop();							/* ����ֹͣ�ź� */
}

/*
*	�� �� ��: GT911_ReadMaxXY
*	����˵��: ���GT911������ķֱ��ʣ�X��Y���ֵ+1.
*	��    ��: *_X : ˮƽ�ֱ���
*			  *_Y : ��ֱ�ֱ���
*	�� �� ֵ: ��
*/
void GT911_ReadMaxXY(uint16_t *_X, uint16_t *_Y)
{
	uint8_t buf[4];
	
	GT911_ReadReg(0x8048, buf, 4);
	
	*_X = buf[0] + buf[1] * 256;
	*_Y = buf[2] + buf[3] * 256;
}

/*
*	�� �� ��: GT911_ReadID
*	����˵��: ���GT911��оƬID
*	��    ��: ��
*	�� �� ֵ: 16λ�汾
*/
uint32_t GT911_ReadID(void)
{
	uint8_t buf[4]; 

	GT911_ReadReg(GT911_PRODUCT_ID_REG, buf, 4); 

	return ((uint32_t)buf[3] << 24) + ((uint32_t)buf[2] << 16) + ((uint32_t)buf[1] <<8) + buf[0]; 
}

/*
*	�� �� ��: GT911_ReadVersion
*	����˵��: ���GT911��оƬ�汾
*	��    ��: ��
*	�� �� ֵ: 16λ�汾
*/
uint16_t GT911_ReadVersion(void)
{
	uint8_t buf[2]; 

	GT911_ReadReg(GT911_FIRMWARE_VERSION_REG, buf, 2); 

	return ((uint16_t)buf[1] << 8) + buf[0]; 
}
/*
*	�� �� ��: GT911_Timer1ms
*	����˵��: ÿ��1ms����1��
*	��    ��: ��
*	�� �� ֵ: ��
*/
void GT911_Timer1ms(void)
{
	g_GT911.TimerCount++;
}
/*
*	�� �� ��: bsp_DetectLcdType
*	����˵��: ͨ��I2C����оƬ��ʶ��LCDģ�����͡���������ȫ�ֱ��� g_LcdType �� g_TouchType
*	��    ��: ��
*	�� �� ֵ: ��
*/
uint8_t g_TouchType;
uint8_t g_LcdType;
void bsp_DetectLcdType(void)
{	
	g_TouchType = 0xFF;
	g_LcdType =   0xFF;		
    /*
		GT911��I2C��ַ0x28,0x29 ���֡�
			ͨ����ȡ����IC��оƬID��ʶ��
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
				printf("��⵽7�������GT911(0x28) 800*480\n");
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
				printf("��⵽7.0�������GT911(0xBA)800x480\n");
			}
		 }
	  }						
	  delay_us(10);
}

/*
*	�� �� ��: TOUCH_IntWakeUpForGT
*	����˵��: GT928������GT911������INT���Ѻ�������λ��INT�����ߵ�ƽ���壬���Ի��Ѵ������ߵ�оƬ
*	��    ��: ��
*	�� �� ֵ: ��
*/
void TOUCH_IntWakeUpForGT(void)
{
	/* ����TP_INT�����ж����� */
	GPIO_InitTypeDef gpio_init;

	/* ��1������GPIOʱ�� */
	TP_INT_GPIO_CLK_ENABLE();
	
	/* ��2�����������еİ���GPIOΪ��������ģʽ */
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;   	  /* ������� */
	gpio_init.Pull = GPIO_NOPULL;                 /* ���������費ʹ�� */
	
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* GPIO�ٶȵȼ� */
	gpio_init.Pin = TP_INT_PIN;
	HAL_GPIO_Init(TP_INT_GPIO_PORT, &gpio_init);	

	TP_INT_GPIO_PORT->BSRR = ((uint32_t)TP_INT_PIN << 16U);	/* INT = 0 */
	delay_us(200);
	
	TP_INT_GPIO_PORT->BSRR = TP_INT_PIN;	/* INT = 1 */
	delay_us(2000);
	TP_INT_GPIO_PORT->BSRR = ((uint32_t)TP_INT_PIN << 16U);	/* INT = 0 */
	delay_us(200);
	
	gpio_init.Mode = GPIO_MODE_INPUT;   		/* �������� */
	gpio_init.Pull = GPIO_NOPULL;                /* ���������費ʹ�� */
	HAL_GPIO_Init(TP_INT_GPIO_PORT, &gpio_init);	
	
}
/*
��ʼ��GT9271������
����ֵ:0,��ʼ���ɹ�;1,��ʼ��ʧ�� 
*/
unsigned char GT911_Init(void)
{   
	__HAL_RCC_GPIOD_CLK_ENABLE();			//����GPIODʱ��
    __HAL_RCC_GPIOE_CLK_ENABLE();			//����GPIOEʱ��   

	GPIO_InitTypeDef gpio_init;
		
    /*�������еİ���GPIOΪ��������ģʽ */
    gpio_init.Mode = GPIO_MODE_INPUT;   		/* �������� */
    gpio_init.Pull = GPIO_NOPULL;               /* ���������費ʹ�� */
    
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* GPIO�ٶȵȼ� */
    gpio_init.Pin = TP_INT_PIN;
    HAL_GPIO_Init(TP_INT_GPIO_PORT, &gpio_init);	    
		
     bsp_DetectLcdType();	/* �Զ�ʶ����оƬ�ͺ� */
    #if 0	/* ���Դ��� */
        printf("GT911 ID :%08X, Version :%04X \n", GT911_ReadID(), GT911_ReadVersion());
    #endif    
	g_GT911.TimerCount = 0;
	g_GT911.TimerCount = 0;
	g_GT911.Enable = 1;

	return 1;  
}


