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
/*
*	�� �� ��: TOUCH_PenInt
*	����˵��: �жϴ�������
*	��    ��: ��
*	�� �� ֵ: 0��ʾ�޴��ʰ��£�1��ʾ�д��ʰ���
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
*	�� �� ��: GT911_Scan
*	����˵��: ��ȡGT911�������ݡ���ȡȫ�������ݣ���Ҫ 720us����
*	��    ��: ��
*	�� �� ֵ: ��
*/
unsigned char GT911_Scan(void)
{
	uint8_t buf[48];
	uint8_t clear_flag = 0;
	g_GT911.TimerCount = 0;

	if (TOUCH_PenInt() == 1)
	{		
        //	/* ����INT���ţ���״̬�Ĵ��� */
        //	GT911_ReadReg(GT911_READ_XY_REG, buf, 1);
        //	if (buf[0] == 0)
        //	{

        #if 1		/* ֻ��1�� */
            GT911_ReadReg(GT911_READ_XY_REG, buf, 8);
        #else		/* ��5�������� */
            GT911_ReadReg(GT911_READ_XY_REG, buf, 40);
        #endif
            
            GT911_WriteReg(GT911_READ_XY_REG, &clear_flag,	1);		/* ������������д0��� */
	
	/*
		0x814E R/W Bufferstatus Large_Detect number of touch points 
			Bit7: Buffer status��1��ʾ���꣨�򰴼����Ѿ�׼���ã����ؿ��Զ�ȡ��0��ʾδ������������Ч�������ض�ȡ������󣬱���ͨ��I2C���˱�־���������ֽڣ�дΪ0��
			Bit4: HaveKey, 1��ʾ�а�����0��ʾ�ް������Ѿ��ɼ�����
			Bit3~0: Number of touch points, ���ϵ���������
	
		0x814F R Point1 track id                    0x8157 R Point2 track id 
		0x8150 R Point1Xl ������ 1��X ����� 8 λ   0x8158 R Point2Xl ������ 2��X ����� 8 λ 
		0x8151 R Point1Xh ������ 1��X ����� 8 λ   0x8159 R Point2Xh ������ 2��X ����� 8 λ 
		0x8152 R Point1Yl ������ 1��Y ����� 8 λ   0x815A R Point2Yl ������ 2��Y ����� 8 λ 
		0x8153 R Point1Yh ������ 1��Y ����� 8 λ   0x815B R Point2Yh ������ 2��Y ����� 8 λ 
		0x8154 R Point1 ������ 1����������� 8 λ   0x815C R Point2 ������ 2����������� 8 λ 
		0x8155 R Point1 ������ 1����������� 8 λ   0x815D R Point2 ������ 2����������� 8 λ 
		0x8156 ----                                 0x815E ---- 

		0x815F R Point3 track id         		  0x8167 R Point4 track id          
		0x8160 R Point3Xl ������ 3��X ����� 8 λ 0x8168 R Point4Xl ������ 4��X ����� 8 λ 
		0x8161 R Point3Xh ������ 3��X ����� 8 λ 0x8169 R Point4Xh ������ 4��X ����� 8 λ 
		0x8162 R Point3Yl ������ 3��Y ����� 8 λ 0x816A R Point4Yl ������ 4��Y ����� 8 λ 
		0x8163 R Point3Yh ������ 3��Y ����� 8 λ 0x816B R Point4Yh ������ 4��Y ����� 8 λ
		0x8164 R Point3 ������ 3����������� 8 λ 0x816C R Point4 ������ 4����������� 8 λ 
		0x8165 R Point3 ������ 3����������� 8 λ 0x816D R Point4 ������ 4����������� 8 λ 
		0x8166 ----                               0x816E ----

		0x816F R Point5 track id 
		0x8170 R Point5Xl ������ 5��X ����� 8 λ 
		0x8171 R Point5Xh ������ 5��X ����� 8 λ 
		0x8172 R Point5Yl ������ 5��Y ����� 8 λ 
		0x8173 R Point5Yh ������ 5��Y ����� 8 λ 
		0x8174 R Point5 ������ 5����������� 8 λ 
		0x8175 R Point5 ������ 5����������� 8 λ 
		0x8176 --
		
	*/
	g_GT911.TouchpointFlag = buf[0];
	g_GT911.Touchkeystate = buf[1];

	g_GT911.X0 = ((uint16_t)buf[3] << 8) + buf[2];
	g_GT911.Y0 = ((uint16_t)buf[5] << 8) + buf[4];
	g_GT911.P0 = ((uint16_t)buf[7] << 8) + buf[6];

	#if 0	/* ����4��һ�㲻�� */
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

		/* ����ת�� :
			���ݴ��������½��� (0��0);  ���Ͻ��� (479��799)
			��Ҫת��LCD���������� (���Ͻ��� (0��0), ���½��� (799��479)
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

