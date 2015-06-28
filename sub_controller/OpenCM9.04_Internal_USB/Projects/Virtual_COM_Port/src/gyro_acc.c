/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : gyro_acc.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 2011/01/15
* Description        : functions about sensor control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "hw_config.h"
#include "gyro_acc.h"
#include "CM_DXL_COM.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// raw data -32000~32000
s16 Gyro_X_raw;
s16 Gyro_Y_raw;
s16 Gyro_Z_raw;
s16 ACC_X_raw;
s16 ACC_Y_raw;
s16 ACC_Z_raw;

// convert data 0 ~ 1023
u16 Gyro_X;
u16 Gyro_Y;
u16 Gyro_Z;
u16 ACC_X;
u16 ACC_Y;
u16 ACC_Z;

vu8 SPI_TxBuffer[9]={	0xE8,0xFF,0xFF,
			0xEA,0xFF,0xFF,
			0xEC,0xFF,0xFF };

vu8 SPI_RxBuffer[18];

vu8 SPI_RxBufferPointer=0;
vu8 SPI_TxBufferPointer=0;

vu8 GYRO_ACC_ENABLE = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Gyro_Acc_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SIG_ACC_CS_GPIO_PIN | SIG_GYRO_CS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SIG_SCK_GPIO_PIN | SIG_MOSI_GPIO_PIN | SIG_MISO_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitTypeDef SPI_InitStructure;

	GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);
	GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
}

void Push_SPI_Data(u16 dat)
{
	SPI_RxBuffer[SPI_RxBufferPointer++] = (u8)( dat & 0x00FF );
}

void Clear_SPI_Data(void)
{
	SPI_RxBufferPointer = 0;
}

void CovertData(void)
{
	s32 temp;

	Gyro_X_raw = (s16)((SPI_RxBuffer[2] << 8) + SPI_RxBuffer[1]);
	Gyro_Y_raw = (s16)((SPI_RxBuffer[5] << 8) + SPI_RxBuffer[4]);
	Gyro_Z_raw = (s16)((SPI_RxBuffer[8] << 8) + SPI_RxBuffer[7]);

	ACC_X_raw = (s16)((SPI_RxBuffer[11] << 8) + SPI_RxBuffer[10]);
	ACC_Y_raw = (s16)((SPI_RxBuffer[14] << 8) + SPI_RxBuffer[13]);
	ACC_Z_raw = (s16)((SPI_RxBuffer[17] << 8) + SPI_RxBuffer[16]);

	// 400dps /1023 0.39
	// 500dps /65535 0.007

	// convert data s16 -> u10
	temp = (Gyro_X_raw /64);
	temp = temp * 5 / 4;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_GYRO_X = Gyro_X = temp;

	// convert data s16 -> u10
	temp = Gyro_Y_raw /64;
	temp = temp * 5 / 4;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_GYRO_Y = Gyro_Y = temp;

	// convert data s16 -> u10
	temp = Gyro_Z_raw /64;
	temp = temp * 5 / 4;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_GYRO_Z = Gyro_Z = temp;


	// convert data s16 -> u10
	temp = (-1)*(ACC_X_raw /64);
	//temp = temp * 4 / 3;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_ACC_X = ACC_X = temp;

	// convert data s16 -> u10
	temp = (-1)*(ACC_Y_raw /64);
	//temp = temp * 4 / 3;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_ACC_Y = ACC_Y = temp;

	// convert data s16 -> u10
	temp = ACC_Z_raw /64;
	//temp = temp * 4 / 3;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_ACC_Z  = ACC_Z = temp;
}

void Gyro_Configuration(void)
{
	// write 0x20FF
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

	SPI_I2S_SendData(SPI1,0x20);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	SPI_I2S_SendData(SPI1,0xFF);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


	//write 0x2310
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

	SPI_I2S_SendData(SPI1,0x23);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	SPI_I2S_SendData(SPI1,0x20);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


	Clear_SPI_Data();
}

void ACC_Configuration(void)
{
	// write 0x202F
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

	SPI_I2S_SendData(SPI1,0x20);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	SPI_I2S_SendData(SPI1,0xA7);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


	//write 0x2310
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

	SPI_I2S_SendData(SPI1,0x21);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	SPI_I2S_SendData(SPI1,0x08);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

	GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


	Clear_SPI_Data();


	GYRO_ACC_ENABLE = 1;
}

void __GYRO_ACC_READ_ISR(void)
{
	int i;

	if (!GYRO_ACC_ENABLE)
		return;


	//gyro read
	for(i=0;i<9;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

		SPI_I2S_SendData(SPI1,SPI_TxBuffer[i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

		if( (i+1)%3 == 0 )
			 GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);
	}


	//acc read
	for(i=0;i<9;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

		SPI_I2S_SendData(SPI1,SPI_TxBuffer[i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		Push_SPI_Data( SPI_I2S_ReceiveData(SPI1) );

		if( (i+1)%3 == 0 )
			 GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);
	}


	CovertData();

	Clear_SPI_Data();
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
