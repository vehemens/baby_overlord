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
#include "CM_DXL_COM.h"
#include "gyro_acc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t GYRO_Enable = 0;
volatile uint8_t ACC_Enable = 0;

uint16_t GYRO_X;
uint16_t GYRO_Y;
uint16_t GYRO_Z;

uint16_t ACC_X;
uint16_t ACC_Y;
uint16_t ACC_Z;

/* Private function prototypes -----------------------------------------------*/
void GYRO_ConvertData(uint8_t rxbuf[]);
void ACC_ConvertData(uint8_t rxbuf[]);
void GYRO_SPI_TR(uint8_t txbuf[], uint8_t rxbuf[], uint16_t len);
void ACC_SPI_TR(uint8_t txbuf[], uint8_t rxbuf[], uint16_t len);
void SPI_TR(uint8_t txbuf[], uint8_t rxbuf[], uint16_t len);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Gyro_Config
* Description    : Configures GYRO
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_Config(void)
{
  uint16_t i;
  uint8_t txbuf[4] =
    {
    0x20, 0xFF,
    0x23, 0x20
    };
  uint8_t rxbuf[4];

  for (i = 0; i < 2; i++)
  {
    GYRO_SPI_TR(&txbuf[2 * i], &rxbuf[2 * i], 2);
  }

  GYRO_Enable = 1;
}

/*******************************************************************************
* Function Name  : ACC_Config
* Description    : Configures accelerometer
* Input          : None.
* Return         : None.
*******************************************************************************/
void ACC_Config(void)
{
  uint16_t i;
  uint8_t txbuf[4] =
    {
    0x20, 0xA7,
    0x21, 0x08
    };
  uint8_t rxbuf[4];

  for (i = 0; i < 2; i++)
  {
    ACC_SPI_TR(&txbuf[2 * i], &rxbuf[2 * i], 2);
  }

  ACC_Enable = 1;
}

/*******************************************************************************
* Function Name  : __GYRO_ISR
* Description    : Reads GYRO rates
* Input          : None.
* Return         : None.
*******************************************************************************/
void __GYRO_ISR(void)
{
  uint16_t i;
  uint8_t txbuf[9] =
    {
    0xE8,0xFF,0xFF,
    0xEA,0xFF,0xFF,
    0xEC,0xFF,0xFF
    };
  uint8_t rxbuf[9];

  if (!GYRO_Enable || !ACC_Enable)
    return;

  for (i = 0; i < 3; i++)
  {
    GYRO_SPI_TR(&txbuf[3 * i], &rxbuf[3 * i], 3);
  }

  GYRO_ConvertData(rxbuf);
}

/*******************************************************************************
* Function Name  : __ACC_ISR
* Description    : Reads accelerometer rates
* Input          : None.
* Return         : None.
*******************************************************************************/
void __ACC_ISR(void)
{
  uint16_t i;
  uint8_t txbuf[9] =
    {
    0xE8,0xFF,0xFF,
    0xEA,0xFF,0xFF,
    0xEC,0xFF,0xFF
    };
  uint8_t rxbuf[9];

  if (!GYRO_Enable || !ACC_Enable)
    return;

  for (i = 0; i < 3; i++)
  {
    ACC_SPI_TR(&txbuf[3 * i], &rxbuf[3 * i], 3);
  }

  ACC_ConvertData(rxbuf);
}

/*******************************************************************************
* Function Name  : GYRO_ConvertData
* Description    : Converts raw data to u10
* Input          : Message buffer
* Return         : None.
*******************************************************************************/
void GYRO_ConvertData(uint8_t rxbuf[])
{
  int16_t GYRO_X_raw;
  int16_t GYRO_Y_raw;
  int16_t GYRO_Z_raw;

  int32_t temp;

  GYRO_X_raw = ((int16_t)rxbuf[2] << 8) | rxbuf[1];
  GYRO_Y_raw = ((int16_t)rxbuf[5] << 8) | rxbuf[4];
  GYRO_Z_raw = ((int16_t)rxbuf[8] << 8) | rxbuf[7];

  temp = (GYRO_X_raw / 64);
  temp = temp * 5 / 4;
  temp = 512 + temp;
  if (temp > 1023)
    temp = 1023;
  if (temp < 0)
    temp = 0;
  GW_GYRO_X = GYRO_X = temp;

  temp = GYRO_Y_raw / 64;
  temp = temp * 5 / 4;
  temp = 512 + temp;
  if (temp > 1023)
    temp = 1023;
  if (temp < 0)
    temp = 0;
  GW_GYRO_Y = GYRO_Y = temp;

  temp = GYRO_Z_raw / 64;
  temp = temp * 5 / 4;
  temp = 512 + temp;
  if (temp > 1023)
    temp = 1023;
  if (temp < 0)
    temp = 0;
  GW_GYRO_Z = GYRO_Z = temp;
}

/*******************************************************************************
* Function Name  : ACC_ConvertData
* Description    : Converts raw data to u10
* Input          : Message buffer
* Return         : None.
*******************************************************************************/
void ACC_ConvertData(uint8_t rxbuf[])
{
  int16_t ACC_X_raw;
  int16_t ACC_Y_raw;
  int16_t ACC_Z_raw;

  int32_t temp;

  ACC_X_raw = ((int16_t)rxbuf[2] << 8) | rxbuf[1];
  ACC_Y_raw = ((int16_t)rxbuf[5] << 8) | rxbuf[4];
  ACC_Z_raw = ((int16_t)rxbuf[8] << 8) | rxbuf[7];

  temp = (-1)*(ACC_X_raw / 64);
  //temp = temp * 4 / 3;
  temp = 512 + temp;
  if (temp > 1023)
    temp = 1023;
  if (temp < 0)
    temp = 0;
  GW_ACC_X = ACC_X = temp;

  temp = (-1)*(ACC_Y_raw / 64);
  //temp = temp * 4 / 3;
  temp = 512 + temp;
  if (temp > 1023)
    temp = 1023;
  if (temp < 0)
    temp = 0;
  GW_ACC_Y = ACC_Y = temp;

  temp = ACC_Z_raw / 64;
  //temp = temp * 4 / 3;
  temp = 512 + temp;
  if (temp > 1023)
    temp = 1023;
  if (temp < 0)
    temp = 0;
  GW_ACC_Z = ACC_Z = temp;
}

/*******************************************************************************
* Function Name  : GYRO_SPI_TR
* Description    : TR GYRO
* Input          : Message buffers and length
* Return         : None.
*******************************************************************************/
void GYRO_SPI_TR(uint8_t txbuf[], uint8_t rxbuf[], uint16_t len)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

  SPI_TR(txbuf, rxbuf, len);

  GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : ACC_SPI_TR
* Description    : TR accelerometer
* Input          : Message buffers and length
* Return         : None.
*******************************************************************************/
void ACC_SPI_TR(uint8_t txbuf[], uint8_t rxbuf[], uint16_t len)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

  SPI_TR(txbuf, rxbuf, len);

  GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : SPI_TR
* Description    : TR device
* Input          : Message buffers and length
* Return         : None.
*******************************************************************************/
void SPI_TR(uint8_t txbuf[], uint8_t rxbuf[], uint16_t len)
{
  uint16_t i;

  for (i = 0; i < len; i++)
  {
    SPI_I2S_SendData(SPI1, txbuf[i]);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    rxbuf[i] = (uint8_t)(SPI_I2S_ReceiveData(SPI1) & (uint16_t)0x00FF);
  }
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
