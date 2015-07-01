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
#define RX_BUFFER_SIZE 9

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t GYRO_TxBuffer[9] = {
  0xE8,0xFF,0xFF,
  0xEA,0xFF,0xFF,
  0xEC,0xFF,0xFF};

volatile uint8_t ACC_TxBuffer[9] = {
  0xE8,0xFF,0xFF,
  0xEA,0xFF,0xFF,
  0xEC,0xFF,0xFF};

volatile uint8_t GYRO_RxBuffer[RX_BUFFER_SIZE];
volatile uint8_t ACC_RxBuffer[RX_BUFFER_SIZE];

volatile uint8_t GYRO_RxBufferPointer = 0;
volatile uint8_t ACC_RxBufferPointer = 0;

volatile uint8_t GYRO_Enable = 0;
volatile uint8_t ACC_Enable = 0;

uint16_t GYRO_X;
uint16_t GYRO_Y;
uint16_t GYRO_Z;

uint16_t ACC_X;
uint16_t ACC_Y;
uint16_t ACC_Z;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : X_ClearData
* Description    : Resets buffer pointer
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_ClearData(void)
{
  GYRO_RxBufferPointer = 0;
}

void ACC_ClearData(void)
{
  ACC_RxBufferPointer = 0;
}

/*******************************************************************************
* Function Name  : X_PushData
* Description    : Reads register and places data into a buffer
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_PushData(uint16_t dat)
{
  GYRO_RxBuffer[(GYRO_RxBufferPointer++)%RX_BUFFER_SIZE] = (uint8_t)(dat & 0x00FF);
}

void ACC_PushData(uint16_t dat)
{
  ACC_RxBuffer[(ACC_RxBufferPointer++)%RX_BUFFER_SIZE] = (uint8_t)(dat & 0x00FF);
}

/*******************************************************************************
* Function Name  : GYRO_ConvertData
* Description    : Converts int16_t data to u10
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_ConvertData(void)
{
  int16_t GYRO_X_raw;
  int16_t GYRO_Y_raw;
  int16_t GYRO_Z_raw;

  int32_t temp;

  GYRO_X_raw = (int16_t)((GYRO_RxBuffer[2] << 8) + GYRO_RxBuffer[1]);
  GYRO_Y_raw = (int16_t)((GYRO_RxBuffer[5] << 8) + GYRO_RxBuffer[4]);
  GYRO_Z_raw = (int16_t)((GYRO_RxBuffer[8] << 8) + GYRO_RxBuffer[7]);

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
* Description    : Converts int16_t data to u10
* Input          : None.
* Return         : None.
*******************************************************************************/
void ACC_ConvertData(void)
{
  int16_t ACC_X_raw;
  int16_t ACC_Y_raw;
  int16_t ACC_Z_raw;

  int32_t temp;

  ACC_X_raw = (int16_t)((ACC_RxBuffer[2] << 8) + ACC_RxBuffer[1]);
  ACC_Y_raw = (int16_t)((ACC_RxBuffer[5] << 8) + ACC_RxBuffer[4]);
  ACC_Z_raw = (int16_t)((ACC_RxBuffer[8] << 8) + ACC_RxBuffer[7]);

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
* Function Name  : Gyro_Config
* Description    : Configures GYRO device
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_Config(void)
{
  GYRO_ClearData();

  // write 0x20FF
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x20);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_PushData(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0xFF);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_PushData(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


  //write 0x2320
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x23);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_PushData(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0x20);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_PushData(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

  GYRO_Enable = 1;
}

/*******************************************************************************
* Function Name  : ACC_Config
* Description    : Configures accelerometer device
* Input          : None.
* Return         : None.
*******************************************************************************/
void ACC_Config(void)
{
  ACC_ClearData();

  // write 0x20A7
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x20);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_PushData(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0xA7);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_PushData(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


  //write 0x2108
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x21);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_PushData(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0x08);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_PushData(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

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
  uint32_t i;

  if (!GYRO_Enable || !ACC_Enable)
    return;

  GYRO_ClearData();

  for (i = 0; i < 9; i++)
  {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

    SPI_I2S_SendData(SPI1, GYRO_TxBuffer[i]);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    GYRO_PushData(SPI_I2S_ReceiveData(SPI1));

    if ((i + 1) % 3 == 0)
      GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);
  }

  GYRO_ConvertData();
}

/*******************************************************************************
* Function Name  : __ACC_ISR
* Description    : Reads accelerometer rates
* Input          : None.
* Return         : None.
*******************************************************************************/
void __ACC_ISR(void)
{
  uint32_t i;

  if (!GYRO_Enable || !ACC_Enable)
    return;

  ACC_ClearData();

  for (i = 0; i < 9; i++)
  {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

    SPI_I2S_SendData(SPI1, ACC_TxBuffer[i]);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    ACC_PushData(SPI_I2S_ReceiveData(SPI1));

    if ((i + 1) % 3 == 0)
      GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);
  }

  ACC_ConvertData();
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
