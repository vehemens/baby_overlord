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
#define RX_BUFFER_SIZE 18

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
vu8 GYRO_TxBuffer[9] = {
  0xE8,0xFF,0xFF,
  0xEA,0xFF,0xFF,
  0xEC,0xFF,0xFF};

vu8 ACC_TxBuffer[9] = {
  0xE8,0xFF,0xFF,
  0xEA,0xFF,0xFF,
  0xEC,0xFF,0xFF};

vu8 GYRO_RxBuffer[RX_BUFFER_SIZE];
vu8 ACC_RxBuffer[RX_BUFFER_SIZE];

vu8 GYRO_RxBufferPointer = 0;
vu8 ACC_RxBufferPointer = 0;

vu8 GYRO_Enable = 0;
vu8 ACC_Enable = 0;

u16 GYRO_X;
u16 GYRO_Y;
u16 GYRO_Z;

u16 ACC_X;
u16 ACC_Y;
u16 ACC_Z;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : X_Clear_Data
* Description    : Resets buffer pointer
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_Clear_Data(void)
{
  GYRO_RxBufferPointer = 0;
}

void ACC_Clear_Data(void)
{
  ACC_RxBufferPointer = 0;
}

/*******************************************************************************
* Function Name  : X_Push_Data
* Description    : Reads register and places data into a buffer
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_Push_Data(u16 dat)
{
  GYRO_RxBuffer[(GYRO_RxBufferPointer++)%RX_BUFFER_SIZE] = (u8)(dat & 0x00FF);
}

void ACC_Push_Data(u16 dat)
{
  ACC_RxBuffer[(ACC_RxBufferPointer++)%RX_BUFFER_SIZE] = (u8)(dat & 0x00FF);
}

/*******************************************************************************
* Function Name  : GYRO_ConvertData
* Description    : Converts s16 data to u10
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_ConvertData(void)
{
  s16 GYRO_X_raw;
  s16 GYRO_Y_raw;
  s16 GYRO_Z_raw;

  s32 temp;

  GYRO_X_raw = (s16)((GYRO_RxBuffer[2] << 8) + GYRO_RxBuffer[1]);
  GYRO_Y_raw = (s16)((GYRO_RxBuffer[5] << 8) + GYRO_RxBuffer[4]);
  GYRO_Z_raw = (s16)((GYRO_RxBuffer[8] << 8) + GYRO_RxBuffer[7]);

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
* Description    : Converts s16 data to u10
* Input          : None.
* Return         : None.
*******************************************************************************/
void ACC_ConvertData(void)
{
  s16 ACC_X_raw;
  s16 ACC_Y_raw;
  s16 ACC_Z_raw;

  s32 temp;

  ACC_X_raw = (s16)((ACC_RxBuffer[11] << 8) + ACC_RxBuffer[10]);
  ACC_Y_raw = (s16)((ACC_RxBuffer[14] << 8) + ACC_RxBuffer[13]);
  ACC_Z_raw = (s16)((ACC_RxBuffer[17] << 8) + ACC_RxBuffer[16]);

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
* Function Name  : Gyro_Configuration
* Description    : Configures GYRO
* Input          : None.
* Return         : None.
*******************************************************************************/
void GYRO_Configure(void)
{
  GYRO_Clear_Data();

  // write 0x20FF
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x20);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_Push_Data(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0xFF);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_Push_Data(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


  //write 0x2320
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x23);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_Push_Data(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0x20);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  GYRO_Push_Data(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

  GYRO_Enable = 1;
}

/*******************************************************************************
* Function Name  : ACC_Configuration
* Description    : Configures accelerometer
* Input          : None.
* Return         : None.
*******************************************************************************/
void ACC_Configure(void)
{
  ACC_Clear_Data();

  // write 0x20A7
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x20);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_Push_Data(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0xA7);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_Push_Data(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


  //write 0x2108
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);


  SPI_I2S_SendData(SPI1, 0x21);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_Push_Data(SPI_I2S_ReceiveData(SPI1));


  SPI_I2S_SendData(SPI1, 0x08);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ACC_Push_Data(SPI_I2S_ReceiveData(SPI1));


  GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

  ACC_Enable = 1;
}

/*******************************************************************************
* Function Name  : __GYRO_READ_ISR
* Description    : Reads GYRO rates
* Input          : None.
* Return         : None.
*******************************************************************************/
void __GYRO_READ_ISR(void)
{
  int i;

  if (!GYRO_Enable || !ACC_Enable)
    return;

  GYRO_Clear_Data();

  for (i = 0; i < 9; i++)
  {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    GPIO_ResetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);

    SPI_I2S_SendData(SPI1, GYRO_TxBuffer[i]);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    GYRO_Push_Data(SPI_I2S_ReceiveData(SPI1));

    if ((i + 1) % 3 == 0)
      GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);
  }

  GYRO_ConvertData();
}

/*******************************************************************************
* Function Name  : __ACC_READ_ISR
* Description    : Reads accelerometer rates
* Input          : None.
* Return         : None.
*******************************************************************************/
void __ACC_READ_ISR(void)
{
  int i;

  if (!GYRO_Enable || !ACC_Enable)
    return;

  ACC_Clear_Data();

  for (i = 0; i < 9; i++)
  {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    GPIO_ResetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);

    SPI_I2S_SendData(SPI1, ACC_TxBuffer[i]);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    ACC_Push_Data(SPI_I2S_ReceiveData(SPI1));

    if ((i + 1) % 3 == 0)
      GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);
  }

  ACC_ConvertData();
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
