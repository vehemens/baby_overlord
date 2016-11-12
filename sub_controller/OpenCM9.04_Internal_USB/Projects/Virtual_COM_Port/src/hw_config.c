/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USART_Rx_Buffer[USART_RX_DATA_SIZE]; 
uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;

/* Extern variables ----------------------------------------------------------*/
extern uint8_t gbpRxCmBuffer[];
extern uint8_t gbRxCmBufferWritePointer;

extern uint8_t gbpTxCmBuffer[];
extern uint8_t gbTxCmBufferReadPointer;
extern uint8_t gbTxCmBufferWritePointer;

/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode(uint32_t value , uint8_t *pbuf , uint8_t len);

/* Private functions ---------------------------------------------------------*/
void USB_To_CM_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes);
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes);

/*******************************************************************************
* Function Name  : Clock_Config
* Description    : Configures system clocks & power
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Clock_Config(void)
{
  /* Peripheral clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#if defined USE_DXL_USART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#if defined USE_DXL_USART3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/*******************************************************************************
* Function Name  : IO_Config
* Description    : Configures GPIO and EXTI interfaces
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void IO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_StructInit(&GPIO_InitStructure);

  EXTI_StructInit(&EXTI_InitStructure);

  /* USB */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT_GPIO_PORT, &GPIO_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line18);

  EXTI_InitStructure.EXTI_Line = EXTI_Line18; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* DXL USART */
  GPIO_InitStructure.GPIO_Pin = DXL_USART_TXD_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(DXL_USART_TXD_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = DXL_USART_RXD_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(DXL_USART_RXD_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = DXL_USART_DIR_GPIO_PIN;
#if defined USE_DXL_USART1
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#elif defined USE_DXL_USART3
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
#endif
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(DXL_USART_DIR_GPIO_PORT, &GPIO_InitStructure);

  /* LEDs */
  GPIO_InitStructure.GPIO_Pin = LED_TEST_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_TEST_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LED_MANAGE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_MANAGE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LED_EDIT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_EDIT_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LED_PLAY_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_PLAY_GPIO_PORT, &GPIO_InitStructure);

  /* Buttons */
  GPIO_InitStructure.GPIO_Pin = SW_TEST_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(SW_TEST_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SW_MODE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SW_MODE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SW_START_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SW_START_GPIO_PORT, &GPIO_InitStructure);

  /* IMU */
  GPIO_InitStructure.GPIO_Pin = SIG_SCK_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SIG_SCK_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SIG_MOSI_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SIG_MOSI_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SIG_MISO_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SIG_MISO_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SIG_GYRO_CS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SIG_GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SIG_ACC_CS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SIG_ACC_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Remap */
  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

  /* DXL USART */
  GPIO_ResetBits(DXL_USART_DIR_GPIO_PORT, DXL_USART_DIR_GPIO_PIN);

  /* LEDs */
  GPIO_SetBits(LED_TEST_GPIO_PORT, LED_TEST_GPIO_PIN);
  GPIO_SetBits(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN);
  GPIO_SetBits(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN);
  GPIO_SetBits(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN);

  /* IMU */
  GPIO_SetBits(SIG_GYRO_CS_GPIO_PORT, SIG_GYRO_CS_GPIO_PIN);
  GPIO_SetBits(SIG_ACC_CS_GPIO_PORT, SIG_ACC_CS_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : Interrupt_Config
* Description    : Configures the interrupts
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Interrupt_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Timer */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* USB */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* DXL USART */
  NVIC_InitStructure.NVIC_IRQChannel = DXL_USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USART_Config
* Description    : Configures USART Devices
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;

  USART_StructInit(&USART_InitStructure);

  /* DXL USART */
  USART_InitStructure.USART_BaudRate = 1000000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(DXL_USART, &USART_InitStructure);

  USART_Cmd(DXL_USART, ENABLE);

  USART_ITConfig(DXL_USART, USART_IT_RXNE, ENABLE);
}

/*******************************************************************************
* Function Name  : SPI_Config
* Description    : Configures SPI Devices
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SPI_Config(void)
{
  SPI_InitTypeDef SPI_InitStructure;

  SPI_StructInit(&SPI_InitStructure);

  /* IMU */
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

/*******************************************************************************
* Function Name  : Timer_Config
* Description    : Configures Timer Devices
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
volatile uint16_t CCR4_Val = 12; // 12 us

void Timer_Config(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_OCStructInit(&TIM_OCInitStructure);

  TIM_DeInit(TIM2);

  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);

  TIM_Cmd(TIM2, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
  /*Enable SystemCoreClock*/
  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Cable_Config(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT_GPIO_PORT, USB_DISCONNECT_GPIO_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT_GPIO_PORT, USB_DISCONNECT_GPIO_PIN);
  }
}

/*******************************************************************************
* Function Name  : USB_To_CM_USART_Send_Data.
* Description    : send USB data to CM and USART.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_To_CM_USART_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes)
{
  USB_To_USART_Send_Data(data_buffer, Nb_bytes);

  USB_To_CM_Send_Data(data_buffer, Nb_bytes);
}

/*******************************************************************************
* Function Name  : USB_To_CM_Send_Data.
* Description    : send USB data to CM.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_To_CM_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes)
{
  uint32_t i;

  for (i = 0; i < Nb_bytes; i++)
  {
    gbpRxCmBuffer[gbRxCmBufferWritePointer++] = *(data_buffer + i);
  }
}

/*******************************************************************************
* Function Name  : CNTR_To_USB_Send_Data.
* Description    : send CNTR data to USB.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CNTR_To_USB_Send_Data()
{
  uint8_t data;
  uint32_t tmp_in = USART_Rx_ptr_in;

  while (gbTxCmBufferReadPointer != gbTxCmBufferWritePointer)
  {
    data = gbpTxCmBuffer[gbTxCmBufferReadPointer++];

    USART_Rx_Buffer[(tmp_in++)%USART_RX_DATA_SIZE] = data;
  }
  USART_Rx_ptr_in = tmp_in;
}

/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send USB data to USART.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes)
{
  uint32_t i;

  GPIO_SetBits(DXL_USART_DIR_GPIO_PORT, DXL_USART_DIR_GPIO_PIN);
  for (i = 0; i < Nb_bytes; i++)
  {
    USART_SendData(DXL_USART, *(data_buffer + i));
    while(USART_GetFlagStatus(DXL_USART, USART_FLAG_TXE) == RESET);
  }
  while(USART_GetFlagStatus(DXL_USART, USART_FLAG_TC) == RESET);
  GPIO_ResetBits(DXL_USART_DIR_GPIO_PORT, DXL_USART_DIR_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : USART_To_USB_Send_Data.
* Description    : send USART data to USB.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USART_To_USB_Send_Data(uint8_t data)
{
  uint32_t tmp_in = USART_Rx_ptr_in;

  USART_Rx_Buffer[(tmp_in++)%USART_RX_DATA_SIZE] = data;

  USART_Rx_ptr_in = tmp_in;
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;  

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode(uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
