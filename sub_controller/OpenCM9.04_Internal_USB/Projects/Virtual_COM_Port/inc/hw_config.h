/**
  ******************************************************************************
  * @file    hw_config.h
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

#define USB_DISCONNECT                      GPIOC
#define USB_DISCONNECT_PIN                  GPIO_Pin_13
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOC

#define DXL_USART			USART1
#define DXL_USART_CLK			RCC_APB2Periph_USART1
#define DXL_USART_IRQHandler		USART1_IRQHandler
#define DXL_USART_IRQn			USART1_IRQn
#define DXL_USART_GPIO_CLK		RCC_APB2Periph_GPIOB
#define DXL_USART_TXD_GPIO_PIN		GPIO_Pin_6
#define DXL_USART_TXD_GPIO_PORT		GPIOB
#define DXL_USART_RXD_GPIO_PIN		GPIO_Pin_7
#define DXL_USART_RXD_GPIO_PORT		GPIOB
#define DXL_USART_DIR_GPIO_PIN		GPIO_Pin_5
#define DXL_USART_DIR_GPIO_PORT		GPIOB

/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USART_RX_DATA_SIZE   2048

/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void USART_Config_Default(void);
bool USART_Config(void);
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes);
void USART_To_USB_Send_Data(void);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
