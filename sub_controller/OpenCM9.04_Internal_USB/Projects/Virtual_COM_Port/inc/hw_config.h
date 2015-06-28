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


#if defined USE_DXL_USART1

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

#elif defined USE_DXL_USART3

#define DXL_USART			USART3
#define DXL_USART_CLK			RCC_APB1Periph_USART3
#define DXL_USART_IRQHandler		USART3_IRQHandler
#define DXL_USART_IRQn			USART3_IRQn
#define DXL_USART_GPIO_CLK		(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC)
#define DXL_USART_TXD_GPIO_PIN		GPIO_Pin_10
#define DXL_USART_TXD_GPIO_PORT		GPIOB
#define DXL_USART_RXD_GPIO_PIN		GPIO_Pin_11
#define DXL_USART_RXD_GPIO_PORT		GPIOB
#define DXL_USART_DIR_GPIO_PIN		GPIO_Pin_14
#define DXL_USART_DIR_GPIO_PORT		GPIOC

#endif


#define PORT_SW_MODE			GPIOB
#define PORT_SW_START			GPIOB

#define PIN_SW_START			GPIO_Pin_3
#define PIN_SW_MODE			GPIO_Pin_4


#define PORT_LED2			GPIOB
#define PORT_LED3			GPIOB
#define PORT_LED4			GPIOB

#define PIN_LED2			GPIO_Pin_14
#define PIN_LED3			GPIO_Pin_13
#define PIN_LED4			GPIO_Pin_12

#define PIN_LED_MANAGE			PIN_LED2
#define PIN_LED_EDIT			PIN_LED3
#define PIN_LED_PLAY			PIN_LED4

#define PORT_LED_MANAGE			PORT_LED2
#define PORT_LED_EDIT			PORT_LED3
#define PORT_LED_PLAY			PORT_LED4


#define PORT_SIG_SCK			GPIOA
#define PORT_SIG_MISO			GPIOA
#define PORT_SIG_MOSI			GPIOA

#define PORT_SIG_ACC_CS			GPIOA
#define PORT_SIG_GYRO_CS		GPIOA

#define PIN_SIG_SCK			GPIO_Pin_5
#define PIN_SIG_MISO			GPIO_Pin_6
#define PIN_SIG_MOSI			GPIO_Pin_7

#define PIN_SIG_ACC_CS			GPIO_Pin_1
#define PIN_SIG_GYRO_CS			GPIO_Pin_0

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
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes);
void Handle_USBAsynchXfer (void);
void CNTR_To_USB_Send_Data(uint8_t data);
void USART_To_USB_Send_Data(void);
void Get_SerialNum(void);

/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
