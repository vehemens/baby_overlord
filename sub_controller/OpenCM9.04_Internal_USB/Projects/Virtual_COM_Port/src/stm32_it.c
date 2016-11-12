/**
  ******************************************************************************
  * @file    stm32_it.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "hw_config.h"
#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "button.h"
#include "gyro_acc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                       */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                                */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}

#include "CM_DXL_COM.h"

extern volatile uint16_t CCR4_Val;

volatile uint32_t capture = 0;
volatile uint8_t Counter = 0;
volatile uint16_t gwCounter1 = 0;

extern volatile uint8_t gbMiliSec;

void TIM2_IRQHandler(void)
{
  static int b1Sec = 0;

  if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) // 120us, 8000Hz
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    //ISR_ADC();
    capture = TIM_GetCapture4(TIM2);
    TIM_SetCompare4(TIM2, capture + CCR4_Val);

    if (!(gwCounter1 & 7)) // 840us
    {
      //ISR_1ms_TIMER();
      gbMiliSec++;
    }

    if (!(gwCounter1 & 3)) // 480us, 2000Hz
    {
      //ISR_LED_RGB_TIMER();
    }

    if (!(gwCounter1 & 31)) // 3840us, 250Hz
    {
      //ISR_SPI_READ();
      __GYRO_ISR();
      __ACC_ISR();
      //__ISR_Buzzer_Manage();
      __Button_ISR();
    }

    if (!(gwCounter1 & 0x3FF)) // 125ms
    {
      //LED_SetState(LED_RX,OFF);
      //LED_SetState(LED_TX,OFF);

      if (!(b1Sec & 0x07))
      {
        //ISR_BATTERY_CHECK();
      }

      b1Sec++;
    }

#if 0
    if (!(Counter1 & 32)) // 3960us, 250Hz
    {
    }
#endif

    gwCounter1++;
  }
}

void DXL_USART_IRQHandler(void)
{
  uint8_t data;

  if (USART_GetITStatus(DXL_USART, USART_IT_RXNE) != RESET)
  {
    /* Send data to the USB */
    data = (uint8_t)(USART_ReceiveData(DXL_USART) & (uint16_t)0x00FF);

    USART_To_USB_Send_Data(data);
  }

  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(DXL_USART, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(DXL_USART);
  }
}

void USBWakeUp_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

