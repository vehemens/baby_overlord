/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
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
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Tx_State = 0;

/* Extern variables ----------------------------------------------------------*/
extern uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_in;
extern uint32_t USART_Rx_ptr_out;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Asynch_Xfer
* Description    : send buffered data to USB.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Asynch_Xfer(void)
{
  uint32_t USART_Rx_length;
  uint8_t USB_Tx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
  uint16_t USB_Tx_length;
  uint32_t i;

  if(USB_Tx_State == 0)
  {
    USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;

    if(USART_Rx_length != 0)
    {
      USB_Tx_length = USART_Rx_length < VIRTUAL_COM_PORT_DATA_SIZE ?
        USART_Rx_length : VIRTUAL_COM_PORT_DATA_SIZE;

      for (i = 0; i < USB_Tx_length; i++)
      {
        USB_Tx_Buffer[i] = USART_Rx_Buffer[(USART_Rx_ptr_out++)%USART_RX_DATA_SIZE];
      }

      UserToPMABufferCopy(USB_Tx_Buffer, ENDP1_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP1, USB_Tx_length);
      SetEPTxValid(ENDP1);
      USB_Tx_State = 1;
    }
  }
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  uint32_t USART_Rx_length;
  uint8_t USB_Tx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
  uint16_t USB_Tx_length;
  uint32_t i;

  if (USB_Tx_State != 0)
  {
    USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;

    if (USART_Rx_length != 0)
    {
      USB_Tx_length = USART_Rx_length < VIRTUAL_COM_PORT_DATA_SIZE ?
        USART_Rx_length : VIRTUAL_COM_PORT_DATA_SIZE;

      for (i = 0; i < USB_Tx_length; i++)
      {
        USB_Tx_Buffer[i] = USART_Rx_Buffer[(USART_Rx_ptr_out++)%USART_RX_DATA_SIZE];
      }

      UserToPMABufferCopy(USB_Tx_Buffer, ENDP1_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP1, USB_Tx_length);
      SetEPTxValid(ENDP1);
    }
    else
    {
      USB_Tx_State = 0;
    }
  }
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
  uint32_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the USART Xfer */
  
  USB_To_CM_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);
 
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
}

/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
  if(bDeviceState == CONFIGURED)
  {
    /* Check the data to be sent through IN pipe */
    EP1_IN_Asynch_Xfer();
  }  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

