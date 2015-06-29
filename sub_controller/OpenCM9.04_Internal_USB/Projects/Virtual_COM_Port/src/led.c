/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : led.c
* Author             : zerom
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : functions about led control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "hw_config.h"
#include "CM_DXL_COM.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void LED_SetState(u8 LED_PORT, PowerState NewState)
{
  if (NewState == ON)
  { 
    if (LED_PORT & LED_MANAGE)
      GPIO_ResetBits(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN);
    if (LED_PORT & LED_EDIT)
      GPIO_ResetBits(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN);
    if (LED_PORT & LED_PLAY)
      GPIO_ResetBits(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN);
  }
  else
  {
    if (LED_PORT & LED_MANAGE)
      GPIO_SetBits(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN);
    if (LED_PORT & LED_EDIT)
      GPIO_SetBits(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN);
    if (LED_PORT & LED_PLAY)
      GPIO_SetBits(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN);
  }
}

PowerState LED_GetState(u8 LED_PORT)
{
  if (LED_PORT == LED_MANAGE)
  {
    if (GPIO_ReadOutputDataBit(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN) != SET)
      return ON;
    else
      return OFF;
  }
  else if (LED_PORT == LED_EDIT)
  {
    if (GPIO_ReadOutputDataBit(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN) != SET)
      return ON;
    else
      return OFF;
  }
  else if (LED_PORT == LED_PLAY)
  {
    if (GPIO_ReadOutputDataBit(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN) != SET)
      return ON;
    else
      return OFF;
  }

  return OFF;
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
