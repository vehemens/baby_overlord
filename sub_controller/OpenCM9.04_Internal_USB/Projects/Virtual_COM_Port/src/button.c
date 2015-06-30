/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : button.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : functions about button control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "hw_config.h"
#include "CM_DXL_COM.h"
#include "button.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : __Button_ISR
* Description    : Reads button state
* Input          : None.
* Return         : Button state
*******************************************************************************/
void __Button_ISR(void)
{
  uint8_t val = 0;

  if (GPIO_ReadInputDataBit(SW_START_GPIO_PORT, SW_START_GPIO_PIN) == SET)
    val |= BUTTON_START;

  if (GPIO_ReadInputDataBit(SW_MODE_GPIO_PORT, SW_MODE_GPIO_PIN) == SET)
    val |= BUTTON_MODE;

  GB_BUTTON = val;
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
