/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : button.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : functions about button control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "common_type.h"
#include "hw_config.h"
#include "button.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
* Function Name  : ReadKey
* Description    : Reads key from demoboard.
* Input          : None
* Output         : None
* Return         : Return RIGHT, LEFT, SEL, UP, DOWN or NOKEY
*******************************************************************************/

void InitButton(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SW_MODE_GPIO_PIN | SW_START_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

u8 ReadButton(void)
{
	u8 retval=0;

	if( GPIO_ReadInputDataBit(SW_START_GPIO_PORT, SW_START_GPIO_PIN) == SET )  retval |= BUTTON_START;
	if( GPIO_ReadInputDataBit(SW_MODE_GPIO_PORT,  SW_MODE_GPIO_PIN)  == SET )  retval |= BUTTON_MODE;
	
	return retval;
}




/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
