/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : led.c
* Author             : zerom
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : functions about led control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "common_type.h"
#include "hw_config.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void InitLED(void)
{
	int bCount;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED_MANAGE_GPIO_PIN | LED_EDIT_GPIO_PIN | LED_PLAY_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
}

void LED_SetState(u8 LED_PORT, PowerState NewState)
{
	if( NewState == ON )
	{ 
		if( LED_PORT & LED_MANAGE )
			GPIO_ResetBits(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN);
		if( LED_PORT & LED_EDIT )
		 	GPIO_ResetBits(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN);
		if( LED_PORT & LED_PLAY )
		 	GPIO_ResetBits(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN);
	}
	else
	{
		if( LED_PORT & LED_MANAGE )
			GPIO_SetBits(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN);
		if( LED_PORT & LED_EDIT )
		 	GPIO_SetBits(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN);
		if( LED_PORT & LED_PLAY )
		 	GPIO_SetBits(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN);
	}
}

PowerState LED_GetState(u8 LED_PORT)
{
	if( LED_PORT == LED_MANAGE )
	{
		if( GPIO_ReadOutputDataBit(LED_MANAGE_GPIO_PORT, LED_MANAGE_GPIO_PIN) != SET )
			return ON;
		else
			return OFF;
	}
	else if( LED_PORT == LED_EDIT )
	{
		if( GPIO_ReadOutputDataBit(LED_EDIT_GPIO_PORT, LED_EDIT_GPIO_PIN) != SET )
			return ON;
		else
			return OFF;
	}
	else if( LED_PORT == LED_PLAY )
	{
		if( GPIO_ReadOutputDataBit(LED_PLAY_GPIO_PORT, LED_PLAY_GPIO_PIN) != SET )
			return ON;
		else
			return OFF;
	}

	return OFF;
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
