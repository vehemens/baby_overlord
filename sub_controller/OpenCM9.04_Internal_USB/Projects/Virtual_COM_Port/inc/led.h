/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : led.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/24
* Description        : This file contains the defines used for LED fuctions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

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

#define LED_MANAGE			0x1
#define LED_EDIT			0x2
#define	LED_PLAY			0x4
#define LED_TX				0x8
#define LED_RX				0x10
//#define LED_AUX				0x20
#define LED_POWER			0x40
#define LED_ALL				0x7F

#define LED_R		0x1
#define LED_G		0x2
#define LED_B		0x4


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void LED_SetState(u8 LED_PORT, PowerState NewState);
PowerState LED_GetState(u8 LED_PORT);
void LED_RGB_SetState(u8 RGB);
u8 LED_RGB_GetState();

#endif /* __LED_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
