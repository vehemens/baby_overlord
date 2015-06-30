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
#define LED_MANAGE		0x1
#define LED_EDIT		0x2
#define LED_PLAY		0x4

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LED_SetState(uint8_t LED_PORT, PowerState NewState);
PowerState LED_GetState(uint8_t LED_PORT);

#endif /* __LED_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
