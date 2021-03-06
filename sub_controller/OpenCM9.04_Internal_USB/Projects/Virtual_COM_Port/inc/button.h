/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : button.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : This file contains the defines used for button fuctions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_H
#define __BUTTON_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define BUTTON_START		0x1
#define BUTTON_MODE		0x2

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void __Button_ISR(void);

#endif /* __BUTTON_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
