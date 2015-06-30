/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : spi.h
* Author             : danceww
* Version            : V0.1
* Date               : 2011/01/15
* Description        : This file contains the defines used for sensor functions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GYRO_SPI_H
#define __GYRO_SPI_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void GYRO_ClearData(void);
void ACC_ClearData(void);

void GYRO_PushData(uint16_t dat);
void ACC_PushData(uint16_t dat);

void GYRO_ConvertData(void);
void ACC_ConvertData(void);

void GYRO_Config(void);
void ACC_Config(void);

void __GYRO_ISR(void);
void __ACC_ISR(void);

#endif /* __GYRO_SPI_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
