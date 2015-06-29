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
void GYRO_Clear_Data(void);
void ACC_Clear_Data(void);

void GYRO_Push_Data(u16 dat);
void ACC_Push_Data(u16 dat);

void GYRO_Convert_Data(void);
void ACC_Convert_Data(void);

void GYRO_Configure(void);
void ACC_Configure(void);

void __GYRO_READ_ISR(void);
void __ACC_READ_ISR(void);

#endif /* __GYRO_SPI_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
