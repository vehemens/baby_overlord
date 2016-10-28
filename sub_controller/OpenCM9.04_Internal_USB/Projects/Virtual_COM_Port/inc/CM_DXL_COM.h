/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : CM_DXL_COM.h
* Author             : dudung
* Version            : V0.1
* Date               : 2010/11/03
* Description        : This file contains the inteligent toss mode between
                       host and dynamixel.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DYNAMIXEL_CM_DXL_COM_HEADER
#define _DYNAMIXEL_CM_DXL_COM_HEADER

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {OFF = 0, ON = !OFF} PowerState;

/* Exported constants --------------------------------------------------------*/
//EEPROM AREA
#define P_MODEL_NUMBER_L	0
#define P_MODEL_NUMBER_H	1
#define P_VERSION		2
#define P_ID			3
#define P_BAUD_RATE		4
#define P_RETURN_DELAY_TIME	5
#define P_RETURN_LEVEL		16

//RAM AREA
#define P_DYNAMIXEL_POWER	24

#define P_LED_PANNEL		25
#define P_LED_HEAD_L		26
#define P_LED_HEAD_H		27
#define P_LED_EYE_L		28
#define P_LED_EYE_H		29

#define P_BUTTON		30

#define P_GYRO_Z_L		38
#define P_GYRO_Z_H		39
#define P_GYRO_Y_L		40
#define P_GYRO_Y_H		41
#define P_GYRO_X_L		42
#define P_GYRO_X_H		43

#define P_ACC_X_L		44
#define P_ACC_X_H		45
#define P_ACC_Y_L		46
#define P_ACC_Y_H		47
#define P_ACC_Z_L		48
#define P_ACC_Z_H		49

#define P_ADC0_VOLTAGE		50
#define P_ADC1_MIC1_L		51
#define P_ADC2_L		53
#define P_ADC3_L		55
#define P_ADC4_L		57
#define P_ADC5_L		59
#define P_ADC6_L		61
#define P_ADC7_L		63
#define P_ADC8_L		65
#define P_ADC9_MIC2_L		67
#define P_ADC10_L		69
#define P_ADC11_L		71
#define P_ADC12_L		73
#define P_ADC13_L		75
#define P_ADC14_L		77
#define P_ADC15_L		79

/* Exported macro ------------------------------------------------------------*/
#define WORD_CAST(AA)           (*(uint16_t *)(&(AA)))

#define GW_MODEL_NUMBER           WORD_CAST(gbpControlTable[P_MODEL_NUMBER_L])
#define GB_GB_VERSION             gbpControlTable[P_VERSION]
#define GB_ID                     gbpControlTable[P_ID]
#define GB_BAUD_RATE              gbpControlTable[P_BAUD_RATE]
#define GB_RETURN_DELAY_TIME      gbpControlTable[P_RETURN_DELAY_TIME]
#define GB_RETURN_LEVEL           gbpControlTable[P_RETURN_LEVEL]

#define	GB_DYNAMIXEL_POWER	gbpControlTable[P_DYNAMIXEL_POWER]

#define	GB_LED_MODE	gbpControlTable[P_LED_PANNEL]
#define	GW_LED_HEAD	WORD_CAST(gbpControlTable[P_LED_HEAD_L])
#define	GW_LED_EYE	WORD_CAST(gbpControlTable[P_LED_EYE_L])

#define	GB_BUTTON	gbpControlTable[P_BUTTON]

#define	GW_GYRO_Z	WORD_CAST(gbpControlTable[P_GYRO_Z_L])
#define	GW_GYRO_Y	WORD_CAST(gbpControlTable[P_GYRO_Y_L])
#define	GW_GYRO_X	WORD_CAST(gbpControlTable[P_GYRO_X_L])

#define	GW_ACC_X	WORD_CAST(gbpControlTable[P_ACC_X_L])
#define	GW_ACC_Y	WORD_CAST(gbpControlTable[P_ACC_Y_L])
#define	GW_ACC_Z	WORD_CAST(gbpControlTable[P_ACC_Z_L])

#define	GB_ADC0_VOLTAGE	gbpControlTable[P_ADC0_VOLTAGE]
#define	GW_ADC1_MIC1	WORD_CAST(gbpControlTable[P_ADC1_MIC1_L])
#define	GW_ADC2			WORD_CAST(gbpControlTable[P_ADC2_L])
#define	GW_ADC3			WORD_CAST(gbpControlTable[P_ADC3_L])
#define	GW_ADC4			WORD_CAST(gbpControlTable[P_ADC4_L])
#define	GW_ADC5			WORD_CAST(gbpControlTable[P_ADC5_L])
#define	GW_ADC6			WORD_CAST(gbpControlTable[P_ADC6_L])
#define	GW_ADC7			WORD_CAST(gbpControlTable[P_ADC7_L])
#define	GW_ADC8			WORD_CAST(gbpControlTable[P_ADC8_L])
#define	GW_ADC9_MIC2	WORD_CAST(gbpControlTable[P_ADC9_MIC2_L])
#define	GW_ADC10		WORD_CAST(gbpControlTable[P_ADC10_L])
#define	GW_ADC11		WORD_CAST(gbpControlTable[P_ADC11_L])
#define	GW_ADC12		WORD_CAST(gbpControlTable[P_ADC12_L])
#define	GW_ADC13		WORD_CAST(gbpControlTable[P_ADC13_L])
#define	GW_ADC14		WORD_CAST(gbpControlTable[P_ADC14_L])
#define	GW_ADC15		WORD_CAST(gbpControlTable[P_ADC15_L])

#define HIGH_LIMIT 1
#define LOW_LIMIT 0

#define MAX_PACKET_LENGTH (256)

#define ROM_CONTROL_TABLE_LEN 24
#define RAM_CONTROL_TABLE_LEN 67

#define CONTROL_TABLE_LEN (ROM_CONTROL_TABLE_LEN + RAM_CONTROL_TABLE_LEN)

#define BROADCASTING_ID 0xfe

/* Exported functions ------------------------------------------------------- */
void ProcessInit(void);
void ProcessPackets(void);

/* External variables --------------------------------------------------------*/
extern volatile uint8_t gbpControlTable[];

#endif /* _DYNAMIXEL_CM_DXL_COM_HEADER */

/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/

