/******************************************************************************
* File Name          : CM_DXL_COM.c
* Author             : X
* Version            : X
* Date               : X
* Description        : This file contains the inteligent toss mode between
                       host and dynamixel.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "hw_config.h"
#include "CM_DXL_COM.h"
#include "dynamixel.h"
#include "led.h"

/* Private define ------------------------------------------------------------*/
#define TIMEOUT_CHECK
#define TIMEOUT_MILISEC 100

#define BROADCAST_PING_DISABLE

#define VOLTAGE_ERROR_BIT     0x01
#define ANGLE_LIMIT_ERROR_BIT 0x02
#define OVERHEATING_ERROR_BIT 0x04
#define RANGE_ERROR_BIT       0x08
#define CHECKSUM_ERROR_BIT    0x10
#define OVERLOAD_ERROR_BIT    0x20
#define INSTRUCTION_ERROR_BIT 0x40

#define RETURN_NO_PACKET 0
#define RETURN_READ_PACKET 1
#define RETURN_ALL_PACKET 2

/* Private macro -------------------------------------------------------------*/
#define SYSTEM_RESET NVIC_GenerateSystemReset()

/* Private variables ---------------------------------------------------------*/
uint8_t gbpParameterRange[][2] =
{
  //EEPROM area
  {1, 0},	//P_MODEL_NUMBER_L	0
  {1, 0},	//P_MODEL_NUMBER_H	1
  {1, 0},	//P_VERSION		2
  {0, 253},	//P_ID			3
  {1, 254},	//P_BAUD_RATE		4
  {0, 254},	//P_RETURN_DELAY_TIME	5
  {1, 0},	//			6
  {1, 0},	//			7
  {1, 0},	//			8
  {1, 0},	//			9
  {1, 0},	//			10
  {1, 0},	//			11
  {1, 0},	//			12
  {1, 0},	//			13
  {1, 0},	//			14
  {1, 0},	//			15
  {0, 2},	//P_RETURN_LEVEL	16
  {1, 0},	//			17
  {1, 0},	//			18
  {1, 0},	//			19
  {1, 0},	//			20
  {1, 0},	//			21
  {1, 0},	//			22
  {1, 0},	//			23

  //RAM area
  {0, 1},	//P_DYNAMIXEL_POWER	24
  {0, 7},	//P_LED_PANNEL		25
  {0, 255},	//P_LED_HEAD_L		26
  {0, 127},	//P_LED_HEAD_H		27
  {0, 255},	//P_LED_EYE_L		28
  {0, 127},	//P_LED_EYE_H		29
  {1, 0},	//P_BUTTON		30
  {1, 0},	//			31
  {1, 0},	//			32
  {1, 0},	//			33
  {1, 0},	//			34
  {1, 0},	//			35
  {1, 0},	//			36
  {1, 0},	//			37
  {1, 0},	//P_GYRO_Z_L		38
  {1, 0},	//P_GYRO_Z_H		39
  {1, 0},	//P_GYRO_Y_L		40
  {1, 0},	//P_GYRO_Y_H		41
  {1, 0},	//P_GYRO_X_L		42
  {1, 0},	//P_GYRO_X_H		43
  {1, 0},	//P_ACC_X_L		44
  {1, 0},	//P_ACC_X_H		45
  {1, 0},	//P_ACC_Y_L		46
  {1, 0},	//P_ACC_Y_H		47
  {1, 0},	//P_ACC_Z_L		48
  {1, 0},	//P_ACC_Z_H		49
  {1, 0},	//P_ADC0_VOLTAGE	50
  {1, 0},	//P_ADC1_MIC1_L		51
  {1, 0},	//			52
  {1, 0},	//P_ADC2_L		53
  {1, 0},	//			54
  {1, 0},	//P_ADC3_L		55
  {1, 0},	//			56
  {1, 0},	//P_ADC4_L		57
  {1, 0},	//			58
  {1, 0},	//P_ADC5_L		59
  {1, 0},	//			60
  {1, 0},	//P_ADC6_L		61
  {1, 0},	//			62
  {1, 0},	//P_ADC7_L		63
  {1, 0},	//			64
  {1, 0},	//P_ADC8_L		65
  {1, 0},	//			66
  {1, 0},	//P_ADC9_MIC2_L		67
  {1, 0},	//			68
  {1, 0},	//P_ADC10_L		69
  {1, 0},	//			70
  {1, 0},	//P_ADC11_L		71
  {1, 0},	//			72
  {1, 0},	//P_ADC12_L		73
  {1, 0},	//			74
  {1, 0},	//P_ADC13_L		75
  {1, 0},	//			76
  {1, 0},	//P_ADC14_L		77
  {1, 0},	//			78
  {1, 0},	//P_ADC15_L		79
  {1, 0},	//			80
};

uint8_t gbpDataSize[] =
{
  //EEPROM area
  2,		//P_MODEL_NUMBER_L	0
  0,		//P_MODEL_NUMBER_H	1
  1,		//P_VERSION		2
  1,		//P_ID			3
  1,		//P_BAUD_RATE		4
  1,		//P_RETURN_DELAY_TIME	5
  1,		//			6
  1,		//			7
  1,		//			8
  1,		//			9
  1,		//			10
  1,		//			11
  1,		//			12
  1,		//			13
  1,		//			14
  1,		//			15
  1,		//P_RETURN_LEVEL	16
  1,		//			17
  1,		//			18
  1,		//			19
  1,		//			20
  1,		//			21
  1,		//			22
  1,		//			23

  //RAM area
  1,		//P_DYNAMIXEL_POWER	24
  1,		//P_LED_PANNEL		25
  2,		//P_LED_HEAD_L		26
  0,		//P_LED_HEAD_H		27
  2,		//P_LED_EYE_L		28
  0,		//P_LED_EYE_H		29
  1,		//P_BUTTON		30
  1,		//			31
  1,		//			32
  1,		//			33
  1,		//			34
  1,		//			35
  1,		//			36
  1,		//			37
  2,		//P_GYRO_Z_L		38
  0,		//P_GYRO_Z_H		39
  2,		//P_GYRO_Y_L		40
  0,		//P_GYRO_Y_H		41
  2,		//P_GYRO_X_L		42
  0,		//P_GYRO_X_H		43
  2,		//P_ACC_X_L		44
  0,		//P_ACC_X_H		45
  2,		//P_ACC_Y_L		46
  0,		//P_ACC_Y_H		47
  2,		//P_ACC_Z_L		48
  0,		//P_ACC_Z_H		49
  1,		//P_ADC0_VOLTAGE	50
  2,		//P_ADC1_MIC1_L		51
  0,		//			52
  2,		//P_ADC2_L		53
  0,		//			54
  2,		//P_ADC3_L		55
  0,		//			56
  2,		//P_ADC4_L		57
  0,		//			58
  2,		//P_ADC5_L		59
  0,		//			60
  2,		//P_ADC6_L		61
  0,		//			62
  2,		//P_ADC7_L		63
  0,		//			64
  2,		//P_ADC8_L		65
  0,		//			66
  2,		//P_ADC9_MIC2_L		67
  0,		//			68
  2,		//P_ADC10_L		69
  0,		//			70
  2,		//P_ADC11_L		71
  0,		//			72
  2,		//P_ADC12_L		73
  0,		//			74
  2,		//P_ADC13_L		75
  0,		//			76
  2,		//P_ADC14_L		77
  0,		//			78
  2,		//P_ADC15_L		79
  0,		//			80
};

uint8_t ROM_INITIAL_DATA[] =
{
  0,		//P_MODEL_NUMBER_L	0
  0x73,		//P_MODEL_NUMBER_H	1
  0x13,		//P_VERSION		2
  200,		//P_ID			3
  1,		//P_BAUD_RATE		4
  0,		//P_RETURN_DELAY_TIME	5
  0,		//			6
  0,		//			7
  0,		//			8
  0,		//			9
  0,		//			10
  0,		//			11
  0,		//			12
  0,		//			13
  0,		//			14
  0,		//			15
  2,		//P_RETURN_LEVEL	16
  0,		//			17
  0,		//			18
  0,		//			19
  0,		//			20
  0,		//			21
  0,		//			22
  0		//			23
};

volatile uint8_t gbpRxCmBuffer[256];
volatile uint8_t gbpTxCmBuffer[256];
volatile uint8_t gbRxCmBufferWritePointer;
volatile uint8_t gbRxCmBufferReadPointer;
volatile uint8_t gbTxCmBufferWritePointer;
volatile uint8_t gbTxCmBufferReadPointer;

volatile uint8_t gbpRxD0Buffer[256];
volatile uint8_t gbpTxD0Buffer[256];
volatile uint8_t gbRxD0BufferWritePointer;
volatile uint8_t gbRxD0BufferReadPointer;
volatile uint8_t gbTxD0BufferWritePointer;
volatile uint8_t gbTxD0BufferReadPointer;

volatile uint8_t gbMiliSec;

uint8_t gbRxID;

uint8_t gbParameterLength;

uint8_t gbInstruction;

volatile uint8_t gbpParameter[256];

uint8_t gbStartAddress;

uint8_t gbInterruptCheckError;

volatile uint8_t gbpControlTable[CONTROL_TABLE_LEN + 1];

volatile uint8_t gbLEDHeadR;
volatile uint8_t gbLEDHeadG;
volatile uint8_t gbLEDHeadB;

volatile uint8_t gbLEDEyeR;
volatile uint8_t gbLEDEyeG;
volatile uint8_t gbLEDEyeB;

/* Private function prototypes -----------------------------------------------*/
void ProcessInstruction(uint8_t length);
void WriteControlTable(void);
uint8_t WriteControlTableRangeCheck(void);
void ReturnPacket(uint8_t bError);
void ProcessAfterWriting(void);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : ProcessInit
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
void ProcessInit(void)
{
  uint8_t bCount;

  for (bCount = 0; bCount < ROM_CONTROL_TABLE_LEN; bCount++)
  {
    gbpControlTable[bCount] = ROM_INITIAL_DATA[bCount];
  }

  gbRxCmBufferWritePointer = 0;
  gbRxCmBufferReadPointer = 0;
  gbTxCmBufferWritePointer = 0;
  gbTxCmBufferReadPointer = 0;

  gbRxD0BufferWritePointer = 0;
  gbRxD0BufferReadPointer = 0;
  gbTxD0BufferWritePointer = 0;
  gbTxD0BufferReadPointer = 0;

  gbInterruptCheckError = 0;

  GW_LED_HEAD = ((0 >> 3) << 10) | ((255 >> 3) << 5) | (0 >> 3);
  GW_LED_EYE =  ((255 >> 3) << 10) | ((0 >> 3) << 5) | (0 >> 3);

  gbLEDHeadR = GW_LED_HEAD & 0x1f;
  gbLEDHeadG = (GW_LED_HEAD >> 5) & 0x1f;
  gbLEDHeadB = (GW_LED_HEAD >> 10) & 0x1f;

  gbLEDEyeR = GW_LED_EYE & 0x1f;
  gbLEDEyeG = (GW_LED_EYE >> 5) & 0x1f;
  gbLEDEyeB = (GW_LED_EYE >> 10) & 0x1f;
}

/*******************************************************************************
* Function Name  : ProcessPackets
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
void ProcessPackets(void)
{
  uint8_t bCount, bLength, bCount0xff, bCheckSum;

  while (1)
  {
RX_PACKET_START:
RX_PACKET_TIMEOUT:

    bCount0xff = 0;
    while (1)
    {
      #ifdef TIMEOUT_CHECK
      gbMiliSec = 0;
      #endif
      while (gbRxCmBufferReadPointer == gbRxCmBufferWritePointer)
      {
        #ifdef TIMEOUT_CHECK
        if (gbMiliSec > TIMEOUT_MILISEC)
        {
          goto RX_PACKET_TIMEOUT;
        }
        #endif
      }

      if ((gbRxID = gbpRxCmBuffer[gbRxCmBufferReadPointer++]) == 0xff)
      {
        bCount0xff++;
      }
      else
      {
        if (bCount0xff >= 2)
        {
          break;
        }
        bCount0xff = 0;
      }
    }

    if (gbRxID == GB_ID || gbRxID == BROADCASTING_ID)
    {
      #ifdef TIMEOUT_CHECK
      gbMiliSec = 0;
      #endif
      while (gbRxCmBufferReadPointer == gbRxCmBufferWritePointer)
      {
        #ifdef TIMEOUT_CHECK
        if (gbMiliSec > TIMEOUT_MILISEC)
        {
          goto RX_PACKET_TIMEOUT;
        }
        #endif
      }
      bLength = gbpRxCmBuffer[gbRxCmBufferReadPointer++];

      gbParameterLength = bLength - 2;
      if (gbParameterLength > MAX_PACKET_LENGTH)
      {
        goto RX_PACKET_START;
      }

      //from this state, status packet will be returned

      #ifdef TIMEOUT_CHECK
      gbMiliSec = 0;
      #endif
      while (gbRxCmBufferReadPointer == gbRxCmBufferWritePointer)
      {
        #ifdef TIMEOUT_CHECK
        if (gbMiliSec > TIMEOUT_MILISEC)
        {
          goto RX_PACKET_TIMEOUT;
        }
        #endif
      }
      gbInstruction = gbpRxCmBuffer[gbRxCmBufferReadPointer++];
      bCheckSum = gbRxID + bLength + gbInstruction;

      for (bCount = 0; bCount < gbParameterLength + 1; bCount++)
      {
        #ifdef TIMEOUT_CHECK
        gbMiliSec = 0;
        #endif
        while (gbRxCmBufferReadPointer == gbRxCmBufferWritePointer)
        {
          #ifdef TIMEOUT_CHECK
          if (gbMiliSec > TIMEOUT_MILISEC)
          {
            goto RX_PACKET_TIMEOUT;
          }
          #endif
        }
        bCheckSum += (gbpParameter[bCount] = gbpRxCmBuffer[gbRxCmBufferReadPointer++]);
      }
      //Packet Receiving End

      gbStartAddress = gbpParameter[0];

      for (bCount = 0; bCount < GB_RETURN_DELAY_TIME; bCount++)
      {
        // __delay_cycles(32);//2usec
        //uDelay(2);
      }
      if (bCheckSum != 0xff)
      {
        //buffer initialize
        gbRxCmBufferWritePointer = gbRxCmBufferReadPointer;

        if (gbInstruction == INST_PING) {
        }
        else {
          ReturnPacket(CHECKSUM_ERROR_BIT);
        }
      }
      else
      {
        ProcessInstruction(bLength);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : ProcessInstruction
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
void ProcessInstruction(uint8_t length)
{
  uint8_t bCount, bLength, bEndAddress, bCount0xff, bCheckSum, bReturn, bPrevID;

  bLength = length;

  if (gbInstruction == INST_BULK_READ) //INST_SYNC_WR only 2009.12.11.buche
  {
    gbRxD0BufferWritePointer = gbRxD0BufferReadPointer = 0;

    bPrevID = 0xFF;
    for (bCount = 2; bCount < bLength - 3; bCount += 3)
    {
      if (gbpParameter[bCount] == GB_ID)
      {
        gbRxID = GB_ID;
        gbInstruction = INST_READ;
        bLength = 4;
        gbStartAddress = gbpParameter[bCount + 1];
        gbpParameter[1] = gbpParameter[bCount - 1];

        // waiting
        if (bPrevID == 0xFF)
        {
          break;
        }
        else
        {
          while (1)
          {
            //RX_PACKET_START:
            //RX_PACKET_TIMEOUT:

            uint8_t bWaitRxID, bWaitLength, bWaitParameterLength, bWaitInstruction, bWaitCheckSum;

            bCount0xff = 0;

            while (1)
            {
              gbMiliSec = 0;
              while (gbRxD0BufferWritePointer == gbRxD0BufferReadPointer)
              {
                if (gbMiliSec > TIMEOUT_MILISEC)
                {
                  goto RX_PACKET_TIMEOUT;
                }
              }

              if ((bWaitRxID = gbpRxD0Buffer[gbRxD0BufferReadPointer++]) == 0xff)
              {
                bCount0xff++;
              }
              else
              {
                if (bCount0xff >= 2)
                {
                  break;
                }
                bCount0xff = 0;
              }
            }

            if (bWaitRxID == bPrevID)
            {
              gbMiliSec = 0;
              while (gbRxD0BufferWritePointer == gbRxD0BufferReadPointer)
              {
                if (gbMiliSec > TIMEOUT_MILISEC)
                {
                  goto RX_PACKET_TIMEOUT;
                }
              }

              bWaitLength = gbpRxD0Buffer[gbRxD0BufferReadPointer++];

              bWaitParameterLength = bWaitLength - 2;
              if (bWaitParameterLength > MAX_PACKET_LENGTH)
              {
                goto RX_PACKET_START; //Ver8
              }

              //from this state, status packet will be returned

              gbMiliSec = 0;
              while (gbRxD0BufferWritePointer == gbRxD0BufferReadPointer)
              {
                if (gbMiliSec > TIMEOUT_MILISEC)
                {
                  goto RX_PACKET_TIMEOUT;
                }
              }

              bWaitInstruction = gbpRxD0Buffer[gbRxD0BufferReadPointer++];
              bWaitCheckSum = bWaitRxID + bWaitLength + bWaitInstruction;

              for (bCount = 0; bCount < bWaitParameterLength + 1; bCount++)
              {
                gbMiliSec = 0;
                while (gbRxD0BufferWritePointer == gbRxD0BufferReadPointer)
                {
                  if (gbMiliSec > TIMEOUT_MILISEC)
                  {
                    goto RX_PACKET_TIMEOUT;
                  }
                }
                bWaitCheckSum += (gbpRxD0Buffer[gbRxD0BufferReadPointer++]);
              }

              for (bCount = 0; bCount < GB_RETURN_DELAY_TIME; bCount++)
              {
                //uDelay(2);
                //__delay_cycles(32);//2usec
#ifdef  PLL_CLOCK_48MHZ
                DelayCycle(16);
#endif
#ifdef  PLL_CLOCK_32MHZ
                DelayCycle(10);
#endif
              }
              break;
            }
          }
        }
      }

      bPrevID = gbpParameter[bCount];
    }
  }

  if (gbInstruction == INST_SYNC_WRITE || gbInstruction == INST_SYNC_REG_WRITE)
  {
    uint8_t bTmpLength, bCount0;
    bTmpLength = gbpParameter[1];
    for (bCount = 2; bCount < bLength - 3; bCount += (bTmpLength + 1))
    {
      if (gbpParameter[bCount] == GB_ID)
      {
        bCount++; //point to Data_N
        for (bCount0 = 1; bCount0 <= bTmpLength; bCount0++)
        {
          gbpParameter[bCount0] = gbpParameter[bCount++];
        }
        gbInstruction &= 0x7f; //change to INST_WRITE or INST_REG_WRITE
        bLength = bTmpLength + 3;
        gbParameterLength = bLength - 2;
        break;
      }
    }
  }

  //else
  if (gbInstruction == INST_WRITE)
  {
    bReturn = WriteControlTableRangeCheck();
    ReturnPacket(bReturn);
    if (bReturn != RANGE_ERROR_BIT)
    {
      WriteControlTable();
      ProcessAfterWriting();
    }
  }

  else if (gbInstruction == INST_READ)
  {
    if (gbRxID != BROADCASTING_ID && GB_RETURN_LEVEL >= RETURN_READ_PACKET)
    {
      bEndAddress = gbStartAddress + gbpParameter[1] - 1;

      bLength = gbpParameter[1] + 2; //Errorstatus,Checksum

      bCheckSum = GB_ID + bLength + gbInterruptCheckError;

      gbpTxCmBuffer[gbTxCmBufferWritePointer++] = 0xff;
      gbpTxCmBuffer[gbTxCmBufferWritePointer++] = 0xff;
      gbpTxCmBuffer[gbTxCmBufferWritePointer++] = GB_ID;
      gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bLength;
      gbpTxCmBuffer[gbTxCmBufferWritePointer++] = gbInterruptCheckError;

      gbpTxD0Buffer[gbTxD0BufferWritePointer++]= 0xff;
      gbpTxD0Buffer[gbTxD0BufferWritePointer++]= 0xff;
      gbpTxD0Buffer[gbTxD0BufferWritePointer++]= GB_ID;
      gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bLength;
      gbpTxD0Buffer[gbTxD0BufferWritePointer++]= gbInterruptCheckError;

      for (bCount = gbStartAddress; bCount <= bEndAddress; bCount++)
      {
        uint8_t bFixedData;
        uint16_t wFixedData;

        if (gbpDataSize[bCount] == 2 && bCount < bEndAddress)
        {
          wFixedData = WORD_CAST(gbpControlTable[bCount]);

          bFixedData = (uint8_t)(wFixedData & 0xff);

          gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bFixedData;
          gbpTxD0Buffer[gbTxD0BufferWritePointer++] = bFixedData;

          bCheckSum += bFixedData;

          bFixedData = (uint8_t)((wFixedData >> 8) & 0xff);

          gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bFixedData;
          gbpTxD0Buffer[gbTxD0BufferWritePointer++] = bFixedData;

          bCheckSum += bFixedData;

          bCount++;
        }
        else //length == 1 or 0
        {
          bFixedData = gbpControlTable[bCount];

          gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bFixedData;
          gbpTxD0Buffer[gbTxD0BufferWritePointer++] = bFixedData;

          bCheckSum += bFixedData;
        }
      }
      bCheckSum ^= 0xff;

      gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bCheckSum;
      gbpTxD0Buffer[gbTxD0BufferWritePointer++] = bCheckSum;

      CNTR_To_USB_Send_Data();
    }
  }

  else if (gbInstruction == INST_SYSTEM_WRITE)
  { //[Addr] [Data] [0xf0] [0x55] [0x0f] [0xaa]
    if ((gbpParameter[2] == 0xf0) && (gbpParameter[3] == 0x55) &&
      (gbpParameter[4] == 0x0f) && (gbpParameter[5] == 0xaa) &&
      (gbParameterLength == 6))
    {
      if (gbStartAddress < CONTROL_TABLE_LEN)
      {
        BKP_WriteBackupRegister((gbStartAddress + 1) << 2, gbpParameter[1]);
      }
    }
  }

  else if (gbInstruction == INST_PING)
  {
    if (gbRxID == BROADCASTING_ID) //for avoiding data crush
    {
      //mDelay((uint16_t)(GB_ID << 0)); //Ver0x14
    }
    ReturnPacket(0);
  }

  else if (gbInstruction == INST_RESET)
  {
    ReturnPacket(0);
    //SYSTEM_RESET;
  }

  else if (gbInstruction == INST_DIGITAL_RESET)
  {
    ReturnPacket(0);
    //SYSTEM_RESET;
  }

  else
  {
    ReturnPacket(INSTRUCTION_ERROR_BIT);
  }

RX_PACKET_START:
RX_PACKET_TIMEOUT:

  return;
}

/*******************************************************************************
* Function Name  : WriteControlTable
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
void WriteControlTable(void)
{
  uint8_t bCount, bPointer;

  for (bCount = 1; bCount < gbParameterLength; bCount++) //Writing
  {
    bPointer = gbStartAddress + bCount - 1;

    if (gbpDataSize[bPointer] == 2) //&& bCount < gbParameterLength-2) //length was already checked.
    {
      WORD_CAST(gbpControlTable[bPointer]) = WORD_CAST(gbpParameter[bCount]);

      if (bPointer < ROM_CONTROL_TABLE_LEN)
      {
        BKP_WriteBackupRegister((bPointer + 1) << 2, WORD_CAST(gbpParameter[bCount]));
      }
      bCount++;
    }
    else //if (gbpDataSize[bPointer] == 1)//length was already checked.
    {
      gbpControlTable[bPointer] = gbpParameter[bCount];

      if (bPointer < ROM_CONTROL_TABLE_LEN)
      {
        BKP_WriteBackupRegister((bPointer + 1) << 2, WORD_CAST(gbpParameter[bCount]));
      }
    }
  }
}

/*******************************************************************************
* Function Name  : WriteControlTableRangeCheck
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
uint8_t WriteControlTableRangeCheck(void)
{
  uint8_t bCount, bPointer;

  if (gbpDataSize[gbStartAddress] == 0 || gbpDataSize[gbStartAddress + gbParameterLength - 2] == 2)
  {
    return RANGE_ERROR_BIT;
  }

  for (bCount = 1; bCount < gbParameterLength; bCount++) //Range Check
  {
    bPointer = gbStartAddress + bCount - 1;

    if (bPointer > CONTROL_TABLE_LEN ||
      gbpParameterRange[bPointer][LOW_LIMIT] > gbpParameter[bCount] ||
      gbpParameter[bCount] > gbpParameterRange[bPointer][HIGH_LIMIT])
    {
      return RANGE_ERROR_BIT;
    }
  }

  return 0;
}

/*******************************************************************************
* Function Name  : ReturnPacket
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
void ReturnPacket(uint8_t bError)
{
  uint8_t bCheckSum;

  if (gbInstruction == INST_PING || (gbRxID != BROADCASTING_ID && GB_RETURN_LEVEL >= RETURN_ALL_PACKET))
  {
    bError |= gbInterruptCheckError;

    bCheckSum = ~(GB_ID + 2 + bError);

    gbpTxCmBuffer[gbTxCmBufferWritePointer++] = 0xff;
    gbpTxCmBuffer[gbTxCmBufferWritePointer++] = 0xff;
    gbpTxCmBuffer[gbTxCmBufferWritePointer++] = GB_ID;
    gbpTxCmBuffer[gbTxCmBufferWritePointer++] = 2;
    gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bError;
    gbpTxCmBuffer[gbTxCmBufferWritePointer++] = bCheckSum;

    CNTR_To_USB_Send_Data();
  }
}

/*******************************************************************************
* Function Name  : ProcessAfterWriting
* Description    : XX
* Input          : None.
* Return         : None.
*******************************************************************************/
void ProcessAfterWriting(void)
{
  uint8_t bCount;

  for (bCount = 0; bCount < gbParameterLength - 1; bCount++) //Range Check
  {
    switch (gbStartAddress + bCount)
    {
    case P_LED_PANNEL:
      LED_SetState(GB_LED_MODE, ON);
      LED_SetState(~GB_LED_MODE, OFF);
      break;

    case P_LED_HEAD_L:
      gbLEDHeadR = GW_LED_HEAD & 0x1f;
      gbLEDHeadG = (GW_LED_HEAD >> 5) & 0x1f;
      gbLEDHeadB = (GW_LED_HEAD >> 10) & 0x1f;
      break;

    case P_LED_EYE_L:
      gbLEDEyeR = GW_LED_EYE & 0x1f;
      gbLEDEyeG = (GW_LED_EYE >> 5) & 0x1f;
      gbLEDEyeB = (GW_LED_EYE >> 10) & 0x1f;
      break;

    default:
      break;
    }
  }
}
