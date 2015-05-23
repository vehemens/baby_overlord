/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : usart.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : functions about usart control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "common_type.h"
#include "usart.h"
#include "system_init.h"
#include "led.h"
#include "CM_DXL_COM.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// usart buffer
/*
volatile u16 gwpUSART_DXL_Buffer[USART_BUFFER_SIZE+1];
volatile u16 gwUSART_DXL_ReadPtr, gwUSART_DXL_WritePtr;
*/
#if 0
volatile u16 gwpUSART_ZIGBEE_Buffer[USART_BUFFER_SIZE+1];
volatile u16 gwUSART_ZIGBEE_ReadPtr, gwUSART_ZIGBEE_WritePtr;
#endif

/*
volatile u16 gwpUSART_PC_Buffer[USART_BUFFER_SIZE+1];
volatile u16 gwUSART_PC_ReadPtr, gwUSART_PC_WritePtr;
*/
extern u8 gbDxlPwr;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



void BufferClear(u8 PORT)
{
#if 0
	if( PORT == USART_ZIGBEE ) // || PORT == USART_IR
	{
		gwUSART_ZIGBEE_ReadPtr = gwUSART_ZIGBEE_WritePtr = 0;
	}
#endif

}

u8 IsRXD_Ready(u8 PORT)
{
#if 0
	if( PORT == USART_ZIGBEE ) // || PORT == USART_IR
	{
		return (gwUSART_ZIGBEE_ReadPtr != gwUSART_ZIGBEE_WritePtr);
	}
#endif

	return -1;
}

u8 RxDBuffer(u8 PORT)
{

#if 0
	if( PORT == USART_ZIGBEE ) // || PORT == USART_IR
	{
		gwUSART_ZIGBEE_ReadPtr++;
		return (u8)(gwpUSART_ZIGBEE_Buffer[gwUSART_ZIGBEE_ReadPtr&USART_BUFFER_SIZE]);
	}
#endif

	return -1;
}

void TxDData(u8 PORT, u8 dat)
{
	if( PORT == USART_DXL )
	{
#if 0
		GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
		GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable
#endif
		GPIO_SetBits(PORT_DXL_DIR, PIN_DXL_DIR);	// TX Enable

		USART_SendData(DXL_USART,dat);		
		while( USART_GetFlagStatus(DXL_USART, USART_FLAG_TC)==RESET );

#if 0
		GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
		GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
#endif
		GPIO_ResetBits(PORT_DXL_DIR, PIN_DXL_DIR);	// TX Disable
	}
#if 0
	else if( PORT == USART_ZIGBEE )
	{
		USART_SendData(UART5,dat);
		while( USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET );
	}
#endif
	else if( PORT == USART_PC )
	{
		USART_SendData(PC_USART,dat);		
		while( USART_GetFlagStatus(PC_USART, USART_FLAG_TC)==RESET );
	}
}


#if 0
void __ISR_USART_ZIGBEE(void)
{
	u16 ReceivedData;
	
	//GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		ReceivedData = USART_ReceiveData(UART5);
		//USART_SendData(USART1,(u8)ReceivedData);
		gwpUSART_ZIGBEE_Buffer[(++gwUSART_ZIGBEE_WritePtr)&USART_BUFFER_SIZE] = ReceivedData;	
		//TxDHex8(ReceivedData);
	}
}
#endif


void __ISR_USART_PC(void)
{

	u16 ReceivedData;

	if(USART_GetITStatus(PC_USART, USART_IT_RXNE) != RESET)
	{
		// Read one byte from the receive data register
		ReceivedData = USART_ReceiveData(PC_USART); 

#if 0
		LED_SetState(LED_TX, ON);
#endif

		gbpTxD0Buffer[gbTxD0BufferWritePointer] = gbpRxInterruptBuffer[gbRxBufferWritePointer] =   gbpRxD1Buffer[gbRxD1BufferWritePointer] = ReceivedData;
		gbRxBufferWritePointer++;
		gbRxD1BufferWritePointer++;
		gbTxD0BufferWritePointer++;


		//if (TXD0_READY) {


					//if( GB_DYNAMIXEL_POWER)
#if 0
		if ( GPIO_ReadOutputDataBit(PORT_ENABLE_TXD, PIN_ENABLE_TXD) == Bit_RESET) {
			//if (TXD0_FINISH) {
			GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
			GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable
#endif
		if ( GPIO_ReadOutputDataBit(PORT_DXL_DIR, PIN_DXL_DIR) == Bit_RESET) {
			GPIO_SetBits(PORT_DXL_DIR, PIN_DXL_DIR);	// TX Enable

			USART_SendData(DXL_USART, gbpTxD0Buffer[gbTxD0BufferReadPointer++]);
			//gbTxD0BufferReadPointer &= 0x3FF;
			USART_ITConfig(DXL_USART, USART_IT_TC, ENABLE);

		}

	}
	else if(USART_GetITStatus(PC_USART, USART_IT_TC) != RESET)
	{
		if (gbTxD1BufferReadPointer!=gbTxD1BufferWritePointer) {
			USART_SendData(PC_USART, gbpTxD1Buffer[gbTxD1BufferReadPointer++]);
			//gbTxD1BufferReadPointer&= 0x3FF;
		}
		else {
			//LED_SetState(LED_RX, OFF);
			gbTxD1Transmitting = 0;
			USART_ITConfig(PC_USART, USART_IT_TC, DISABLE);
		}
	}

}


void __ISR_USART_DXL(void)
{


	u16 ReceivedData;



	//GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	if(USART_GetITStatus(DXL_USART, USART_IT_RXNE) != RESET)
	{
		// Read one byte from the receive data register
		ReceivedData = USART_ReceiveData(DXL_USART);

		//if( DxlPwr == Bit_RESET ) return ;

		//USART_SendData(USART1,(u8)ReceivedData);



		gbpTxD1Buffer[gbTxD1BufferWritePointer] = gbpRxD0Buffer[gbRxD0BufferWritePointer] = ReceivedData;
		gbRxD0BufferWritePointer++;
		gbTxD1BufferWritePointer++;

		//gbRxD0BufferWritePointer &= 0x3FF;
		//gbTxD1BufferWritePointer &= 0x3FF;




		//if( DxlPwr == Bit_SET )
		//{
		//}

		if (gbTxD1Transmitting==0) {
			gbTxD1Transmitting = 1;
			if( gbDxlPwr == Bit_SET )
			{
				USART_SendData(PC_USART, gbpTxD1Buffer[gbTxD1BufferReadPointer]);
#if 0
				LED_SetState(LED_RX, ON);
#endif
			}
			gbTxD1BufferReadPointer++;
			//gbTxD1BufferReadPointer &= 0x3FF;

			USART_ITConfig(PC_USART, USART_IT_TC, ENABLE);
		}

		//if (TXD1_READY) {


	}




	else if(USART_GetITStatus(DXL_USART, USART_IT_TC) != RESET)
	{
		if (gbTxD0BufferReadPointer!=gbTxD0BufferWritePointer) {
			USART_SendData(DXL_USART, gbpTxD0Buffer[gbTxD0BufferReadPointer++]);
			//gbTxD0BufferReadPointer &= 0x3FF;

			//TxDHex8(data);
			//TxDString(USART_ZIGBEE,"\r\n");
		//    TXD0_FINISH = 1;
		}
		else {
#if 0
			GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
			GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
#endif
			GPIO_ResetBits(PORT_DXL_DIR, PIN_DXL_DIR);	// TX Disable
			//LED_SetState(LED_TX, OFF);
			//gbTxD0Transmitting = 0;
			USART_ITConfig(DXL_USART, USART_IT_TC, DISABLE);
		}
	}

}


/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
