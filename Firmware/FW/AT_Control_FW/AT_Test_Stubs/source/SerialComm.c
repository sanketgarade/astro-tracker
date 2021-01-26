/************************ Includes ************************/
#include <avr/interrupt.h>
#include "C:\Users\Vedashree\Documents\AVRStudio\AT_Control_FW\AT_Test_Stubs\include/Common.h"

/************************ Global variables ************************/

volatile uint8_t gu8SerialDataLatestRx = 1;

uint8_t gau8RxBuffer[16];
uint8_t gau8TxBuffer[16];
uint8_t gu8RxReadPtr			= 0;
uint8_t gu8RxWritePtr			= 0;
uint8_t gu8TxPtr				= 0;
uint8_t gu8TxPtrTarget			= 0;

uint8_t gu8SendQuadrantResp		= FALSE;
uint8_t gu8SerialTxDone			= TRUE;

/********************** File static variables ***********************/
static uint8_t fsu8RxFrameID	= 0x00;
static uint8_t fsu8RxDLC		= 0;
static uint8_t fsau8RxDataBuff[4];
static uint8_t fsu8RxChkSumCalc	= 0x00;

static RX_FRAME_READ_STATES fseRxFrameReadState = RX_SOF_WAIT;

static uint8_t fsu8TxFrameID	= 0x00;
static uint8_t fsu8TxDLC		= 0;
static uint8_t fsu8TxChkSumAdd	= 0x00;


/************************ Function declarations ************************/
void rvReadRequest(void);
void rvProcessRequest(void);
void rvClearRxData();

void rvSendResponse(void);
uint8_t ru8PrepareTxFrame(uint8_t);
/************************ Function definitions ************************/

void rvReadRequest(void)
{
	uint8_t lu8ReadPending = FALSE;
	uint8_t lu8ReadByte = 0x00;
	uint8_t lu8RxChkSum = 0x00;
	
	
	// If latest Rx data not yet read.
	if(gu8RxWritePtr > gu8RxReadPtr)
	{
		lu8ReadByte = gau8RxBuffer[gu8RxReadPtr];
		gu8RxReadPtr++;
		
		lu8ReadPending = TRUE;
	}
	
	// if read is pending
	if(TRUE == lu8ReadPending)
	{
		// If valid SOF detected 
		if((RX_SOF_WAIT == fseRxFrameReadState) && (COMM_SOF == lu8ReadByte))
		{
			fseRxFrameReadState = RX_SOF_READ;
			
			fsu8RxChkSumCalc = lu8ReadByte;
		}
		// If valid Rx Frame No detected 		
		else if((RX_SOF_READ == fseRxFrameReadState) && ((lu8ReadByte > 0x10) && (lu8ReadByte < 0x19)) && (0x00 == fsu8RxFrameID))
		{
			fsu8RxFrameID = lu8ReadByte;
			fsu8RxChkSumCalc += lu8ReadByte;
			
			fseRxFrameReadState = RX_FRAME_NO_READ;
		}
		// If valid DLC detected 
		else if((RX_FRAME_NO_READ == fseRxFrameReadState) && ((lu8ReadByte > 0) && (lu8ReadByte <= 4)))
		{
			fsu8RxDLC = lu8ReadByte;
			fsu8RxChkSumCalc += lu8ReadByte;
			
			fseRxFrameReadState = RX_DLC_READ;
		}
		// Read Rx data bytes
		else if(RX_DLC_READ == fseRxFrameReadState)
		{
			if(fsu8RxDLC > 0)
			{
				fsau8RxDataBuff[fsu8RxDLC - 1] = lu8ReadByte;
				fsu8RxChkSumCalc += lu8ReadByte;
				
				fsu8RxDLC--;
				
				// If Rx data bytes completely read
				if(0 == fsu8RxDLC)
				{
					fseRxFrameReadState = RX_DATA_READ;
				}
			}
		}
		// Read checksum
		else if(RX_DATA_READ == fseRxFrameReadState)
		{
			lu8RxChkSum = lu8ReadByte;
			
			// verify checksum
			if(fsu8RxChkSumCalc == lu8RxChkSum)
			{
				fseRxFrameReadState = RX_CHKSUM_READ;
			}
		}
		// if valid EOF is detected		
		else if((RX_CHKSUM_READ == fseRxFrameReadState) && (COMM_EOF == lu8ReadByte))
		{
			gu8RxReadPtr = 0;
			gu8RxWritePtr = 0;

			fseRxFrameReadState = RX_EOF_READ;
		}
		// invalid data
		else
		{
			// reset frame reading
			rvClearRxData();

		}
		
	}
	
	
	// process data from the valid frame
	if(RX_EOF_READ == fseRxFrameReadState)
	{
		rvProcessRequest();				// Extract Information from Request frames
		rvClearRxData();				// Clear Rx data for next reception
	}
	
}


void rvProcessRequest(void)
{
	// process as per Frame ID 
	switch(fsu8RxFrameID)
	{
		case REQ_START_STOP	:	// Request - Start Stop 
						gu8ReqStart = fsau8RxDataBuff[0];
							
						LED1_ON;
						LED2_OFF;

						break;
			
		case REQ_SPEED	:		// Request - Speed
						gu8ReqSpeedRef = fsau8RxDataBuff[0];
							
						LED1_OFF;
						LED2_ON;

						break;
							
		case REQ_MODE	:		// Request - Mode
						gu8ReqMode = fsau8RxDataBuff[0];

						break;
							
		case REQ_DIRECTION	:	// Request - Direction
						gu8ReqDirectionH = fsau8RxDataBuff[0] & 0x0F;
						gu8ReqDirectionV = (fsau8RxDataBuff[0] & 0xF0) >> 4;

						break;

		case REQ_PULSE_COUNT :	// Request - Pulse Count
						gu16ReqPulseTargetH = fsau8RxDataBuff[0] | ((uint16_t)fsau8RxDataBuff[1] << 8);
						gu16ReqPulseTargetV = fsau8RxDataBuff[2] | ((uint16_t)fsau8RxDataBuff[3] << 8);

						break;

		case REQ_ZERO	:		// Request - Zero Set
						gu8ReqZeroSet = fsau8RxDataBuff[0];

						break;
							
		case REQ_QUADRANT :		// Request - Quadrant Set
						gu8ReqQuadrantSet = fsau8RxDataBuff[0];

						gu8SendQuadrantResp = TRUE;				// Set response request for quadrant feedback
						
						break;

		case REQ_DEGREES :	// Request - Degrees Count
						gu16ReqDegreesSetH = fsau8RxDataBuff[0] | ((uint16_t)fsau8RxDataBuff[1] << 8);
						gu16ReqDegreesSetV = fsau8RxDataBuff[2] | ((uint16_t)fsau8RxDataBuff[3] << 8);

						break;

		default		:	break;
	}

}

void rvClearRxData(void)
{
	
	gu8RxReadPtr = 0;
	gu8RxWritePtr = 0;

	fseRxFrameReadState = RX_SOF_WAIT;

	fsu8RxChkSumCalc = 0x00;
	fsu8RxDLC = 0x00;
	fsu8RxFrameID = 0x00;
}

void rvSendResponse(void)
{
	static uint8_t lsu8NextTxFrame = RESP_SYS_STATUS;
	static uint32_t lsu32SendTimer = 0;
	
	
	
	
	// Every Tx cycle - 200 msec 
	if(gu32TimeTicks - lsu32SendTimer >= T_200MS)
	{
		// load timer for next count
		lsu32SendTimer = gu32TimeTicks;
		
		// If previous Tx was complete
		if(TRUE == gu8SerialTxDone)
		{
			// If Quadrant response was requested
			if(TRUE == gu8SendQuadrantResp)
			{
				// clear request
				gu8SendQuadrantResp = FALSE;

				lsu8NextTxFrame = RESP_QUADRANT;					// Select Quadrant frame
			}
			else if(RESP_SYS_STATUS == lsu8NextTxFrame)
			{
				if(MODE_NORMAL == gu8ReqMode)
				{
					lsu8NextTxFrame = RESP_PULSE_COUNT;				// Select Pulse elapsed frame
				}
				else if(MODE_SCAN == gu8ReqMode)					
				{
					lsu8NextTxFrame = RESP_DEGREES;					// Select degrees elapsed frame
				}					
			}
			else if((RESP_DEGREES == lsu8NextTxFrame) ||
					(RESP_PULSE_COUNT == lsu8NextTxFrame))
			{
				lsu8NextTxFrame = RESP_SYS_STATUS;					// Select System status frame
			}
			else if(RESP_QUADRANT == lsu8NextTxFrame)
			{
				lsu8NextTxFrame = RESP_SYS_STATUS;					// Select System status frame
			}
	
			// Indicate serial Tx request is pending
			gu8SerialTxDone = FALSE;					
			
			gu8TxPtr = 0;
			
			// Prepare frame data as per selected frame ID.
			// Load number of bytes to be Tx.
			// Updating target pointer of Tx buffer initiates transmission
			gu8TxPtrTarget = ru8PrepareTxFrame(lsu8NextTxFrame);
			
			
		}
	}
	
}


uint8_t ru8PrepareTxFrame(uint8_t lu8TxFrameID)
{
	uint8_t lu8ChkSum = 0, lu8Loop = 0, lu8TxDLC = 0;
	
	// Prepare frame for Transmission 
	gau8TxBuffer[0] = COMM_SOF;			// SOF
	gau8TxBuffer[1] = lu8TxFrameID;		// Frame ID
	
	// select DLC according to frame ID
	switch(lu8TxFrameID)
	{
		case RESP_SYS_STATUS	:
		case RESP_PULSE_COUNT	:
		case RESP_DEGREES		: 
									lu8TxDLC = 4;
									break;
		case RESP_QUADRANT		:
									lu8TxDLC = 1;
									break;
									
		default					:	break;									
	}
	
	gau8TxBuffer[2] = lu8TxDLC;			// DLC

	// Data fields
	switch(lu8TxFrameID)
	{
		case RESP_SYS_STATUS	:		// System status													// DLC - 4
						gau8TxBuffer[3] = gu8ReqDirectionH | ((gu8ReqDirectionV & 0x0F) << 4);				// D3 - Direction : V and H
						gau8TxBuffer[4] = 0x00;																// D2 - reserved
						gau8TxBuffer[5] = gu8ReqSpeedRef;													// D1 - Speed divider
						gau8TxBuffer[6] = (gu8ReqStart	& 0x01) |											// D0.0 - Status
										 ((gu8ReqMode	& 0x01) << 1);										// D0.1 - Mode
						
						
						break;
						
		case RESP_PULSE_COUNT	:		// pulse elapsed count	- normal mode								// DLC - 4
						gau8TxBuffer[3] = (gu16PulseElapsedV & 0xFF00) >> 8;								// D3 - V pulse elapsed - MSB
						gau8TxBuffer[4] = (gu16PulseElapsedV & 0x00FF) >> 0;								// D2 - V pulse elapsed - LSB
						gau8TxBuffer[5] = (gu16PulseElapsedH & 0xFF00) >> 8;								// D1 - H pulse elapsed - MSB
						gau8TxBuffer[6] = (gu16PulseElapsedH & 0x00FF) >> 0;								// D0 - H pulse elapsed - LSB

						break;
						
		case RESP_QUADRANT		:		// Select quadrant status											// DLC - 1
						gau8TxBuffer[3] = gu8ReqQuadrantSet;												// D0 - Quadrant selected
						
						break;
												
		case RESP_DEGREES	:		// degrees elapsed count - scan mode									// DLC - 4
						gau8TxBuffer[3] = (gu16DegreesElapsedV & 0xFF00) >> 8;								// D3 - V pulse elapsed - MSB
						gau8TxBuffer[4] = (gu16DegreesElapsedV & 0x00FF) >> 0;								// D2 - V pulse elapsed - LSB
						gau8TxBuffer[5] = (gu16DegreesElapsedH & 0xFF00) >> 8;								// D1 - H pulse elapsed - MSB
						gau8TxBuffer[6] = (gu16DegreesElapsedH & 0x00FF) >> 0;								// D0 - H pulse elapsed - LSB

						break;
						
		default		:	break;
	}
	

	// Calculate checksum
	for(lu8Loop = 0; lu8Loop < (3 + fsu8TxDLC); lu8Loop++)
	{
		lu8ChkSum += gau8TxBuffer[lu8Loop];
	}
	
	gau8TxBuffer[lu8Loop++]	= lu8ChkSum;		// Check-sum
	gau8TxBuffer[lu8Loop]	= COMM_EOF;			// EOF
	
	return lu8Loop;
}

ISR(USART_RXC_vect)
{
	/* Write serial RX data in read Buff */
	/* If data received */
	if(UCSRA & (1<<RXC))
	{
		gau8RxBuffer[gu8RxWritePtr] = UDR;
		gu8RxWritePtr++;
		
		if(gu8RxWritePtr > 15)
		{
			gu8RxWritePtr = 0;
		}
	}
}


// Driver function for serial data Tx
void rvTxSerialData(void)
{
	// If data is pending for transmission 
	if(gu8TxPtr < gu8TxPtrTarget)
	{
		/* If transmit buffer is empty */
		if(UCSRA & (1<<UDRE))
		{
			/* Put data into buffer, sends the data */
			UDR = gau8TxBuffer[gu8TxPtr];
		
			gu8TxPtr++;
		}
	}
	else
	{
		// Indicate the Tx is complete.
		gu8SerialTxDone = TRUE;
		
		gu8TxPtrTarget = 0;
		gu8TxPtr = 0;
	}
}