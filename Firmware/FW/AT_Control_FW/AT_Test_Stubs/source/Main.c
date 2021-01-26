/************************ Includes ************************/

#include <avr/interrupt.h>
#include "C:\Users\Vedashree\Documents\AVRStudio\AT_Control_FW\AT_Test_Stubs\include/Common.h"

/************************ Global variables ************************/

volatile uint32_t	gu32TimeTicks		= 0;
//volatile uint32_t	gu32Time100msTicks	= 0;

// Request
uint8_t gu8ReqStart			= FALSE;
uint8_t gu8ReqSpeedRef		= 0x00;
uint8_t gu8ReqMode			= MODE_NORMAL;
uint8_t gu8ReqDirectionH	= 0;
uint8_t gu8ReqDirectionV	= 0;
uint16_t gu16ReqPulseTargetH= 0;
uint16_t gu16ReqPulseTargetV= 0;
uint8_t gu8ReqZeroSet		= 0;
uint8_t gu8ReqQuadrantSet	= 1;
uint16_t gu16ReqDegreesSetH	= 0;
uint16_t gu16ReqDegreesSetV	= 0;

// Response
uint16_t gu16PulseElapsedH	= 0;
uint16_t gu16PulseElapsedV	= 0;
uint16_t gu16DegreesElapsedH= 0;
uint16_t gu16DegreesElapsedV= 0;


/************************ Function declarations ************************/



/************************ Function definitions ************************/


void main(void)
{
	//Initialize the hardware
	rvHardwareInit();
	
	H_ENABLE;
	
	H_DIR_CLOCKWISE;

	V_ENABLE;
	
	V_DIR_CLOCKWISE;

	
	//Infinite loop
	while(1)
	{
		rvReadRequest();
		rvSendResponse();
	}
	
}



/*	This is the interrupt service routine for TIMER0 Compare Match Interrupt.
	ISR frequency : 100 usec */
ISR(TIMER0_COMP_vect)
{
	static uint16_t lu16ISRDivCounter = 0;

		
#if 0	
	rvMotorTest();
#endif
	
	if(TRUE == gu8ReqStart)
	{
		// data Rx is time in 1ms multiple. (0~255)
		// used to clock the motor and LED
		if(lu16ISRDivCounter > (gu8ReqSpeedRef * 10))
		{
			LED3_TOGGLE;
	
			H_CLK_TOGGLE;
	
			V_CLK_TOGGLE;
	
	

	
			lu16ISRDivCounter = 0;
		}
	}

	rvTxSerialData();
	
	lu16ISRDivCounter++;

	// Increment global tick
	gu32TimeTicks++;


}



// Test stub to test both motors - Run CW - Stop - Run CCW
void rvMotorTest(void)
{
	static uint16_t lu16TestStubCounter = 0;

	if((lu16TestStubCounter > 0) && (lu16TestStubCounter <= 20000))
	{
		H_ENABLE;
		H_DIR_ANTICLOCKWISE;

		V_ENABLE;
		V_DIR_CLOCKWISE;
		
		LED1_ON;
		LED2_OFF;
	} 
	else if((lu16TestStubCounter > 20000) && (lu16TestStubCounter <= 25000))
	{
		H_DISABLE;
		V_DISABLE;

		LED1_OFF;
		LED2_OFF;
	}
	if((lu16TestStubCounter > 25000) && (lu16TestStubCounter <= 45000))
	{
		H_ENABLE;
		H_DIR_CLOCKWISE;

		V_ENABLE;
		V_DIR_ANTICLOCKWISE;
		
		LED1_OFF;
		LED2_ON;
	} 
	else if((lu16TestStubCounter > 45000) && (lu16TestStubCounter <= 50000))
	{
		H_DISABLE;
		V_DISABLE;
		
		LED1_OFF;
		LED2_OFF;
	}
	else if (lu16TestStubCounter > 50000)
	{
		lu16TestStubCounter = 0;
	}
	
	lu16TestStubCounter++;

}