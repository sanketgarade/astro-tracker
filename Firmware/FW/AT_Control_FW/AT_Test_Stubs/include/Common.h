/*
 * Common.h
 *
 * Created: 10/2/2014 3:47:58 PM
 *  Author: Sanket
 */ 


#ifndef COMMON_H_
#define COMMON_H_


#include <avr/io.h>


#define TRUE	1
#define FALSE	0

#define B7		7
#define B6		6
#define B5		5
#define B4		4
#define B3		3
#define B2		2
#define B1		1
#define B0		0




/*************** ISR related ***************/
#define ISR_TICK_US				100		//Timer ISR period (for high priority jobs) (in micro sec.)
#define ISR_TIMER_CLOCK_US		8		//NG-> approx 122uS after prescaler (CPU clk = 8.33 MHz found by trial error experiment)
#define ISR_TICK_VALUE			(uint8_t)(((uint32_t)ISR_TICK_US)/ISR_TIMER_CLOCK_US)


/*************** Time in 100 us ticks ***************/
#define T_100MS					1000	// 100u x 1000 = 100ms
#define T_200MS					2000	// 100u x 2000 = 200ms

/*************** UART related ***************/

#define FOSC1 8000000
#define BAUD 9600
#define MYUBRR FOSC1/16/BAUD-1

/*************** LED Blink related ***************/
#define LED_BLINK_MS			(uint16_t)500	// blink rate of LED in mSec
#define LED_BLINK_INTERVAL		(uint16_t)((uint16_t)LED_BLINK_MS / (uint16_t)ISR_TICK_MS)

/*************** I/O related ***************/

#define PORT_PU_D				(SFIOR |= (1<<PUD))
#define PORT_PU_E				(SFIOR &= ~(1<<PUD))

#define MOTOR_PORT_OUT			PORTC
#define MOTOR_PORT_IN			PINC
#define MOTOR_PORT_DIR			DDRC

#define LAMP_PORT_OUT			PORTD
#define LAMP_PORT_IN			PIND
#define LAMP_PORT_DIR			DDRD


#define SW_PORT_OUT				PORTA
#define SW_PORT_IN				PINA
#define SW_PORT_DIR				DDRA

/*************** Port operations related ***************/

/*** LED ***/

#define LED1_OFF				LAMP_PORT_OUT|=(1<<B5)
#define LED2_OFF				LAMP_PORT_OUT|=(1<<B6)
#define LED3_OFF				LAMP_PORT_OUT|=(1<<B7)

#define LED1_ON					LAMP_PORT_OUT&=~(1<<B5)
#define LED2_ON					LAMP_PORT_OUT&=~(1<<B6)
#define LED3_ON					LAMP_PORT_OUT&=~(1<<B7)

#define LED1_TOGGLE				LAMP_PORT_OUT^=(1<<B5)
#define LED2_TOGGLE				LAMP_PORT_OUT^=(1<<B6)
#define LED3_TOGGLE				LAMP_PORT_OUT^=(1<<B7)

/*** Stepper Motor Interface ***/

/*** Horizontal Motion ***/
#define H_CLK_TOGGLE			MOTOR_PORT_OUT^=(1<<B0)		// Clock

#define H_DIR_CLOCKWISE			MOTOR_PORT_OUT|=(1<<B1)		// Direction
#define H_DIR_ANTICLOCKWISE		MOTOR_PORT_OUT&=~(1<<B1)	// 1 : CW , 0 : CCW

#define H_ENABLE				MOTOR_PORT_OUT|=(1<<B2)		// Enable
#define H_DISABLE				MOTOR_PORT_OUT&=~(1<<B2)	// 1 : En , 0 : Dis

#define H_STEP_HALF				MOTOR_PORT_OUT|=(1<<B3)		// Step size
#define H_STEP_FULL				MOTOR_PORT_OUT&=~(1<<B3)	// 1 : Half , 0 : Full

/*** Vertical Motion ***/
#define V_CLK_TOGGLE			MOTOR_PORT_OUT^=(1<<B4)		// Clock

#define V_DIR_CLOCKWISE			MOTOR_PORT_OUT|=(1<<B5)		// Direction
#define V_DIR_ANTICLOCKWISE		MOTOR_PORT_OUT&=~(1<<B5)	// 1 : CW , 0 : CCW

#define V_ENABLE				MOTOR_PORT_OUT|=(1<<B6)		// Enable
#define V_DISABLE				MOTOR_PORT_OUT&=~(1<<B6)	// 1 : En , 0 : Dis

#define V_STEP_HALF				MOTOR_PORT_OUT|=(1<<B7)		// Step size
#define V_STEP_FULL				MOTOR_PORT_OUT&=~(1<<B7)	// 1 : Half , 0 : Full


/*************** Serial Comm related ***************/
#define COMM_SOF				0x55
#define COMM_EOF				0xAA


// Frame IDs - Request
#define REQ_START_STOP			0x11
#define REQ_SPEED				0x12
#define REQ_MODE				0x13
#define REQ_DIRECTION			0x14
#define REQ_PULSE_COUNT			0x15
#define REQ_ZERO				0x16
#define REQ_QUADRANT			0x17
#define REQ_DEGREES				0x18

// Frame IDs - Response
#define  RESP_SYS_STATUS		0x21
#define  RESP_PULSE_COUNT		0x25
#define  RESP_QUADRANT			0x27
#define  RESP_DEGREES			0x28


/*************** Application related ***************/
// Operation Modes
#define MODE_NORMAL				0
#define MODE_SCAN				1


/*************** Enums ***************/

typedef enum
{
	RX_SOF_WAIT = 0,
	RX_SOF_READ,
	RX_FRAME_NO_READ,
	RX_DLC_READ,
	RX_DATA_READ,
	RX_CHKSUM_READ,
	RX_EOF_READ	
}RX_FRAME_READ_STATES;


/*************** Variables related ***************/

extern volatile uint32_t gu32TimeTicks;

extern uint8_t gau8RxBuffer[16];
extern uint8_t gau8TxBuffer[16];
extern uint8_t gu8TxPtr;
extern uint8_t gu8TxPtrTarget;

extern uint8_t gu8SendQuadrantResp;
extern uint8_t gu8SerialTxDone;


// Request
extern uint8_t gu8ReqStart;
extern uint8_t gu8ReqSpeedRef;
extern uint8_t gu8ReqMode;
extern uint8_t gu8ReqDirectionH;
extern uint8_t gu8ReqDirectionV;
extern uint16_t gu16ReqPulseTargetH;
extern uint16_t gu16ReqPulseTargetV;
extern uint8_t gu8ReqZeroSet;
extern uint8_t gu8ReqQuadrantSet;
extern uint16_t gu16ReqDegreesSetH;
extern uint16_t gu16ReqDegreesSetV;


// Response
extern uint16_t gu16PulseElapsedH;
extern uint16_t gu16PulseElapsedV;
extern uint16_t gu16DegreesElapsedH;
extern uint16_t gu16DegreesElapsedV;



#endif /* COMMON_H_ */