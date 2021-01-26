#include <avr/interrupt.h>
#include "C:\Users\Vedashree\Documents\AVRStudio\AT_Control_FW\AT_Test_Stubs\include/Common.h"

void rvTimer0Config(void);
void rvHardwareInit(void);
void rvADCInit(void);
void rvGPIOInit(void);
void USART_Init(void);

void rvHardwareInit(void)
{
	uint8_t lu8loopCount = 0;
	
	// GPIO Init 
	rvGPIOInit();
	
	for(lu8loopCount = 0; lu8loopCount < 16; lu8loopCount++ )
	{
		gau8RxBuffer[lu8loopCount] = 0;
		gau8TxBuffer[lu8loopCount] = 0;
	}
	
	// Config timer for ISR
	rvTimer0Config();
#if 0
	//Config ADC
	rvADCInit();
#endif

	USART_Init();

	//Enable Global Interrupts
	sei();

}

void rvTimer0Config(void)
{
	// Set Prescaler = FCPU/1024
	// tick = 1Mhz/1024 = 1ms approx
	// set CTC mode for custom TOP value
	// Div by 64 prescaler 
	TCCR0 |= (1<<CS01) | (1<<CS00) | (1<<WGM01);

	//Enable Compare Match Interrupt Enable
	TIMSK |= (1<<OCIE0);

	//Initialize Timer 0 gu16LEDTickser
	TCNT0 = 0;

	// Initialise Compare value (value = time(ms)/0.25ms)
	OCR0 = ISR_TICK_VALUE;	// for 10ms ISR
//	OCR0 = 4;	// for 1ms ISR

	
}


void USART_Init(void)
{
	 unsigned int ubrr = MYUBRR;
	/* Set baud rate */
	UBRRH = (unsigned char)(ubrr>>8);
	UBRRL = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
}


void rvGPIOInit(void)
{
	//Enable internal pull-ups for ports
	PORT_PU_E;

	//Set o/p direction of LED port
	LAMP_PORT_DIR = 0xFF;

	//Set o/p direction of LED port
	MOTOR_PORT_DIR = 0xFF;

	//Set i/p direction of Switch port
	SW_PORT_DIR = 0x00;
	
	// Clear motor interface signals 
	MOTOR_PORT_OUT = 0;

	// Clear LED port
	LAMP_PORT_OUT = 0;

	// Turn OFF all LEDs
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	
}

void rvADCInit(void)
{
	uint8_t lu8Temp = 0x00;
	
	//ADC ref selection : AVcc (REFS1:0 = 01)
	ADMUX = (1<<REFS0);
	
	//ADC channel selection : ADC1 single ended
	ADMUX |= 0x01;
	
	//ADC result left adjust
	ADMUX |= (1<<ADLAR);
	
	//ADC trigger mode : auto trigger
	ADCSRA |= (1<< ADATE);

	//ADC SOC auto trigger source : Timer 0 compare match
	SFIOR |= 0x60;
	
	//ADC Enable
	ADCSRA |= (1<<ADEN);
	
#if 1
	//Do a dummy conversion
	ADCSRA |= (1<<ADSC);

	while(0 == (ADCSRA & (1<<ADIF)));

	lu8Temp = ADCH;
#endif	
	

}

