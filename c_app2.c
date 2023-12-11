
#define VER_MAJOR 1
#define VER_MINOR 1

#include "./main.h"
#include "./port.h"
#include "./uart.h"
#include "./xmem.h"
#include "./adc.h"
#include "./timer.h"
#include "./int.h"
#include "./tools.h"
#include "./eprom.h"

#include <avr/interrupt.h>
#include <stddef.h>
#include <stdbool.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

enum _UartStatusT
{
	UartTransBeg,
	UartTransEnd,
	UartRecvBeg,
	UartRecvEnd,
	UartIdle
}
UartStatus = UartIdle;

#define IN_BUFF_LEN  64
#define OUT_BUFF_LEN 64
static uint8_t InBuff [ IN_BUFF_LEN ];
static uint8_t OutBuff[ OUT_BUFF_LEN ];
static volatile uint8_t SendLen;
static volatile uint8_t SendNum;

#define DEV_ISPR	0x01
#define IO_ISPR		0x02
#define TYPE	    0x04
#define TYPE_P    	0x00

static uint8_t AdcLimitX1 = ( uint8_t )( 1.2 * 256. / 5. );
static uint8_t AdcLimitX2 = ( uint8_t )( 2.0 * 256. / 5. );

#define ADC_LIMIT_X1_ADDR_EPROM 0x00
#define ADC_LIMIT_X2_ADDR_EPROM 0x01

static volatile uint8_t IsStart;

#define ADC_PERIOD 20 /*ms*/
#define ADC_T_WORK 150 /*ms*/

static volatile uint8_t  CountC80 = 0;
static volatile bool     LevelC80 = false;
static volatile unsigned VvdUpNum;
static volatile unsigned PodgUpNum;
static volatile unsigned AzUpNum;

static volatile uint8_t PNum = 0;
static volatile uint8_t Izp  = 0;
static volatile bool    fIzp;

#define ADC_ANALIZ( __adc__, __limit__, __num__ ){ if( __adc__ > __limit__ ) __num__++; }

void Timer1CompAPS( void )
{
	static uint8_t adc;
	//( 375mks )
	PORTF |= ( 1 << PF4 );
	ADC_GET_DATE_CHAN0( adc );
	PORTF &= ~( 1 << PF4 );
	ADC_ANALIZ( adc, AdcLimitX1, VvdUpNum );
	TIMER1_STOP;
}

void Timer3CompAPS( void )
{
	static uint8_t adc;
	// ( 875mks )
	PORTF |= ( 1 << PF4 );
	ADC_GET_DATE_CHAN0( adc );
	PORTF &= ~( 1 << PF4 );
	ADC_ANALIZ( adc, AdcLimitX1, PodgUpNum );
	TIMER3_STOP;
}

ISR( INT0_vect ) // C_80
{
	if(( EICRA & (( 1 << ISC01 )|( 1 << ISC00 ))) == (( 1 << ISC01 )|( 1 << ISC00 )))
	{	// восходящий уровень
		EIMSK &= ~( 1 << INT0  );  // int off
		EICRA &= ~( 1 << ISC00 );  // настройка срабатывания по спадающему фронту
		EIMSK |=  ( 1 << INT0  );  // int on

		LevelC80 = true;		

		if( 0 < PNum )
		{
			CountC80++;
		}
	}
	else
	{	// нисходящий уровень
		LevelC80 = false;
		EIMSK &= ~( 1 << INT0  ); // int off
		EICRA |=  ( 1 << ISC00 ); // настройка срабатывания по наростающему фронту
		EIMSK |=  ( 1 << INT0  ); // int on
	}
}

ISR( INT1_vect ) // 1 kHz
{
	if(( EICRA & (( 1 << ISC11 )|( 1 << ISC10 ))) == (( 1 << ISC11 )|( 1 << ISC10 )))
	{	// восходящий уровень
		EIMSK &= ~( 1 << INT1  ); // int off
		EICRA &= ~( 1 << ISC10 ); // настройка срабатывания по спадающему фронту
		EIMSK |=  ( 1 << INT1  ); // int on

		if( fIzp && LevelC80 )
		{
			*(( uint8_t* )( XMEM_BEG + 0xC4 )) = Izp;
			 fIzp = false;
		}

		if(( 0 < CountC80 ) && ( 3 >= CountC80 ))
		{
			TIMERS_START;
		}
		else
		{
			static uint8_t i = 0;

			if( 6 == CountC80 )
			{
				if( 10 > i )
				{
					static uint8_t adc;
					ADC_GET_DATE_CHAN1( adc );
					ADC_ANALIZ( adc, AdcLimitX1, AzUpNum );
					i++;
				}
				else
				{
					i = 0;
					CountC80 = 0;
				}
			}
		}
	}
	else
	{	// нисходящий уровень
		EIMSK &= ~( 1 << INT1  ); // int off
		EICRA |=  ( 1 << ISC10 ); // настройка срабатывания по наростающему фронту
		EIMSK |=  ( 1 << INT1  ); // int on
	}
}

void Timer1CompA( void )
{
	static uint8_t msec = 0;
	static uint8_t adc;
	static unsigned num = 0;

	msec++;

	if( ADC_T_WORK > msec )
	{
		if( 0 == ( msec % ADC_PERIOD ))
		{
			ADC_GET_DATE_CHAN0( adc );
			ADC_ANALIZ( adc, AdcLimitX1, num );
		}
	}
	else
	{
		TIMER1_STOP;
		msec = 0;
		uint8_t* s;
		num = 0;
	}
}

void Timer3CompA( void )
{
	static uint8_t msec = 0;
	static unsigned num = 0;
	static uint8_t adc;

	msec++;

	if( ADC_T_WORK > msec )
	{
		if( 0 == ( msec % ADC_PERIOD ))
		{
			ADC_GET_DATE_CHAN1( adc );
			ADC_ANALIZ( adc, AdcLimitX1, num );
		}
	}
	else
	{
		TIMER3_STOP;
		msec = 0;
		uint8_t* s = NULL;
		num = 0;
	}
}

#define BLOCK1 0x40
#define BLOCK2 0x80

void( *Start       )( void );
void( *Timer1CompA )( void );
void( *Timer3CompA )( void );
void( *DelayForS   )( void );

ISR( TIMER1_COMPA_vect ){ Timer1CompA();}
ISR( TIMER3_COMPA_vect ){ Timer3CompA();}

void DelayForS  ( void ){ _delay_us(  10. );}
void DelayForSPS( void ){ _delay_us( 600. );}

ISR( USART1_RX_vect )
{
	static uint8_t udr;
	static uint8_t len;
	static uint8_t num;

	cli();
	udr = UDR1;

	if( UartIdle == UartStatus )
	{
		switch( udr )
		{
			case 0xA0:
			case 0xA1:
			case 0xA2: len = 2; break;
			case 0xA3: len = 3; break;
			case 0xA4: len = 5; break;
			case 0xA5: len = 2; break;
			case 0xAE: len = 2; break;
			case 0xAF: len = 3; break;

			default: len = 0;
		}

		if( 0 < len )
		{
			UartStatus = UartRecvBeg;
			InBuff[ num = 0 ] = udr;
			num++, len--;
		}
	}
	else
	{
		if( 0 < len )
		InBuff[ num ] = udr, num++, len--;
		
		if( 0 == len )
		{
			num--;

			if( Crc( InBuff, num ) == InBuff[ num ])
			OutBuff[ 1 ] |= IO_ISPR;
			else
			OutBuff[ 1 ] &= ~IO_ISPR;

			UartStatus = UartRecvEnd;
		}
	}

	sei();
}

ISR( USART1_UDRE_vect )
{
	cli();
	UDR1 = OutBuff[ SendNum ];
	SendNum++;

	if( SendNum == SendLen )
	{
		UCSR1B &= ~( 1 << UDRIE1 );
		UCSR1B |=  ( 1 << TXCIE1 );
		UartStatus = UartTransEnd;
	}

	sei();
}

ISR( USART1_TX_vect )
{
	cli();
	UartStatus = UartIdle;
	UCSR1B &= ~( 1 << TXCIE1 );
	sei();
}

void setup( void )
{
	cli();
	PORT_INIT;
	XMEM_INIT;
	
	EEPROM_READ( ADC_LIMIT_X1_ADDR_EPROM, AdcLimitX1 );
	EEPROM_READ( ADC_LIMIT_X2_ADDR_EPROM, AdcLimitX2 );

	if( PORT_TYPE & ( 1 << PIN_TYPE ))
	{
		*(( uint8_t* )( XMEM_BEG + 0xD5 )) = 0x02; // тип
		PORT_LED    |= ( 1 << LED_ISPR );
		OutBuff[ 1 ] = DEV_ISPR | IO_ISPR | TYPE;
		Timer1CompA  = Timer1CompA;
		Timer3CompA  = Timer3CompA;
		DelayForS    = DelayForS;
		TIMER_INIT;
	}
	else
	{
		*(( uint8_t* )( XMEM_BEG + 0xD5 )) = 0x01; // тип
		PORT_LED    &= ~( 1 << LED_ISPR );
		OutBuff[ 1 ] = DEV_ISPR | IO_ISPR | TYPE_PUSK;
		Timer1CompA  = Timer1CompAPS;
		Timer3CompA  = Timer3CompAPS;
		DelayForS    = DelayForSPS;
		TIMER_INIT_PUSK;
		INT_INIT;
	}
	UART_INIT;
	ADC_INIT;
	sei();
}

int main( void )
{
	setup();

    while( 1 )
    {
		__asm volatile ("nop");

		if( UartRecvEnd == UartStatus )
		{
			switch( InBuff[ 0 ])
			{	// версия
				case 0xA0:
					OutBuff[ 0 ] = 0xC0;
					memcpy( &OutBuff[ 2 ], ( void* )( XMEM_BEG + 0xD6 ), 5 ); // версия по плис
					OutBuff[ 7 ] = ( VER_MAJOR << 4 )| VER_MINOR;             // версия по контроллера
					SendLen = 9;
					break;

				case 0xA1:
					memcpy( &OutBuff[ 2 ], ( void* )( XMEM_BEG + 0xC0 ), 4 ); // шина D
					ADC_GET_DATE_CHAN0( OutBuff[ 6 ]); // АЦП 0-й канал
					ADC_GET_DATE_CHAN1( OutBuff[ 7 ]); // АЦП 1-й канал
					OutBuff[ 0 ] = 0xC1;
					SendLen = 9;
					break;

				case 0xA2:
					memcpy( &OutBuff[ 2 ], ( void* )( XMEM_BEG + 0xCF ), 6 ); // S
					OutBuff[ 0 ] = 0xC2;
					SendLen = 9;
					break;

				case 0xA3: // BS
					SetBS( InBuff[ 1 ]);
					DelayForS(); // Ожидание установки S
					memcpy( &OutBuff[ 2 ], ( void* )( XMEM_BEG + 0xCF ), 6 ); // S
					OutBuff[ 0 ] = 0xC3;
					SendLen = 9;
					break;
					
				case 0xA4: // Старт ц
					Start();
					DelayForS();// Ожидание установки S
					memcpy( &OutBuff[ 2 ], ( void* )( XMEM_BEG + 0xC0 ), 4 ); // шина D
					memcpy( &OutBuff[ 6 ], ( void* )( XMEM_BEG + 0xCF ), 6 ); // S
					OutBuff[ 0 ] = 0xC4;
					SendLen = 13;
					break;

				case 0xA5:// сигналы ок
					OutBuff[ 0 ] = 0xC5;
					memcpy( &OutBuff[ 2 ], SigObrKontr, 6 );
					SendLen = 9;
					break;

				case 0xAE:
					OutBuff[ 0 ] = 0xCE;
					memcpy( &OutBuff[ 2 ], __DATE__, 11 );
					SendLen = 14;
					break;

				case 0xAF:
					if( 0x00 != InBuff[ 1 ])
					{
						EEPROM_WRITE( ADC_LIMIT_X1_ADDR_EPROM, InBuff[ 1 ]);
						EEPROM_READ ( ADC_LIMIT_X1_ADDR_EPROM, AdcLimitX1 );
					}

					if( 0x00 != InBuff[ 2 ])
					{
						EEPROM_WRITE( ADC_LIMIT_X2_ADDR_EPROM, InBuff[ 2 ]);
						EEPROM_READ ( ADC_LIMIT_X2_ADDR_EPROM, AdcLimitX2 );
					}

					OutBuff[ 0 ] = 0xCF;
					OutBuff[ 2 ] = AdcLimitX1;
					OutBuff[ 3 ] = AdcLimitX2;
					SendLen = 5;
					break;

				default:
					continue;
			}

			OutBuff[ SendLen - 1 ] = Crc( OutBuff, SendLen - 1 );

			cli();
			UartStatus = UartTransBeg;
			SendNum = 0;
			UCSR1B |= 1 << UDRIE1;
			sei();
			PORT_LED ^= 1 << LED_OBMEN;
		}

		if( PORT_BLK & (( 1 << PIN_BLK1 )|( 1 << PIN_BLK2 )))
			PORT_LED &=	~( 1 << LED_BLK );
		else
			PORT_LED |=	1 << LED_BLK;
    }
}
