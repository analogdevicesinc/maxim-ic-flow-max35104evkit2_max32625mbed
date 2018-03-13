/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/


// Smart Valve

#include "global.h"

#include <tmr.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "spim.h"
#include "uart.h"

#include "pwrseq_regs.h"
#include "pwrman_regs.h"
#include "flc.h"
#include "rtc_regs.h"
#include "trim_regs.h"
#include "board.h"
#include "transducer.h"
#include "crc.h"



// The MAX35104EVKIT2 is a Arduino-style shield.
#define UART_NDX								1
#define UART									MXC_UART1
#define UART_IRQ								UART1_IRQn

#define SWITCH_DEBOUNCE_COUNT 2

static volatile bool s_done = true;

static uint32_t 	s_timestamp;
static uint16_t 	s_max3510x_status;

const gpio_cfg_t s_gpio_cfg_led[] = 
{ 
	{ PORT_3, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL },
	{ PORT_3, PIN_1, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL },
	{ PORT_3, PIN_2, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL },
	{ PORT_3, PIN_3, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL }
};

static const gpio_cfg_t s_gpio_cfg_button_s1 = { PORT_2, PIN_3, GPIO_FUNC_GPIO, GPIO_PAD_INPUT };
static const gpio_cfg_t s_gpio_cfg_button_s2 = { PORT_2, PIN_2, GPIO_FUNC_GPIO, GPIO_PAD_INPUT };

typedef struct _board_switch_t
{
	bool		state;
	bool		changed;
	uint8_t		debounce_count;
	const gpio_cfg_t *p_gpio_cfg;
}
board_switch_t;

static board_switch_t s_switches[2] =
{
	{ .p_gpio_cfg = &s_gpio_cfg_button_s1 },
	{ .p_gpio_cfg = &s_gpio_cfg_button_s2 }
};

static const gpio_cfg_t s_gpio_max35104_int = { PORT_0, PIN_5, GPIO_FUNC_GPIO, GPIO_PAD_INPUT_PULLUP };

static const gpio_cfg_t s_gpio_4mx = { PORT_0, PIN_7, GPIO_FUNC_TMR, GPIO_PAD_FAST };

static const ioman_cfg_t spi_cfg = IOMAN_SPIM1(1, 1, 0, 0, 0, 0 );
static const spim_cfg_t max3510x_spim_cfg = { 1, SPIM_SSEL0_LOW, 12000000 };
static const gpio_cfg_t max3510x_spi = { PORT_1, (PIN_0 | PIN_1 | PIN_2 | PIN_3), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

static const ioman_cfg_t g_uart_cfg = IOMAN_UART(UART_NDX, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0);
static const gpio_cfg_t max3510x_uart = { PORT_0, (PIN_0 | PIN_1), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };


//extern void (* const __isr_vector[])(void);

void GPIO_P0_IRQHandler(void)
{
	GPIO_Handler(0);
}
void UART1_IRQHandler(void)
{
    UART_Handler(UART);
}

void board_init( void )
{
	// brings up board-specific ports

	SYS_IOMAN_UseVDDIOH( &max3510x_spi );
	SYS_IOMAN_UseVDDIOH( &s_gpio_4mx );
//	SYS_IOMAN_UseVDDIOH( &max3510x_uart );
	SYS_IOMAN_UseVDDIOH( &s_gpio_max35104_int );

	uint8_t i;
	for(i=0;i<ARRAY_COUNT(s_gpio_cfg_led);i++)
	{
		SYS_IOMAN_UseVDDIOH( &s_gpio_cfg_led[i] );
		board_led(i,false);
		GPIO_Config(&s_gpio_cfg_led[i]);
	}
	GPIO_Config(&s_gpio_4mx);
	GPIO_Config(&s_gpio_max35104_int);
	GPIO_Config(&max3510x_spi);
	GPIO_Config(&max3510x_uart);
	GPIO_Config(&s_gpio_cfg_button_s1);
	GPIO_Config(&s_gpio_cfg_button_s2);

	FLC_Init();

	{
		// initialize the SPI port connected to the MAX35103
		sys_cfg_t sys_cfg;
		sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
		sys_cfg.io_cfg = spi_cfg;
		if( SPIM_Init(MXC_SPIM1, &max3510x_spim_cfg, &sys_cfg) != E_NO_ERROR )
		{
			while( 1 );	// initialization failed -- step into CSL to determine the reason
		}
	}

	{
		tmr32_cfg_t cont_cfg;
 		cont_cfg.mode = TMR32_MODE_CONTINUOUS;
		cont_cfg.polarity = TMR_POLARITY_INIT_LOW;  //start GPIO low
		cont_cfg.compareCount = 96000000/4000000/2 ;
		while( TMR_Init(MXC_TMR1, TMR_PRESCALE_DIV_2_0, NULL) != E_NO_ERROR );
		TMR32_Config(MXC_TMR1, &cont_cfg);
	}

	TMR32_Start(MXC_TMR1);
	{
		// use this timer as a 96MHz 32-bit timestamp
		tmr32_cfg_t cont_cfg;
 		cont_cfg.mode = TMR32_MODE_CONTINUOUS;
		cont_cfg.polarity = TMR_POLARITY_INIT_LOW;  //start GPIO low
		cont_cfg.compareCount = ~0;
		while( TMR_Init(MXC_TMR0, TMR_PRESCALE_DIV_2_0, NULL) != E_NO_ERROR );
		TMR32_Config(MXC_TMR0, &cont_cfg);
	}

	TMR32_Start(MXC_TMR0);

	NVIC_DisableIRQ(UART_IRQ);
	
	{
		// initialize the UART connected to the HDK serial port
		uart_cfg_t cfg;
		cfg.parity = UART_PARITY_DISABLE;
		cfg.size = UART_DATA_SIZE_8_BITS;
		cfg.extra_stop = 0;
		cfg.cts = 0;
		cfg.rts = 0;
		cfg.baud = 115200;

		sys_cfg_uart_t sys_cfg;
		sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
		sys_cfg.io_cfg = g_uart_cfg;

		while( UART_Init(UART, &cfg, &sys_cfg) != E_NO_ERROR );
	}
	NVIC_ClearPendingIRQ(UART_IRQ);
	NVIC_EnableIRQ(UART_IRQ);
	GPIO_IntConfig(&s_gpio_max35104_int, GPIO_INT_FALLING_EDGE);
	
	board_printf("\033cMAX35104EVKIT2 v0.1\r\n");

}

void board_wait_ms( uint32_t ms )
{
	// delay at least 'ms' milliseconds using the timestamp timer
	uint32_t delay;
	TMR32_TimeToTicks(MXC_TMR0,ms, TMR_UNIT_MILLISEC, &delay);
	uint32_t now = TMR32_GetCount(MXC_TMR0);
	while( TMR32_GetCount(MXC_TMR0) - now < delay );
}
	
void max3510x_spi_xfer( max3510x_t p, void *pv_in, const void *pv_out, uint8_t count )
{
	// used by the MAX3510x module to interface with the hardware

//	tdc_shield_t *p_shield = (tdc_shield_t *)p;
	spim_req_t req;
	req.ssel = 0;
	req.deass = 1;
	req.tx_data = pv_out;
	req.rx_data = pv_in;
	req.width = SPIM_WIDTH_1;
	req.len = count;
	
	if( (SPIM_Trans( MXC_SPIM1, &req )) != count )
	{
		while( 1 ); // fatal error -- step into CSL to determine reason
	}

	// Wait for transaction to complete
	while( SPIM_Busy( MXC_SPIM1 ) != E_NO_ERROR )
	{
		// fatal
	}
}

void write_complete(uart_req_t* p, int a)
{
	s_done = true;
}

void board_printf( const char *p_format, ... )
{
	// prints to the UART connected HDK serial port

	static char s_buff[256];
	static uart_req_t s_write_req;

	va_list args;
	va_start(args, p_format);

	while( !s_done );
	vsnprintf( s_buff, sizeof(s_buff)-1, p_format, args );
    s_write_req.data = (uint8_t*)s_buff;
    s_write_req.len = strlen(s_buff);
	s_write_req.callback = write_complete;
	s_done = false;
	UART_WriteAsync( UART, &s_write_req );
	va_end(args);
}

// board-specific UART implimenation 

uint16_t board_uart_write( const void *pv, uint16_t length )
{
	return UART_Write(UART, (uint8_t *)pv, length);
}

uint16_t board_uart_read( void *pv, uint16_t length )
{
	NVIC_DisableIRQ(UART_IRQ);
	int count = UART_Read2(UART, (uint8_t *)pv, length, NULL);
	NVIC_EnableIRQ(UART_IRQ);
	if(  count < 0 )
		count = 0;
	return count;
}

uint32_t board_timestamp(void)
{
	return s_timestamp;
}


uint16_t board_max3510x_interrupt_status( void )
{
	return s_max3510x_status;
}


bool board_flash_write( const void *p_data, uint16_t size )
{
	// The last 8K page in flash is reserved for max3510x configuration
	if( size <= MXC_FLASH_PAGE_SIZE )
	{
		if( E_NO_ERROR == FLC_PageErase( (MXC_FLASH_MEM_BASE+MXC_FLASH_FULL_MEM_SIZE-MXC_FLASH_PAGE_SIZE), MXC_V_FLC_ERASE_CODE_PAGE_ERASE, MXC_V_FLC_FLSH_UNLOCK_KEY ) )
			if( E_NO_ERROR == FLC_Write( (MXC_FLASH_MEM_BASE + MXC_FLASH_FULL_MEM_SIZE - MXC_FLASH_PAGE_SIZE), p_data, size + (((4 - size) & 3) & 3), MXC_V_FLC_FLSH_UNLOCK_KEY ) )
				return true;
	}
	return false;
}

void board_flash_read( void *p_data, uint16_t size )
{
	if( size <= MXC_FLASH_PAGE_SIZE )
	{
		memcpy( p_data, (void*)(MXC_FLASH_MEM_BASE+MXC_FLASH_FULL_MEM_SIZE-MXC_FLASH_PAGE_SIZE), size );
	}
}


void board_led( uint8_t ndx, bool on )
{
	if( ndx < ARRAY_COUNT(s_gpio_cfg_led) )
	{
		GPIO_OutPut( &s_gpio_cfg_led[ndx], on ? 0 : ~0 );
	}
}

float_t board_temp_sensor_resistance( float_t therm_time, float_t ref_time )
{
	const float_t comparison_resistance = 1000.0f;  // R1
	return comparison_resistance * therm_time / ref_time ;
}


bool board_switch( uint8_t switch_ndx, bool * p_changed )
{
	// returns the deobounced switch state
	// true = pressed
	while( switch_ndx >= ARRAY_COUNT( s_switches ) );
	board_switch_t *p_switch = &s_switches[switch_ndx];
	if( p_changed )
	{
		*p_changed = p_switch->changed;
	}
	p_switch->changed = false;
	return p_switch->state;
}

uint16_t board_crc( const void * p_data, uint32_t size )
{
	CRC16_Init( true, true );
	CRC16_Reseed(0);
	CRC16_AddDataArray( (uint32_t*)p_data, size>>2 );
	return CRC16_GetCRC();
}

uint32_t board_sleep( void )
{
	uint8_t i;
	uint32_t event = 0;

//#if defined(RELEASE_BUILD)
//  LP_ClearWakeUpConfig();
//	LP_ClearWakeUpFlags();
//	__disable_irq();
//	LP_EnterLP1();
//	__enable_irq();
//#endif


	while( !event )
	{
		s_timestamp =  TMR32_GetCount( MXC_TMR0 );
		if( UART_NumReadAvail(UART) )
		{
			event |= BOARD_EVENT_UART;
		}

		if( SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk )
		{
			event |= BOARD_EVENT_SYSTICK;
		}
		if( GPIO_IntStatus(&s_gpio_max35104_int) )
		{
			s_max3510x_status = max3510x_interrupt_status(NULL);
			event |= BOARD_EVENT_MAX35104;
			GPIO_IntClr(&s_gpio_max35104_int);
		}
		for( i = 0; i < ARRAY_COUNT( s_switches ); i++ )
		{
			// debounce push-button switches

			board_switch_t *p_switch = &s_switches[i];
			const gpio_cfg_t *p_button_cfg = p_switch->p_gpio_cfg;
			if( GPIO_IntStatus( p_button_cfg ) )
			{
				GPIO_IntClr( p_button_cfg );
				p_switch->debounce_count = SWITCH_DEBOUNCE_COUNT;
			}
			else if( (event & BOARD_EVENT_SYSTICK) && p_switch->debounce_count )
			{
				bool current_state = GPIO_InGet( p_button_cfg ) ? false : true ;  // 0V = pushed
				if( current_state != p_switch->state )
				{
					if( !--p_switch->debounce_count )
					{
						p_switch->state = current_state;
						p_switch->changed = true;
						event |= BOARD_EVENT_BUTTON;
					}
				}
				else
				{
					p_switch->debounce_count = 0;
				}
			}
		}
	}
	return event;
}

float_t board_elapsed_time( uint32_t timestamp, float_t *p_elapsed )
{
	static const float_t c_period = 1.0/96.0E6;

	uint32_t then = timestamp;
	uint32_t now = board_timestamp();
 	if( p_elapsed )
	{
		*p_elapsed =  ((float_t)(now - then)) * c_period;
	}
	return now;
}

void board_reset( void )
{
	NVIC_SystemReset();
}

void board_clock_enable( bool enable )
{
	SysTick->CTRL  = enable ? SysTick_CTRL_ENABLE_Msk : 0;
	SysTick->VAL   = 0;
}

float_t board_clock_set( float_t frequency )
{
	// callee requires a periodic clock
	uint32_t div;
	if( frequency )
		div = ( 32768.0f / frequency) - 1;
	else
		div = 0;
	MXC_PWRSEQ->reg0 |= (MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN);
	if( SysTick->VAL > div )
	{
		SysTick->CTRL = 0;
		SysTick->VAL %= div;
		SysTick->LOAD = div;
		SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
	}
	else
		SysTick->LOAD = div;
	return 32768.0f / (float_t)div;
}

