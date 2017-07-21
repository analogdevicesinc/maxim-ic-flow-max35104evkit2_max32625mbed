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



#define TIMESTAMP_TIMER	MXC_TMR1

// The MAX35104EVKIT2 is a Arduino-style shield.
// The shield is mounted on the MAX32625MBED board.

const gpio_cfg_t g_board_led = { PORT_3, PIN_1, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

static const gpio_cfg_t s_board_max3510x_int = { PORT_0, PIN_6, GPIO_FUNC_GPIO, GPIO_PAD_INPUT_PULLUP };

static const ioman_cfg_t spi_cfg = IOMAN_SPIM1(1, 1, 0, 0, 0, 0 );
static const spim_cfg_t max3510x_spim_cfg = { 1, SPIM_SSEL0_LOW, 12000000 };
static const gpio_cfg_t max3510x_spi = { PORT_1, (PIN_0 | PIN_1 | PIN_2 | PIN_3), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

static const ioman_cfg_t g_uart_cfg = IOMAN_UART(1, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0);
static const gpio_cfg_t max3510x_uart = { PORT_2, (PIN_0 | PIN_1), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };



extern void (* const __isr_vector[])(void);

void GPIO_P0_IRQHandler(void)
{
	GPIO_Handler(0);
}

void UART1_IRQHandler(void)
{
    UART_Handler(MXC_UART1);
}

void board_init( void )
{
	// brings up board-specific ports

	SYS_IOMAN_UseVDDIOH( &max3510x_spi );
	SYS_IOMAN_UseVDDIOH( &max3510x_uart );
	SYS_IOMAN_UseVDDIOH( &s_board_max3510x_int );
	SYS_IOMAN_UseVDDIOH( &g_board_led );

	GPIO_OutPut( &g_board_led, ~0 );

	GPIO_Config(&s_board_max3510x_int);
	GPIO_Config(&max3510x_spi);
	GPIO_Config(&max3510x_uart);
	GPIO_Config(&g_board_led);

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
		// use this timer as a 96MHz 32-bit timestamp
		tmr32_cfg_t cont_cfg;
 		cont_cfg.mode = TMR32_MODE_CONTINUOUS;
		cont_cfg.polarity = TMR_POLARITY_INIT_LOW;  //start GPIO low
		cont_cfg.compareCount = ~0;
		while( TMR_Init(TIMESTAMP_TIMER, TMR_PRESCALE_DIV_2_0, NULL) != E_NO_ERROR );
		TMR32_Config(TIMESTAMP_TIMER, &cont_cfg);
	}

	TMR32_Start(TIMESTAMP_TIMER);
	
	board_tdc_interrupt_enable(false);
	GPIO_IntConfig(&s_board_max3510x_int, GPIO_INT_FALLING_EDGE);
	GPIO_RegisterCallback(&s_board_max3510x_int, max3510x_int_isr, NULL);
	GPIO_IntClr(&s_board_max3510x_int);

	NVIC_DisableIRQ(UART1_IRQn);
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

		while( UART_Init(MXC_UART1, &cfg, &sys_cfg) != E_NO_ERROR );
	}
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);

}

void board_wait( uint32_t us )
{
	// delay at least 'us' microseconds using the timestamp timer
	uint32_t one_us = SYS_SysTick_GetFreq()/1000000;
	uint32_t delay = us*one_us;
	uint32_t now = board_timestamp();
	while( board_timestamp() - now < delay );
}
	
void max3510x_spi_xfer( max3510x_t *p, void *pv_in, const void *pv_out, uint8_t count )
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

	if( SPIM_Trans( MXC_SPIM1, &req ) != count )
	{
		while( 1 ); // fatal error -- step into CSL to determine reason
	}

	// Wait for transaction to complete
	while( SPIM_Busy( MXC_SPIM1 ) != E_NO_ERROR )
	{
		// fatal
	}
}

static volatile bool s_done = true;

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
	UART_WriteAsync( MXC_UART1, &s_write_req );
	va_end(args);
}

// board-specific UART implimenation 

uint16_t board_uart_write( void *pv, uint16_t length )
{
	return UART_Write(MXC_UART1, (uint8_t *)pv, length);
}

uint16_t board_uart_read( void *pv, uint16_t length )
{
	int count = UART_Read2(MXC_UART1, (uint8_t *)pv, length, NULL);
	if(  count < 0 )
		count = 0;
	return count;
}

uint32_t board_timestamp(void)
{
	return TMR32_GetCount(TIMESTAMP_TIMER);
}

void board_tdc_interrupt_enable(bool b)
{
	if( b )
	{
		NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(s_board_max3510x_int.port));
		GPIO_IntEnable( &s_board_max3510x_int );
	}
	else
	{
		GPIO_IntDisable( &s_board_max3510x_int );
	}
}

bool board_flash_write( const void *p_data, uint16_t size )
{
	// The last 8K page in flash is reserved for max3510x configuration data supported by the cmd.c module
	if( size <= MXC_FLASH_PAGE_SIZE )
	{
		if( E_NO_ERROR == FLC_PageErase( MXC_FLASH_MEM_BASE+MXC_FLASH_FULL_MEM_SIZE-MXC_FLASH_PAGE_SIZE, MXC_V_FLC_ERASE_CODE_PAGE_ERASE, MXC_V_FLC_FLSH_UNLOCK_KEY ) )
			if( E_NO_ERROR == FLC_Write( MXC_FLASH_MEM_BASE + MXC_FLASH_FULL_MEM_SIZE - MXC_FLASH_PAGE_SIZE, p_data, size + ((4 - size & 3) & 3), MXC_V_FLC_FLSH_UNLOCK_KEY ) )
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
