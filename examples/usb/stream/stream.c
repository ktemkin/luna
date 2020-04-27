/**
 * This file is part of LUNA.
 *
 */

#include <stdbool.h>
#include "resources.h"

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(*array))

/**
 * Transmits a single charater over our example UART.
 */
void print_char(char c)
{
	while(!uart_tx_rdy_read());
	uart_tx_data_write(c);
}


/**
 * Transmits a string over our UART.
 */
void uart_puts(char *str)
{
	for (char *c = str; *c; ++c) {
		if (*c == '\n') {
			print_char('\r');
		}

		print_char(*c);
	}
}


/**
 * Prints a hex character over our UART.
 */
void print_nibble(uint8_t nibble)
{
	static const char hexits[] = "0123456789abcdef";
	print_char(hexits[nibble & 0xf]);
}


/**
 * Prints a single byte, in hex, over our UART.
 */
void print_byte(uint8_t byte)
{
	print_nibble(byte >> 4);
	print_nibble(byte & 0xf);
}


//
// Core application.
//

int main(void)
{
	uart_puts("eptri demo started! (built: " __TIME__ ")\n");
	uart_puts("Connecting USB device...\n");
	controller_connect_write(1);
	uart_puts("Connected.\n");


	while (1) {
	}
}
