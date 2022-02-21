/*
 *
 ******************************************************************************
 *
 * UART port control functions (C)Shun Hattori, 2021
 * @brief HAL Driver wrapper (F303k8 peripheral)
 *
 ******************************************************************************
 *
 */

#ifndef UART_WRAPPER_DEF
#define UART_WRAPPER_DEF

uint8_t uart_getc(void);
void uart_putc(uint8_t c);
void uart_puts(char *str);
extern UART_HandleTypeDef huart1;

uint8_t uart_getc(void) {
	uint8_t c = 0;
	char buf[1];
	HAL_UART_Receive(&huart1, (uint8_t*) buf, sizeof(buf), 0xFFFF);
	c = buf[0];
	return c;
}
void uart_putc(uint8_t c) {
	char buf[1];
	buf[0] = c;
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, sizeof(buf), 0xFFFF);
}
void uart_puts(char *str) {
	while (*str) {
		uart_putc(*str++);
	}
}

#endif
