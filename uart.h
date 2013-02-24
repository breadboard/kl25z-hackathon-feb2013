#ifndef UART_H
#define UART_H

#define UART0_RX_BUF_SIZE		32
#define UART0_TX_BUF_SIZE		32

void uart_init (uint_32 id);
void uart_deinit (uint_32 id);
void uart_send_buffer (uint_32 id, uint_8 *ptr, uint_32 length);
void uart_interrupt (uint_32 id);
void UART0_IRQHandler (void);

#endif
