#include "types.h"
#include "derivative.h" 
#include "uart.h"
#include <string.h>

static uint_8 uart0_rx[UART0_RX_BUF_SIZE];
volatile uint_32 uart0_rx_idx = 0;

static uint_8 uart0_tx[UART0_TX_BUF_SIZE];
volatile uint_32 uart0_tx_idx = 0;
volatile uint_32 uart0_tx_pos = 0;

void uart_init (uint_32 id) {
	/*
	 * this function configures and enables uart ports,
	 * will be available for uart 0 and uart 1
	 */
	
	switch (id) {
		case 0:
			uart0_rx_idx = 0;
			uart0_tx_idx = 0;
			uart0_tx_pos = 0;
			/* SIM_SCGC4: UART0=1 */
			SIM_SCGC4 |= SIM_SCGC4_UART0_MASK; // enable clock gate     
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
			/* PORTA_PCR1: (PTA1) ISF=0,MUX=2 */
			PORTA_PCR1 = (uint32_t)((PORTA_PCR1 & (uint32_t)~(uint32_t)(
										PORT_PCR_ISF_MASK |
										PORT_PCR_MUX(0x05)
									 )) | (uint32_t)(
										PORT_PCR_MUX(0x02)
									 ));                                                  
			/* PORTA_PCR2: (PTA2) ISF=0,MUX=2 */
			PORTA_PCR2 = (uint32_t)((PORTA_PCR2 & (uint32_t)~(uint32_t)(
										PORT_PCR_ISF_MASK |
										PORT_PCR_MUX(0x05)
									 )) | (uint32_t)(
										PORT_PCR_MUX(0x02)
									 ));                                                  
			/* NVIC_IPR3: PRI_12=0x80 */
			NVIC_IPR3 = (uint32_t)((NVIC_IPR3 & (uint32_t)~(uint32_t)(
									 NVIC_IP_PRI_12(0x7F)
									)) | (uint32_t)(
									 NVIC_IP_PRI_12(0x80)
									));                                                  
			/* NVIC_ISER: SETENA|=0x1000 */
			NVIC_ISER |= NVIC_ISER_SETENA(0x1000);
				 
			UART0_C2 &= ~UART0_C2_TE_MASK; /* Disable transmitter. */
			UART0_C2 &= ~UART0_C2_RE_MASK; /* Disable receiver. */
  
			/* UART0_C1: LOOPS=0,DOZEEN=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
			UART0_C1 = 0x00U;                    /*  Set the C1 register */
			/* UART0_C3: R8T9=0,R9T8=0,TXDIR=0,TXINV=0,ORIE=0,NEIE=0,FEIE=0,PEIE=0 */
			UART0_C3 = 0x00U;                    /*  Set the C3 register */
			/* UART0_S2: LBKDIF=0,RXEDGIF=0,MSBF=0,RXINV=0,RWUID=0,BRK13=0,LBKDE=0,RAF=0 */
			UART0_S2 = 0x00U;                    /*  Set the S2 register */
  
			// config UART0 to use external clock source (0x2)
			SIM_SOPT2 = (SIM_SOPT2 & 0xF3FFFFFF) + (0x2 << SIM_SOPT2_UART0SRC_SHIFT);
			
			// BR[12:0] -> baud rate = (baud clock = 8MHz)/((OSR + 1)*BR), 8000000/(5*14) = 114285baud
			UART0_BDH &= 0xE0; // bits 4:0 = 0, BR[12:0] = 14
			UART0_BDL = 14; // = 14		
			
			UART0_C4 = (UART0_C4 & 0xE0) + 0x4; // set over sampling rate to 4 (OSR = 4)
			UART0_C5 |= UART0_C5_BOTHEDGE_MASK;	// enable sampling on both edges
			UART0_C2 |= UART0_C2_TE_MASK; // enable transmitter 
			UART0_C2 |= UART0_C2_RE_MASK; // enable receiver 
			UART0_C2 |= UART0_C2_RIE_MASK; // enable RX interrupt
			
			break;
			
		default:
			break;
	}
}

void uart_deinit (uint_32 id) {
	/*
	 * this function disables the clock gate for uart0 or uart1
	 */
	switch (id) {
		case 0:
			/* SIM_SCGC4: UART0=0 */
			SIM_SCGC4 &= (uint32_t)~(uint32_t)(SIM_SCGC4_UART0_MASK);   
			break;
		
		default:
			break;		
	}
}

void uart_send_buffer (uint_32 id, uint_8 *ptr, uint_32 length) {
	/*
	 * this function copies length data to the uart0 tx buffer
	 * and then arms the TX interrupt
	 */
	switch (id) {
		case 0:
			if (length >= UART0_TX_BUF_SIZE) {
				memcpy(uart0_tx, ptr, UART0_TX_BUF_SIZE);
				uart0_tx_idx = UART0_TX_BUF_SIZE;
			}
			else {
				memcpy(uart0_tx, ptr, length);
				uart0_tx_idx = length;
			}
			
			uart0_tx_pos = 0;
			UART0_C2 |= UART0_C2_TIE_MASK; // enable interrupt
			break;
		
		default:
			break;
	}
}

void uart_interrupt (uint_32 id) {
	/*
	 * this function handles interrupt requests
	 */
	uint_32 status;
	uint_8 c;
	
	switch (id) {
		case 0:
			status = UART0_S1;
			if (status & (UART0_S1_NF_MASK | UART0_S1_OR_MASK | UART0_S1_FE_MASK | UART0_S1_PF_MASK)) {
				//          noise flag         overflow flag      framing error flag parity error flag
				// write logic 1 to bits in UART0_S1 to clear
				UART0_S1 |= (UART0_S1_NF_MASK | UART0_S1_OR_MASK | UART0_S1_FE_MASK | UART0_S1_PF_MASK);
				c = UART0_D;
				status &= ~UART0_S1_RDRF_MASK; /* Clear the receive data flag to discard the errorneous data */
			}
			
			if (status & UART0_S1_RDRF_MASK) {
				// process RX interrupt
				c = UART0_D; // clear buffer
				if (uart0_rx_idx < UART0_RX_BUF_SIZE) {
					uart0_rx[uart0_rx_idx++] = c; // store data if there is space
				}
				
				// echo data back if there is buffer space
				if (status & UART0_S1_TDRE_MASK) {
					UART0_D = c;
				}
				
				if (c == 0x61) { // 'a'
					uart_send_buffer(0, "\r\nHELLO\r\nz",strlen("\r\nHELLO\r\nz"));
				}
			}
			
			if (status & UART0_S1_TDRE_MASK) {
				// process TX interrupt, need to print out from low byte to high byte
				if (uart0_tx_idx && (uart0_tx_pos < uart0_tx_idx)) {
					UART0_D = uart0_tx[uart0_tx_pos++];
				}
				else {
					// make sure TX interrupt is disabled
					uart0_tx_pos = 0;
					uart0_tx_idx = 0;
					UART0_C2 &= ~UART0_C2_TIE_MASK; // disable TX interrupt
				}
			}
		  break;
		
		default:
			break;
	}
}

void UART0_IRQHandler (void) {
	/*
	 * this is the UART0 interrupt handler, a wrapper function
	 */
	uart_interrupt(0);
}
