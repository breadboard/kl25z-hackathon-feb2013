#include <stdio.h>
#include <string.h>
#include "types.h"
#include "derivative.h" 
#include "adc.h"
#include "uart.h"
#include "pwm.h"

#define filter_limit 20
#define min_distance 2000

uint_32 adcval = 0, adcval_prev = 0;
uint_32 adcval_filtered = 0;
uint_8 buf[128];
uint_8 counter = 0, i = 0;

void SysTick_Handler (void) {
	static uint_32 tick = 0;
	
	tick++;
	if (tick%100 == 0) {
		//uart_send_buffer(0, "BEEP\r\n", strlen("BEEP\r\n"));	
		sprintf((char *)buf, "ADC=%d\r\n\0", adcval_filtered);
		
		uart_send_buffer(0, buf, strlen((const char *)buf));
	}
	
	//pwm_systick_control(90);
}

int main (void) {		
	
	adc_init();
	pwm_init();
	uart_init(0);
	
	SysTick_Config(48000000/1000); // systick every 1 ms
	//             Hz      /ticks per sec
	
	while (1) {
		adc_measure(&adcval);
		counter++;
		if(counter == 1)
			adcval_prev = adcval;
		else
			adcval_prev = (adcval_prev + adcval)/2;

		if(counter > filter_limit-1)
		{
			adcval_filtered = adcval_prev;
			if(adcval_filtered < min_distance)
				pwm_systick_control(10);
			else if (adcval_filtered < 10000)
				pwm_systick_control(adcval_filtered%100);
			else
				pwm_systick_control(100);
			counter = 0;
		}
		
	}
	
	return(0);
}
