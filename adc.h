#ifndef ADC_H
#define ADC_H

void adc_init (void);
void adc_deinit (void);
uint_32 adc_measure (uint_32 *sample);

#endif
