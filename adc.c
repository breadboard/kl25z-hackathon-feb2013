#include "types.h"
#include "derivative.h" 
#include "adc.h"

void adc_init (void) {
	/*
	 * this initializes the ADC to record from PTE20, the
	 * MKL25Z only has one ADC peripheral
	 */
	/* SIM_SCGC6: ADC0=1 */
  SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;      
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;	
  /* PORTE_PCR20: ISF=0,MUX=0 */
  PORTE_PCR20 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));   
  
	/* ADC0_CFG1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADLPC=0,ADIV=1,ADLSMP=0,MODE=3,ADICLK=3 */
  ADC0_CFG1 = ADC_CFG1_ADIV(0x01) |
              ADC_CFG1_MODE(0x03) |
              ADC_CFG1_ADICLK(0x03);      
	
  /* ADC0_CFG2: ADACKEN=0,ADHSC=0,ADLSTS=0 */
  ADC0_CFG2 &= (uint32_t)~(uint32_t)(
                ADC_CFG2_ADACKEN_MASK |
                ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_ADLSTS(0x03)
               );                                                   
	
	//
	// Need to drive Vrefh and Vrefh, already done on FRDM board
	// 
	
  /* ADC0_SC2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADACT=0,ADTRG=0,ACFE=0,ACFGT=0,ACREN=0,DMAEN=0,REFSEL=0 */
  ADC0_SC2 = 0x00U;                                                   
  /* ADC0_SC3: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CAL=0,CALF=1,??=0,??=0,ADCO=0,AVGE=0,AVGS=0 */
  ADC0_SC3 = ADC_SC3_CALF_MASK;
}

void adc_deinit (void) { 
	/*
	 * this disables the ADC peripheral
	 */
	/* ADC0_SC1A: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=0,DIFF=0,ADCH=0x1F */
  ADC0_SC1A = ADC_SC1_ADCH(0x1F);                                                   
  /* SIM_SCGC6: ADC0=0 */
  SIM_SCGC6 &= ~SIM_SCGC6_ADC0_MASK;  
}

uint_32 adc_measure (uint_32 *sample) {
	/*
	 * this function measures a single sample from 
	 * the configured port, blocking until the conversion is done
	 */
	
	if (ADC0_SC2 & ADC_SC2_ADACT_MASK) { // conversion pending
		return(E_BUSY);
	}
	
	ADC0_SC3 &= ~ADC_SC3_ADCO_MASK; // single conversion mode (ADCO = 0)
	// may want to set up AVGE, AVGS here to turn on hardware averaging, number of samples
	
	ADC0_SC2 &= ~ADC_SC2_ADTRG_MASK; // ADTRG = 0, sw triggerd conversion on SC1A write
	
	ADC0_SC1A = 0x00; // select ADC_SE0/DP0 as input, disable diff, trigger conversion
	
	while (ADC0_SC2 & ADC_SC2_ADACT_MASK); // wait for conversion to complete
	
	*sample = ADC0_RA; // read data from ADC0_RA
	
	return(E_OK);
}
