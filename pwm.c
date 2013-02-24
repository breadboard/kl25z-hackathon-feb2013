#include "types.h"
#include "derivative.h" 
#include "pwm.h"

void pwm_init (void) {
	/*
	 * initializes TMP0 for PWM operation on PTC1
	 */
	
	/* SIM_SOPT2:TPMSRC = 0x01 */
	SIM_SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM_SOPT2 |= (0x11 << SIM_SOPT2_TPMSRC_SHIFT);
	
	/* SIM_SCGC6: TPM0=1 */
  SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;    
  /* SIM_SCGC5: PORTC=1 */
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
  /* SIM_SCGC5: PORTC=1 */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;	
	
  /* TPM0_SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,DMA=0,TOF=0,TOIE=0,CPWMS=0,CMOD=0,PS=0 */
  TPM0_SC = 0x00U;                     /* Clear status and control register */
  /* TPM0_CNT: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COUNT=0 */
  TPM0_CNT = 0x00U;                    /* Reset counter register */
  /* TPM0_C0SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=0,MSA=0,ELSB=0,ELSA=0,??=0,DMA=0 */
  TPM0_C0SC = 0x00U;                   /* Clear channel status and control register */
  /* TPM0_C1SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=0,MSA=0,ELSB=0,ELSA=0,??=0,DMA=0 */
  TPM0_C1SC = 0x00U;                   /* Clear channel status and control register */
  /* TPM0_C2SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=0,MSA=0,ELSB=0,ELSA=0,??=0,DMA=0 */
  TPM0_C2SC = 0x00U;                   /* Clear channel status and control register */
  /* TPM0_C3SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=0,MSA=0,ELSB=0,ELSA=0,??=0,DMA=0 */
  TPM0_C3SC = 0x00U;                   /* Clear channel status and control register */
  /* TPM0_C4SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=0,MSA=0,ELSB=0,ELSA=0,??=0,DMA=0 */
  TPM0_C4SC = 0x00U;                   /* Clear channel status and control register */
  /* TPM0_C5SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=0,MSA=0,ELSB=0,ELSA=0,??=0,DMA=0 */
  TPM0_C5SC = 0x00U;                   /* Clear channel status and control register */
  /* TPM0_MOD: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,MOD=0xFFFF */
  TPM0_MOD = TPM_MOD_MOD(0xFFFF);      /* Set up modulo register */
  /* TPM0_C0SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CHF=0,CHIE=0,MSB=1,MSA=0,ELSB=1,ELSA=1,??=0,DMA=0 */
	// capture on both falling and rising edge, edge aligned PWM
  TPM0_C0SC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK); /* Set up channel status and control register */
  /* TPM0_C0V: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,VAL=0x3333 */
  TPM0_C0V = TPM_CnV_VAL(0x3333);      /* Set up channel value register */
  /* PORTC_PCR1: ISF=0,MUX=4 */
	
  PORTC_PCR1 = (uint32_t)((PORTC_PCR1 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_MUX(0x04)
                )) | (uint32_t)(
                PORT_PCR_MUX(0x04)
               ));        

  // PTC7 used as GPIO (output)
	/* PORTD_PCR7: ISF=0,MUX=1 */
  PORTC_PCR7 = (uint32_t)((PORTC_PCR7 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_MUX(0x06)
               )) | (uint32_t)(
                PORT_PCR_MUX(0x01)
               )); 
							 
	GPIOC_PDDR |= 0x00000080; // PRC7 as output

  // PTB18 used as Red LED (output)
	/* PORTB_PCR18: ISF=0,MUX=1 */
  PORTB_PCR18 = (uint32_t)((PORTB_PCR18 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_MUX(0x06)
               )) | (uint32_t)(
                PORT_PCR_MUX(0x01)
               )); 
							 
	GPIOB_PDDR |= 1<<18; // PRC7 as output
	
  /* TPM0_SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,DMA=0,TOF=0,TOIE=0,CPWMS=0,CMOD=1,PS=3 */
	// incementes counter each LPTPM counter, clock divide by 8
  TPM0_SC = (TPM_SC_CMOD(0x01) | TPM_SC_PS(0x03)); /* Set up status and control register */
	TPM0_SC |= 0x40; // TOIE
}

void pwm_deinit (void) {
	/*
	 * deinitialize routine, not much to do here
	 */
}

void pwm_set_ratio (uint_16 ratio) {
	/*
	 * sets the pwm duty cycle ratio, valid values are
	 * 0x0000-0xFFFF which correspond to 0-100% respectively
	 */
	
}

void pwm_systick_control (uint_32 duty) {
	/*
	 * this function depends on systick interrupt
	 * to bitbang a PWM waveform, duty 0-100 correspond to 0-100%
	 */
	static uint_32 state = 0;
	static uint_32 r;
	
	if (duty <= 100) {
		r = ((PWM_SYSTICK_FREQ*duty)/100); // rounding errors
	}
	else {
		r = PWM_SYSTICK_FREQ;
	}
	
	if (!state) { // these are not mutually exclusive, so you can do 0% duty
		GPIOC_PSOR |= 0x80;
		GPIOB_PSOR |= 1<<18;
	}
	
	if (state == r) {
		GPIOC_PCOR |= 0x80;
		GPIOB_PCOR |= 1<<18;
	}
	
	state++;
	if (state >= PWM_SYSTICK_FREQ) {
		state = 0;
	}
}

void TPM0_IRQHandler (void) {
	int i = 0;
	
}
