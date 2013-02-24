#ifndef PWM_H
#define PWM_H

#define PWM_SYSTICK_FREQ 10

void pwm_init (void);
void pwm_deinit (void);
void pwm_systick_control (uint_32 duty);

#endif
