
#ifndef INC_SIGNAL_LEDS_H_
#define INC_SIGNAL_LEDS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum signal_sel_enum
{
	Signal_1,
	Signal_2
} SignalSel;

typedef enum signal_mode_enum
{
	Signal_GPIO,
	Signal_PWM
} SignalMode;


#define TIMER_PSC 		(0)
#define TIMER_ARR		(3199) // 64MHz / 3200 = 20kHz

void signal_init(void); // Sets both signals in GPIO but also starts their PWM timers
void signal_mode(SignalSel sig, SignalMode mod); // changes between GPIO or PWM mode
void signal_set(SignalSel sig, bool on_or_off); // sets GPIO state of signal. if in PWM mode, no effect
void signal_set_pwm(SignalSel sig, uint16_t duty); // sets PWM duty of signal. if in GPIO mode, no effect


#endif /* INC_SIGNAL_LEDS_H_ */
