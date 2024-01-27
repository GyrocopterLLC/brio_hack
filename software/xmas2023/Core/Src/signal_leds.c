
#include "stm32g0xx.h"
#include "signal_leds.h"
#include "main.h"

inline void GPIO_AF_SET(GPIO_TypeDef* port, uint32_t pin, uint32_t af) {
	const uint32_t bit_pos = __builtin_ctz(pin);
	uint32_t temp = port->AFR[(bit_pos >> 3)];
	temp &= ~(0xFu << ((bit_pos & 0x07u) * 4u));
	temp |= ((af) << ((bit_pos & 0x07u) * 4u));
	port->AFR[(bit_pos >> 3)] = temp;
}

// Note: LED1 is on PB6, LED2 is PB8
// For PWM, PB6 is connected to TIM1_CH3 and PB8 is on TIM16_CH1
void signal_init(void){
	// Enable GPIOB
	__HAL_RCC_GPIOB_CLK_ENABLE();
	// And the two timers
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM16_CLK_ENABLE();

	// Place both signals in GPIO mode
	// Ensure they are off, and the speed is
	// set to maximum (required to make sure
	// pull-down current is high enough)
	LED1_GPIO_Port->OSPEEDR |= GPIO_OSPEEDR_OSPEED0 *(LED1_Pin*LED1_Pin);
	LED2_GPIO_Port->OSPEEDR |= GPIO_OSPEEDR_OSPEED0 *(LED2_Pin*LED2_Pin);
	LED1_GPIO_Port->OTYPER |= LED1_Pin; // Open drain
	LED2_GPIO_Port->OTYPER |= LED2_Pin;
	// Alternate function selection for when the pins are in PWM mode
	GPIO_AF_SET(LED1_GPIO_Port, LED1_Pin, 1); // PB6, AF1 = TIM1_CH3
	GPIO_AF_SET(LED2_GPIO_Port, LED2_Pin, 2); // PB8, AF2 = TIM16_CH1

	signal_set(Signal_1, false);
	signal_set(Signal_2, false);
	signal_mode(Signal_1, Signal_GPIO);
	signal_mode(Signal_2, Signal_GPIO);

	// Simple timer setup for PWM output
	TIM1->CR1 = 0; // Timer turned off
	TIM1->PSC = TIMER_PSC;
	TIM1->ARR = TIMER_ARR;
	TIM1->CCMR2 = (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM mode 1 - active when counting up to CCR value
	TIM1->CCER = TIM_CCER_CC3E; // enable the CH3 output
	TIM1->CCR3 = (TIMER_ARR >> 2); // 50% duty cycle
	TIM1->BDTR = TIM_BDTR_MOE; // all outputs enabled
	TIM1->CR1 = TIM_CR1_CEN; // start counting!

	TIM16->CR1 = 0;
	TIM16->PSC = TIMER_PSC;
	TIM16->ARR = TIMER_ARR;
	TIM16->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
	TIM16->CCER = TIM_CCER_CC1E;
	TIM16->CCR1 = (TIMER_ARR >> 2);
	TIM16->BDTR = TIM_BDTR_MOE;
	TIM16->CR1 = TIM_CR1_CEN;
}


void signal_mode(SignalSel sig, SignalMode mod)
{
	switch(sig)
	{
	case Signal_1:
		// clear the GPIO MODER for the pin,
		// then set the correct output
		LED1_GPIO_Port->MODER &= ~(GPIO_MODER_MODE0 * (LED1_Pin*LED1_Pin));

		if(mod == Signal_GPIO) {
			// GPIO = bit 0 set
			LED1_GPIO_Port->MODER |= (GPIO_MODER_MODE0_0* (LED1_Pin*LED1_Pin));
		} else if(mod == Signal_PWM) {
			// Alt function = bit 1 set
			LED1_GPIO_Port->MODER |= (GPIO_MODER_MODE0_1* (LED1_Pin*LED1_Pin));
		}
		break;
	case Signal_2:
		LED2_GPIO_Port->MODER &= ~(GPIO_MODER_MODE0 * (LED2_Pin*LED2_Pin));
		if(mod == Signal_GPIO) {
			// GPIO = bit 0 set
			LED2_GPIO_Port->MODER |= (GPIO_MODER_MODE0_0 * (LED2_Pin*LED2_Pin));
		} else if(mod == Signal_PWM) {
			// Alt function = bit 1 set
			LED2_GPIO_Port->MODER |= (GPIO_MODER_MODE0_1 * (LED2_Pin*LED2_Pin));
		}
		break;
	}
}

void signal_set(SignalSel sig, bool on_or_off)
{
	// note: LEDs are active low, so a GPIO Reset is needed to
	// turn it on, and GPIO Set to turn off

	switch(sig)
	{
	case Signal_1:
		if(on_or_off){
			LED1_GPIO_Port->BRR = LED1_Pin;
		} else {
			LED1_GPIO_Port->BSRR = LED1_Pin;
		}
		break;
	case Signal_2:
		if(on_or_off) {
			LED2_GPIO_Port->BRR = LED2_Pin;
		} else {
			LED2_GPIO_Port->BSRR = LED2_Pin;
		}
		break;
	}
}

// Note: duty is considered a 0.16 fixed point number
// where 0xFFFF is approximately 1
// example: desired duty cycle = 15.5%
// duty should be 0xFFFF * 0.155 = 10158 = 0x27AE
void signal_set_pwm(SignalSel sig, uint16_t duty)
{
	// Convert 32-bit duty to timer size / auto-reload value
	uint32_t temp = ((uint32_t)duty) * ((uint32_t)TIMER_ARR);
	temp = temp >> 16;

	/*
	HAL_RCC_GetSysClockFreq(); // discard the return, we just need it to update internal values
	uint32_t pclk = HAL_RCC_GetPCLK1Freq();
	if((RCC->CFGR & RCC_CFGR_PPRE) != 0) {
		pclk = pclk * 2; // timers run at 2x PCLK frequency if any PCLK divider is anything other than 1x
	}
	*/

	switch(sig)
	{
	case Signal_1:
		TIM1->CCR3 = temp;
		break;
	case Signal_2:
		TIM16->CCR1 = temp;
		break;
	}
}
