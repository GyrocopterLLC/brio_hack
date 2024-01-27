/**
 * Adopted from OneButtonTiny. Part of the OneButton Arduino library
 * available at https://github.com/mathertel/OneButton/
 *
 * Converted to generic microcontroller (non-Arduino) use and C language
 * @author David Miller
 *
 * Original header information follows:
 */

// -----
// OneButtonTiny.h - Library for detecting button clicks, doubleclicks and long
// press pattern on a single button. This class is implemented for use with the
// Arduino environment. Copyright (c) by Matthias Hertel,
// http://www.mathertel.de This work is licensed under a BSD style license. See
// http://www.mathertel.de/License.aspx More information on:
// http://www.mathertel.de/Arduino
// -----
// 01.12.2023 created from OneButtonTiny to support tiny environments.
// -----

#ifndef OneButton_h
#define OneButton_h

#include "stm32g0xx_hal.h"
#include <stdbool.h>

// ----- Callback function types -----

typedef void (*callbackFunction)(void);

// define FiniteStateMachine
typedef enum stateMachine_e {
  OCS_INIT = 0,
  OCS_DOWN = 1,  // button is down
  OCS_UP = 2,    // button is up
  OCS_COUNT = 3,
  OCS_PRESS = 6,  // button is hold down
  OCS_PRESSEND = 7,
} OBState;

// ----- Struct types -----
typedef struct OneButton_s
{
	bool _initDone;
	GPIO_TypeDef* _port;
	uint32_t _pin;                   // hardware pin number.
	unsigned int _debounce_ms;  // number of msecs for debounce times.
	unsigned int _click_ms;    // number of msecs before a click is detected.
	unsigned int _press_ms;    // number of msecs before a long button press is detected

	bool _buttonPressed;  // this is the level of the input pin when the button is pressed.
						   // LOW if the button connects the input pin to GND when pressed.
						   // HIGH if the button connects the input pin to VCC when pressed.

	// These variables will hold functions acting as event source.
	callbackFunction _clickFunc;
	callbackFunction _doubleClickFunc;
	callbackFunction _longPressStartFunc;

	OBState _state;

	bool debouncedPinLevel;
	bool _lastDebouncePinLevel;       // used for pin debouncing
	unsigned long _lastDebounceTime;  // tick counts
	unsigned long now;                // tick counts

	unsigned long _startTime;  // start of current input change to checking debouncing
	int _nClicks;              // count the number of clicks with this variable

} OneButton;


/**
* Initialize the OneButtonTiny library.
* @param btn Pointer to a OneButton struct
* @param port The GPIO port to be used
* @param pin The pin to be used for input from a momentary button.
* @param activeLow Set to true when the input level is LOW when the button is pressed, Default is true.
* @param pullupActive Activate the internal pullup when available. Default is true.
*/
void OneButtonInit(OneButton* btn, GPIO_TypeDef* const port, const uint32_t pin, const bool activeLow, const bool pullupActive);

/**
* set # millisec after safe click is assumed.
*/
void setDebounceMs(OneButton* btn, const unsigned int ms);

/**
* set # millisec after single click is assumed.
*/
void setClickMs(OneButton* btn, const unsigned int ms);

/**
* set # millisec after press is assumed.
*/
void setPressMs(OneButton* btn, const unsigned int ms);

// ----- Attach events functions -----

/**
* Attach an event to be called when a single click is detected.
* @param newFunction This function will be called when the event has been detected.
*/
void attachClick(OneButton* btn, callbackFunction newFunction);

/**
* Attach an event to be called after a double click is detected.
* @param newFunction This function will be called when the event has been detected.
*/
void attachDoubleClick(OneButton* btn, callbackFunction newFunction);

/**
* Attach an event to be called after a multi click is detected.
* @param newFunction This function will be called when the event has been detected.
*/
void attachMultiClick(OneButton* btn, callbackFunction newFunction);

/**
* Attach an event to fire when the button is pressed and held down.
* @param newFunction
*/
void attachLongPressStart(OneButton* btn, callbackFunction newFunction);


// ----- State machine functions -----

/**
* @brief Call this function every some milliseconds for checking the input
* level at the initialized digital pin.
*/
void tick(OneButton* btn);

/**
* @brief Call this function every time the input level has changed.
* Using this function no digital input pin is checked because the current
* level is given by the parameter.
* Run the finite state machine (FSM) using the given level.
*/
void tick_mock(OneButton* btn, bool level);


/**
* Reset the button state machine.
*/
void reset(OneButton* btn);


/**
* @return true if we are currently handling button press flow
* (This allows power sensitive applications to know when it is safe to power down the main CPU)
*/
bool isIdle(OneButton* btn);

/**
 * @brief Debounce input pin level for use in SpecialInput.
 */
int debounce(OneButton* btn, const bool value);


// Private functions not intended to be called directly

/**
* Run the finite state machine (FSM) using the given level.
*/
void _fsm(OneButton* btn, bool activeLevel);

/**
*  Advance to a new state.
*/
void _newState(OneButton* btn, OBState nextState);

#endif
