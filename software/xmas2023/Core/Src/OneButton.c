/**
 * Adopted from OneButtonTiny. Part of the OneButton Arduino library
 * available at https://github.com/mathertel/OneButton/
 *
 * Converted to generic microcontroller (non-Arduino) use and C language
 * @author David Miller
 *
 * Original header information follows:
 */

/**
 * @file OneButton.cpp
 *
 * @brief Library for detecting button clicks, doubleclicks and long press
 * pattern on a single button.
 *
 * @author Matthias Hertel, https://www.mathertel.de
 * @Copyright Copyright (c) by Matthias Hertel, https://www.mathertel.de.
 *                          Ihor Nehrutsa, Ihor.Nehrutsa@gmail.com
 *
 * This work is licensed under a BSD style license. See
 * http://www.mathertel.de/License.aspx
 *
 * More information on: https://www.mathertel.de/Arduino/OneButtonLibrary.aspx
 *
 * Changelog: see OneButtonTiny.h
 */



#include "OneButton.h"

// ----- Initialization and Default Values -----
/**
 * Enable a GPIO clock for a given port
 */
static void OneButtonGPIOClkInit(const GPIO_TypeDef* port)
{
	if(port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(port == GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	else if(port == GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
	else if(port == GPIOF) {
		__HAL_RCC_GPIOF_CLK_ENABLE();
	}
}

/**
 * Initialize the OneButton library.
 * @param pin The pin to be used for input from a momentary button.
 * @param activeLow Set to true when the input level is LOW when the button is pressed, Default is true.
 * @param pullupActive Activate the internal pullup when available. Default is true.
 */
void OneButtonInit(OneButton* btn, GPIO_TypeDef* const port, const uint32_t pin, const bool activeLow, const bool pullupActive) {
	// Initialize default values
	btn->_debounce_ms = 50;
	btn->_click_ms = 400;
	btn->_press_ms = 800;

	btn->_clickFunc = NULL;
	btn->_doubleClickFunc = NULL;
	btn->_longPressStartFunc = NULL;

	btn->_state = OCS_INIT;

	btn->debouncedPinLevel = false;
	btn->_lastDebouncePinLevel = false;
	btn->_lastDebounceTime = 0;
	btn->now = 0;

	btn->_startTime = 0;
	btn->_nClicks = 0;

	btn->_port = port;
	btn->_pin = pin;

	// Select GND = pressed with activeLow, VCC = pressed otherwise
	btn->_buttonPressed = !(activeLow);

	OneButtonGPIOClkInit(btn->_port);

	(btn->_port)->MODER &= ~(3UL << (2*btn->_pin)); // clear 2-bit MODE for this pin = input mode
	if (pullupActive) {
		// use the given pin as input and activate internal PULLUP resistor.
		(btn->_port)->PUPDR &= ~(3UL << (2*btn->_pin));
	}

	btn->_initDone = true;
}  // OneButton


// explicitly set the number of millisec that have to pass by before a click is assumed stable.
void setDebounceMs(OneButton* btn, const unsigned int ms) {
	btn->_debounce_ms = ms;
}  // setDebounceMs


// explicitly set the number of millisec that have to pass by before a click is detected.
void setClickMs(OneButton* btn, const unsigned int ms) {
	btn->_click_ms = ms;
}  // setClickMs


// explicitly set the number of millisec that have to pass by before a long button press is detected.
void setPressMs(OneButton* btn, const unsigned int ms) {
	btn->_press_ms = ms;
}  // setPressMs


// save function for click event
void attachClick(OneButton* btn, callbackFunction newFunction) {
	btn->_clickFunc = newFunction;
}  // attachClick


// save function for doubleClick event
void attachDoubleClick(OneButton* btn, callbackFunction newFunction) {
	btn->_doubleClickFunc = newFunction;
}  // attachDoubleClick


// save function for longPressStart event
void attachLongPressStart(OneButton* btn, callbackFunction newFunction) {
	btn->_longPressStartFunc = newFunction;
}  // attachLongPressStart


void reset(OneButton* btn) {
	btn->_state = OCS_INIT;
	btn->_nClicks = 0;
	btn->_startTime = 0;
}

bool isIdle(OneButton* btn) {
  return btn->_state == OCS_INIT;
}
/**
 * @brief Debounce input pin level for use in SpecialInput.
 */
int debounce(OneButton* btn, const bool value) {
  btn->now = HAL_GetTick();  // current (relative) time in msecs.
  if (btn->_lastDebouncePinLevel == value) {
    if (btn->now - btn->_lastDebounceTime >= btn->_debounce_ms)
    	btn->debouncedPinLevel = value;
  } else {
	  btn->_lastDebounceTime = btn->now;
	  btn->_lastDebouncePinLevel = value;
  }
  return btn->debouncedPinLevel;
};


/**
 * @brief Check input of the configured pin,
 * debounce input pin level and then
 * advance the finite state machine (FSM).
 */
void tick(OneButton* btn) {
	bool pressed = false;
	GPIO_TypeDef* port = btn->_port;
	if (((port->IDR) & (1UL << (btn->_pin))) != 0x00u) {
		// button level is high / VCC
		// so we are pressed if we are active high
		pressed = btn->_buttonPressed;
	} else {
		// button level is low / GND
		// so we are pressed only if active low
		pressed = !(btn->_buttonPressed);
	}
	_fsm(btn, debounce(btn, pressed));
}  // tick()


void tick_mock(OneButton* btn, bool activeLevel) {
  _fsm(btn, debounce(btn, activeLevel));
}


/**
 *  @brief Advance to a new state and save the last one to come back in cas of bouncing detection.
 */
void _newState(OneButton* btn, OBState nextState) {
  btn->_state = nextState;
}  // _newState()


/**
 * @brief Run the finite state machine (FSM) using the given level.
 */
void _fsm(OneButton* btn, bool activeLevel) {
  unsigned long waitTime = (btn->now - btn->_startTime);

  // Implementation of the state machine
  switch (btn->_state) {
    case OCS_INIT:
      // waiting for level to become active.
      if (activeLevel) {
        btn->_state = OCS_DOWN;
        btn->_startTime = btn->now;  // remember starting time
        btn->_nClicks = 0;
      }  // if
      break;

    case OCS_DOWN:
      // waiting for level to become inactive.

      if (!activeLevel) {
        btn->_state = OCS_UP;
        btn->_startTime = btn->now;  // remember starting time

      } else if ((activeLevel) && (waitTime > btn->_press_ms)) {
        if (btn->_longPressStartFunc) btn->_longPressStartFunc();
        btn->_state = OCS_PRESS;
      }  // if
      break;

    case OCS_UP:
      // level is inactive

      // count as a short button down
    	btn->_nClicks++;
      btn->_state = OCS_COUNT;
      break;

    case OCS_COUNT:
      // dobounce time is over, count clicks

      if (activeLevel) {
        // button is down again
        btn->_state = OCS_DOWN;
        btn->_startTime = btn->now;  // remember starting time

      } else if ((waitTime >= btn->_click_ms) || (btn->_nClicks == 2)) {
        // now we know how many clicks have been made.

        if (btn->_nClicks == 1) {
          // this was 1 click only.
          if (btn->_clickFunc) btn->_clickFunc();

        } else if (btn->_nClicks == 2) {
          // this was a 2 click sequence.
          if (btn->_doubleClickFunc) btn->_doubleClickFunc();

        }  // if

        reset(btn);
      }  // if
      break;

    case OCS_PRESS:
      // waiting for pin being release after long press.

      if (!activeLevel) {
        btn->_state = OCS_PRESSEND;
        btn->_startTime = btn->now;
      }  // if
      break;

    case OCS_PRESSEND:
      // button was released.
      reset(btn);
      break;

    default:
      // unknown state detected -> reset state machine
      btn->_state = OCS_INIT;
      break;
  }  // if

}  // OneButton.tick()


// end.
