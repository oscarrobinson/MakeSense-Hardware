/**
 * \addtogroup orisen-stamp-io-pins Orisen Stamp IO Pins
 *
 * @{
 */

/**
 * \file
 *				This code allows manipulation of the io pins on
 *				the Orisen Stamp
 * \author
 *         Stephen Hailes
 */


#include "contiki.h"
#include "io-pins.h"
#include "kbi_events.h"


/*
 * This is where we store the configuration information.
 * Note, there are 16 pins numbered 1-16 rather than 0-17.
 */
static ioPinConfig ioPins[16];

/*
 * All variables and functions that are visible outside of the file
 * should have the module name prepended to them. This makes it easy
 * to know where to look for function and variable definitions.
 *
 * Put dividers (a single-line comment consisting only of dashes)
 * between functions. 
 */
/*---------------------------------------------------------------------------*/
/**
 * \brief      Use Doxygen documentation for functions.
 * \param c    Briefly describe all parameters.
 * \return     Briefly describe the return value.
 * \retval 0   Functions that return a few specified values
 * \retval 1   can use the \retval keyword instead of \return.
 *
 *             Put a longer description of what the function does
 *             after the preamble of Doxygen keywords.
 *
 *             This template should always be used to document
 *             functions. The text following the introduction is used
 *             as the function's documentation.
 *
 *             Function prototypes have the return type on one line,
 *             the name and arguments on one line (with no space
 *             between the name and the first parenthesis), followed
 *             by a single curly bracket on its own line.
 */
void ioPins_init()
{
  // There are 16 pins to configure. We choose the initial values
  // as set in the header file. Note that the pin number runs from
  // 1-16 rather than 0-15
  //
  uint8_t i;
  for (i = 0; i < 16; i++) {
    ioPinConfig p = initialSettings[i].config;
	  ioPins[i] = p;
		ioPins_configurePin(i+1, p.function, p.direction, p.pullup, p.hysteresis);
  }

	kbi_event_init();			  // Deal with the kbi6/7 setup
	adc_init();							// Initialise the ADC channels
}

/*---------------------------------------------------------------------------*/
/*
 * Static (non-global) functions do not need Doxygen comments. The
 * name should not be prepended with the module name - doing so would
 * create confusion. 
 */
void ioPins_configurePin(uint8_t pin, pinFunction function, pinDirection direction, pinPullup pullup, pinHysteresis hysteresis)
{ uint8_t f, d, p, e, h;

	// Pins run from T1-T16. Do nothing if we're outside that range
  //
  if (pin < 1 || pin > 16)
    return;

  // Set the function - it can only be one of two things on this
  // processor - GPIO or an alternate.
  //
  
	switch (function) {
		case USEGPIO:
			f = 0;
			break;
		case ALTERNATE:
      f = 1;
			break;
  }

	ioPins[pin-1].function = function;
	
	// Set direction, pullup and hysteresis.
  //
  // We restrict users to setting hysteresis and pullup/down
  // if and only if this is an input. Else we switch them
  // both off.
  //
	switch (direction) {
		case INPUT:
			d = 0;

			switch (pullup) {
				case PULLUP:
					p = 1;
					e = 1;
					break;
				case PULLDOWN:
					p = 0;
					e = 1;
					break;
				case NOPULLUP:
					p = 0;
					e = 0;
					break;
			}

			switch (hysteresis) {
				case HYSTERESIS_ON:
					h = 1;
					break;
				case HYSTERESIS_OFF:
					h = 0;
					break;
			}	

			ioPins[pin-1].direction  = direction;
			ioPins[pin-1].pullup     = pullup;
			ioPins[pin-1].hysteresis = hysteresis;
			break;

		case OUTPUT:
      d = 1;
			p = 0;
			e = 0;
			h = 0;

			ioPins[pin-1].direction  = direction;
			ioPins[pin-1].pullup     = NOPULLUP;
			ioPins[pin-1].hysteresis = HYSTERESIS_OFF;
			break;
  }

  switch (pin) {
    case 1:
			SET_IO_PIN(GPIO_13, f, d, p, e, h);
			break;
    case 2:
			SET_IO_PIN(GPIO_12, f, d, p, e, h);
			break;
    case 3:
			SET_IO_PIN(GPIO_09, f, d, p, e, h);
			break;
    case 4:
			SET_IO_PIN(GPIO_08, f, d, p, e, h);
			break;
    case 5:
			SET_IO_PIN(GPIO_18, f, d, p, e, h);
			break;
    case 6:
			SET_IO_PIN(GPIO_19, f, d, p, e, h);
			break;
    case 7:
			SET_IO_PIN(GPIO_21, f, d, p, e, h);
			break;
    case 8:
			SET_IO_PIN(GPIO_20, f, d, p, e, h);
			break;
    case 9:
			SET_IO_PIN(GPIO_29, f, d, p, e, h);
			break;
    case 10:
			SET_IO_PIN(GPIO_28, f, d, p, e, h);
			break;
    case 11:
			SET_IO_PIN(GPIO_04, f, d, p, e, h);
			break;
    case 12:
			SET_IO_PIN(GPIO_05, f, d, p, e, h);
			break;
    case 13:
			SET_IO_PIN(GPIO_06, f, d, p, e, h);
			break;
    case 14:
			SET_IO_PIN(GPIO_07, f, d, p, e, h);
			break;
    case 15:
			SET_IO_PIN(GPIO_30, f, d, p, e, h);
			break;
    case 16:
			SET_IO_PIN(GPIO_31, f, d, p, e, h);
			break;
  }
}


/*---------------------------------------------------------------------------*/
/*
 * Static (non-global) functions do not need Doxygen comments. The
 * name should not be prepended with the module name - doing so would
 * create confusion. 
 */
void ioPins_setValue(uint8_t pin, uint8_t value)
{
	// Pins run from T1-T16. Do nothing if we're outside that range
  //
  if (pin < 1 || pin > 16)
    return;

  // Check to see whether the pin is an output. If not, we don't
  // set its value
  //
  if (ioPins[pin-1].direction != OUTPUT)
    return;

  uint8_t on = (value & 0x01);

  // Now set the value from the lowest bit of the pin.
  //
  switch (pin) {
    case 1:
			if (on)
				GPIO->DATA_SET.GPIO_13   = 1;
			else
				GPIO->DATA_RESET.GPIO_13 = 1;
			break;
    case 2:
			if (on)
				GPIO->DATA_SET.GPIO_12   = 1;
			else
				GPIO->DATA_RESET.GPIO_12 = 1;
			break;
    case 3:
			if (on)
				GPIO->DATA_SET.GPIO_09   = 1;
			else
				GPIO->DATA_RESET.GPIO_09 = 1;
			break;
    case 4:
			if (on)
				GPIO->DATA_SET.GPIO_08   = 1;
			else
				GPIO->DATA_RESET.GPIO_08 = 1;
			break;
    case 5:
			if (on)
				GPIO->DATA_SET.GPIO_18   = 1;
			else
				GPIO->DATA_RESET.GPIO_18 = 1;
			break;
    case 6:
			if (on)
				GPIO->DATA_SET.GPIO_19   = 1;
			else
				GPIO->DATA_RESET.GPIO_19 = 1;
			break;
    case 7:
			if (on)
				GPIO->DATA_SET.GPIO_21   = 1;
			else
				GPIO->DATA_RESET.GPIO_21 = 1;
			break;
    case 8:
			if (on)
				GPIO->DATA_SET.GPIO_20   = 1;
			else
				GPIO->DATA_RESET.GPIO_20 = 1;
			break;
    case 9:
			if (on)
				GPIO->DATA_SET.GPIO_29   = 1;
			else
				GPIO->DATA_RESET.GPIO_29 = 1;
			break;
    case 10:
			if (on)
				GPIO->DATA_SET.GPIO_28   = 1;
			else
				GPIO->DATA_RESET.GPIO_28 = 1;
			break;
    case 11:
			if (on)
				GPIO->DATA_SET.GPIO_04   = 1;
			else
				GPIO->DATA_RESET.GPIO_04 = 1;
			break;
    case 12:
			if (on)
				GPIO->DATA_SET.GPIO_05   = 1;
			else
				GPIO->DATA_RESET.GPIO_05 = 1;
			break;
    case 13:
			if (on)
				GPIO->DATA_SET.GPIO_06   = 1;
			else
				GPIO->DATA_RESET.GPIO_06 = 1;
			break;
    case 14:
			if (on)
				GPIO->DATA_SET.GPIO_07   = 1;
			else
				GPIO->DATA_RESET.GPIO_07 = 1;
			break;
    case 15:
			if (on)
				GPIO->DATA_SET.GPIO_30   = 1;
			else
				GPIO->DATA_RESET.GPIO_30 = 1;
			break;
    case 16:
			if (on)
				GPIO->DATA_SET.GPIO_31   = 1;
			else
				GPIO->DATA_RESET.GPIO_31 = 1;
			break;
  }
}


/*---------------------------------------------------------------------------*/
/*
 * Static (non-global) functions do not need Doxygen comments. The
 * name should not be prepended with the module name - doing so would
 * create confusion. 
 */
uint32_t ioPins_getValue(uint8_t pin)
{
	// Pins run from T1-T16. Do nothing if we're outside that range
  //
  if (pin < 1 || pin > 16)
    return;

  // Now get the value from the pin... if it's configured in an
  // appropriate way
  //
  switch (pin) {
    case 1:
      if (ioPins[0].function == USEGPIO)
			  return (GPIO->DATA.GPIO_13);
			break;
    case 2:
      if (ioPins[1].function == USEGPIO)
			  return (GPIO->DATA.GPIO_12);
			break;
    case 3:
      if (ioPins[2].function == USEGPIO)
			  return (GPIO->DATA.GPIO_09);
			break;
    case 4:
      if (ioPins[3].function == USEGPIO)
			  return (GPIO->DATA.GPIO_08);
			break;
    case 5:
      if (ioPins[4].function == USEGPIO)
			  return (GPIO->DATA.GPIO_18);
			break;
    case 6:
      if (ioPins[5].function == USEGPIO)
			  return (GPIO->DATA.GPIO_19);
			break;
    case 7:
      if (ioPins[6].function == USEGPIO)
			  return (GPIO->DATA.GPIO_21);
			break;
    case 8:
      if (ioPins[7].function == USEGPIO)
			  return (GPIO->DATA.GPIO_20);
			break;
    case 9:
      if (ioPins[8].function == USEGPIO)
			  return (GPIO->DATA.GPIO_29);
			break;
    case 10:
      if (ioPins[9].function == USEGPIO)
			  return (GPIO->DATA.GPIO_28);
			break;
    case 11:
      if (ioPins[10].function == USEGPIO)
			  return (GPIO->DATA.GPIO_04);
			break;
    case 12:
      if (ioPins[11].function == USEGPIO)
			  return (GPIO->DATA.GPIO_05);
			break;
    case 13:
      if (ioPins[12].function == USEGPIO)
			  return (GPIO->DATA.GPIO_06);
			break;
    case 14:
      if (ioPins[13].function == USEGPIO)
			  return (GPIO->DATA.GPIO_07);
			break;
    case 15:
      if (ioPins[14].function == USEGPIO)
			  return (GPIO->DATA.GPIO_30);
			else {
				adc_service();
				return (adc_reading[0]);
			}
			break;
    case 16:
      if (ioPins[15].function == USEGPIO)
			  return (GPIO->DATA.GPIO_31);
			else {
				adc_service();
				return (adc_reading[1]);
			}
			break;
  }

  return (0xFFFFFFFF);
}

uint32_t ioPins_getBatt()
{
	adc_service();
	return(adc_reading[8]);
}

/*---------------------------------------------------------------------------*/

/** @} */
