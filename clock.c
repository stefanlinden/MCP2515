/*
 * clock.c
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#include "clock.h"

#include <driverlib.h>

/** Start the clock on HSMCLK */
void startClockOnPin( void ) {
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4,
	GPIO_PRIMARY_MODULE_FUNCTION);
}

