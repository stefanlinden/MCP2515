/*
 * mcputil.c
 *
 *  Created on: 27 Jun 2016
 *      Author: Stefan van der Linden
 */

#include <stdint.h>
#include <stdlib.h>
#include "canutil.h"

MCP_CANMessage createEmptyMessage( void ) {
	MCP_CANMessage msg = {
			0,
			0,
			0,
			0,
			(uint_fast8_t *) malloc(8)
	};
	return msg;
}
