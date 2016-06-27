/*
 * mcputil.h
 *
 *  Created on: 27 Jun 2016
 *      Author: Stefan van der Linden
 */

#ifndef INCLUDE_CANUTIL_H_
#define INCLUDE_CANUTIL_H_

/*** TYPEDEFs ***/

typedef struct {
	uint_fast32_t ID;
	uint_fast8_t isExtended;
	uint_fast8_t isRequest;
	uint_fast8_t length;
	uint_fast8_t * data;
} MCP_CANMessage;


/*** PROTOTYPES ***/

MCP_CANMessage createEmptyMessage( void );
void deleteMessage( MCP_CANMessage * );

#endif /* INCLUDE_CANUTIL_H_ */
