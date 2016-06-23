/*
 * mcp2515.h
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#ifndef MCP2515_H_
#define MCP2515_H_

#define RTS_TXB0 0x01
#define RTS_TXB1 0x02
#define RTS_TXB2 0x04

/**
 * Initialise the SPI connection and MCP2515
 */
void MCP_init(void);

/**
 * Send a reset command
 */
void MCP_reset(void);

/**
 * Read the general status register
 */
uint_fast8_t MCP_readStatus(void);

/**
 * Read the specified register
 *
 * Parameters:
 * uint_fast8_t address: register address to read from
 */
uint_fast8_t MCP_readRegister(uint_fast8_t);

/**
 * Write to the specified register
 *
 * Parameters:
 * uint_fast8_t address: register address to write to
 * uint_fast8_t value: value to write to the register
 */
void MCP_writeRegister(uint_fast8_t, uint_fast8_t);

/**
 * Write to a register through a bit modify operation
 *
 * Parameters:
 * uint_fast8_t address: register address to perform the operation on
 * uint_fast8_t mask: the bit mask
 * uint_fast8_t value: the new values (taking into account the mask)
 */
void MCP_modifyBit(uint_fast8_t, uint_fast8_t, uint_fast8_t);

/**
 * Send an RTS command
 *
 * Parameters:
 * uint_fast8_t whichBuffer: bitwise OR of any of the following: RTS_TXB0, RTS_TXB1, RTS_TXB2
 */
void MCP_sendRTS(uint_fast8_t whichBuffer);


#endif /* MCP2515_H_ */
