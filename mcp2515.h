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

/* Often-used register addresses */
#define REFLG			0x2D
#define RCANSTAT		0x0E
#define RCANCTRL 		0x0F
#define RCANINTE 		0x2B
#define RCANINTF 		0x2C
#define RCNF1 			0x2A
#define RCNF2			0x29
#define RCNF3			0x28
#define RREC			0x1D
#define RTEC			0x1C

#define RTXB0CTRL 		0x30
#define RTXB1CTRL 		0x40
#define RTXB2CTRL 		0x50
#define RRXB0CTRL 		0x60
#define RRXB1CTRL		0x70

/* Buffers */
#define TXB0			0x01
#define TXB1			0x02
#define TXB2			0x04
#define RXB0			0x08
#define RXB1			0x10

/* Modes */
#define MODE_NORMAL 	0x00
#define MODE_SLEEP  	0x01
#define MODE_LOOPBACK	0x02
#define MODE_LISTENONLY	0x03
#define MODE_CONFIG		0x04

/* Interrupts */
#define MCP_ISR_MERRE 	0x80
#define MCP_ISR_WAKIE 	0x40
#define MCP_ISR_ERRIE	0x20
#define MCP_ISR_TX2IE	0x10
#define MCP_ISR_TX1IE	0x08
#define MCP_ISR_TX0IE	0x04
#define MCP_ISR_RX1IE	0x02
#define MCP_ISR_RX0IE	0x01

/*** TYPEDEFS ***/

typedef struct {
	/* All values are in time quanta (TQ), except F_OSC */

	/* Oscillator frequency */
	uint_fast32_t F_OSC;
	/* Baud rate prescaler */
	uint_fast8_t BRP;
	/* Propagation delay */
	uint_fast8_t PROP;
	/* Phase segment 1 */
	uint_fast8_t PS1;
	/* Phase segment 2 */
	uint_fast8_t PS2;
	/* Synchronisation jump width */
	uint_fast8_t SJW;
} MCP_CANTimingConfig;

/*** HIGHER LEVEL FUNCTION PROTOTYPES ***/

/**
 * Configure the timing values for the CAN protocol
 *
 * Parameters:
 * const MCP_CANTimingConfig * TimingConfig: struct containing the timing settings
 */
void MCP_setTiming(const MCP_CANTimingConfig *);

/**
 * Set the mode of the MCP2515
 *
 * Parameters:
 * uint_fast8_t mode: one of the following: MODE_NORMAL, MODE_SLEEP, MODE_LOOPBACK, MODE_LISTENONLY, MODE_CONFIG
 */
void MCP_setMode(uint_fast8_t);

/**
 * Enable interrupts
 *
 * Parameters:
 * uint_fast8_t interrupts: the interrupt(s) to enable
 */
void MCP_enableInterrupt(uint_fast8_t interrupts);

/**
 * Disable interrupts
 *
 * Parameters:
 * uint_fast8_t interrupts: the interrupt(s) to disable
 */
void MCP_disableInterrupt(uint_fast8_t interrupts);

/**
 * Clear interrupts
 *
 * Parameters:
 * uint_fast8_t interrupts: the interrupt(s) to clear
 */
void MCP_clearInterrupt(uint_fast8_t interrupts);

/**
 * Get the status byte containing all the interrupt flags
 *
 * Returns:
 * uint_fast8_t: the interrupt status (the CANINTF register)
 */
uint_fast8_t MCP_getInterruptStatus(void);

/**
 * Fill the next available buffer with the given headers and data
 *
 * Parameters:
 * uint_fast16_t sid: the standard identifier (only 11 bits supported)
 * uint_fast8_t * data: byte array with data
 * uint_fast8_t length: number of bytes in the data array
 *
 * Returns:
 * The buffer id of the loaded buffer or 0xFF if failed
 */
uint_fast8_t MCP_fillBuffer(uint_fast16_t sid, uint_fast8_t * data, uint_fast8_t length);
/*** LOWER LEVEL FUNCTION PROTOTYPES ***/

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
