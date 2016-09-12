/*
 * mcp2515.h
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#ifndef MCP2515_H_
#define MCP2515_H_

#include "canutil.h"

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
#define RTXB0SIDH		0x31
#define RTXB0SIDL		0x32
#define RTXB0EID8		0x33
#define RTXB0EID0		0x34
#define RTXB0DLC		0x35

#define RTXB1CTRL 		0x40
#define RTXB1SIDH		0x41
#define RTXB1SIDL		0x42
#define RTXB1EID8		0x43
#define RTXB1EID0		0x44
#define RTXB1DLC		0x45

#define RTXB2CTRL 		0x50
#define RTXB2SIDH		0x51
#define RTXB2SIDL		0x52
#define RTXB2EID8		0x53
#define RTXB2EID0		0x54
#define RTXB2DLC		0x55

#define RRXB0CTRL 		0x60
#define RRXB1CTRL		0x70

/* Buffers */
#define TXB0			0x01
#define TXB1			0x02
#define TXB2			0x04
#define RXB0			0x01
#define RXB1			0x02

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
 * Enable the master interrupt for the MCP2515
 */
void MCP_enableMasterInterrupt(void);

/**
 * Disable the master interrupt for the MCP2515
 */
void MCP_disableMasterInterrupt(void);

/**
 * Register the received message handler
 *
 * Parameters:
 * void (*handle)(MCP_CANMessage *): a function pointer to be called
 */
void MCP_setReceivedMessageHandler(void (*handle)(MCP_CANMessage *));

/**
 * Returns whether a TX buffer is available or not
 *
 * Returns:
 * uint_fast8_t: true if there is a TX buffer available, false otherwise
 */
uint_fast8_t MCP_isTXBufferAvailable(void);

/**
 * Fill the next available buffer with the given headers and data
 *
 * Parameters:
 * uint_fast16_t * msg: the message to transmit
 *
 * Returns:
 * The buffer id of the loaded buffer or 0xFF if failed
 */
uint_fast8_t MCP_fillBuffer(MCP_CANMessage *);
/*** LOWER LEVEL FUNCTION PROTOTYPES ***/

/**
 * Initialise the SPI connection and MCP2515
 */
void MCP_init(void);

/**
 * Send a reset command
 *
 * Returns:
 * uint_fast8_t: 0 on success or an error code otherwise
 */
uint_fast8_t MCP_reset(void);

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
 *
 * Returns:
 * uint_fast8_t: 0 on success or an error code otherwise
 */
uint_fast8_t MCP_writeRegister(uint_fast8_t, uint_fast8_t);

/**
 * Write to a register through a bit modify operation
 *
 * Parameters:
 * uint_fast8_t address: register address to perform the operation on
 * uint_fast8_t mask: the bit mask
 * uint_fast8_t value: the new values (taking into account the mask)
 *
 * Returns:
 * uint_fast8_t: 0 on success or an error code otherwise
 */
uint_fast8_t MCP_modifyBit(uint_fast8_t, uint_fast8_t, uint_fast8_t);

/**
 * Send an RTS command
 *
 * Parameters:
 * uint_fast8_t whichBuffer: bitwise OR of any of the following: RTS_TXB0, RTS_TXB1, RTS_TXB2
 *
 * Returns:
 * uint_fast8_t: 0 on success or an error code otherwise
 */
uint_fast8_t MCP_sendRTS(uint_fast8_t whichBuffer);

#endif /* MCP2515_H_ */
