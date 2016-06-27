/*
 * mcp2515.c
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#include <driverlib.h>
#include <stdio.h>
#include <stdlib.h>
#include "mcp2515.h"

#include "simple_spi.h"

#define BUFFER_SIZE 	14

#define CMD_WRITE 		0x02
#define CMD_READ 		0x03
#define CMD_RESET 		0xC0
#define CMD_READ_STATUS 0xA0
#define CMD_BIT_MODIFY 	0x05
#define CMD_RTS 		0x80
#define CMD_LOAD_TX		0x40
#define CMD_READ_RX		0x90

volatile uint_fast8_t RXData;
volatile uint_fast8_t mode;
volatile uint_fast8_t TXData[BUFFER_SIZE];
volatile uint_fast8_t TXSize;

volatile uint_fast8_t BufferState;

void (*rcvdMsgHandler)(MCP_CANMessage *);

/*** PROTOTYPES ***/
uint_fast8_t _getAvailableTXB(void);

/*** HIGHER LEVEL FUNCTIONS ***/

void MCP_setTiming(const MCP_CANTimingConfig * TimingConfig) {
	uint_fast8_t buffer;
	uint_fast8_t CNF1 = 0;
	/* Make sure PS2 is always determined from the setting in CNF3 */
	uint_fast8_t CNF2 = 0x80;
	uint_fast8_t CNF3 = 0;

	/* CNF1: SJW(7:6) BRP(5:0) */
	CNF1 = 0x3F & (TimingConfig->BRP - 1);
	buffer = TimingConfig->SJW - 1;
	buffer <<= 6;
	CNF1 |= buffer;

	/* CNF2: BTLMODE(7) SAM(6) PHSEG1(5:3) PRSEG(2:0) */
	buffer = 0x07 & (TimingConfig->PS1 - 1);
	buffer <<= 3;
	CNF2 |= buffer;
	buffer = 0x07 & (TimingConfig->PROP - 1);

	/* CNF3: SOF(7) WAKFIL(6) PHSEG2(2:0) */
	CNF3 = 0x07 & (TimingConfig->PS2 - 1);

	/* Write to the register */
	MCP_writeRegister(RCNF1, CNF1);
	MCP_writeRegister(RCNF2, CNF2);
	MCP_writeRegister(RCNF3, CNF3);
}

void MCP_setMode(uint_fast8_t mode) {
	mode <<= 5;
	MCP_modifyBit(RCANCTRL, 0xE0, mode);
}

void MCP_enableInterrupt(uint_fast8_t interrupts) {
	MCP_modifyBit(RCANINTE, interrupts, interrupts);
}

void MCP_disableInterrupt(uint_fast8_t interrupts) {
	MCP_modifyBit(RCANINTE, interrupts, 0x00);
}

void MCP_clearInterrupt(uint_fast8_t interrupts) {
	MCP_modifyBit(RCANINTF, interrupts, 0x00);
}

uint_fast8_t MCP_getInterruptStatus(void) {
	return MCP_readRegister(RCANINTF);
}

void MCP_setReceivedMessageHandler(void (*handle)(MCP_CANMessage *)) {
	rcvdMsgHandler = handle;
}

/*** LOWER LEVEL FUNCTIONS ***/

void MCP_init(void) {
	/* Start the SPI module */
	MCP_SPI_startSPI();

	/* Enable the dedicated INT pin (active low) */
	MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN0);
	MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN0,
	GPIO_HIGH_TO_LOW_TRANSITION);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN0);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN0);
	MAP_Interrupt_enableInterrupt(INT_PORT5);

	/* Enabling MASTER interrupts */
	MAP_Interrupt_enableMaster();

	/* Initialise variables */
	RXData = 0;
	BufferState = 0;
}

uint_fast8_t MCP_reset(void) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 1;
	mode = CMD_RESET;

	/* Send the command */
	MCP_CS_LOW

	MCP_SPI_transmitByte(0xC0);

	MCP_CS_HIGH
	mode = 0;
	return 0;
}

uint_fast8_t MCP_readStatus(void) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 0xFF;

	mode = CMD_READ_STATUS;

	/* Populate the TX buffer */
	TXData[0] = CMD_READ_STATUS;
	TXData[1] = 0x00;
	TXData[2] = 0x00;

	/* Perform transaction */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, 3);
	MCP_CS_HIGH

	mode = 0;

	return RXData;
}

uint_fast8_t MCP_readRegister(uint_fast8_t address) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 0xFF;

	mode = CMD_READ;

	TXData[0] = CMD_READ;
	TXData[1] = address;
	TXData[2] = 0x00;

	/* Perform transaction */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, 3);
	MCP_CS_HIGH

	mode = 0;

	return RXData;
}

uint_fast8_t MCP_writeRegister(uint_fast8_t address, uint_fast8_t value) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 1;

	mode = CMD_WRITE;

	// By setting TXData, the data byte is sent once the tx buffer is available
	TXData[0] = CMD_WRITE;
	TXData[1] = address;
	TXData[2] = value;

	/* Perform transaction */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, 3);
	MCP_CS_HIGH

	mode = 0;

	return 0;
}

uint_fast8_t MCP_modifyBit(uint_fast8_t address, uint_fast8_t mask, uint_fast8_t value) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 1;

	mode = CMD_BIT_MODIFY;

	TXData[0] = CMD_BIT_MODIFY;
	TXData[1] = address;
	TXData[2] = mask;
	TXData[3] = value;
	TXSize = 4;

	/* Perform transaction */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, 4);
	MCP_CS_HIGH

	mode = 0;

	return 0;
}

uint_fast8_t MCP_sendRTS(uint_fast8_t whichBuffer) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 1;

	mode = CMD_RTS;

	TXData[0] = CMD_RTS;
	TXData[0] |= whichBuffer;

	/* Send the command */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitByte(TXData[0]);
	MCP_CS_HIGH
	mode = 0;

	return 0;
}

uint_fast8_t MCP_fillBuffer(uint_fast16_t sid, uint_fast8_t * data,
		uint_fast8_t length) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 0xFF;
	if (length > 8)
		return 0xFF;

	mode = CMD_LOAD_TX;

	uint_fast8_t TXB = _getAvailableTXB();
	uint_fast8_t ii;
	if (TXB != 0xFF) {
		BufferState |= TXB;
		/* TXB0 is a special case since it is equal to 0x00 */
		if (TXB == TXB0)
			TXB = 0;

		/* Prepare the transmit queue */
		TXData[0] = CMD_LOAD_TX | TXB; /* Command + Address */
		TXData[1] = (uint_fast8_t) (sid >> 3); /* SIDH */
		TXData[2] = (uint_fast8_t) (sid << 5); /* SIDL */
		TXData[3] = 0x00; /* EID8 */
		TXData[4] = 0x00; /* EID0 */
		TXData[5] = 0x0F & length; /* DLC  */

		/* Transmit actual data to buffer */
		for (ii = 0; ii < length; ii++) {
			TXData[6 + ii] = data[ii];
		}

		/* Do transaction */
		TXSize = 6 + length;

		/* Perform transaction */
		MCP_CS_LOW
		;
		RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, TXSize);
		MCP_CS_HIGH
		;

		mode = 0;
	}
	return TXB;
}

uint_fast8_t MCP_readBuffer(MCP_CANMessage * msgBuffer, uint_fast8_t RXB) {
	DELAY_WITH_TIMEOUT(mode);
	if (mode)
		return 1;
	mode = CMD_READ_RX;

	if (RXB == RXB0)
		RXB = 0x00;

	uint_fast8_t it;
	uint_fast8_t rxbuffer[9];
	if (!rxbuffer)
		return 1;

	TXData[0] = CMD_READ_RX + (RXB << 1);

	MCP_CS_LOW
	;
	MCP_SPI_transmitBytesReadAll(rxbuffer, (uint_fast8_t *) TXData, 6);
	MCP_CS_HIGH
	;
	msgBuffer->length = 0x0F & rxbuffer[5];
	msgBuffer->isExtended = 0x40 & rxbuffer[5];
	msgBuffer->isRequest = 0x10 & rxbuffer[2];

	if (msgBuffer->isExtended) {
		msgBuffer->ID = (uint_fast32_t) rxbuffer[4];
		msgBuffer->ID |= ((uint_fast32_t) rxbuffer[3]) << 8;
	} else {
		msgBuffer->ID = (uint_fast32_t) rxbuffer[2] >> 5;
		msgBuffer->ID |= ((uint_fast32_t) rxbuffer[1]) << 3;
	}

	// If there is no data attached to this message, then clean up return
	if (!msgBuffer->length) {
		//free(rxbuffer);
		return 0;
	}

	RXB++;
	TXData[0] = CMD_READ_RX + (RXB << 1);
	MCP_CS_LOW
	;
	MCP_SPI_transmitBytesReadAll(rxbuffer, (uint_fast8_t *) TXData,
			msgBuffer->length + 1);
	MCP_CS_HIGH
	;

	for (it = 0; it < msgBuffer->length; it++)
		msgBuffer->data[it] = rxbuffer[it + 1];

	mode = 0;
	return 0;
}

/*** PRIVATE FUNCTIONS ***/

uint_fast8_t _getAvailableTXB(void) {
	if (!(BufferState & TXB0))
		return 0x01;
	if (!(BufferState & TXB1))
		return 0x02;
	if (!(BufferState & TXB2))
		return 0x04;
	return 0xFF;
}

/*** ISR HANDLERS ***/

void GPIOP5_ISR(void) {
	uint32_t status;

	/* Check whether no other transaction is ongoing, skip (and return later) if this is the case */
	if (mode)
		return;

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

	if (status & GPIO_PIN0) {
		/* Read and clear the interrupt flags */
		uint_fast8_t CANStatus = MCP_getInterruptStatus();
		MCP_clearInterrupt(CANStatus);

		if (CANStatus & MCP_ISR_TX0IE) {
			BufferState &= ~TXB0;
		}

		if (CANStatus & MCP_ISR_TX1IE) {
			BufferState &= ~TXB1;
		}

		if (CANStatus & MCP_ISR_TX2IE) {
			BufferState &= ~TXB2;
		}

		if (CANStatus & MCP_ISR_RX0IE) {
			MCP_CANMessage msg = createEmptyMessage();
			MCP_readBuffer(&msg, RXB0);
			if (rcvdMsgHandler) {
				(*rcvdMsgHandler)(&msg);
			}
		}

		if (CANStatus & MCP_ISR_RX1IE) {
			MCP_CANMessage msg = createEmptyMessage();
			MCP_readBuffer(&msg, RXB1);
			if (rcvdMsgHandler) {
				(*rcvdMsgHandler)(&msg);
			}
		}
	}
}
