/*
 * mcp2515.c
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#include <driverlib.h>
#include <stdio.h>
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

volatile uint_fast8_t RXData;
volatile uint_fast8_t mode;
volatile uint_fast8_t TXData[BUFFER_SIZE];
volatile uint_fast8_t TXCount;
volatile uint_fast8_t TXSize;
volatile uint_fast8_t DoTX;

volatile uint_fast8_t BufferState;



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
	DoTX = 0;
	TXCount = 0;
	BufferState = 0;
}

void MCP_reset(void) {
	if (mode)
		return;
	mode = CMD_RESET;

	TXCount = 1;
	TXSize = 1;

	/* Send the command */
	MCP_CS_LOW

	MCP_SPI_transmitByte(0xC0);

	MCP_CS_HIGH
	mode = 0;
	return;
}

uint_fast8_t MCP_readStatus(void) {
	if (mode)
		return 0xFF;

	mode = CMD_READ_STATUS;

	/* Populate the TX buffer */
	TXData[0] = CMD_READ_STATUS;
	TXData[1] = 0x00;
	TXData[2] = 0x00;

	/* Perform transaction */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitBytes((uint_fast8_t *)TXData, 3);
	MCP_CS_HIGH

	mode = 0;

	return RXData;
}

uint_fast8_t MCP_readRegister(uint_fast8_t address) {
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

void MCP_writeRegister(uint_fast8_t address, uint_fast8_t value) {
	if (mode)
		return;

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

	return;
}

void MCP_modifyBit(uint_fast8_t address, uint_fast8_t mask, uint_fast8_t value) {
	if (mode)
		return;

	mode = CMD_BIT_MODIFY;

	TXData[0] = CMD_BIT_MODIFY;
	TXData[1] = address;
	TXData[2] = mask;
	TXData[3] = value;
	TXSize = 4;
	TXCount = 1;
	DoTX = 1;

	/* Perform transaction */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, 4);
	MCP_CS_HIGH

	mode = 0;

	return;
}

void MCP_sendRTS(uint_fast8_t whichBuffer) {
	if (mode)
		return;

	mode = CMD_RTS;

	TXData[0] = CMD_RTS;
	TXData[0] |= whichBuffer;

	/* Send the command */
	MCP_CS_LOW
	RXData = MCP_SPI_transmitByte(TXData[0]);
	MCP_CS_HIGH
	mode = 0;

	return;
}

uint_fast8_t MCP_fillBuffer(uint_fast16_t sid, uint_fast8_t * data,
		uint_fast8_t length) {
	if (mode)
		return 0xFF;
	if (length > 8)
		return 0xFF;

	mode = CMD_LOAD_TX;

	uint_fast8_t TXB = _getAvailableTXB();
	uint_fast8_t ii;
	if (TXB != 0xFF) {
		BufferState |= TXB;
		if(TXB == TXB0)
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
		TXSize = 6+length;

		/* Perform transaction */
		MCP_CS_LOW;
		RXData = MCP_SPI_transmitBytes((uint_fast8_t *) TXData, TXSize);
		MCP_CS_HIGH;

		mode = 0;
	}
	return TXB;
}

/*** PRIVATE FUNCTIONS ***/

uint_fast8_t _getAvailableTXB(void) {
	if (!(BufferState & 0x01))
		return 0x01;
	if (!(BufferState & 0x02))
		return 0x02;
	if (!(BufferState & 0x04))
		return 0x04;
	return 255;
}

/*** ISR HANDLERS ***/

void GPIOP5_ISR(void) {
	uint32_t status;

	if(mode)
		return;

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

	if (status & GPIO_PIN0) {
		uint_fast8_t result = MCP_getInterruptStatus();
		printf("Got an interrupt: 0x%x\n", result);
	}
}
