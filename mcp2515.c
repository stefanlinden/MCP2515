/*
 * mcp2515.c
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#include <driverlib.h>
#include <stdio.h>
#include "mcp2515.h"

#define MODULE EUSCI_B0_BASE
#define CS_PORT GPIO_PORT_P5
#define CS_PIN GPIO_PIN1
#define SPI_PORT GPIO_PORT_P1
#define SPI_PIN GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7

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

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig = {
EUSCI_B_SPI_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
		24000000, // SMCLK = DCO = 24MHz
		2000000, // SPICLK = 1MHz
		EUSCI_B_SPI_MSB_FIRST, // MSB First
		EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT, // Phase
		EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // Low polarity
		EUSCI_B_SPI_3PIN // 3Wire SPI Mode
		};

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
	/* Initialise the pins */
	MAP_GPIO_setAsOutputPin(CS_PORT, CS_PIN);
	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);

	/* Selecting P1.5 P1.6 and P1.7 in SPI mode */
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(SPI_PORT,
	SPI_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

	/* Configuring SPI in 3wire master mode */
	MAP_SPI_initMaster(MODULE, &spiMasterConfig);

	/* Enable SPI module */
	MAP_SPI_enableModule(MODULE);

	/* Enabling interrupts */
	MAP_SPI_clearInterruptFlag(MODULE,
	EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);
	MAP_SPI_enableInterrupt(MODULE,
	EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);
	MAP_Interrupt_enableInterrupt(INT_EUSCIB0);
	MAP_Interrupt_enableSleepOnIsrExit();

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
	DoTX = 1;

	/* Send the command */
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);

	MAP_SPI_transmitData(MODULE, 0xC0);
	while (DoTX)
		;
	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
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
	TXCount = 1;
	TXSize = 3;
	DoTX = 1;

	/* Perform transaction */
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
	MAP_SPI_transmitData(MODULE, TXData[0]);
	while (DoTX)
		;
	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);

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
	TXCount = 1;
	TXSize = 3;
	DoTX = 1;

	/* Perform transaction */
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
	MAP_SPI_transmitData(MODULE, TXData[0]);
	while (DoTX)
		;
	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);

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
	TXSize = 3;
	TXCount = 1;
	DoTX = 1;

	/* Perform transaction */
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
	MAP_SPI_transmitData(MODULE, CMD_WRITE);

	while (DoTX)
		;

	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
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
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
	MAP_SPI_transmitData(MODULE, CMD_BIT_MODIFY);

	while (DoTX)
		;

	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
	mode = 0;

	return;
}

void MCP_sendRTS(uint_fast8_t whichBuffer) {
	if (mode)
		return;

	mode = CMD_RTS;

	TXData[0] = CMD_RTS;
	TXData[0] |= whichBuffer;

	printf("Sending RTS: 0x%x\n", TXData[0]);

	TXCount = 1;
	TXSize = 1;
	DoTX = 1;

	/* Send the command */
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);

	MAP_SPI_transmitData(MODULE, 0xC0);
	while (DoTX)
		;
	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
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
		BufferState &= TXB;
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
		TXCount = 1;
		DoTX = 1;

		MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
		MAP_SPI_transmitData(MODULE, TXData[0]);

		while (DoTX)
			;

		MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
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

void EUSCIB0_ISR(void) {

	uint32_t status = MAP_SPI_getEnabledInterruptStatus(MODULE);
	MAP_SPI_clearInterruptFlag(MODULE, status);

	if (status & EUSCI_B_SPI_TRANSMIT_INTERRUPT) {
		if (DoTX && TXCount < TXSize) {
			MAP_SPI_transmitData(MODULE, TXData[TXCount]);
			TXCount++;
		}
	}

	if (status & EUSCI_B_SPI_RECEIVE_INTERRUPT) {
		RXData = MAP_SPI_receiveData(MODULE);
		if (DoTX && TXCount == TXSize) {
			TXCount = 0;
			DoTX = false;
		}
	}
}

void GPIOP5_ISR(void) {
	uint32_t status;

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

	if (status & GPIO_PIN0) {
		if (!mode) {
			//uint_fast8_t result = MCP_readStatus();
			//printf("Got an interrupt: 0x%x\n", result);
			printf("Got an interrupt.\n");
		}

	}
}
