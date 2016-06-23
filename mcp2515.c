/*
 * mcp2515.c
 *
 *  Created on: 17 Jun 2016
 *      Author: Stefan van der Linden
 */

#include <driverlib.h>
#include <stdio.h>

#define MODULE EUSCI_B0_BASE
#define CS_PORT GPIO_PORT_P5
#define CS_PIN GPIO_PIN1
#define SPI_PORT GPIO_PORT_P1
#define SPI_PIN GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7

#define BUFFER_SIZE 4

#define CMD_WRITE 0x02
#define CMD_READ 0x03
#define CMD_RESET 0xC0
#define CMD_READ_STATUS 0xA0
#define CMD_BIT_MODIFY 0x05
#define CMD_RTS 0x80

volatile uint_fast8_t RXData;
volatile uint_fast8_t mode;
volatile uint_fast8_t TXData[BUFFER_SIZE];
volatile uint_fast8_t TXCount;
volatile uint_fast8_t TXSize;
volatile uint_fast8_t DoTX;

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig = {
EUSCI_B_SPI_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
		24000000, // SMCLK = DCO = 24MHz
		1000000, // SPICLK = 1MHz
		EUSCI_B_SPI_MSB_FIRST, // MSB First
		EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT, // Phase
		EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // Low polarity
		EUSCI_B_SPI_3PIN // 3Wire SPI Mode
		};

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
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN0);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN0);
	MAP_Interrupt_enableInterrupt(INT_PORT5);

	/* Initialise variables */
	RXData = 0;
	DoTX = 0;
	TXCount = 0;
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
	MAP_SPI_transmitData(MODULE, CMD_WRITE);

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
	DoTX = 0;

	TXData[0] = CMD_RTS;
	TXData[0] |= whichBuffer;

	/* Perform transaction */
	MAP_GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
	MAP_SPI_transmitData(MODULE, CMD_WRITE);
	MAP_GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);

	mode = 0;

	return;
}

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

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

	/* Toggling the output on the LED */
	if (status & GPIO_PIN0) {
		printf("Got an interrupt!");
	}
}
