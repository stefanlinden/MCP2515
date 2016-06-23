//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "msp.h"
#include <stdio.h>
#include <driverlib.h>
#include "mcp2515.h"
#include "clock.h"

uint_fast8_t RXData;
int i;
uint8_t mode;
uint_fast8_t result;

/**
 * Current CAN Timing Settings
 *
 * F_OSC: 24 MHz
 * BRP:   6
 * Sync:  1 TQ
 * Prop:  1 TQ
 * PS1:   3 TQ
 * PS2:   3 TQ
 * SJW:   1 TQ
 */

void main(void) {

	/* Halting the watchdog */
	MAP_WDT_A_holdTimer();

	printf("Starting...\r\n");

	/* Start the clock */
	startClockOnPin();

	/* Init the CAN controller */
	MCP_init();

	/* RESET */
	MCP_reset();
	printf("RESET\n");

	uint_fast8_t CANCTRL = MCP_readRegister(0x0F);
	printf("CANCTRL 0x0F: 0x%x\n", CANCTRL);

	/* Make sure configuration mode is set */
	CANCTRL |= 0x80;
	CANCTRL &= ~0x60;
	MCP_writeRegister(0x0F, CANCTRL);

	CANCTRL = MCP_readRegister(0x0F);
	printf("CANCTRL 0x0F: 0x%x\n", CANCTRL);

	/* Set the CAN BRP to 6 (five plus one) */
	uint_fast8_t CNF1 = 0x05;
	uint_fast8_t CNF2 = 0x90;
	uint_fast8_t CNF3 = 0x03;

	MCP_writeRegister(0x2A, CNF1);
	MCP_writeRegister(0x29, CNF2);
	MCP_writeRegister(0x28, CNF3);

	/* Register an interrupt on RX0*/
	MCP_writeRegister(0x2B, 0x01);
	result = MCP_readRegister(0x2B);
	printf("CANINTE: 0x%x\n", result);

	result = MCP_readRegister(0x2A);
	printf("CNF1: 0x%x\n", result);
	result = MCP_readRegister(0x29);
	printf("CNF2: 0x%x\n", result);
	result = MCP_readRegister(0x28);
	printf("CNF3: 0x%x\n", result);

	/* Activate loopback mode */
	CANCTRL |= 0x40;
	CANCTRL &= ~0xA0;
	MCP_writeRegister(0x0F, CANCTRL);

	result = MCP_readRegister(0x0E);
	printf("CANSTAT: 0x%x\n", result);

	/* Do the main loop */
	while (1) {
		/*for (i = 0; i < 500000; i++)
		 ;*/
		MAP_PCM_gotoLPM0();
		/*result = MCP_readRegister(0x0F);
		 printf("CANCTRL 0x0F: 0x%x\n", result);*/
	}

}
