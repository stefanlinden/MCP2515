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

/* SPI Timing Config */
const MCP_CANTimingConfig CANTimingConfig = { 24000000, /* Oscillator Frequency */
6, /* Baud Rate Prescaler */
1, /* Propagation Delay */
3, /* Phase Segment 1 */
3, /* Phase Segment 2 */
1 /* Synchronisation Jump Width */
};

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

	/* Make sure configuration mode is set */
	MCP_setMode(MODE_CONFIG);

	MCP_setTiming(&CANTimingConfig);

	/* Register an interrupt on RX0 and TX0 */
	result = MCP_readRegister(RCANINTE);
	printf("CANINTE: 0x%x\n", result);
	MCP_enableInterrupt(MCP_ISR_TX0IE | MCP_ISR_RX0IE);
	result = MCP_readRegister(RCANINTE);
	printf("CANINTE: 0x%x\n", result);

	result = MCP_readRegister(RCNF1);
	printf("CNF1: 0x%x\n", result);
	result = MCP_readRegister(RCNF2);
	printf("CNF2: 0x%x\n", result);
	result = MCP_readRegister(RCNF3);
	printf("CNF3: 0x%x\n", result);

	/* Activate loopback mode */
	MCP_setMode(MODE_LOOPBACK);

	result = MCP_readRegister(0x0E);
	printf("CANSTAT: 0x%x\n", result);

	uint_fast8_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	MCP_fillBuffer(0x0F, data, 8);

	result = MCP_readRegister(RTXB0SIDH);
	printf("TXB0SIDH: 0x%x\n", result);

	result = MCP_readRegister(RTXB0SIDL);
	printf("TXB0SIDL: 0x%x\n", result);

	result = MCP_readRegister(RTXB0DLC);
	printf("TXB0DLC: 0x%x\n", result);

	result = MCP_getInterruptStatus();
	printf("CANINTF: 0x%x\n", result);

	MCP_sendRTS(RTS_TXB0);


	/* Do the main loop */
	while (1) {
		/*for (i = 0; i < 500000; i++)
		 ;*/
		MAP_PCM_gotoLPM0InterruptSafe();
		/*result = MCP_readRegister(0x0F);
		 printf("CANCTRL 0x0F: 0x%x\n", result);*/
	}

}
