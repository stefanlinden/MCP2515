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
#include "canutil.h"

//#define ISMASTER

uint_fast8_t RXData;
int i;
uint8_t mode;
uint_fast8_t result;

/* SPI Timing Config */
const MCP_CANTimingConfig CANTimingConfig = {
16000000, /* Oscillator Frequency */
4, /* Baud Rate Prescaler */
1, /* Propagation Delay */
3, /* Phase Segment 1 */
3, /* Phase Segment 2 */
1 /* Synchronisation Jump Width */
};

void msgHandler(MCP_CANMessage *);

void main(void) {

	/* Halting the watchdog */
	MAP_WDT_A_holdTimer();

	printf("Starting...\r\n");

	/* Start the clock */
	//startClockOnPin();

	/* Init the CAN controller */
	MCP_init();

	/* RESET */
	MCP_reset();
	printf("RESET\n");

	/* Make sure configuration mode is set */
	MCP_setMode(MODE_CONFIG);

	MCP_setTiming(&CANTimingConfig);

	/* Register an interrupt on TX0 and RX0 */
	MCP_enableInterrupt(MCP_ISR_TX0IE | MCP_ISR_RX0IE | MCP_ISR_ERRIE | MCP_ISR_MERRE);

	/* Set the handler to be called when a message is received */
	MCP_setReceivedMessageHandler(&msgHandler);

	/* Disable all filters */
	MCP_writeRegister(0x60, 0x60);

	/* Enable one-shot mode */
	//MCP_modifyBit(RCANCTRL, BIT3, BIT3);

	/* Activate loopback mode */
	//MCP_setMode(MODE_LOOPBACK);
	MCP_setMode(MODE_NORMAL);


	uint_fast8_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	MCP_CANMessage msg = {
	0x7B, /* Address */
	0, /* isExtended */
	0, /* isRequest */
	8, /* length */
	data /* data */
	};

	result = MCP_readRegister(RCANSTAT);
	printf("CANSTAT: 0x%x\n", result);

	/* Do the main loop */
	while (1) {
#ifndef ISMASTER
		MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
		MAP_PCM_gotoLPM0InterruptSafe();
#endif
#ifdef ISMASTER
		MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
		for(i = 0; i<8; i++)
			data[i]++;

		MCP_fillBuffer(&msg);

		printf("Transmitting: ");
		printf("(ID: 0x%x)", msg.ID);
		for (i = 0; i < 8; i++) {
			printf(" 0x%x", data[i]);
		}
		printf("\n");

		MCP_sendRTS(TXB0);

		MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		for(i = 0; i<2500000; i++);
		MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
		for(i = 0; i<2500000; i++);
		MAP_PCM_gotoLPM0();
#endif
	}
}

void msgHandler(MCP_CANMessage * msg) {
	printf("Received: ");
	printf("(ID: 0x%x)", msg->ID);
	for (i = 0; i < msg->length; i++) {
		printf(" 0x%x", msg->data[i]);
	}
	printf("\n");
			if(msg->isRequest)
				printf(" (this is a request)");
			else
				printf(" (this is NOT a request)");
	printf("\n");

	if(msg->isExtended)
		printf(" (this is extended)");
	else
		printf(" (this is NOT extended)");
	printf("\n");
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
	for(i = 0; i<2500000; i++);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
}
