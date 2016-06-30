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

uint_fast8_t RXData;
int i;
uint8_t mode;
uint_fast8_t result;

/* SPI Timing Config */
const MCP_CANTimingConfig CANTimingConfig = { 24000000, /* Oscillator Frequency */
6, /* Baud Rate Prescaler */
1, /* Propagation Delay */
2, /* Phase Segment 1 */
2, /* Phase Segment 2 */
1 /* Synchronisation Jump Width */
};

void msgHandler(MCP_CANMessage *);

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

	/* Register an interrupt on TX0 and RX0 */
	MCP_enableInterrupt(MCP_ISR_TX0IE | MCP_ISR_RX0IE);

	/* Set the handler to be called when a message is received */
	MCP_setReceivedMessageHandler(&msgHandler);

	/* Activate loopback mode */
	MCP_setMode(MODE_LOOPBACK);
	//MCP_setMode(MODE_NORMAL);

	/* Disable all filters */
	MCP_writeRegister(0x60, 0x60);

	uint_fast8_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	MCP_CANMessage msg = createEmptyMessage();
	msg.ID = 0xAAAA;
	msg.isExtended = 0;
	msg.isRequest = 0;
	msg.length = 8;
	msg.data = data;


	/* Do the main loop */
	while (1) {
		//MAP_PCM_gotoLPM0InterruptSafe();
		MCP_fillBuffer(&msg);

		printf("Transmitting: ");
		printf("(ID: 0x%x)", msg.ID);
		for (i = 0; i < 8; i++) {
			printf(" 0x%x", data[i]);
		}
		printf("\n");


		MCP_sendRTS(TXB0);

		for(i = 0; i<5000000; i++);
	}
}

void msgHandler(MCP_CANMessage * msg) {
	printf("Received: ");
	printf("(ID: 0x%x)", msg->ID);
	for (i = 0; i < 8; i++) {
		printf(" 0x%x", msg->data[i]);
	}
	printf("\n");
	if(msg->isExtended)
		printf(" (this is extended)");
	else
		printf(" (this is NOT extended)");

	printf("\n");
		if(msg->isRequest)
			printf(" (this is a request)");
		else
			printf(" (this is NOT a request)");

	printf("\n");
	deleteMessage(msg);
}
