//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "msp.h"
#include <stdio.h>
#include <driverlib.h>
#include "mcp2515.h"
#include "canutil.h"
#include "delay.h"

//#define ISMASTER

uint_fast8_t RXData;
int i;
uint8_t mode;
uint_fast8_t result, msgcount;

/* SPI Timing Config */
const MCP_CANTimingConfig CANTimingConfig = { 20000000, /* Oscillator Frequency */
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

	msgcount = 0;

	/* Start the clock */
	//startClockOnPin();
	/* Init the CAN controller */
	MCP_init();

	/* RESET */
	MCP_reset();
	printf("RESET\n");

	while ((MCP_readRegister(RCANSTAT) >> 5) != MODE_CONFIG)
		;

	/* Make sure configuration mode is set */
	//MCP_setMode(MODE_CONFIG);
	MCP_setTiming(&CANTimingConfig);

	/* Register an interrupt on TX0 and RX0 */
	MCP_enableInterrupt(
			MCP_ISR_RX0IE | MCP_ISR_RX1IE | MCP_ISR_TX0IE | MCP_ISR_TX1IE
					| MCP_ISR_TX2IE);

	/* Set the handler to be called when a message is received */
	MCP_setReceivedMessageHandler(&msgHandler);

	/* Disable all filters */
	MCP_writeRegister(RRXB0CTRL, 0x64);
	//MCP_writeRegister(RRXB1CTRL, 0x60);

	printf("RXB0CTRL: 0x%x\n", MCP_readRegister(RRXB0CTRL));

	/* Enable one-shot mode */
	//MCP_modifyBit(RCANCTRL, BIT3, BIT3);
	/* Activate loopback mode */
	//MCP_setMode(MODE_LOOPBACK);
	MCP_setMode(MODE_NORMAL);

	uint_fast8_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	MCP_CANMessage msg = { 0x7A, /* Address */
	0, /* isExtended */
	0, /* isRequest */
	8, /* length */
	data /* data */
	};

	result = MCP_readRegister(RCANSTAT);
	printf("CANSTAT: 0x%x\n", result);

	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);

	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

#ifdef ISMASTER
	/* Do the main loop */
	uint_fast8_t it;
	for (it = 0; it < 10; it++) {
		for (i = 0; i < 8; i++)
			data[i]++;

		//if (MCP_sendMessage(&msg))
		//printf("Could not send message\n");
		//result = MCP_fillGivenBuffer(&msg, TXB0);
		result = MCP_sendBulk(&msg, 1);
		//MCP_sendRTS(TXB0);
		if(result == 0xFF)
			printf("TX Error\n");
		else
			printf("TX Result: 0x%x\n", result);

		SysCtlDelay(1000000);
	}
	printf("Done sending %d messages.\n", it);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
#endif

	//printf("Filter Result: 0x%x\n", MCP_readRegister(RTXB1SIDL));
	while (1) {
		if (msgcount == 10) {
			printf("Got 10 messages!\n");
			MAP_GPIO_toggleOutputOnPin(GPIO_PIN2, GPIO_PIN0);
			msgcount = 0;
		}
		//MAP_PCM_gotoLPM0InterruptSafe();
	}
}

void msgHandler(MCP_CANMessage * msg) {
	msgcount++;
	printf("Received message\n");
	//printf("(ID: 0x%x)", msg->ID);
	//for (i = 0; i < msg->length; i++) {
	//	printf(" 0x%x", msg->data[i]);
	//}
	//printf("\n");
	/*if (msg->isRequest)
	 printf(" (this is a request)");
	 else
	 printf(" (this is NOT a request)");
	 printf("\n");

	 if (msg->isExtended)
	 printf(" (this is extended)");
	 else
	 printf(" (this is NOT extended)");*/
	//printf("\n");
	/*MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
	 for (i = 0; i < 2500000; i++)
	 ;
	 MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);*/
}
