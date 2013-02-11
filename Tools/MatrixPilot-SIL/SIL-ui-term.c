//
//  SIL-ui-term.c
//  MatrixPilot-SIL
//
//  Created by Ben Levitt on 2/10/13.
//  Copyright (c) 2013 MatrixPilot. All rights reserved.
//

#include "SIL-udb.h"
#include "defines.h"
#include <stdio.h>


UDBSocket stdioSocket;

uint8_t lastLedBits = 0;
boolean showLEDs = 0;
uint8_t inputState = 0;


void print_help(void)
{
	printf("1/2/3 = mode manual/stabilized/waypoint\n");
	printf("w/s   = throttle up/down\n");
	printf("a/d   = rudder left/right\n");
	printf("i/k   = elevetor forward/back\n");
	printf("j/l   = aileron left/right\n");
	printf("\n");
	printf("z     = zero the sticks\n");
	printf("L     = toggle LEDs\n");
#if (FLIGHT_PLAN_TYPE == FP_LOGO)
	printf("xN    = execute LOGO subroutine N(0-9)\n");
#endif
	printf("r     = reset\n");
	printf("?     = show this help message\n");
}


void print_LED_status(void)
{
	printf("LEDs: %c %c %c %c\n",
		   (leds[0]) ? 'R' : '-',
		   (leds[1]) ? 'G' : '-',
		   (leds[2]) ? 'O' : '-',
		   (leds[3]) ? 'B' : '-');
}


void checkForLedUpdates(void)
{
	uint8_t newLedBits = 0;
	if (leds[0]) newLedBits |= 1;
	if (leds[1]) newLedBits |= 2;
	if (leds[2]) newLedBits |= 4;
	if (leds[3]) newLedBits |= 8;
	
	if (lastLedBits	!= newLedBits) {
		if (showLEDs) {
			print_LED_status();
		}
		lastLedBits	= newLedBits;
	}
}


void sil_rc_input_adjust(char *inChannelName, int inChannelIndex, int delta)
{
	udb_pwIn[inChannelIndex] = udb_servo_pulsesat(udb_pwIn[inChannelIndex] + delta);
	if (inChannelIndex == THROTTLE_INPUT_CHANNEL) {
		printf("\n%s = %d%%\n", inChannelName, (udb_pwIn[inChannelIndex]-udb_pwTrim[inChannelIndex])/20);
	}
	else {
		printf("\n%s = %d%%\n", inChannelName, (udb_pwIn[inChannelIndex]-udb_pwTrim[inChannelIndex])/10);
	}
}


#define KEYPRESS_INPUT_DELTA 50

void sil_handle_key_input(char c)
{
	switch (inputState) {
		case 0:
		{
			switch (c) {
				case '?':
					printf("\n");
					print_help();
					break;
					
				case 'w':
					sil_rc_input_adjust("throttle", THROTTLE_INPUT_CHANNEL, KEYPRESS_INPUT_DELTA*2);
					break;
					
				case 's':
					sil_rc_input_adjust("throttle", THROTTLE_INPUT_CHANNEL, -KEYPRESS_INPUT_DELTA*2);
					break;
					
				case 'a':
					sil_rc_input_adjust("rudder", RUDDER_INPUT_CHANNEL, KEYPRESS_INPUT_DELTA);
					break;
					
				case 'd':
					sil_rc_input_adjust("rudder", RUDDER_INPUT_CHANNEL, -KEYPRESS_INPUT_DELTA);
					break;
					
				case 'i':
					sil_rc_input_adjust("elevator", ELEVATOR_INPUT_CHANNEL, KEYPRESS_INPUT_DELTA);
					break;
					
				case 'k':
					sil_rc_input_adjust("elevator", ELEVATOR_INPUT_CHANNEL, -KEYPRESS_INPUT_DELTA);
					break;
					
				case 'j':
					sil_rc_input_adjust("aileron", AILERON_INPUT_CHANNEL, KEYPRESS_INPUT_DELTA);
					break;
					
				case 'l':
					sil_rc_input_adjust("aileron", AILERON_INPUT_CHANNEL, -KEYPRESS_INPUT_DELTA);
					break;
					
				case 'z':
					printf("\naileron, elevator, rudder = 0%%\n");
					udb_pwIn[AILERON_INPUT_CHANNEL] = udb_pwTrim[AILERON_INPUT_CHANNEL];
					udb_pwIn[ELEVATOR_INPUT_CHANNEL] = udb_pwTrim[ELEVATOR_INPUT_CHANNEL];
					udb_pwIn[RUDDER_INPUT_CHANNEL] = udb_pwTrim[RUDDER_INPUT_CHANNEL];
					break;
					
				case '1':
					udb_pwIn[MODE_SWITCH_INPUT_CHANNEL] = MODE_SWITCH_THRESHOLD_LOW - 1;
					break;
					
				case '2':
					udb_pwIn[MODE_SWITCH_INPUT_CHANNEL] = MODE_SWITCH_THRESHOLD_LOW + 1;
					break;
					
				case '3':
					udb_pwIn[MODE_SWITCH_INPUT_CHANNEL] = MODE_SWITCH_THRESHOLD_HIGH + 1;
					break;
					
				case 'L':
					showLEDs = !showLEDs;
					if (showLEDs) {
						printf("\n");
						print_LED_status();
					}
					break;
					
#if (FLIGHT_PLAN_TYPE == FP_LOGO)
				case 'x':
					inputState = 1;
					break;
#endif
					
				case 'r':
					printf("\nReally reset? (y/N)");
					fflush(stdout);
					inputState = 2;
					break;
					
				default:
					break;
			}
			break;
		}
			
		case 1:
		{
			if (c >= '0' && c <= '9') {
				printf("\nExecuting LOGO subroutine #%c\n", c);
				flightplan_live_begin() ;
				flightplan_live_received_byte(10) ; // Exec command
				flightplan_live_received_byte(c-'0') ; // Subroutine #
				flightplan_live_received_byte(0) ; // Don't use param or set fly bit
				flightplan_live_received_byte(0) ; // Exec command
				flightplan_live_received_byte(0) ; // Exec command
				flightplan_live_commit() ;
			}
			inputState = 0;
			break;
		}
			
		case 2:
		{
			if (c == 'y') {
				printf("\nReset MatrixPilot...\n\n\n");
				sil_reset();
			}
			inputState = 0;
		}
	}
}

