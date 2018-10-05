// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#include "defines.h"
#include "../libDCM/gpsData.h"
#include "../libDCM/gpsParseCommon.h"
#include "../libDCM/rmat.h"
#include "../libDCM/libDCM.h"


#define MIN_THROTTLE        2240
#define MID_THROTTLE        3000
#define CLS_TIME            60

enum PLANE_FLIGHT_MODE
{
	PLANE_ON_GROUND,
    PLANE_LAUNCHED,
	PLANE_IN_FLIGHT,
    PLANE_TURN_1,
    PLANE_CRUISE_1,
    PLANE_CRUISE_2,
    PLANE_DESCENDING, 
	PLANE_LANDED
};

static int16_t flight_mode = PLANE_ON_GROUND;
static int16_t state_counter = 0;
static int16_t home_saved = false;
static int32_t last_lat = 0;
static int32_t last_lon = 0;

boolean inflight_state(void)
{
	return (flight_mode == PLANE_IN_FLIGHT);
}

// called every 0.5 sec
static void update_in_flight(void)
{
	static int16_t counter = 0;

	// simple state machine
	//   flight_mode = PLANE_ON_GROUND -> if moved 7 meters and throttle > 50% -> PLANE_IN_FLIGHT

	int32_t lat = lat_gps.WW >> 6;  // cut off unecessary digits, it's about 7 meters
	int32_t log = lat_gps.WW >> 6;
	int16_t throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];

//	if (flight_mode == PLANE_ON_GROUND || flight_mode == PLANE_IN_FLIGHT)   // we are on ground before or after flight

	if (flight_mode == PLANE_ON_GROUND)                 // we are on ground before or after flight
	{                                                   // we need 7 meters of movement
		if (last_lon != log && last_lat != lat)         // we are moving 
		{
			if (throttleIn < MID_THROTTLE)              // but without throttle
			{
				last_lon = log;                         // save
				last_lat = lat;
			}
			else                                        // with throttle so we are flying
			{
				flight_mode = PLANE_IN_FLIGHT;
				printf("PLANE_IN_FLIGHT\r\n");
			}
		}
	}
	else if (flight_mode == PLANE_IN_FLIGHT)
	{
		if (last_lon != log && last_lat != lat)         // we are moving
		{
			last_lon = log;                             // save
			last_lat = lat;
			counter = 0;
		}
		else
		{
			counter++;
		}
		if (counter > 20 && throttleIn < MIN_THROTTLE)  // no location change for 10 seconds and throttle low
		{
			flight_mode = PLANE_LANDED;
			printf("PLANE_LANDED\r\n");
		}
	}
	if (flight_mode == PLANE_LANDED)
	{
		if (counter++ > 40)
		{
			flight_mode = PLANE_ON_GROUND;
		}
	}
}

void flight_state_8hz(void)
{
	if (home_saved)
	{
		if (state_counter & 1)
		{
			// 4 Hz
		}
		if (state_counter % 4 == 0)
		{
			// 2 Hz
			update_in_flight();
		}
		if (state_counter % 8 == 0)
		{
			// 1 Hz
		}
	}
	else
	{
		if (gps_nav_valid())
		{
			static int16_t init_counter = 0;

			if (init_counter++ == 14)
			{
				home_saved = true;
				state_counter = 0;  // used to run initial cls
			}
		}
	}

	if (state_counter == 24 || state_counter % (CLS_TIME * 8) == 0)
	{
		// 3 secs after home save or about every CLS_TIME secs
	}

	++state_counter;
}


void flightState(void)
{
#define FLIGHT_CLIMB_TIMER           40  // 1 second  
#define FLIGHT_CRUISE_TIME_1         40
#define FLIGHT_TURN_TIMER           160 
#define FLIGHT_DESCEND_TIMER        800  // 20 seconds
    
#define FLIGHT_CLIMB_TRIM_DELTA     200
#define FLIGHT_TURN_RATE_UDB_UNITS 1000  
#define FLIGHT_DESCEND_TRIM_DELTA   100
    
static int16_t climb_timer = 0;
static int16_t turn_timer = 0;
static int16_t cruise_timer_1 = 0;
static int16_t cruise_timer_2 = 0;
    
    if (flight_mode == PLANE_ON_GROUND)
    {
       if ((gravity_axis_at_startup == GRAVITY_X_POSITIVE) ||
            (gravity_axis_at_startup == GRAVITY_X_NEGATIVE))
        {
            led_on(LED_GREEN);
            if (abs(acceleration_plane_y()) > GRAVITY / 2)
            {
                flight_mode = PLANE_LAUNCHED ;
                udb_pwTrim[ELEVATOR_INPUT_CHANNEL] -= FLIGHT_CLIMB_TRIM_DELTA;
            }
        }
    }
    if ( flight_mode == PLANE_LAUNCHED )
    {
        if (climb_timer++ > FLIGHT_CLIMB_TIMER)
        {
            udb_pwTrim[ELEVATOR_INPUT_CHANNEL] = ELEVATOR_TRIMPOINT;
            flight_mode = PLANE_IN_FLIGHT;
        }
    }
    if ( flight_mode == PLANE_IN_FLIGHT)
    {
        if (cruise_timer_1++ > FLIGHT_CRUISE_TIME_1)
        {
            if (gravity_axis_at_startup == GRAVITY_X_POSITIVE)
            {
                udb_pwIn[AILERON_INPUT_CHANNEL] -= FLIGHT_TURN_RATE_UDB_UNITS;
            }
            else
            {
                udb_pwIn[AILERON_INPUT_CHANNEL] += FLIGHT_TURN_RATE_UDB_UNITS;
            }
            flight_mode = PLANE_TURN_1;
        }
    }
    if ( flight_mode == PLANE_TURN_1)
    {
         if ( turn_timer++ > FLIGHT_TURN_TIMER)
         {
             udb_pwIn[AILERON_INPUT_CHANNEL] = AILERON_TRIMPOINT ;
             flight_mode = PLANE_CRUISE_2 ;
         }
    }
    if (flight_mode == PLANE_CRUISE_2)
    {
        if ( cruise_timer_2++ > FLIGHT_DESCEND_TIMER)
         {
             udb_pwTrim[ELEVATOR_INPUT_CHANNEL] += FLIGHT_DESCEND_TRIM_DELTA ;
             udb_pwIn[AILERON_INPUT_CHANNEL] += FLIGHT_TURN_RATE_UDB_UNITS;
             flight_mode = PLANE_DESCENDING ;
         }
    }
}
