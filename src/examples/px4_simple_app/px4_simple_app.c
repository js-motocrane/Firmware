/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/gen_status.h>
#include <uORB/topics/debug_key_value.h>


#define GEN_ENERGY_THRESH_KJ 100.0 // number of kJ in the full tank
#define GEN_ENERGY_MAX_KJ 200.0 // max number of kJ to log up to

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{

	PX4_INFO("ADC app start");

	PX4_ERR("TEST ERR");

	//struct debug_key_value_s dbg = {.key = "velx", .value = 0.0f};
	//orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value),&dbg);

	// subscribe to adc_report topic
	int adc_sub_fd = orb_subscribe(ORB_ID(adc_report));
	// limit the update rate to 20 Hz
	orb_set_interval(adc_sub_fd, 50);

	// subscribe to battery voltage topic
	int bat_volt_fd = orb_subscribe(ORB_ID(battery_status));
	// limit update rate to 20 Hz
	orb_set_interval(bat_volt_fd,1500);

	// set the custom uORB message up for publishing and advertising
	struct gen_status_s gen_struct;
	memset(&gen_struct, 0, sizeof(gen_struct));
	orb_advert_t gen_pub = orb_advertise(ORB_ID(gen_status), &gen_struct);

	// one could wait for multiple topics with this technique, just using one here
	px4_pollfd_struct_t fds[] = {
		{ .fd = adc_sub_fd,   .events = POLLIN },
		{ .fd = bat_volt_fd,  .events = POLLIN },
		// there could be more file descriptors here, in the form like:
		// { .fd = other_sub_fd,   .events = POLLIN },
		 //
	};


	uint8_t error_counter = 0;

	double pwrIntegral = 0.0;
	double sysVoltage = 50.0; // temporary
	double sysCurrent = 0.0;

	float fuelpct_local = 100.0; // start at 100 percent

	uint64_t prevTime;
	uint8_t timeCaptured = 0;

	uint8_t timeCaptureCnt = 0;

	while(true)
	{
		// do a non-blocking poll for the new ADC/system voltage value
		int poll_ret = px4_poll(fds,2,1500); // second argument is important!! for multiple


		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within interval");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) { // only update integral on current reception
				/* obtained data for the first file descriptor */
				struct adc_report_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(adc_report), adc_sub_fd, &raw);

				double dt;
				uint64_t curTime;

				if (!timeCaptured)
				{
					if (timeCaptureCnt < 2)
					{
						timeCaptureCnt++;
					}
					else
					{
						timeCaptured = 1;
					}

					dt = 0.0;
					prevTime = raw.timestamp;

				}
				else
				{
					// we got the value once already
					curTime = raw.timestamp;
					dt = ((double)(curTime - prevTime)) * 0.000001; // convert to seconds
					// update previous time
					prevTime = curTime;
				}

				// convert the analog value to current (ADC x 3.3/4096 ( to V) - 0.6(offset)) * 1/(0.06) (sensitivity)
				sysCurrent = (((double)(raw.raw_data[8]))*0.000805664 - 0.5) * 16.667;
				if (sysCurrent < 0)
				{
					sysCurrent = 0.0;
				}
				// cap the pwrIntegral to prevent it from overflowing
				if(pwrIntegral > GEN_ENERGY_MAX_KJ)
				{
					pwrIntegral = GEN_ENERGY_MAX_KJ;
				}
				else
				{
					// read system voltage?
					pwrIntegral += sysCurrent * sysVoltage * dt * 0.001; // converting to kJ
				}
				//PX4_INFO("Analog:\t%8.4f",(double)raw.raw_data[8]);

				//PX4_INFO("(A): %4.2f, (V): %3.1f, (J): %8.4f",sysCurrent,sysVoltage,pwrIntegral);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/

				// calculate total percentage remaining
				fuelpct_local = 100.0-(uint8_t)(pwrIntegral/GEN_ENERGY_THRESH_KJ*100.0);
				if (fuelpct_local < 0)
				{
					fuelpct_local = 0.0;
				}

				gen_struct.timestamp = raw.timestamp;
				gen_struct.genpower = (float)(sysCurrent*sysVoltage);
				gen_struct.genenergy = (float)(pwrIntegral);
				gen_struct.gencurrent = (float)(sysCurrent);
				gen_struct.fuelpct = fuelpct_local;

				// TO-DO - figure out more logic here in the future for detecting stalls
				if (sysCurrent < 2.0)
				{
					gen_struct.genrunning = 0;
				}
				else
				{
					gen_struct.genrunning = 1;
				}

				// DEBUG

				//dbg.value = pwrIntegral;
				//orb_publish(ORB_ID(debug_key_value),pub_dbg,&dbg);

				//adc_struct.raw_data[8] = raw.raw_data[8];

				//orb_publish(ORB_ID(adc_report), adc_pub, &adc_struct);
				orb_publish(ORB_ID(gen_status),gen_pub,&gen_struct);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */

			if (fds[1].revents & POLLIN)
			{
				struct battery_status_s rawBatt;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(battery_status), bat_volt_fd, &rawBatt);

				// get the data and assign it to the system voltage value
				sysVoltage = rawBatt.voltage_filtered_v;

				// TO-DO - check for dropouts to zero

			}
		} // end of else - from handling poll event

	} // while (true) loop
	// TO-DO - figure out how to exit this based on other conditions?


	PX4_INFO("exiting");

	return 0;
} // end of px4_simple_app_main
