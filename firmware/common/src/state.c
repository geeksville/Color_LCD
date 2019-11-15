/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <math.h>
#include "stdio.h"
#include "main.h"
#include "utils.h"
#include "screen.h"
#include "rtc.h"
#include "fonts.h"
#include "uart.h"
#include "mainscreen.h"
#include "eeprom.h"
#include "buttons.h"
#include "fault.h"
#include <stdlib.h>

static uint8_t ui8_m_usart1_received_first_package = 0;
uint16_t ui16_g_battery_soc_watts_hour;
volatile uint32_t ui32_g_layer_2_can_execute;
volatile uint8_t motorVariablesStabilized = 0;

bool has_seen_motor; // true once we've received a packet from a real motor
bool is_sim_motor; // true if we are simulating a motor (and therefore not talking on serial at all)

// kevinh: I don't think volatile is probably needed here
volatile l2_vars_t l2_vars;

l3_vars_t l3_vars;

l3_vars_t* get_l3_vars(void) {
	return &l3_vars;
}

/// Set correct backlight brightness for current headlight state
void set_lcd_backlight() {
	lcd_set_backlight_intensity(
			l3_vars.ui8_lights ?
					l3_vars.ui8_lcd_backlight_on_brightness :
					l3_vars.ui8_lcd_backlight_off_brightness);
}

static uint16_t fake(uint16_t minv, uint16_t maxv) {
	static uint16_t seed = 1; // Just generate some slightly increasing data, scaled to fit the required range

	uint16_t numval = maxv - minv + 1;

	return (seed++ % numval) + minv;
}

/// Generate a fake value that slowly loops between min and max and then back to min.  You must provide static storage for this routine to use
static uint16_t fakeWave(uint32_t *storage, uint16_t minv, uint16_t maxv) {
	(*storage)++;

	uint16_t numval = maxv - minv + 1;

	return (*storage % numval) + minv;
}

/// Generate a fake value that randomly oscillates between min and max and then back to min.  You must provide static storage for this routine to use
static uint16_t fakeRandom(uint32_t *storage, uint16_t minv, uint16_t maxv) {
    int32_t rnd = (rand() - RAND_MAX / 2) % ((maxv - minv) / 20);
    (*storage) += rnd;
    if (*storage > maxv) {
        *storage = (uint32_t)maxv;
    }
    if (*storage < minv) {
        *storage = (uint32_t)minv;
    }
    return *storage;
}

/**
 * Pretend we just received a randomized motor packet
 */
void parse_simmotor() {
  static uint32_t counter;

  // execute at a slow rate so values can be seen on the graph
  counter++;
  if (counter % (3 * 10)) // 3 seconds
    return;

	const uint32_t min_bat10x = 400;
	const uint32_t max_bat10x = 546;
	const uint32_t max_cur10x = 140;
    static uint32_t voltstore, curstore, speedstore, cadencestore, tempstore, diststore;

	// per step of ADC ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000
	// l2_vars.ui16_adc_battery_voltage = battery_voltage_10x_get() * 1000L / ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
	l2_vars.ui16_adc_battery_voltage = fakeWave(&voltstore, min_bat10x,
			max_bat10x) * 1000L / ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
	// l2_vars.ui16_adc_battery_voltage = max_bat10x * 1000L / ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;

	// battery current drain x5
	l2_vars.ui8_battery_current_x5 = fakeRandom(&curstore, 0, max_cur10x) / 2;

	l2_vars.ui16_wheel_speed_x10 = fakeRandom(&speedstore, 80, 300);
    diststore += l2_vars.ui16_wheel_speed_x10 * 2.6; // speed x 10 to millimeters per 100 ms
//	l2_vars.ui16_wheel_speed_x10 = 200; // for testing, just leave speed fixed

	l2_vars.ui8_braking = 0; // fake(0, 1);

	l2_vars.ui8_adc_throttle = fake(0, 100);

	if (l2_vars.ui8_temperature_limit_feature_enabled) {
		l2_vars.ui8_motor_temperature = fakeWave(&tempstore, 20, 120);
	} else {
		l2_vars.ui8_throttle = fake(0, 100);
	}

	l2_vars.ui8_adc_pedal_torque_sensor = fake(0, 100);

	l2_vars.ui8_pedal_torque_sensor = fake(0, 100);

	l2_vars.ui8_pedal_cadence = fakeRandom(&cadencestore, 0, 120);
	l2_vars.ui8_pedal_human_power = fake(0, 100);
	l2_vars.ui8_duty_cycle = fake(0, 100);
	l2_vars.ui16_motor_speed_erps = fake(0, 4000);
	l2_vars.ui8_foc_angle = fake(0, 100);

	// error states
	l2_vars.ui8_error_states = NO_ERROR; // fake(0, ERROR_MAX);

	// temperature actual limiting value
	l2_vars.ui8_temperature_current_limiting_value = fake(0, 100);

	// wheel_speed_sensor_tick_counter

    if (diststore > l2_vars.ui16_wheel_perimeter) {
        l2_vars.ui32_wheel_speed_sensor_tick_counter += 1;
        diststore -= l2_vars.ui16_wheel_perimeter;
    }

	// ui16_pedal_torque_x10
	l2_vars.ui16_pedal_torque_x10 = fake(10, 1000);

	// ui16_pedal_power_x10
	l2_vars.ui16_pedal_power_x10 = fake(10, 1000);

}

void process_rx(void) {
	static uint32_t num_missed_packets = 0;

	const uint8_t *p_rx_buffer = uart_get_rx_buffer_rdy();

	// process rx package if we are simulating or the UART had a packet
	if (is_sim_motor || p_rx_buffer) {
		if (is_sim_motor)
			parse_simmotor();
		else if (p_rx_buffer) {
			// now process rx data
			// only if first byte is equal to package start byte
			if (*p_rx_buffer == 67) {
				has_seen_motor = true;
				num_missed_packets = 0; // reset missed packet count

				p_rx_buffer++;

				l2_vars.ui16_adc_battery_voltage = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui16_adc_battery_voltage |= ((uint16_t) (*p_rx_buffer
						& 0x30)) << 4;
				p_rx_buffer++;

				l2_vars.ui8_battery_current_x5 = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui16_wheel_speed_x10 = (uint16_t) *p_rx_buffer;
				p_rx_buffer++;
				l2_vars.ui16_wheel_speed_x10 += ((uint16_t) *p_rx_buffer << 8);
				p_rx_buffer++;

				uint8_t ui8_temp = *p_rx_buffer;
				l2_vars.ui8_braking = ui8_temp & 1;
				p_rx_buffer++;

				l2_vars.ui8_adc_throttle = *p_rx_buffer;
				p_rx_buffer++;

				if (l2_vars.ui8_temperature_limit_feature_enabled) {
					l2_vars.ui8_motor_temperature = *p_rx_buffer;
				} else {
					l2_vars.ui8_throttle = *p_rx_buffer;
				}
				p_rx_buffer++;

				l2_vars.ui8_adc_pedal_torque_sensor = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui8_pedal_torque_sensor = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui8_pedal_cadence = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui8_pedal_human_power = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui8_duty_cycle = *p_rx_buffer;
				p_rx_buffer++;

				l2_vars.ui16_motor_speed_erps = (uint16_t) *p_rx_buffer;
				p_rx_buffer++;
				l2_vars.ui16_motor_speed_erps += ((uint16_t) *p_rx_buffer << 8);
				p_rx_buffer++;

				l2_vars.ui8_foc_angle = *p_rx_buffer;
				p_rx_buffer++;

				// error states
				l2_vars.ui8_error_states = *p_rx_buffer;
				p_rx_buffer++;

				// temperature actual limiting value
				l2_vars.ui8_temperature_current_limiting_value = *p_rx_buffer;
				p_rx_buffer++;

				// wheel_speed_sensor_tick_counter
				uint32_t ui32_wheel_speed_sensor_tick_temp;
				ui32_wheel_speed_sensor_tick_temp = ((uint32_t) *p_rx_buffer);
				p_rx_buffer++;
				ui32_wheel_speed_sensor_tick_temp |= (((uint32_t) *p_rx_buffer)
						<< 8);
				p_rx_buffer++;
				ui32_wheel_speed_sensor_tick_temp |= (((uint32_t) *p_rx_buffer)
						<< 16);
				l2_vars.ui32_wheel_speed_sensor_tick_counter =
						ui32_wheel_speed_sensor_tick_temp;
				p_rx_buffer++;

				// ui16_pedal_torque_x10
				l2_vars.ui16_pedal_torque_x10 = (uint16_t) *p_rx_buffer;
				p_rx_buffer++;
				l2_vars.ui16_pedal_torque_x10 += ((uint16_t) *p_rx_buffer << 8);
				p_rx_buffer++;

				// ui16_pedal_power_x10
				l2_vars.ui16_pedal_power_x10 = (uint16_t) *p_rx_buffer;
				p_rx_buffer++;
				l2_vars.ui16_pedal_power_x10 += ((uint16_t) *p_rx_buffer << 8);

				// not needed with this implementation (and with ptr flipflop not needed either)
				// usart1_reset_received_package();
			}
		}

		// let's wait for 10 packages, seems that first ADC battery voltages have incorrect values
		ui8_m_usart1_received_first_package++;
		if (ui8_m_usart1_received_first_package > 10)
			ui8_m_usart1_received_first_package = 10;
	} else {
		// We expected a packet during this 100ms window but one did not arrive.  This might happen if the motor is still booting and we don't want to declare failure
		// unless something is seriously busted (because we will be raising the fault screen and eventually forcing the bike to shutdown) so be very conservative
		// and wait for 10 seconds of missed packets.
		if (has_seen_motor && num_missed_packets++ == 50)
			APP_ERROR_HANDLER(FAULT_LOSTRX);
	}
}

void send_tx_package(void) {
	static uint8_t ui8_message_id = 0;

	uint8_t *ui8_usart1_tx_buffer = uart_get_tx_buffer();

	/************************************************************************************************/
	// send tx package
	// start up byte
	ui8_usart1_tx_buffer[0] = 0x59;
	ui8_usart1_tx_buffer[1] = ui8_message_id;

	if (l2_vars.ui8_walk_assist) {
		ui8_usart1_tx_buffer[2] =
				l2_vars.ui8_walk_assist_level_factor[((l2_vars.ui8_assist_level)
						- 1)];
	} else if (l2_vars.ui8_assist_level) {
		ui8_usart1_tx_buffer[2] =
				l2_vars.ui8_assist_level_factor[((l2_vars.ui8_assist_level) - 1)];
	} else {
		ui8_usart1_tx_buffer[2] = 0;
	}

	ui8_usart1_tx_buffer[3] = (l2_vars.ui8_lights & 1)
			| ((l2_vars.ui8_walk_assist & 1) << 1);

	// battery power
	ui8_usart1_tx_buffer[4] = l2_vars.ui8_target_max_battery_power;

	switch (ui8_message_id) {
    case 0:
      // battery low voltage cut-off
      ui8_usart1_tx_buffer[5] =
          (uint8_t) (l2_vars.ui16_battery_low_voltage_cut_off_x10 & 0xff);
      ui8_usart1_tx_buffer[6] =
          (uint8_t) (l2_vars.ui16_battery_low_voltage_cut_off_x10 >> 8);
      break;

    case 1:
      // wheel perimeter
      ui8_usart1_tx_buffer[5] = (uint8_t) (l2_vars.ui16_wheel_perimeter
          & 0xff);
      ui8_usart1_tx_buffer[6] =
          (uint8_t) (l2_vars.ui16_wheel_perimeter >> 8);
      break;

    case 2:
      // wheel max speed
      ui8_usart1_tx_buffer[5] = l2_vars.ui8_wheel_max_speed;

      // battery max current
      ui8_usart1_tx_buffer[6] = l2_vars.ui8_battery_max_current;
      break;

    case 3:
      ui8_usart1_tx_buffer[5] = l2_vars.ui8_motor_type;

      ui8_usart1_tx_buffer[6] = (
          l2_vars.ui8_startup_motor_power_boost_always ? 1 : 0)
          | (l2_vars.ui8_startup_motor_power_boost_limit_power ? 2 : 0);
      break;

    case 4:
      // startup motor power boost
      ui8_usart1_tx_buffer[5] =
          l2_vars.ui8_startup_motor_power_boost_factor[((l2_vars.ui8_assist_level)
              - 1)];
      // startup motor power boost time
      ui8_usart1_tx_buffer[6] = l2_vars.ui8_startup_motor_power_boost_time;
      break;

    case 5:
      // startup motor power boost fade time
      ui8_usart1_tx_buffer[5] =
          l2_vars.ui8_startup_motor_power_boost_fade_time;
      // boost feature enabled
      ui8_usart1_tx_buffer[6] =
          (l2_vars.ui8_startup_motor_power_boost_feature_enabled & 1) ?
              1 : 0;
      break;

    case 6:
      // motor over temperature min and max values to limit
      ui8_usart1_tx_buffer[5] =
          l2_vars.ui8_motor_temperature_min_value_to_limit;
      ui8_usart1_tx_buffer[6] =
          l2_vars.ui8_motor_temperature_max_value_to_limit;
      break;

    case 7:
      ui8_usart1_tx_buffer[5] = l2_vars.ui8_ramp_up_amps_per_second_x10;

      // TODO
      // target speed for cruise
      ui8_usart1_tx_buffer[6] = 0;
      break;

    case 8:
      // motor temperature limit function or throttle
      ui8_usart1_tx_buffer[5] =
          l2_vars.ui8_temperature_limit_feature_enabled & 3;

      // motor assistance without pedal rotation enable/disable when startup
      ui8_usart1_tx_buffer[6] =
          l2_vars.ui8_motor_assistance_startup_without_pedal_rotation;
      break;

    default:
      ui8_message_id = 0;
      break;
	}

	// prepare crc of the package
	uint16_t ui16_crc_tx = 0xffff;
	for (uint8_t ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_SEND; ui8_i++) {
		crc16(ui8_usart1_tx_buffer[ui8_i], &ui16_crc_tx);
	}
	ui8_usart1_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 1] =
			(uint8_t) (ui16_crc_tx & 0xff);
	ui8_usart1_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 2] =
			(uint8_t) (ui16_crc_tx >> 8) & 0xff;

	// send the full package to UART
	if (!is_sim_motor) // If we are simulating received packets never send real packets
		uart_send_tx_buffer(ui8_usart1_tx_buffer);

	// increment message_id for next package
	if (++ui8_message_id > UART_MAX_NUMBER_MESSAGE_ID) {
		ui8_message_id = 0;
	}
}

void l2_low_pass_filter_battery_voltage_current_power(void) {
	static uint32_t ui32_battery_voltage_accumulated_x10000 = 0;
	static uint16_t ui16_battery_current_accumulated_x5 = 0;

	// low pass filter battery voltage
	ui32_battery_voltage_accumulated_x10000 -=
			ui32_battery_voltage_accumulated_x10000
					>> BATTERY_VOLTAGE_FILTER_COEFFICIENT;
	ui32_battery_voltage_accumulated_x10000 +=
			(uint32_t) l2_vars.ui16_adc_battery_voltage
					* ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
	l2_vars.ui16_battery_voltage_filtered_x10 =
			((uint32_t) (ui32_battery_voltage_accumulated_x10000
					>> BATTERY_VOLTAGE_FILTER_COEFFICIENT)) / 1000;

	// low pass filter batery current
	ui16_battery_current_accumulated_x5 -= ui16_battery_current_accumulated_x5
			>> BATTERY_CURRENT_FILTER_COEFFICIENT;
	ui16_battery_current_accumulated_x5 +=
			(uint16_t) l2_vars.ui8_battery_current_x5;
	l2_vars.ui16_battery_current_filtered_x5 =
			ui16_battery_current_accumulated_x5
					>> BATTERY_CURRENT_FILTER_COEFFICIENT;

	// battery power
	l2_vars.ui16_battery_power_filtered_x50 =
			l2_vars.ui16_battery_current_filtered_x5
					* l2_vars.ui16_battery_voltage_filtered_x10;
	l2_vars.ui16_battery_power_filtered =
			l2_vars.ui16_battery_power_filtered_x50 / 50;
}

void l2_low_pass_filter_pedal_torque_and_power(void) {
	static uint32_t ui32_pedal_torque_accumulated = 0;
	static uint32_t ui32_pedal_power_accumulated = 0;

	// low pass filter
	ui32_pedal_torque_accumulated -= ui32_pedal_torque_accumulated
			>> PEDAL_TORQUE_FILTER_COEFFICIENT;
	ui32_pedal_torque_accumulated += (uint32_t) l2_vars.ui16_pedal_torque_x10
			/ 10;
	l2_vars.ui16_pedal_torque_filtered =
			((uint32_t) (ui32_pedal_torque_accumulated
					>> PEDAL_TORQUE_FILTER_COEFFICIENT));

	// low pass filter
	ui32_pedal_power_accumulated -= ui32_pedal_power_accumulated
			>> PEDAL_POWER_FILTER_COEFFICIENT;
	ui32_pedal_power_accumulated += (uint32_t) l2_vars.ui16_pedal_power_x10
			/ 10;
	l2_vars.ui16_pedal_power_filtered =
			((uint32_t) (ui32_pedal_power_accumulated
					>> PEDAL_POWER_FILTER_COEFFICIENT));
}

void l2_calc_battery_voltage_soc(void) {
	uint16_t ui16_fluctuate_battery_voltage_x10;

	// calculate flutuate voltage, that depends on the current and battery pack resistance
	ui16_fluctuate_battery_voltage_x10 =
			(uint16_t) ((((uint32_t) l2_vars.ui16_battery_pack_resistance_x1000)
					* ((uint32_t) l2_vars.ui16_battery_current_filtered_x5))
					/ ((uint32_t) 500));
	// now add fluctuate voltage value
	l2_vars.ui16_battery_voltage_soc_x10 =
			l2_vars.ui16_battery_voltage_filtered_x10
					+ ui16_fluctuate_battery_voltage_x10;
}

void l2_calc_wh(void) {
	static uint8_t ui8_1s_timer_counter = 0;
	uint32_t ui32_temp = 0;

	if (l2_vars.ui16_battery_power_filtered_x50 > 0) {
		l2_vars.ui32_wh_sum_x5 += l2_vars.ui16_battery_power_filtered_x50 / 10;
		l2_vars.ui32_wh_sum_counter++;
	}

	// calc at 1s rate
	if (++ui8_1s_timer_counter >= 10) {
		ui8_1s_timer_counter = 0;

		// avoid zero divisison
		if (l2_vars.ui32_wh_sum_counter != 0) {
			ui32_temp = l2_vars.ui32_wh_sum_counter / 36;
			ui32_temp = (ui32_temp
					* (l2_vars.ui32_wh_sum_x5 / l2_vars.ui32_wh_sum_counter))
					/ 500;
		}

		l2_vars.ui32_wh_x10 = l2_vars.ui32_wh_x10_offset + ui32_temp;
	}
}

static void l2_calc_odometer(void) {
  static uint8_t ui8_1s_timer_counter;
	uint32_t uint32_temp;

	// calc at 1s rate
	if (++ui8_1s_timer_counter >= 10) {
		ui8_1s_timer_counter = 0;

		uint32_temp = (l2_vars.ui32_wheel_speed_sensor_tick_counter
				- l3_vars.ui32_wheel_speed_sensor_tick_counter_offset)
				* ((uint32_t) l2_vars.ui16_wheel_perimeter);
		// avoid division by 0
		if (uint32_temp > 100000) {
			uint32_temp /= 100000;
		}  // milimmeters to 0.1kms
		else {
			uint32_temp = 0;
		}

		// now store the value on the global variable
		// l2_vars.ui16_odometer_distance_x10 = (uint16_t) uint32_temp;

		// calculate how many revolutions since last reset and convert to distance traveled
		uint32_t ui32_temp = (l2_vars.ui32_wheel_speed_sensor_tick_counter
				- l2_vars.ui32_wheel_speed_sensor_tick_counter_offset)
				* ((uint32_t) l2_vars.ui16_wheel_perimeter);

		// if traveled distance is more than 100 meters update all distance variables and reset
		if (ui32_temp >= 100000) { // 100000 -> 100000 mm -> 0.1 km
			// update all distance variables
			// l3_vars.ui16_distance_since_power_on_x10 += 1;
			l2_vars.ui32_odometer_x10 += 1;
			l2_vars.ui32_trip_x10 += 1;

			// reset the always incrementing value (up to motor controller power reset) by setting the offset to current value
			l2_vars.ui32_wheel_speed_sensor_tick_counter_offset =
					l2_vars.ui32_wheel_speed_sensor_tick_counter;
		}
	}
}

static void l2_low_pass_filter_pedal_cadence(void) {
	static uint16_t ui16_pedal_cadence_accumulated = 0;

	// low pass filter
	ui16_pedal_cadence_accumulated -= (ui16_pedal_cadence_accumulated
			>> PEDAL_CADENCE_FILTER_COEFFICIENT);
	ui16_pedal_cadence_accumulated += (uint16_t) l2_vars.ui8_pedal_cadence;

	// consider the filtered value only for medium and high values of the unfiltered value
	if (l2_vars.ui8_pedal_cadence > 20) {
		l2_vars.ui8_pedal_cadence_filtered =
				(uint8_t) (ui16_pedal_cadence_accumulated
						>> PEDAL_CADENCE_FILTER_COEFFICIENT);
	} else {
		l2_vars.ui8_pedal_cadence_filtered = l2_vars.ui8_pedal_cadence;
	}
}

uint8_t first_time_management(void) {
  static uint32_t ui32_counter = 0;
	static uint8_t ui8_motor_controller_init = 1;
	uint8_t ui8_status = 0;


  // wait 5 seconds to help motor variables data stabilize
  if (motorVariablesStabilized == 0)
    if (++ui32_counter > 50) {
      motorVariablesStabilized = 1;
#ifndef SW102
      extern Field *activeGraphs; // FIXME, move this extern someplace better, placing here for review purposes

  	  activeGraphs = &graphs; // allow graph plotting to start
#endif
    }

	// don't update LCD up to we get first communication package from the motor controller
	if (ui8_motor_controller_init
			&& (ui8_m_usart1_received_first_package < 10)) {
		ui8_status = 1;
	}
	// this will be executed only 1 time at startup
  else if(ui8_motor_controller_init &&
      motorVariablesStabilized) {
    // reset Wh value if battery voltage is over ui16_battery_voltage_reset_wh_counter_x10 (value configured by user)
    if (((uint32_t) l3_vars.ui16_adc_battery_voltage *
    ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000)
        > ((uint32_t) l3_vars.ui16_battery_voltage_reset_wh_counter_x10
            * 1000)) {
      l3_vars.ui32_wh_x10_offset = 0;
    }

    if (l3_vars.ui8_offroad_feature_enabled
        && l3_vars.ui8_offroad_enabled_on_startup) {
      l3_vars.ui8_offroad_mode = 1;
    }
  }

	return ui8_status;
}

void l2_calc_battery_soc_watts_hour(void) {
	uint32_t ui32_temp;

	ui32_temp = l2_vars.ui32_wh_x10 * 100;

	if (l2_vars.ui32_wh_x10_100_percent > 0) {
		ui32_temp /= l2_vars.ui32_wh_x10_100_percent;
	} else {
		ui32_temp = 0;
	}

	if (ui32_temp > 100)
		ui32_temp = 100;

	// 100% - current SOC or just current SOC
	if (!l2_vars.ui8_battery_soc_increment_decrement) {
		ui16_g_battery_soc_watts_hour = 100 - ui32_temp;
	} else {
		ui16_g_battery_soc_watts_hour = ui32_temp;
	}
}

// Note: this called from ISR context every 100ms
void realtime_processing(void) {
	process_rx();

#ifndef SW102
	send_tx_package(); // we can't call this in ISR context on SW102 because it uses a semaphore, call from main thread instead
#endif

	/************************************************************************************************/
	// now do all the calculations that must be done every 100ms
	l2_low_pass_filter_battery_voltage_current_power();
	l2_low_pass_filter_pedal_torque_and_power();
	l2_low_pass_filter_pedal_cadence();
	l2_calc_battery_voltage_soc();
	l2_calc_odometer();
	l2_calc_wh();

	graph_realtime_process();
	/************************************************************************************************/

	first_time_management();
	l2_calc_battery_soc_watts_hour();
}

/**
 * Called from the main thread every 100ms
 *
 */
void copy_rt_to_ui_vars(void) {
	l3_vars.ui16_adc_battery_voltage = l2_vars.ui16_adc_battery_voltage;
	l3_vars.ui8_battery_current_x5 = l2_vars.ui8_battery_current_x5;
	l3_vars.ui8_throttle = l2_vars.ui8_throttle;
	l3_vars.ui8_adc_pedal_torque_sensor = l2_vars.ui8_adc_pedal_torque_sensor;
	l3_vars.ui8_pedal_torque_sensor = l2_vars.ui8_pedal_torque_sensor;
	l3_vars.ui8_pedal_human_power = l2_vars.ui8_pedal_human_power;
	l3_vars.ui8_duty_cycle = l2_vars.ui8_duty_cycle;
	l3_vars.ui8_error_states = l2_vars.ui8_error_states;
	l3_vars.ui16_wheel_speed_x10 = l2_vars.ui16_wheel_speed_x10;
	l3_vars.ui8_pedal_cadence = l2_vars.ui8_pedal_cadence;
	l3_vars.ui16_motor_speed_erps = l2_vars.ui16_motor_speed_erps;
	l3_vars.ui8_temperature_current_limiting_value =
			l2_vars.ui8_temperature_current_limiting_value;
	l3_vars.ui8_motor_temperature = l2_vars.ui8_motor_temperature;
	l3_vars.ui32_wheel_speed_sensor_tick_counter =
			l2_vars.ui32_wheel_speed_sensor_tick_counter;
	l3_vars.ui16_pedal_power_x10 = l2_vars.ui16_pedal_power_x10;
	l3_vars.ui16_battery_voltage_filtered_x10 =
			l2_vars.ui16_battery_voltage_filtered_x10;
	l3_vars.ui16_battery_current_filtered_x5 =
			l2_vars.ui16_battery_current_filtered_x5;
	l3_vars.ui16_battery_power_filtered_x50 =
			l2_vars.ui16_battery_power_filtered_x50;
	l3_vars.ui16_battery_power_filtered = l2_vars.ui16_battery_power_filtered;
	l3_vars.ui16_pedal_torque_filtered = l2_vars.ui16_pedal_torque_filtered;
	l3_vars.ui16_pedal_power_filtered = l2_vars.ui16_pedal_power_filtered;
	l3_vars.ui8_pedal_cadence_filtered = l2_vars.ui8_pedal_cadence_filtered;
	l3_vars.ui16_battery_voltage_soc_x10 = l2_vars.ui16_battery_voltage_soc_x10;
	l3_vars.ui32_wh_sum_x5 = l2_vars.ui32_wh_sum_x5;
	l3_vars.ui32_wh_sum_counter = l2_vars.ui32_wh_sum_counter;
	l3_vars.ui32_wh_x10 = l2_vars.ui32_wh_x10;
	l3_vars.ui8_braking = l2_vars.ui8_braking;
	l3_vars.ui8_foc_angle = (((uint16_t) l2_vars.ui8_foc_angle) * 14) / 10; // each units is equal to 1.4 degrees ((360 degrees / 256) = 1.4)

	l2_vars.ui32_wh_x10_offset = l3_vars.ui32_wh_x10_offset;
	l2_vars.ui16_battery_pack_resistance_x1000 =
			l3_vars.ui16_battery_pack_resistance_x1000;
	l2_vars.ui8_assist_level = l3_vars.ui8_assist_level;
	l2_vars.ui8_assist_level_factor[0] = l3_vars.ui8_assist_level_factor[0];
	l2_vars.ui8_assist_level_factor[1] = l3_vars.ui8_assist_level_factor[1];
	l2_vars.ui8_assist_level_factor[2] = l3_vars.ui8_assist_level_factor[2];
	l2_vars.ui8_assist_level_factor[3] = l3_vars.ui8_assist_level_factor[3];
	l2_vars.ui8_assist_level_factor[4] = l3_vars.ui8_assist_level_factor[4];
	l2_vars.ui8_assist_level_factor[5] = l3_vars.ui8_assist_level_factor[5];
	l2_vars.ui8_assist_level_factor[6] = l3_vars.ui8_assist_level_factor[6];
	l2_vars.ui8_assist_level_factor[7] = l3_vars.ui8_assist_level_factor[7];
	l2_vars.ui8_assist_level_factor[8] = l3_vars.ui8_assist_level_factor[8];
	l2_vars.ui8_walk_assist_feature_enabled =
			l3_vars.ui8_walk_assist_feature_enabled;
	l2_vars.ui8_walk_assist_level_factor[0] =
			l3_vars.ui8_walk_assist_level_factor[0];
	l2_vars.ui8_walk_assist_level_factor[1] =
			l3_vars.ui8_walk_assist_level_factor[1];
	l2_vars.ui8_walk_assist_level_factor[2] =
			l3_vars.ui8_walk_assist_level_factor[2];
	l2_vars.ui8_walk_assist_level_factor[3] =
			l3_vars.ui8_walk_assist_level_factor[3];
	l2_vars.ui8_walk_assist_level_factor[4] =
			l3_vars.ui8_walk_assist_level_factor[4];
	l2_vars.ui8_walk_assist_level_factor[5] =
			l3_vars.ui8_walk_assist_level_factor[5];
	l2_vars.ui8_walk_assist_level_factor[6] =
			l3_vars.ui8_walk_assist_level_factor[6];
	l2_vars.ui8_walk_assist_level_factor[7] =
			l3_vars.ui8_walk_assist_level_factor[7];
	l2_vars.ui8_walk_assist_level_factor[8] =
			l3_vars.ui8_walk_assist_level_factor[8];
	l2_vars.ui8_lights = l3_vars.ui8_lights;
	l2_vars.ui8_walk_assist = l3_vars.ui8_walk_assist;
	l2_vars.ui8_offroad_mode = l3_vars.ui8_offroad_mode;
	l2_vars.ui8_battery_max_current = l3_vars.ui8_battery_max_current;
	l2_vars.ui8_ramp_up_amps_per_second_x10 =
			l3_vars.ui8_ramp_up_amps_per_second_x10;
	l2_vars.ui8_target_max_battery_power = l3_vars.ui8_target_max_battery_power;
	l2_vars.ui16_battery_low_voltage_cut_off_x10 =
			l3_vars.ui16_battery_low_voltage_cut_off_x10;
	l2_vars.ui16_wheel_perimeter = l3_vars.ui16_wheel_perimeter;
	l2_vars.ui8_wheel_max_speed = l3_vars.wheel_max_speed_x10 / 10;
	l2_vars.ui8_motor_type = l3_vars.ui8_motor_type;
	l2_vars.ui8_motor_assistance_startup_without_pedal_rotation =
			l3_vars.ui8_motor_assistance_startup_without_pedal_rotation;
	l2_vars.ui8_temperature_limit_feature_enabled =
			l3_vars.ui8_temperature_limit_feature_enabled;
	l2_vars.ui8_startup_motor_power_boost_always =
			l3_vars.ui8_startup_motor_power_boost_always;
	l2_vars.ui8_startup_motor_power_boost_limit_power =
			l3_vars.ui8_startup_motor_power_boost_limit_power;
	l2_vars.ui8_startup_motor_power_boost_time =
			l3_vars.ui8_startup_motor_power_boost_time;
	l2_vars.ui8_startup_motor_power_boost_factor[1] =
			l3_vars.ui8_startup_motor_power_boost_factor[0];
	l2_vars.ui8_startup_motor_power_boost_factor[2] =
			l3_vars.ui8_startup_motor_power_boost_factor[1];
	l2_vars.ui8_startup_motor_power_boost_factor[3] =
			l3_vars.ui8_startup_motor_power_boost_factor[2];
	l2_vars.ui8_startup_motor_power_boost_factor[4] =
			l3_vars.ui8_startup_motor_power_boost_factor[3];
	l2_vars.ui8_startup_motor_power_boost_factor[5] =
			l3_vars.ui8_startup_motor_power_boost_factor[4];
	l2_vars.ui8_startup_motor_power_boost_factor[6] =
			l3_vars.ui8_startup_motor_power_boost_factor[5];
	l2_vars.ui8_startup_motor_power_boost_factor[7] =
			l3_vars.ui8_startup_motor_power_boost_factor[7];
	l2_vars.ui8_startup_motor_power_boost_factor[8] =
			l3_vars.ui8_startup_motor_power_boost_factor[8];
	l2_vars.ui8_startup_motor_power_boost_factor[9] =
			l3_vars.ui8_startup_motor_power_boost_factor[9];
	l2_vars.ui8_startup_motor_power_boost_fade_time =
			l3_vars.ui8_startup_motor_power_boost_fade_time;
	l2_vars.ui8_startup_motor_power_boost_feature_enabled =
			l3_vars.ui8_startup_motor_power_boost_feature_enabled;
	l2_vars.ui8_motor_temperature_min_value_to_limit =
			l3_vars.ui8_motor_temperature_min_value_to_limit;
	l2_vars.ui8_motor_temperature_max_value_to_limit =
			l3_vars.ui8_motor_temperature_max_value_to_limit;
	l2_vars.ui8_offroad_feature_enabled = l3_vars.ui8_offroad_feature_enabled;
	l2_vars.ui8_offroad_enabled_on_startup =
			l3_vars.ui8_offroad_enabled_on_startup;
	l2_vars.ui8_offroad_speed_limit = l3_vars.ui8_offroad_speed_limit;
	l2_vars.ui8_offroad_power_limit_enabled =
			l3_vars.ui8_offroad_power_limit_enabled;
	l2_vars.ui8_offroad_power_limit_div25 =
			l3_vars.ui8_offroad_power_limit_div25;

	// Some l3 vars are derived only from other l3 vars
	uint32_t ui32_battery_cells_number_x10 =
			(uint32_t) (l3_vars.ui8_battery_cells_number * 10);

	uint8_t volt_based_soc;
	if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_90))) {
		volt_based_soc = 95;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_80))) {
		volt_based_soc = 85;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_70))) {
		volt_based_soc = 75;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_60))) {
		volt_based_soc = 65;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_50))) {
		volt_based_soc = 55;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_40))) {
		volt_based_soc = 45;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_30))) {
		volt_based_soc = 35;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_20))) {
		volt_based_soc = 25;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_10))) {
		volt_based_soc = 15;
	} else if (l3_vars.ui16_battery_voltage_soc_x10
			> ((uint16_t) ((float) ui32_battery_cells_number_x10
					* LI_ION_CELL_VOLTS_0))) {
		volt_based_soc = 5;
	} else {
		volt_based_soc = 0;
	}
	l3_vars.volt_based_soc = volt_based_soc;
}

/// must be called from main() idle loop
void automatic_power_off_management(void) {
	static uint32_t ui16_lcd_power_off_time_counter = 0;

	if (l3_vars.ui8_lcd_power_off_time_minutes != 0) {
		// see if we should reset the automatic power off minutes counter
		if ((l3_vars.ui16_wheel_speed_x10 > 0) ||   // wheel speed > 0
				(l3_vars.ui8_battery_current_x5 > 0) || // battery current > 0
				(l3_vars.ui8_braking) ||                // braking
				buttons_get_events())                       // any button active
				{
			ui16_lcd_power_off_time_counter = 0;
		} else {
			// increment the automatic power off ticks counter
			ui16_lcd_power_off_time_counter++;

			// check if we should power off the LCD
			if (ui16_lcd_power_off_time_counter
					>= (l3_vars.ui8_lcd_power_off_time_minutes * 10 * 60)) // have we passed our timeout?
					{
				lcd_power_off(1);
			}
		}
	} else {
		ui16_lcd_power_off_time_counter = 0;
	}
}

