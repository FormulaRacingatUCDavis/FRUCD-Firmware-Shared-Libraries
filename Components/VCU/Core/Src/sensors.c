/*
 * sensors.c
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */
#include "sensors.h"
#include "can_manager.h"
#include "driver_input.h"
#include "config.h"
#include <stdlib.h>
#include "traction_control.h"

CALIBRATED_SENSOR_t throttle1; //APPS1
CALIBRATED_SENSOR_t throttle2; //APPS2
CALIBRATED_SENSOR_t brake1; //BSE_F
CALIBRATED_SENSOR_t brake2; //BSE_R
CALIBRATED_SENSOR_t brake; // used in actual calculations
volatile uint32_t torque_percentage = 0;
volatile uint32_t launch_control_param = 0;
volatile uint32_t prev_torque_percentage = 0;
volatile uint32_t prev_launch_control_param = 0;
volatile uint32_t regen_braking_param = 0;
volatile uint32_t torque_req = 0;

#define RADS_PER_RPM 0.10472
#define MAX_TORQUE_OVERTAKE (uint16_t)(MAX_TORQUE_NM * 0.8)

#define SMOOTHING_STATE_NONE 0
#define SMOOTHING_STATE_DOWN 1
#define SMOOTHING_STATE_UP 2

extern volatile uint8_t traction_control_enabled;
extern volatile int16_t motor_speed;
extern volatile uint16_t acc_current_adc;
extern volatile uint16_t acc_current_ref_adc;
extern volatile int16_t pack_voltage;
extern volatile uint8_t soc;

volatile uint32_t lv_battery_measure_adc = 0;

uint16_t get_max_torque(uint32_t max_power);
uint32_t get_max_power();
int16_t get_regen_torque();

//extern void Error_Handler();



void init_sensors(){
    throttle1.min = 0x7FFF;
    throttle1.max = 0;
    throttle1.range = 1;
    throttle2.min = 0x7FFF;
    throttle2.max = 0;
    throttle2.range = 1;
    brake1.min = 0x7FFF;
    brake1.max = 0;
    brake1.range = 1;
    brake2.min = 0x7FFF;
	brake2.max = 0;
	brake2.range = 1;

	brake.min = 0x7FFF;
	brake.max = 0;
	brake.range = 1;
}

void select_adc_channel(ADC_HandleTypeDef *hadc, ADC_CHAN channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    switch (channel)
    {
        case APPS1:
            sConfig.Channel = ADC_CHANNEL_10;
			sConfig.Rank = 1;

			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;

        case APPS2:
			sConfig.Channel = ADC_CHANNEL_8;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case BSE1:
			sConfig.Channel = ADC_CHANNEL_15;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case BSE2:
			sConfig.Channel = ADC_CHANNEL_9;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case KNOB1:
			sConfig.Channel = ADC_CHANNEL_13;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case KNOB2:
			sConfig.Channel = ADC_CHANNEL_12;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case LV_BAT_MEASURE:
        	sConfig.Channel = ADC_CHANNEL_5;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        default:
            break;
    }
}

uint32_t get_adc_conversion(ADC_HandleTypeDef *hadc, ADC_CHAN channel) {

	select_adc_channel(hadc, channel);

	uint32_t conversion;

	HAL_ADC_Start(hadc);

	// Wait for the conversion to complete
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	// Get the ADC value
	conversion = HAL_ADC_GetValue(hadc);

	return conversion;
}


// Update sensors

void run_calibration() {
    update_minmax(&throttle1);
    update_minmax(&throttle2);
    update_minmax(&brake1);
    update_minmax(&brake2);
}

void update_sensor_vals(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc3) {
	// pedals
    throttle1.raw = get_adc_conversion(hadc1, APPS1);
    update_percent(&throttle1);
    throttle2.raw = get_adc_conversion(hadc3, APPS2);
    update_percent(&throttle2);
    brake1.raw = get_adc_conversion(hadc3, BSE1);
    update_percent(&brake1);
    brake2.raw = get_adc_conversion(hadc3, BSE2);
    update_percent(&brake2);

    update_brake(); // compare the two BSEs to get "one" brake sensor

    // knobs
    prev_torque_percentage = torque_percentage;
	prev_launch_control_param = launch_control_param;
    torque_percentage = get_adc_conversion(hadc1, KNOB2) * 100 / 4095;
    launch_control_param = get_adc_conversion(hadc1, KNOB1) * 100 / 4095;

    if (ALLOW_REGEN_KNOB_TUNING) {
    	regen_braking_param = get_adc_conversion(hadc1, KNOB1) * 100 / 4095;
    }
}

void read_lv_battery(ADC_HandleTypeDef *hadc3) {
	lv_battery_measure_adc = get_adc_conversion(hadc3, LV_BAT_MEASURE);
}

float raw_to_mvolts(uint16_t adc_raw) {
    return ((float)adc_raw / 4095) * 3.3 * 1000;
}

float mvolts_to_amps(float mVolts, float mVolt_ref) {
    return ((mVolts - mVolt_ref) * 7.4 / 4.7) / 6.667;
}

static float get_accumulator_power() {
	float acc_current_amps = mvolts_to_amps(raw_to_mvolts(acc_current_adc), raw_to_mvolts(acc_current_ref_adc));
	float acc_voltage_volt = pack_voltage * 0.018 + 180;

	return acc_current_amps*acc_voltage_volt;
}

int16_t requested_throttle(){
	// variables for power smoothing
	static uint8_t smoothing_flag = SMOOTHING_STATE_NONE; // flag that says if we are currently in a smoothing sequence
	static uint32_t smoothing_start_tick = 0; // stores the tick that we started the smoothing sequence
	static uint32_t smoothing_prev_max_power = 0; // the max power we had when we exceeded 70kW on the accumulator

    uint32_t max_power = get_max_power();

    if (POWER_SMOOTHING_ENABLED) {
    	if (smoothing_flag == SMOOTHING_STATE_DOWN) { // if we are in the smoothing down sequence
			// usually: will go from 70kW max power to 60kW max power over the course of a few seconds

			uint32_t interval_ms = HAL_GetTick() - smoothing_start_tick;

			if (interval_ms > SMOOTHING_DOWN_TIME_MS) { // if we have spent enough time smoothing down, can start smoothing up
				smoothing_flag = SMOOTHING_STATE_UP;
				smoothing_start_tick = HAL_GetTick();
				smoothing_prev_max_power = smoothing_prev_max_power - SMOOTHING_POWER_DELTA_W;
			} else {  // else, smooth down by a set amount of motor power (SMOOTHING_POWER_DELTA_W)
				max_power = smoothing_prev_max_power - abs((float)interval_ms / (float)SMOOTHING_DOWN_TIME_MS) * SMOOTHING_POWER_DELTA_W;
			}

		} else if (smoothing_flag == SMOOTHING_STATE_UP) { // else, we are in the smoothing up sequence
			// usually: will go from 60kW max power back up to 70kW max power

			uint32_t interval_ms = HAL_GetTick() - smoothing_start_tick;

			if (interval_ms > SMOOTHING_UP_TIME_MS) { // if we are done smoothing up, return to normal operation
				smoothing_flag = SMOOTHING_STATE_NONE;
				smoothing_start_tick = 0;
				smoothing_prev_max_power = 0;
			} else { // else gradually allow more motor power, back up to 60kW
				max_power = max_power - (1.0 - (float)interval_ms / (float)SMOOTHING_UP_TIME_MS) * abs(max_power - smoothing_prev_max_power);
			}

			// if we exceed accumulator power on the way up, go back down again
			if (get_accumulator_power() >= MAX_POWER_ACCUMULATOR_W) {
				smoothing_flag = SMOOTHING_STATE_DOWN;
				smoothing_start_tick = HAL_GetTick();
				smoothing_prev_max_power = max_power;
			}

		}
    } // endif power smoothing enabled


    uint16_t max_torque = get_max_torque(max_power);


    // zero throttle if brake is pressed at all, prevents hardware bspd
//	if (brake.percent >= BRAKE_BSPD_THRESHOLD) return 0;


    // if exceed power limit of 70kW, begin smoothing sequence if we haven't already
    if (POWER_SMOOTHING_ENABLED) {

    	if (get_accumulator_power() >= MAX_POWER_ACCUMULATOR_W && smoothing_flag == SMOOTHING_STATE_NONE) {
			smoothing_flag = SMOOTHING_STATE_DOWN;
			smoothing_start_tick = HAL_GetTick();
			smoothing_prev_max_power = max_power;

			// just for this call, limit motor power a lot
			max_torque = get_max_torque(max_power - SMOOTHING_POWER_DELTA_W);
		}

    } else if (get_accumulator_power() >= ACCUMULATOR_POWER_HARD_LIMIT) { // old method of power limit enforcement
    	// MAKE EXTRA SURE 80kW accumulator power draw is not exceeded or FUSE WILL BLOW
    	max_torque = 0;
    }


    torque_req = (throttle1.percent * max_torque * 10) / 100;  //upscale for MC code, Nm times 10

    // use reduced values from TC if TC torque request is lower
    if(is_button_enabled(TC_BUTTON) && (torque_req > TC_torque_req)){
		torque_req = TC_torque_req;
	}



    // regenerative braking
    if (REGEN_BRAKING_ENABLED) {

    	// EV.3.3.3 The powertrain must not regenerate energy when vehicle speed is between 0 and 5 km/hr
		// 0.016349 comes from (60 * pi * tire_diameter) / (FDR * 63360) where 63360 is conversion factor
		// 1.60934 is converting mph to kph
		float car_speed_kph = abs(motor_speed) * 0.016349 * 1.60934;


		// State of charge should also be below 95%, prevent overcharging accumulator
		if (throttle1.percent < APPS_REGEN_THRESHOLD && car_speed_kph > 5 && soc < 95) {
			// negative torque request for regen braking
			int16_t regen_torque = get_regen_torque();

			return clamp(regen_torque*10, -90*10, 0); // max 90 Nm on regen
		}

    }

    return (uint16_t)torque_req;
}

// get maximum power based on power limit
// attenuate for BMS temps between 50 and 60
uint32_t get_max_power(){
	if(PACK_TEMP < 50) {
		return MAX_POWER_MOTOR_W;
	} else if(PACK_TEMP < 58) {
		return (58 - PACK_TEMP)*(MAX_POWER_MOTOR_W / 8);
	} else {
		return 0;
	}
}

uint16_t get_max_torque(uint32_t max_power){
	float motor_speed_rads = (float)motor_speed * RADS_PER_RPM;
	float max_torque_power = max_power / motor_speed_rads; // max torque calculated from max power
	float max_torque_knob = MAX_TORQUE_NM * (float)torque_percentage / 100;

	// if overtake is enabled, return the lower of torque limit set by overtake value and power limit
	if (is_button_enabled(OVERTAKE_BUTTON) && MAX_TORQUE_OVERTAKE < max_torque_power) {
		return MAX_TORQUE_OVERTAKE;
	}

	// return the lower of the torque limit set by the knob and by the power limit
	if (max_torque_knob < max_torque_power) {
		return (uint16_t)max_torque_knob;
	}
	else {
		return (uint16_t)max_torque_power;
	}
}

int16_t get_regen_torque() {
	float current_term = 30; // 40.5 amps max regen current
	float acc_voltage_volt = pack_voltage * 0.018 + 180;
	float voltage_term = abs(acc_voltage_volt);
	float rpm_term = abs(motor_speed);

	int16_t regen_torque = (int16_t)( -1*voltage_term * current_term / (0.10472 * rpm_term) );

	if (ALLOW_REGEN_KNOB_TUNING) {
		regen_torque = (regen_braking_param / 100.0) * regen_torque;
	}

	return regen_torque;
}

bool sensors_calibrated(){
	return throttle1.range > APPS1_MIN_RANGE &&
		   throttle2.range > APPS2_MIN_RANGE &&
		   (brake1.range > BRAKE_MIN_RANGE || brake2.range > BRAKE_MIN_RANGE); // if either BSE is working, we can consider them calibrated
}

bool braking(){
    return brake.raw > BRAKE_LIGHT_THRESHOLD;
}

bool brake_mashed(){
    return brake.percent > RTD_BRAKE_THRESHOLD;
}

// check differential between the throttle sensors
// returns true only if the sensor discrepancy is > 10%
// Note: after verifying there's no discrepancy, can use either sensor(1 or 2) for remaining checks
bool has_discrepancy() {
    if(abs((int)throttle1.percent - (int)throttle2.percent) > 10) return 1;  //percentage discrepancy

    return (throttle1.raw < APPS_OPEN_THRESH)
        || (throttle1.raw > APPS_SHORT_THRESH)
        || (throttle2.raw < APPS_OPEN_THRESH)
        || (throttle2.raw > APPS_SHORT_THRESH);   // checks for wiring fault (open or short circuit).
    											  // not sure why this is in the discrepancy check function but it's needed
	return false;

}

// check for soft BSPD (in rules, called "APPS / Brake Pedal Plausibility Check")
// see EV.4.7 of FSAE 2025 rulebook
bool is_brake_implausible() {
    if (error == BRAKE_IMPLAUSIBLE) {
        // once brake implausibility detected,
        // can only revert to normal if throttle "unapplied"
        return !(throttle1.percent <= APPS1_BSPD_RESET_THRESHOLD);
    }

    // if brake applied and throttle > 25%, brake implausible
    return (brake.percent >= BRAKE_BSPD_THRESHOLD && throttle1.percent > APPS1_BSPD_THRESHOLD);
}

void update_percent(CALIBRATED_SENSOR_t* sensor){
    uint32_t raw = (uint32_t)clamp(sensor->raw, sensor->min, sensor->max);
    sensor->percent = (uint16_t)((100*(raw-sensor->min))/((sensor->range)));
}

// As of the 2026 ruleset, there is no requirement for a second BSE,
// thus we can choose how we handle our redundant BSE sensor.
// Here, we primarily rely on brake1 / BSE_F
void update_brake() {
	// If brake1 failed to calibrate, or if brake2 reads a higher percent than brake1,
	// then use brake2's readings
	if (brake1.range < BRAKE_MIN_RANGE || brake2.percent > brake1.percent ) {

		brake.min = brake2.min;
		brake.max = brake2.max;
		brake.raw = brake2.raw;
		brake.range = brake2.range;
		brake.percent = brake2.percent;

	} else {

		// otherwise, just use brake1's readings
		brake.min = brake1.min;
		brake.max = brake1.max;
		brake.raw = brake1.raw;
		brake.range = brake1.range;
		brake.percent = brake1.percent;

	}
}

void update_minmax(CALIBRATED_SENSOR_t* sensor){
    if (sensor->raw > sensor->max) sensor->max = sensor->raw;
    else if (sensor->raw < sensor->min) sensor->min = sensor->raw;
    if(sensor->max > sensor->min) sensor->range = sensor->max - sensor->min;
}

void add_apps_deadzone(){
	add_deadzone(&throttle1, APPS_DEADZONE_PERCENTAGE);
	add_deadzone(&throttle2, APPS_DEADZONE_PERCENTAGE);
	add_deadzone(&brake1, BRAKE_DEADZONE_PERCENTAGE);
	add_deadzone(&brake2, BRAKE_DEADZONE_PERCENTAGE);
	add_deadzone(&brake, BRAKE_DEADZONE_PERCENTAGE);
}

void add_deadzone(CALIBRATED_SENSOR_t* sensor, uint16_t deadzone_percentage){
	uint16_t deadzone = sensor->range * deadzone_percentage / 100;

	// catch funky cases that would end up with a negative or 0 range
	if(deadzone >= sensor->range) return;

	sensor->min += deadzone;
	sensor->range -= deadzone;
}

int16_t clamp(int16_t in, int16_t min, int16_t max){
    if(in > max) return max;
    if(in < min) return min;
    return in;
}


