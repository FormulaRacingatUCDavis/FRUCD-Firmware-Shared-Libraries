#include "telem_node.h"
#include "adc.h"
#include "pwm.h"
#include "can_manager.h"
#include "main.h"
#include "wheel_speed.h"
#include "math.h"

#define VOLTAGE_DIVIDER_RATIO (12.0 / (12.0 + 6.04))
#define PSI_PER_KPA 0.145038
#define HI8(x) ((x>>8)&0xFF)
#define LO8(x) (x&0xFF);


TelemNodeLocation_t location = LOCATION_FRONT;

// HANDLE TYPE DEFS from main
extern ADC_HandleTypeDef hadc1;
extern uint16_t ADC_RES_BUFFER[3];
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

volatile ADC_Channel_t adc_channels[NUM_ADC_CHANNELS] = {
		{SHOCK_ANGLE1_CHANNEL, 0, 0}, // SHOCK_ANGlE1, 333 Hz
		{SHOCK_ANGLE2_CHANNEL, 0, 0}, // SHOCK_ANGLE2, 333 Hz
		{BRAKE_TEMP_CHANNEL, 0, 0} // BRAKE TEMP 40 Hz
};
uint8_t wheel_speed_flag = 0; // wheel speed is from timers, not ADC; it gets a separate flag

// PRIVATE GLOBALS
WheelSpeed_t wheel_1;
WheelSpeed_t wheel_2;


// PRIVATE FUNCTION PROTOTYPES
uint16_t get_pres(uint16_t adc_val);
int16_t get_temp(uint16_t adc_val);
int16_t get_air_temp(uint16_t adc_val);
//uint16_t set_to_16(uint64_t adc_val);

void TelemNode_Init(){
	CAN_Init();

	WheelSpeed_Init(&wheel_1, &htim1);
	WheelSpeed_Init(&wheel_2, &htim2);
}


void TelemNode_Update()
{
	static uint8_t tx_data[8] = {0};

	// shock angles
	if (adc_channels[0].flag == 1 || adc_channels[1].flag == 1) {
		uint16_t shock_angle1 = adc_channels[0].adc_value;
		uint16_t shock_angle2 = adc_channels[1].adc_value;

		tx_data[0] = HI8(shock_angle1);
		tx_data[1] = LO8(shock_angle1);
		tx_data[2] = HI8(shock_angle2);
		tx_data[3] = LO8(shock_angle2);

		if (location == LOCATION_FRONT) {
			CAN_Send(SHOCK_ANGLES_FRONT, tx_data, 4);
		} else {
			CAN_Send(SHOCK_ANGLES_REAR, tx_data, 4);
		}

		// reset the flags
		adc_channels[0].flag = 0;
		adc_channels[1].flag = 0;
	}


	// brake temp
	// for 5/22/26 track day, only front Telem Node has brake temp, and only has 1
	if (adc_channels[3].flag == 1) {
		uint16_t brake_temp = adc_channels[3].adc_value;

		if (location == LOCATION_FRONT) {
			tx_data[0] = HI8(brake_temp);
			tx_data[1] = LO8(brake_temp);
			tx_data[2] = 0;
			tx_data[3] = 0;
			CAN_Send(BRAKE_TEMP, tx_data, 2);
		}

		// reset the flag
		adc_channels[3].flag = 0;
	}

	// wheel speeds
	if (wheel_speed_flag == 1) {
		uint32_t wheel_speed_1 = WheelSpeed_GetCPS(&wheel_1);
		uint32_t wheel_speed_2 = WheelSpeed_GetCPS(&wheel_2);

		// only sending bottom 16 bits of wheel speed, shouldnt need all 32 bits
		tx_data[0] = HI8(wheel_speed_1);
		tx_data[1] = LO8(wheel_speed_1);
		tx_data[2] = HI8(wheel_speed_2);
		tx_data[3] = LO8(wheel_speed_2);
		if (location == LOCATION_FRONT) {
			CAN_Send(WHEEL_SPEEDS_FRONT, tx_data, 4);
		} else {
			CAN_Send(WHEEL_SPEEDS_REAR, tx_data, 4);
		}

		// reset the flag
		wheel_speed_flag = 0;
	}

}

//uint16_t set_to_16(uint64_t adc_val){
//	return (uint16_t)adc_val;
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // with PSC = 71 and ARR = 999, TIM3 runs at 1000 Hz
    {
    	static uint32_t ticks = 0; // 1000 Hz -> one tick is one ms

    	// 333 Hz -> Period of 3 ms
    	if (ticks % 3 == 0) {
    		adc_channels[0].flag = 1; // SHOCK_ANGLE_1
    		adc_channels[1].flag = 1; // SHOCK_ANGLE_2
    		wheel_speed_flag = 1; // both wheel speeds
    	}

    	// 40 Hz -> Period of 25 ms
		if (ticks % 25 == 0) {
			adc_channels[3].flag = 1; // BRAKE_TEMP
		}

		ticks++;
		// reset the ticks to keep the modulo math clean
		// use the lowest common multiple of all periods above
		if (ticks >= 75) { // 25 * 3 = 75
			ticks = 0;
		}
    }
}



