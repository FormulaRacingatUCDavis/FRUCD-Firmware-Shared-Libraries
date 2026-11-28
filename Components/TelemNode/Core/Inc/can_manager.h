/*
 * can_manager.h
 *
 *  Created on: Apr 18, 2024
 *      Author: leoja
 */

#ifndef INC_CAN_MANAGER_H_
#define INC_CAN_MANAGER_H_

#include "stm32f1xx_hal.h"

typedef enum {
    // PCAN
    //BSPD_FLAGS = 0x0c1,
    VEHICLE_STATE = 0x766,
    TORQUE_REQUEST = 0x0C0,
	CLEAR_FAULT = 0x0C1,
    BMS_STATUS_MSG = 0x380,
    PEI_CURRENT_SHUTDOWN = 0x387,
    MC_VOLTAGE_INFO = 0x0A7,
    MC_INTERNAL_STATES = 0xAA,
    MC_FAULT_CODES = 0xAB,
	MC_TEMPS_1 = 0xA0,
	MC_TEMPS_2 = 0xA1,
	MC_TEMPS_3 = 0xA2,
	COOLING_LOOP_TEMPS = 0x400,
//	WHEEL_SPEED_REAR = 0x401, // TODO should not be in PCAN anymore
	COOLING_LOOP_PRESSURES = 0x402,
	STRAIN_GAUGES_REAR = 0x403,
	PUMP_PWM = 0x505,

} PCAN_ID;

typedef enum{
    WHEEL_SPEED_RR_UPPER = 0x401,
    WHEEL_SPEED_RR_LOWER = 0x402,
    WHEEL_SPEED_RL_UPPER = 0x403,
    WHEEL_SPEED_RL_LOWER = 0x404,

	//this is not updated on CAN index
	RL_SHOCK_ANGLE_UPPER = 0x420,
	RL_SHOCK_ANGLE_LOWER = 0X421,
	RR_SHOCK_ANGLE_UPPER = 0X422,
	RR_SHOCK_ANGLE_LOWER = 0X423,

	SG_UPPERLOWER_FL = 0x430,
	SG_UPPERLOWER_FR = 0x431,
	SG_UPPERLOWER_RL = 0x432,
	SG_UPPERLOWER_RR = 0x433,

	SG_CONTROLSHOCK_FL = 0x440,
	SG_CONTROLSHOCK_FR = 0x441,
	SG_CONTROLSHOCK_RL = 0x442,
	SG_CONTROLSHOCK_RR = 0x443,

//	IR_BRAKE_TEMP_UPPER = 0x450,
//	IR_BRAKE_TEMP_LOWER = 0x451,
//	K_BRAKE_TEMP_UPPER = 0x452,
//	K_BRAKE_TEMP_LOWER = 0x453,

	// for 5/22/26 Blue Max Track Day,
	// only have two Telem Nodes
	// each with 2 wheel speeds, 2 shock
	// front Telem Node also has one brake temp
	WHEEL_SPEEDS_FRONT = 0x460,
	WHEEL_SPEEDS_REAR = 0x461,
	SHOCK_ANGLES_FRONT = 0x464,
	SHOCK_ANGLES_REAR = 0x465,
	BRAKE_TEMP = 0x469,

} TCAN_ID;

typedef enum {
	LOCATION_FRONT,
	LOCATION_REAR,
} TelemNodeLocation_t;


typedef enum {
    LV,
    PRECHARGING,
    HV_ENABLED,
    DRIVE,
	CHARGING,
    VCU_STATE_FAULT = 0x80
} VCU_STATE_t;

typedef struct {
	VCU_STATE_t vcu_state;
	uint8_t bms_temp;

	//Signed 0.1 degree C units (matches raw signals in M160 / M162)
	int16_t mc_temp_module_a;
	int16_t mc_temp_module_b;
	int16_t mc_temp_module_c;
	int16_t mc_temp_max;
	int16_t motor_temp;


	uint8_t inverter_enable;
	uint8_t PWM_requested;

	//New fields from M170 (0x0AA) & M171 (0x0AB)
	uint8_t inv_state;
	uint8_t inv_vsm_state; 
	uint8_t inv_enable_lockout;
	uint16_t inv_post_fault_lo;
	uint16_t inv_post_fault_high;
	uint16_t inv_run_fault_hi;

	//Timestamps 
	uint32_t mc_temps1_last_tick;
	uint32_t mc_temps3_last_tick;
	uint32_t mc_states_last_tick;
	uint32_t mc_faults_last_tick;

	//PWM signals for pump and fan 
	uint8_t pumpPWM;
	uint8_t fanPWM;
} CAN_DATA_t;

void CAN_Init();
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_ptr);
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t* data, uint8_t len);

#endif /* INC_CAN_MANAGER_H_ */
