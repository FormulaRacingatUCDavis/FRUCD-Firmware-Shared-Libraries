/*
 * fsm.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Abhineet
 */


#include "main.h"
#include "cell_interface.h"
#include "fsm.h"
#include "can_manager.h"

extern uint8_t pei_status;

// BMS variables
extern volatile BAT_PACK_t bat_pack;

// Motor controller variables
extern volatile int16_t mc_voltage;
extern volatile uint8_t mc_vsm_state;
extern volatile uint8_t mc_discharge_state;
extern volatile uint32_t mc_post_faults;
extern volatile uint32_t mc_run_faults;

// VCU variables
extern volatile uint8_t vcu_state;
extern volatile uint8_t hv_requested;
extern volatile uint8_t vcu_attached;

// Charger variables
extern volatile uint8_t charger_attached;
extern volatile uint8_t charger_status;

// Tick counters
extern volatile uint32_t ticks_since_vcu_message;
extern volatile uint32_t ticks_since_mc_message;
extern volatile uint32_t ticks_since_charger_message;

uint8_t hv_request() {
	if (vcu_attached) return hv_requested;
	else if (charger_attached) return !(pei_status & SHUTDOWN);

	return 0;
}

uint8_t hv_allowed() {
	static uint8_t hv_lockout = 1;

	if (hv_lockout) {
		if (!hv_request()) hv_lockout = 0; // make sure we cannot go directly to HV
		return 0;
	}

	if (vcu_attached) {
		if (pei_status & VCU_TIMEOUT) return 0;

		if (pei_status & MC_FAULT) return 0;
		if (pei_status & MC_DISCHARGING) return 0;
		if (pei_status & MC_TIMEOUT) return 0;

		if (pei_status & SHUTDOWN) return 0; // shutdown circuit open

		return 1;
	}
	else if (charger_attached) {
		if (pei_status & (CHARGER_FAULT | CHARGER_TIMEOUT | SHUTDOWN)) return 0;

		return 1;
	}

	return 0;
}

uint8_t precharge_ready() {
	return 	(mc_vsm_state == PRECHARGE_INIT) ||
			(mc_vsm_state == PRECHARGE_ACTIVE) ||
			(mc_vsm_state == VSM_WAIT);
}

uint8_t precharge_complete() {
    uint16_t threshold = (uint16_t)((bat_pack.filt_pack_voltage * 10) * 0.9);
    return mc_voltage > threshold;
}

static void add_status(uint8_t status) {
    pei_status |= status;
}

void update_status() {
	const uint16_t MC_HW_GATE_POST_FAULT = 0x0001;
	const uint16_t MC_HW_GATE_RUN_FAULT = 0x0100;

	const uint8_t VCU_SHUTDOWN_OPEN = 0x87;
	const uint8_t VCU_MC_FAULT = 0x8A;

	pei_status = NORMAL; // reset

	if (vcu_attached) {
		if (ticks_since_vcu_message > CAN_TIMEOUT_TICK_COUNT) add_status(VCU_TIMEOUT);
		if ((vcu_state & VCU_ERROR) &&
			(vcu_state != VCU_SHUTDOWN_OPEN) &&
			(vcu_state != VCU_MC_FAULT)) add_status(VCU_FAULT);

		if (ticks_since_mc_message > CAN_TIMEOUT_TICK_COUNT) add_status(MC_TIMEOUT);
		if (((mc_post_faults != MC_HW_GATE_POST_FAULT) && mc_post_faults) ||
			((mc_run_faults != MC_HW_GATE_RUN_FAULT) && mc_run_faults)) add_status(MC_FAULT);
		if (mc_discharge_state == DISCHARGE_ACTIVE) add_status(MC_DISCHARGING); 
	if (charger_attached) {
		if (ticks_since_charger_message > CAN_TIMEOUT_TICK_COUNT) add_status(CHARGER_TIMEOUT);
		if ((charger_status & 0b11111) != 0) add_status(CHARGER_FAULT); // check charger fault bits
		//checks for bit 3, instead of 0b10111 chaned to 0b11111
	}

	}

	if (!HAL_GPIO_ReadPin(SDC_Final_GPIO_Port, SDC_Final_Pin)) add_status(SHUTDOWN); // shutdown circuit open
}
