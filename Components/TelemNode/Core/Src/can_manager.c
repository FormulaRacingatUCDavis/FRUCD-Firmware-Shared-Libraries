/*
 * can_manager.c
 *
 *  Created on: Apr 18, 2024
 *      Author: leoja
 */

#include "can_manager.h"
#include "main.h"
#include "motor_controller.h"

extern CAN_HandleTypeDef hcan;

CAN_DATA_t can_data;

void CAN_Init()
{
	// Filter vehicle state messages into FIFO0
	CAN_FilterTypeDef can_filter;
	can_filter.FilterActivation = CAN_FILTER_ENABLE;
	can_filter.SlaveStartFilterBank = 28;
	can_filter.FilterBank = 0;
	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterIdHigh = 0;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow = 0;
	if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) {
	  Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}
	if ( HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}
}

HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t* data, uint8_t len)
{
	static uint32_t TxMailbox;
	static CAN_TxHeaderTypeDef msg_hdr;
	msg_hdr.IDE = CAN_ID_STD;
	msg_hdr.StdId = id;
	msg_hdr.RTR = CAN_RTR_DATA;
	msg_hdr.DLC = len;

	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) return HAL_OK;
	return HAL_CAN_AddTxMessage(&hcan, &msg_hdr, data, &TxMailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_ptr)
{
	CAN_RxHeaderTypeDef can_rx_header;
	uint8_t can_rx_data[8];

 	if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &can_rx_header, can_rx_data) != HAL_OK) {
		Error_Handler();
	}

	if(can_rx_header.IDE != CAN_ID_STD){
		return;
	}

	switch(can_rx_header.StdId){
		case VEHICLE_STATE: 
			if(can_rx_header.DLC >= 5){
				can_data.vcu_state = can_rx_data[4];
			}
			break;

		case BMS_STATUS_MSG: 
			if(can_rx_header.DLC >= 1){
				can_data.bms_temp = can_rx_data[0];
			}
			break; 
		
		case MOTOR_CONTROLLER_M160_TEMPERATURE_SET_1_FRAME_ID:{ //0x0A0
			struct motor_controller_m160_temperature_set_1_t temps1; 
			if(motor_controller_m160_temperature_set_1_unpack(&temps1, can_rx_data, can_rx_header.DLC) == 0){
				//Raw signals are already signed int16_t sacled at 0.1 deg C 
				can_data.mc_temp_module_a = temps1.inv_module_a_temp; 
				can_data.mc_temp_module_b = temps1.inv_module_b_temp;
				can_data.mc_temp_module_c = temps1.inv_module_c_temp;

				int16_t max_temp = can_data.mc_temp_module_a;
				if(can_data.mc_temp_module_b > max_temp){
					max_temp = can_data.mc_temp_module_b;
				}
				if(can_data.mc_temp_module_c > max_temp){
					max_temp = can_data.mc_temp_module_c;
				}

				can_data.mc_temp_max = max_temp;
				can_data.mc_temps1_last_tick = HAL_GetTick();
			}
			break; 
		}

		case MOTOR_CONTROLLER_M166_CURRENT_INFO_FRAME_ID: { //0x0A6
			struct motor_controller_m166_current_info_t curr;
			if(motor_controller_m166_current_info_unpack(&curr, can_rx_data, can_rx_header.DLC) == 0){
				//Raw signals are signed int16_t scaled at 0.1A
				can_data.inv_dc_bus_current = curr.inv_dc_bus_current;
				can_data.inv_phase_a_current = curr.inv_phase_a_current; 
				can_data.inv_phase_b_current = curr.inv_phase_b_current; 
				can_data.inv_phase_c_current = curr.inv_phase_c_current; 
				can_data.mc_current_last_tick = HAL_GetTick();
			}
			break; 
		}

		case MOTOR_CONTROLLER_M167_VOLTAGE_INFO_FRAME_ID: { //0x0A7
			struct motor_controller_m167_voltage_info_t volt;
			if(motor_controller_m167_voltage_info_unpack(&volt, can_rx_data, can_rx_header.DLC) == 0){
				//Raw signal is signed int16_t scaled at 0.1V 
				can_data.inv_dc_bus_voltage = volt.inv_dc_bus_voltage;
				can_data.mc_voltage_last_tick = HAL_GetTick();
			}
			break;
		}

		case MOTOR_CONTROLLER_M170_INTERNAL_STATES_FRAME_ID: { //0x0AA
			struct motor_controller_m170_internal_states_t states;
			if(motor_controller_m170_internal_states_unpack(&states, can_rx_data, can_rx_header.DLC) == 0){
				can_data.inv_state = states.inv_inverter_state;
				can_data.inv_vsm_state = states.inv_vsm_state;
				can_data.inv_enable_lockout = states.inv_inverter_enable_lockout;
				can_data.mc_states_last_tick = HAL_GetTick();
			}
			break;
		}

		case MOTOR_CONTROLLER_M171_FAULT_CODES_FRAME_ID:{ //0x0AB
			struct motor_controller_m171_fault_codes_t faults;
			if(motor_controller_m171_fault_codes_unpack(&faults, can_rx_data, can_rx_header.DLC) == 0){
				can_data.inv_post_fault_lo = faults.inv_post_fault_lo;
				can_data.inv_post_fault_hi = faults.inv_post_fault_hi;
				can_data.inv_run_fault_lo = faults.inv_run_fault_lo;
				can_data.inv_run_fault_hi = faults.inv_run_fault_hi;
				can_data.mc_faults_last_tick = HAL_GetTick();
			}
			break;
		}

		case TORQUE_REQUEST: // 0x0C0 
			if(can_rx_header.DLC >= 0){
				can_data.inverter_enable = can_rx_data[5] & 0x01;
			}
			break; 

		case PUMP_PWM: 
			if(can_rx_header.DLC >= 1){
				can_data.PWM_requested = (can_rx_data[0] == 1) ? 1 : 0;
			}
			break;

		default: 
			break;

	}

	/*
	switch(can_rx_header.StdId)
	{
		case VEHICLE_STATE:
			can_data.vcu_state = can_rx_data[4];
			break;
		case BMS_STATUS_MSG:
			can_data.bms_temp = can_rx_data[0];
			break;
		case MC_TEMPS_1:
			can_data.mc_temp_module_a = (can_rx_data[1] << 8);
			can_data.mc_temp_module_a |= can_rx_data[0];

			can_data.mc_temp_module_c = (can_rx_data[3] << 8);
			can_data.mc_temp_module_c |= can_rx_data[2];

			can_data.mc_temp_module_c = (can_rx_data[5] << 8);
			can_data.mc_temp_module_c |= can_rx_data[4];

			can_data.mc_temp_max = can_data.mc_temp_module_a;
			if(can_data.mc_temp_module_b > can_data.mc_temp_max){
				can_data.mc_temp_max = can_data.mc_temp_module_b;
			}
			if(can_data.mc_temp_module_c > can_data.mc_temp_max){
				can_data.mc_temp_max = can_data.mc_temp_module_c;
			}
			break;
		case MC_TEMPS_3:
			can_data.motor_temp = (can_rx_data[5] << 8);
			can_data.motor_temp |= can_rx_data[4];
			break;
		case TORQUE_REQUEST:
			can_data.inverter_enable = can_rx_data[5] & 0x01;
			break;
		case PUMP_PWM:
			// 1 means override to use hardcoded value, 0 means we can use our own algoritm for PWM values
			if (can_rx_data[0] == 1) {
				can_data.PWM_requested = 1;
			} else {
				can_data.PWM_requested = 0;
			}
			break;
	}
			*/
}

