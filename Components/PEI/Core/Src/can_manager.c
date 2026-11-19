/*
 * can_manager.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Abhineet
 */


#include "can_manager.h"
#include "main.h"
#include "cell_interface.h"
#include "charger.h"
#include "fsm.h"
#include "utils.h"
#include "frucd.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern volatile uint8_t hv_requested;
extern volatile uint8_t vcu_state;
extern volatile uint16_t vcu_attached;
extern volatile uint32_t ticks_since_vcu_message;

extern volatile int16_t mc_voltage;
extern volatile uint8_t mc_vsm_state;
extern volatile uint8_t mc_discharge_state;
extern volatile uint32_t mc_post_faults;
extern volatile uint32_t mc_run_faults;
extern volatile uint32_t ticks_since_mc_message;

extern volatile uint8_t charger_attached;
extern volatile uint8_t charger_status;
extern uint16_t charger_max_current;
extern uint8_t charge_control;
extern volatile uint32_t ticks_since_charger_message;

extern volatile BAT_PACK_t bat_pack;
extern uint8_t max_fault_addr;
extern uint8_t max_faults;
extern int8_t comm_bk_id;

volatile uint32_t vcu_tickstart = 0;
volatile uint32_t mc_tickstart = 0;
volatile uint32_t charger_tickstart = 0; 

static void can_send_std(CAN_HandleTypeDef *hcan, uint32_t frame_id, const uint8_t *data, int dlc) {
    if (dlc < 0) {
        return; // Pack failed (e.g., -EINVAL)
    }

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = frame_id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = (uint32_t)dlc,
    };

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
}

static void set_tx_header_defaults(CAN_TxHeaderTypeDef* const tx_header_ptr) {
	tx_header_ptr->IDE = CAN_ID_STD;
	tx_header_ptr->RTR = CAN_RTR_DATA;
}

void can_send_PEI_Status(uint8_t shutdown_flags) {
	const uint32_t PEI_STATUS_MSG_ID = 0x387;
	static uint32_t PEI_STATUS_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = PEI_STATUS_MSG_ID;
	tx_header.DLC = 5;

	uint8_t tx_data[8];
	tx_data[0] = shutdown_flags;
	tx_data[1] = HI8(bat_pack.current_raw);
	tx_data[2] = LO8(bat_pack.current_raw);
	tx_data[3] = HI8(bat_pack.current_ref_raw);
	tx_data[4] = LO8(bat_pack.current_ref_raw);

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &PEI_STATUS_TX_MAILBOX);
}

void can_send_PEI_Current() {
	const uint32_t PEI_CURRENT_MSG_ID = 0x388;
	static uint32_t PEI_CURRENT_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = PEI_CURRENT_MSG_ID;
	tx_header.DLC = 2;

	int16_t current = bat_pack.current * 100; // 2 decimals of precision

	uint8_t tx_data[8];
	tx_data[0] = HI8(current);
	tx_data[1] = LO8(current);

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &PEI_CURRENT_TX_MAILBOX);
}

void can_send_Charger() {
	const uint32_t PEI_CHARGER_TX_MSG_ID = 0x1806E5F4;
	static uint32_t PEI_CHARGER_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.ExtId = PEI_CHARGER_TX_MSG_ID;
	tx_header.DLC = 5;

	uint8_t tx_data[8];
	tx_data[0] = HI8(CHARGER_MAX_VOLTAGE);
	tx_data[1] = LO8(CHARGER_MAX_VOLTAGE);
	tx_data[2] = HI8(charger_max_current);
	tx_data[3] = LO8(charger_max_current);
	tx_data[4] = charge_control;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &PEI_CHARGER_TX_MAILBOX);
}

void can_send_BMS_Status() {
	const uint32_t BMS_STATUS_MSG_ID = 0x380;
	static uint32_t BMS_STATUS_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = BMS_STATUS_MSG_ID;
	tx_header.DLC = 5;

	update_max_fault_data();

	uint8_t tx_data[8];
	tx_data[0] = bat_pack.status;
	tx_data[1] = HI8(bat_pack.spi_fault_addresses);
	tx_data[2] = LO8(bat_pack.spi_fault_addresses);
	tx_data[3] = max_fault_addr;
	tx_data[4] = max_faults;
	tx_data[5] = (uint8_t)comm_bk_id;

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &BMS_STATUS_TX_MAILBOX);
}

void can_send_BMS_Diagnostics() {
	const uint32_t BMS_DIAGNOSTIC_DATA_MSG_ID = 0x381;
	static uint32_t BMS_DIAGNOSTIC_DATA_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = BMS_DIAGNOSTIC_DATA_MSG_ID;
	tx_header.DLC = 4;

	uint8_t tx_data[8];
	tx_data[0] = bat_pack.HI_temp_c;
	tx_data[1] = bat_pack.SOC_percent;
	tx_data[2] = HI8(bat_pack.pack_voltage_raw);
	tx_data[3] = LO8(bat_pack.pack_voltage_raw);

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &BMS_DIAGNOSTIC_DATA_TX_MAILBOX);
}

void can_send_BMS_High_Level_Data() {
	const uint32_t BMS_HIGH_LEVEL_DATA_MSG_ID = 0x382;
	static uint32_t BMS_HIGH_LEVEL_DATA_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = BMS_HIGH_LEVEL_DATA_MSG_ID;
	tx_header.DLC = 6;

	uint16_t HI_temp_c = bat_pack.HI_temp_c * 1000; // 3 decimals of precision
	uint16_t pack_voltage = bat_pack.filt_pack_voltage * 100; // 2 decimals of precision
	uint16_t pack_balance = bat_pack.filt_pack_balance * 100000; // 5 decimals of precision (2 if interpreted in mV)

	uint8_t tx_data[8];
	tx_data[0] = HI8(HI_temp_c);
	tx_data[1] = LO8(HI_temp_c);
	tx_data[2] = HI8(pack_voltage);
	tx_data[3] = LO8(pack_voltage);
	tx_data[4] = HI8(pack_balance);
	tx_data[5] = LO8(pack_balance);

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &BMS_HIGH_LEVEL_DATA_TX_MAILBOX);
}

void can_send_BMS_Subpack_Data(uint8_t subpack_num) {
	const uint32_t BMS_SUBPACK_DATA_MSG_ID = 0x383;
	static uint32_t BMS_SUBPACK_DATA_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = BMS_SUBPACK_DATA_MSG_ID;
	tx_header.DLC = 5;

	uint16_t subpack_voltage = bat_pack.subpacks[subpack_num].filt_subpack_voltage * 100; // 2 decimals of precision
	uint16_t subpack_balance = bat_pack.subpacks[subpack_num].filt_subpack_balance * 100000; // 5 decimals of precision

	uint8_t tx_data[8];
	tx_data[0] = subpack_num;
	tx_data[1] = HI8(subpack_voltage);
	tx_data[2] = LO8(subpack_voltage);
	tx_data[3] = HI8(subpack_balance);
	tx_data[4] = LO8(subpack_balance);

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &BMS_SUBPACK_DATA_TX_MAILBOX);
}

void can_send_BMS_Temps(uint8_t subpack_num, uint8_t group) {
	const uint32_t BMS_TEMPS_MSG_ID = 0x384;
	static uint32_t BMS_TEMPS_TX_MAILBOX;

	// Configure TX header
	CAN_TxHeaderTypeDef tx_header;
	set_tx_header_defaults(&tx_header);
	tx_header.StdId = BMS_TEMPS_MSG_ID;
	tx_header.DLC = 8;

	uint8_t starting_cell = group * 3;
	uint16_t temp1 = get_filtered_cell_temp(subpack_num, starting_cell) * 1000;
	uint16_t temp2 = group < 5 ? get_filtered_cell_temp(subpack_num, starting_cell + 1) * 1000 : 0;
	uint16_t temp3 = group < 5 ? get_filtered_cell_temp(subpack_num, starting_cell + 2) * 1000 : 0;

	uint8_t tx_data[8];
	tx_data[0] = subpack_num;
	tx_data[1] = group;
	tx_data[2] = HI8(temp1);
	tx_data[3] = LO8(temp1);
	tx_data[4] = HI8(temp2);
	tx_data[5] = LO8(temp2);
	tx_data[6] = HI8(temp3);
	tx_data[7] = LO8(temp3);

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &BMS_TEMPS_TX_MAILBOX);
}

//This is the refactored FIFO 0 callback for PEI 
//eliminates the confusion between Little Endian and Big Endian bit shifting by getting rid of manual bit shifting in place of using cantools 
//allocates DBC struct onto call stack instead of allocating hcan1 and hcan2 to the same CAN_Rx interrupt 
//new DBC call stack will save can1 to stack frame A, then can2 to stack frame b if can2 interrupt happens when can1 is in progress, letting can1 finish cleanly before can2 goes from stack B to A and finishes 
//this prevents overriting on the bus, as hcan1 and hcan2 use the same interrupt 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_ptr){
	CAN_RxHeaderTypeDef rx0_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx0_header, rx_data);

	if(hcan_ptr == &hcan1){
		if(rx0_header.IDE == CAN_ID_STD && rx0_header.StdID == FRUCD_VEHICLE_STATE_FRAME_ID){
			struct frucd_vehicle_state_t vcu_msg;
			if(frucd_vehicle_state_unpack(&vcu_msg, rx_data, rx0_header.DLC) == 0){
				vcu_tickstart = HAL_GetTick();
				ticks_since_vcu_message = 0;
				vcu_attached = 1;

				hv_requested = vcu_msg.dashboard_hv_requested;
				vcu_state = vcu_msg.dashboard_state;

				bat_pack.status &= ~CHARGEMODE;
			}
		}
		else if(hcan_ptr == &hcan2){
			if(rx0_header.DLC >= 5){
				charger_tickstart = HAL_GetTick();
				ticks_since_charger_message = 0;
				charger_attached = 1;

				bat_pack.status |= CHARGEMODE;

				charger_status = rx_data[4];
				if((charger_status & 0b1111) == 0){ //no faults, charger active  
					vcu_state = CHARGING;
				}
				else if((charger_status & 0b1011) == 0){ //no faults, charger inactive
					vcu_state = LV;
				}
				else{
					//charger faults, handled in fsm.c
				}
			}
		}
	}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan_ptr) {
	CAN_RxHeaderTypeDef rx1_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO1, &rx1_header, rx_data);

	const uint32_t MC_VOLTAGE_MSG_ID = 0x0A7;
	const uint32_t MC_STATE_MSG_ID = 0x0AA;
	const uint32_t MC_FAULT_MSG_ID = 0x0AB;

	mc_tickstart = HAL_GetTick();
	ticks_since_mc_message = 0;

	uint32_t id = rx1_header.StdId;
	if (id == MC_VOLTAGE_MSG_ID) {
		mc_voltage = CAN_RX1_BUFFER[1] << 8;
		mc_voltage |= CAN_RX1_BUFFER[0];
	}
	else if (id == MC_STATE_MSG_ID) {
		if (rx1_header.DLC >= 5) {
			mc_vsm_state = rx_data[0];
			mc_discharge_state = (rx_data[4] >> 5) & 0x07;
		}
	}
	else if (id == MC_FAULT_MSG_ID) {
		if (rx1_header.DLC >= 8) {
			mc_post_faults = ((uint32_t)rx_data[3] << 24) |
			                 ((uint32_t)rx_data[2] << 16) |
			                 ((uint32_t)rx_data[1] << 8)  |
			                 ((uint32_t)rx_data[0]);

		mc_run_faults = CAN_RX1_BUFFER[7] << 24;
		mc_run_faults |= CAN_RX1_BUFFER[6] << 16;
		mc_run_faults |= CAN_RX1_BUFFER[5] << 8;
		mc_run_faults |= CAN_RX1_BUFFER[4];
	}
}

void update_can_status() {
	ticks_since_vcu_message = HAL_GetTick() - vcu_tickstart;
	ticks_since_mc_message = HAL_GetTick() - mc_tickstart;
	ticks_since_charger_message = HAL_GetTick() - charger_tickstart;
}
