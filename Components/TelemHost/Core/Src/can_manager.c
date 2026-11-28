#include "can_manager.h"
#include "serial_print.h"
#include "motor_controller.h"

volatile uint8_t mc_lockout;
volatile uint8_t mc_enabled;
volatile int16_t capacitor_volt_x10 = 0;
volatile uint8_t shutdown_flags = 0b0011111000;  //start with shutdown flags OK
volatile uint8_t estop_flags = 0;
volatile uint8_t switches = 0xC0;   //start with switches on to stay in startup state
volatile uint8_t PACK_TEMP;
volatile uint8_t mc_fault;
volatile uint8_t soc;
volatile uint8_t bms_status;
volatile uint8_t mc_fault_clear_success = 0;
volatile uint16_t pack_voltage;
volatile uint16_t motor_temp;
volatile uint16_t mc_temp;
volatile int16_t glv_v;

volatile int16_t motor_speed = 0;
volatile uint16_t rear_right_wheel_speed = 0;
volatile uint16_t rear_left_wheel_speed = 0;
volatile uint8_t wheel_updated[2] = {1,0};
volatile int16_t inlet_temp = 0;
volatile int16_t outlet_temp = 0;
volatile int16_t inlet_pres = 0;
volatile int16_t outlet_pres = 0;
volatile uint16_t telem_id = 0;
volatile uint16_t sg_rear = 0;
volatile uint16_t max_power = 0;

volatile uint16_t pack0_voltages[24];
volatile uint16_t pack1_voltages[24];
volatile uint16_t pack2_voltages[24];
volatile uint16_t pack3_voltages[24];
volatile uint16_t pack4_voltages[24];
volatile uint16_t pack0_temps[16];
volatile uint16_t pack1_temps[16];
volatile uint16_t pack2_temps[16];
volatile uint16_t pack3_temps[16];
volatile uint16_t pack4_temps[16];
volatile int16_t pei_current;

uint8_t pack_num;
uint8_t pack_group_index;

float AVG_DIAGNOSTIC_BMS_DATA;


static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];


//my task: take the average of the packs and make id and use sd card to write

static void save_can_rx_data(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	save_can_rx_data(RxHeader, RxData);
}


/************ CAN RX ************/
// TODO ADD CASES FOR NEW IDS AND ADD TO ENUM
static void save_can_rx_data(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]) {
    // gets message and updates values
//	sd_card_write_can_rx(rxHeader, rxData);  // TODO REMOVE
//	switch (rxHeader.StdId) {
//		case BMS_STATUS_MSG:
//			bms_status = rxData[0];
//			sd_card_write_can_rx(rxHeader, rxData);
//
//
//			break;
//		case DIAGNOSTIC_BMS_DATA:
//			PACK_TEMP = rxData[0];
//			soc = rxData[1];
//			pack_voltage = (rxData[2] << 8);
//			pack_voltage += rxData[3];
//
//
//			sd_card_write_can_rx(rxHeader, rxData);
//
//		AVG_DIAGNOSTIC_BMS_DATA = (float)(PACK_TEMP + soc + pack_voltage) / 3.0f;
//
//
//
//			break;
//
//
//		case PEI_STATUS_MSG:
//			shutdown_flags = rxData[0];
//			sd_card_write_can_rx(rxHeader, rxData);
//			break;
//		case MC_VOLTAGE_INFO:
//			static uint8_t mc_voltage_msg_counter = 0;
//
//			capacitor_volt_x10 = (rxData[1] << 8); // upper bits
//			capacitor_volt_x10 += rxData[0]; // lower bits
//
//			if (mc_voltage_msg_counter == 0)
//				sd_card_write_can_rx(rxHeader, rxData);
//
//			mc_voltage_msg_counter++;
//			mc_voltage_msg_counter %= 50;
//
//			break;
//		case MC_INTERNAL_STATES:
//			static uint8_t mc_state_msg_counter = 0;
//
//			mc_lockout = rxData[6] & 0b1000000;
//			mc_enabled = rxData[6] & 0b1;
//
//			if (mc_state_msg_counter == 0)
//				sd_card_write_can_rx(rxHeader, rxData);
//
//			mc_state_msg_counter++;
//			mc_state_msg_counter %= 50;
//
//			break;
//		case MC_FAULT_CODES:
//			static uint8_t first_fault = 1;
//			for (uint8_t i = 0; i < 8; ++i) {
//				if (rxData[i] > 0) {
//					mc_fault = 1;
//
//					if (first_fault) {
//						sd_card_write_can_rx(rxHeader, rxData);
//						first_fault = 0;
//					}
//
//					break;
//				}
//				else {
//					mc_fault = 0;
//					first_fault = 1;
//				}
//			}
//			break;
//		case MC_PARAM_RESPONSE:
//			//static uint8_t mc_param_msg_counter = 0;
//
//			if (rxData[0] == 0x20 && rxData[2] == 1) {
//				mc_fault_clear_success = 1;
//			}
//			break;
////		case WHEEL_SPEED_REAR:
////			rear_right_wheel_speed = (rxData[0] << 8);
////			rear_right_wheel_speed += rxData[1];
////			rear_left_wheel_speed = (rxData[2] << 8);
////			rear_left_wheel_speed += rxData[3];
////			wheel_updated[1] = 1;
////			telem_id = 0;
////			break;
//		case MC_MOTOR_POSITION:
//			static uint8_t mc_motor_pos_msg_counter = 0;
//
//			motor_speed = (rxData[3] << 8);
//			motor_speed |= rxData[2];
//			motor_speed *= -1;
//
//			// TEMPORARY?
//			rear_right_wheel_speed = (rxData[3] << 8);
//			rear_right_wheel_speed += rxData[2];
//			rear_right_wheel_speed *= -1;
//			wheel_updated[1] = 1;
//			telem_id = 0;
//
//			if (mc_motor_pos_msg_counter == 0)
//				sd_card_write_can_rx(rxHeader, rxData);
//
//			++mc_motor_pos_msg_counter;
//			mc_motor_pos_msg_counter %= 10;
//
//			break;
//		case COOLING_LOOP:
//			inlet_temp = (rxData[0] << 8);
//			inlet_temp += rxData[1];
//			outlet_temp = (rxData[2] << 8);
//			outlet_temp += rxData[3];
//			inlet_pres = (rxData[4] << 8);
//			inlet_pres += rxData[5];
//			outlet_pres = (rxData[6] << 8);
//			outlet_pres += rxData[7];
//			telem_id = 1;
//
//			sd_card_write_can_rx(rxHeader, rxData);
//			break;
//		case MC_TEMP_3:
//			static uint8_t motor_temp_msg_counter = 0;
//
//			motor_temp = rxData[5] << 8;
//			motor_temp += rxData[4];
//
//			if (motor_temp_msg_counter == 0)
//				sd_card_write_can_rx(rxHeader, rxData);
//			++motor_temp_msg_counter;
//			motor_temp_msg_counter %= 50;
//
//			break;
//		case MC_TEMP_1:
//			static uint8_t mc_temp_msg_counter = 0;
//
//			uint16_t module_a_temp = (rxData[1] << 8) + rxData[0];
//			uint16_t module_b_temp = (rxData[3] << 8) + rxData[2];
//			uint16_t module_c_temp = (rxData[5] << 8) + rxData[4];
//			mc_temp = (module_a_temp + module_b_temp + module_c_temp) / 3; // no unit conversion, don't want to store float
//
//			if (mc_temp_msg_counter == 0)
//				sd_card_write_can_rx(rxHeader, rxData);
//			++mc_temp_msg_counter;
//			mc_temp_msg_counter %= 50;
//
//			break;
//		case MC_INTERNAL_VOLTS:
//			static uint8_t mc_glv_msg_counter = 0;
//
//			glv_v = rxData[7] << 8;
//			glv_v += rxData[6]; // no unit conversion, don't want to store float
//
//			if (mc_glv_msg_counter == 0)
//				sd_card_write_can_rx(rxHeader, rxData);
//
//			++mc_glv_msg_counter;
//			mc_glv_msg_counter %= 100;
//
//			break;
//		case MC_INTERNAL_CURRENTS:
//			int16_t current_x10 = (rxData[7] << 8) + rxData[6];
//			if(capacitor_volt_x10 > 0 && current_x10 > 0){
//				uint16_t power = (capacitor_volt_x10 / 10) * (current_x10 / 10) / 1000;
//				if(power > max_power) max_power = power;
//			}
//			break;
//		case STRAIN_GAUGE_REAR:
//			sg_rear = rxData[0] << 8;
//			sg_rear += rxData[1];
//			break;
//		case PEI_CURRENT:
//			pei_current = (rxData[0] << 8) | rxData[1];
//			break;
//		case BMS_VOLTAGES:
//			pack_num = rxData[0];
//			pack_group_index = rxData[1] * 3;
//			switch (pack_num){
//			case 0:
//				pack0_voltages[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 100;
//				pack0_voltages[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 100;
//				pack0_voltages[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 100;
//				break;
//			case 1:
//				pack1_voltages[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 100;
//				pack1_voltages[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 100;
//				pack1_voltages[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 100;
//				break;
//			case 2:
//				pack2_voltages[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 100;
//				pack2_voltages[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 100;
//				pack2_voltages[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 100;
//				break;
//			case 3:
//				pack3_voltages[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 100;
//				pack3_voltages[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 100;
//				pack3_voltages[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 100;
//				break;
//			case 4:
//				pack4_voltages[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 100;
//				pack4_voltages[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 100;
//				pack4_voltages[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 100;
//				break;
//			}
//			break;
//		case BMS_TEMPS:
//			pack_num = rxData[0];
//			pack_group_index = rxData[1] * 3;
//			switch (pack_num){
//			case 0:
//				pack0_temps[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 1000;
//				if(pack_group_index == 15) break;
//				pack0_temps[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 1000;
//				pack0_temps[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 1000;
//				break;
//			case 1:
//				pack1_temps[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 1000;
//				if(pack_group_index == 15) break;
//				pack1_temps[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 1000;
//				pack1_temps[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 1000;
//				break;
//			case 2:
//				pack2_temps[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 1000;
//				if(pack_group_index == 15) break;
//				pack2_temps[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 1000;
//				pack2_temps[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 1000;
//				break;
//			case 3:
//				pack3_temps[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 1000;
//				if(pack_group_index == 15) break;
//				pack3_temps[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 1000;
//				pack3_temps[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 1000;
//				break;
//			case 4:
//				pack4_temps[pack_group_index] = ((rxData[2] << 8) | rxData[3]) / 1000;
//				if(pack_group_index == 15) break;
//				pack4_temps[pack_group_index + 1] = ((rxData[4] << 8) | rxData[5]) / 1000;
//				pack4_temps[pack_group_index + 2] = ((rxData[6] << 8) | rxData[7]) / 1000;
//				break;
//			}
//			break;
//		default:
//			// no valid input received
//			break;
//	}

}


/************ CAN TX ************/

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox;

//  transmit state
//void can_tx_vcu_state(CAN_HandleTypeDef *hcan){
//	static uint8_t vcu_state_msg_counter = 0;
//
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.StdId = VEHICLE_STATE;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 8;
//	uint16_t tick = (uint16_t)HAL_GetTick();
//	uint8_t data_tx_state[8] = {
//        0,
//        is_hv_requested(),
//        throttle1.percent,
//        throttle2.percent,
//		brake.percent,
//        one_byte_state(),
//		(tick >> 8) & 0xFF,
//		tick & 0xFF
//    };
//
//	if (vcu_state_msg_counter == 0)
//		sd_card_write_can_tx(TxHeader, data_tx_state);
//
//	vcu_state_msg_counter++;
//	vcu_state_msg_counter %= 50;
//
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_state, &TxMailbox) != HAL_OK)
//	{
//	  print("CAN Tx failed\r\n");
//	}
////    write_tx_to_sd(TxHeader, data_tx_state);
//}

HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t* data, uint8_t len)
{
	static CAN_TxHeaderTypeDef msg_hdr;
	msg_hdr.IDE = CAN_ID_STD;
	msg_hdr.StdId = id;
	msg_hdr.RTR = CAN_RTR_DATA;
	msg_hdr.DLC = len;

	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return HAL_OK;
	return HAL_CAN_AddTxMessage(hcan, &msg_hdr, data, &TxMailbox);
}

////  transmit state
//void can_tx_sg(CAN_HandleTypeDef *hcan, uint16_t adc){
//	static uint8_t tc_sg_msg_counter = 0;
//
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.StdId = 0x500;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 6;
//	uint8_t data_tx_state[6] = {
//		(adc >> 8) & 0xFF,
//		(adc & 0xFF),
//		front_right_wheel_speed >> 8,
//		front_right_wheel_speed & 0xff,
////		front_left_wheel_speed >> 8,
////		front_left_wheel_speed & 0xff,
//		TC_torque_req  >> 8,
//		TC_torque_req & 0xff,
//    };
//
//	if (tc_sg_msg_counter == 0)
//		sd_card_write_can_tx(TxHeader, data_tx_state);
//
//	tc_sg_msg_counter++;
//	tc_sg_msg_counter %= 2;
//
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_state, &TxMailbox) != HAL_OK)
//	{
//	  print("CAN Tx failed\r\n");
//	}
////    write_tx_to_sd(TxHeader, data_tx_state);
//}



void can_tx_disable_MC(CAN_HandleTypeDef *hcan) {
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = TORQUE_REQUEST;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

	uint8_t data_tx_torque[8] = {0,0,0,0,0,0,0,0};

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_torque, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//	write_tx_to_sd(TxHeader, data_tx_torque);
}

void can_clear_MC_fault(CAN_HandleTypeDef *hcan) {
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = MC_PARAM_COMMAND;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

	const uint16_t param_addr = 20;
	uint8_t data_tx_param_command[8] = {
			param_addr & 0xFF, // address lower (little endian)
			param_addr >> 8, // address upper
			1, // r/w: 1 = write
			0, // reserved
			0, // data
			0, // data
			0, // reserved
			0 // reserved
	};

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_param_command, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//	write_tx_to_sd(TxHeader, data_tx_param_command);
}

void can_tx_override_cooling_request(CAN_HandleTypeDef *hcan, uint8_t bit) {
	uint8_t data[1] = {bit};
	CAN_Send(hcan, 0xA51, data, 1);
}
