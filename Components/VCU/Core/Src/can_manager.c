#include "can_manager.h"
#include "serial_print.h"
#include "traction_control.h"
#include "driver_input.h"
#include "main.h"
#include "frucd.h"

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
volatile uint8_t dashboard_display_mode = 0; // 0 is drive
volatile int16_t pack_voltage;
volatile uint16_t motor_temp;
volatile uint16_t mc_temp;
volatile int16_t glv_v;
volatile uint16_t acc_current_adc;
volatile uint16_t acc_current_ref_adc;

volatile int16_t motor_speed = 0;
volatile uint16_t rear_right_wheel_speed = 0;
volatile uint16_t rear_left_wheel_speed = 0;
volatile uint16_t front_right_wheel_speed = 0;
volatile uint16_t front_left_wheel_speed = 0;
volatile uint8_t wheel_updated[2] = {1,0};
volatile int16_t inlet_temp = 0;
volatile int16_t outlet_temp = 0;
volatile int16_t inlet_pres = 0;
volatile int16_t outlet_pres = 0;
volatile uint16_t telem_id = 0;
volatile uint16_t sg_rear = 0;
volatile uint16_t max_power = 0;

extern volatile uint32_t torque_percentage;
extern volatile uint32_t launch_control_param;

static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];

static void save_can_rx_data(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	save_can_rx_data(RxHeader, RxData);
}


/************ CAN RX ************/

static void save_can_rx_data(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]) {
    // gets message and updates values
	switch (rxHeader.StdId) {
		case FRUCD_BMS_STATUS_FRAME_ID:
		{
			struct frucd_bms_status_t m;
			if (frucd_bms_status_unpack(&m, rxData, rxHeader.DLC) == 0) {
				bms_status = m.pei_bms_status;
			}
			break;
		}
		case DIAGNOSTIC_BMS_DATA:
		{
    		struct frucd_diagnostic_bms_data_t m;
    			if (frucd_diagnostic_bms_data_unpack(&m, rxData, rxHeader.DLC) == 0) {
       				PACK_TEMP    = m.pei_hi_temp;
       				soc          = m.pei_soc;
        			pack_voltage = m.pei_pack_voltage;
    			}
    			break;
		}	

		/*
			PACK_TEMP = rxData[0];
			soc = rxData[1];
			pack_voltage = (rxData[2] << 8);
			pack_voltage += rxData[3];
			break;
		*/
		case FRUCD_STATUS_FRAME_ID: // PEI status: shutdown circuit flags + current (PEI_STATUS_MSG)
		{
			struct frucd_status_t s;
			if (frucd_status_unpack(&s, rxData, rxHeader.DLC) == 0) {
				// rebuild the old flag byte so shutdown_closed() and the
				// precharge/AIR check in main.c keep working unchanged
				shutdown_flags = (s.pei_imd_ok         << 5)
				               | (s.pei_bms_ok         << 4)
				               | (s.pei_shutdown_final << 3)
				               | (s.pei_air_neg        << 2)
				               | (s.pei_air_pos        << 1)
				               |  s.pei_precharge;
				acc_current_adc     = s.pei_current_adc;
				acc_current_ref_adc = (uint16_t)s.pei_current_reference; // DBC says signed, old code unsigned
			}
			break;
		}
		case MC_INTERNAL_STATES:
			static uint8_t mc_int_state_msg_counter = 0;
			mc_int_state_msg_counter++;

			mc_lockout = rxData[6] & 0b1000000;
			mc_enabled = rxData[6] & 0b1;

			if (mc_int_state_msg_counter >= 50) {
				mc_int_state_msg_counter = 0;
			}

			break;
		case MC_FAULT_CODES:
			static uint8_t first_fault = 1;
			for (uint8_t i = 0; i < 8; ++i) {
				if (rxData[i] > 0) {
					mc_fault = 1;

					if (first_fault) {
						first_fault = 0;
					}

					break;
				}
				else {
					mc_fault = 0;
					first_fault = 1;
				}
			}
			break;
		case MC_PARAM_RESPONSE:
			//static uint8_t mc_param_msg_counter = 0;

			if (rxData[0] == 0x20 && rxData[2] == 1) {
				mc_fault_clear_success = 1;
			}
			break;
//		case WHEEL_SPEED_REAR:
//			rear_right_wheel_speed = (rxData[0] << 8);
//			rear_right_wheel_speed += rxData[1];
//			rear_left_wheel_speed = (rxData[2] << 8);
//			rear_left_wheel_speed += rxData[3];
//			wheel_updated[1] = 1;
//			telem_id = 0;
//			break;
		case MC_MOTOR_POSITION:
			static uint8_t mc_motor_msg_counter = 0;
			mc_motor_msg_counter++;

			motor_speed = (rxData[3] << 8);
			motor_speed |= rxData[2];
			motor_speed *= -1;

			// TEMPORARY?
			rear_right_wheel_speed = (rxData[3] << 8);
			rear_right_wheel_speed += rxData[2];
			rear_right_wheel_speed *= -1;
			wheel_updated[1] = 1;
			telem_id = 0;

			if (mc_motor_msg_counter >= 50) {
				mc_motor_msg_counter = 0;
			}

			break;
		case COOLING_LOOP:
			inlet_temp = (rxData[0] << 8);
			inlet_temp += rxData[1];
			outlet_temp = (rxData[2] << 8);
			outlet_temp += rxData[3];
			inlet_pres = (rxData[4] << 8);
			inlet_pres += rxData[5];
			outlet_pres = (rxData[6] << 8);
			outlet_pres += rxData[7];
			telem_id = 1;

			break;
		case MC_TEMP_3:
			static uint8_t mc_temp3_msg_counter = 0;
			mc_temp3_msg_counter++;

			motor_temp = rxData[5] << 8;
			motor_temp += rxData[4];

			if (mc_temp3_msg_counter >= 50) {
				mc_temp3_msg_counter = 0;
			}

			break;
		case MC_TEMP_1:
			static uint8_t mc_temp1_msg_counter = 0;
			mc_temp1_msg_counter++;

			uint16_t module_a_temp = (rxData[1] << 8) + rxData[0];
			uint16_t module_b_temp = (rxData[3] << 8) + rxData[2];
			uint16_t module_c_temp = (rxData[5] << 8) + rxData[4];
			mc_temp = (module_a_temp + module_b_temp + module_c_temp) / 3; // no unit conversion, don't want to store float

			if (mc_temp1_msg_counter >= 50) {
				mc_temp1_msg_counter = 0;
			}

			break;
		case MC_INTERNAL_VOLTS:
			static uint8_t mc_glv_msg_counter = 0;
			mc_glv_msg_counter++;

			glv_v = rxData[7] << 8;
			glv_v += rxData[6]; // no unit conversion, don't want to store float

			if (mc_glv_msg_counter >= 100) {
				mc_glv_msg_counter = 0;
			}

			break;
		case MC_INTERNAL_CURRENTS:
			int16_t current_x10 = (rxData[7] << 8) + rxData[6];
			if(capacitor_volt_x10 > 0 && current_x10 > 0){
				uint16_t power = (capacitor_volt_x10 / 10) * (current_x10 / 10) / 1000;
				if(power > max_power) max_power = power;
			}
			break;
		case FRUCD_STRAIN_GAUGES_REAR_FRAME_ID:
		{
			struct frucd_strain_gauges_rear_t g;
			if (frucd_strain_gauges_rear_unpack(&g, rxData, rxHeader.DLC) == 0) {
				sg_rear = g.telem_node_rl_toe_strain_gauge; // old code only read bytes 0-1
			}
			break;
		}
		default:
			// no valid input received
			break;
	}

}


/************ CAN TX ************/

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox;

//  transmit state (0x766)
void can_tx_vcu_state(CAN_HandleTypeDef *hcan){
	struct frucd_vehicle_state_t m;
	uint8_t data[FRUCD_VEHICLE_STATE_LENGTH];
 
	frucd_vehicle_state_init(&m);
	m.dashboard_hv_requested    = is_hv_requested();
	m.dashboard_throttle1_level = throttle1.percent;
	m.dashboard_throttle2_level = throttle2.percent;
	m.dashboard_brake_level     = brake.percent;
	m.dashboard_state           = one_byte_state();
	m.dashboard_vcu_ticks       = (uint16_t)HAL_GetTick();
 
	frucd_vehicle_state_pack(data, &m, sizeof(data));
 
	if (CAN_Send(hcan, FRUCD_VEHICLE_STATE_FRAME_ID, data, FRUCD_VEHICLE_STATE_LENGTH) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
}

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

//  transmit random shit for testing
// NOT MIGRATED: DBC says 0x500 = strain gauge / front wheel speed / TC torque request,
// but this sends torque %, launch param and the pressed button.
void can_tx_sg(CAN_HandleTypeDef *hcan, uint16_t adc){
	static uint8_t tc_sg_msg_counter = 0;
	tc_sg_msg_counter++;

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x500;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 6;
	uint8_t data_tx_state[6] = {
//		(adc >> 8) & 0xFF,
//		(adc & 0xFF),
//		front_right_wheel_speed >> 8,
//		front_right_wheel_speed & 0xff,
//		TC_torque_req  >> 8,
//		TC_torque_req & 0xff,
		torque_percentage >> 8,
		torque_percentage & 0xFF,
		launch_control_param >> 8,
		launch_control_param & 0xFF,
		(uint8_t)which_button_pressed(),
		0
    };

	if (tc_sg_msg_counter >= 50) {
		tc_sg_msg_counter = 0;
	}

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_state, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
}


// transmit torque request (0x0C0), goes to the motor controller
void can_tx_torque_request(CAN_HandleTypeDef *hcan){
	struct frucd_torque_request_t m;
	uint8_t data[FRUCD_TORQUE_REQUEST_LENGTH];
 
	frucd_torque_request_init(&m);          // every field starts at 0
	m.dashboard_discharge_enable = 1;       // old code: byte5 = 0b010
	if (state == DRIVE) {
		m.dashboard_inverter_enable = 1;    // old code: byte5 |= 0x01
		m.dashboard_torque = requested_throttle();  // raw value = Nm * 10
	}
 
	frucd_torque_request_pack(data, &m, sizeof(data));
 
	if (CAN_Send(hcan, FRUCD_TORQUE_REQUEST_FRAME_ID, data, FRUCD_TORQUE_REQUEST_LENGTH) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
}


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
}

// NOT MIGRATED: MC_PARAM_COMMAND is a motor controller message, not in the DBC
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
}

// NOT MIGRATED (yet): the DBC's 0x501 "Inputs" layout does not match what this sends
// (display mode bits overlap marker/overtake, debug button and knob order differ,
// DBC length is 3 but this sends 8). The dashboard/Pi side must change together
// with this, so get the team to agree on the layout first.
void can_tx_buttons_and_knobs(CAN_HandleTypeDef *hcan) {
//	uint16_t torque_limit_raw = (uint16_t)(torque_percentage / 100 * 4095);
//	uint16_t launch_control_param_raw = (uint16_t)(launch_control_param / 100 * 4095);
	uint8_t practice_mode_bit = dashboard_display_mode; // 00 = drive, 01 = debug, 10 = practice;
	uint8_t debug_mode_bit = is_button_enabled(DEBUG_BUTTON) << 3;
	uint8_t tc_button_bit = is_button_enabled(TC_BUTTON) << 2;
	uint8_t marker_button_bit = is_button_enabled(MARKER_BUTTON) << 1;
	uint8_t overtake_button_bit = is_button_enabled(OVERTAKE_BUTTON);
	uint8_t button_flags = 0 | practice_mode_bit | debug_mode_bit | tc_button_bit | marker_button_bit | overtake_button_bit;
		uint8_t data[8] = {
//			torque_limit_raw >> 8,
//			torque_limit_raw & 0xFF,
//			launch_control_param_raw >> 8,
//			launch_control_param_raw & 0xFF,
				(uint8_t)torque_percentage,
				(uint8_t)launch_control_param,
				button_flags
		};
		CAN_Send(hcan, 0x501, data, 8);
}

// raw pedal ADC values (0x502)
void can_tx_pedals_raw(CAN_HandleTypeDef *hcan) {
	struct frucd_raw_pedals_t m;
	uint8_t data[FRUCD_RAW_PEDALS_LENGTH];
 
	m.dashboard_throttle_1 = throttle1.raw;
	m.dashboard_throttle_2 = throttle2.raw;
	m.dashboard_brake_1    = brake1.raw;
	m.dashboard_brake_2    = brake2.raw;
 
	frucd_raw_pedals_pack(data, &m, sizeof(data));
 
	if (CAN_Send(hcan, FRUCD_RAW_PEDALS_FRAME_ID, data, FRUCD_RAW_PEDALS_LENGTH) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
}

// motor + accumulator power (0x504)
void can_tx_power(CAN_HandleTypeDef *hcan) {
	float acc_current_amps = mvolts_to_amps(raw_to_mvolts(acc_current_adc), raw_to_mvolts(acc_current_ref_adc));
	float acc_voltage_volt = pack_voltage * 0.018 + 180;
 
	float acc_power = acc_current_amps*acc_voltage_volt;
 
	int32_t motor_power = requested_throttle() / 10 * motor_speed * 0.10472; // 0.10472 = RADS_PER_RPM (conversion)
 
	struct frucd_mcac_power_t m;
	uint8_t data[FRUCD_MCAC_POWER_LENGTH];
 
	m.motor_power = motor_power;
	m.acc_power   = (int32_t)acc_power;
 
	frucd_mcac_power_pack(data, &m, sizeof(data));
 
	if (CAN_Send(hcan, FRUCD_MCAC_POWER_FRAME_ID, data, FRUCD_MCAC_POWER_LENGTH) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
}

