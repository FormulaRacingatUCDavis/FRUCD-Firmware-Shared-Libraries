/*
 * battery_gui.c
 *
 *  Created on: Dec 6, 2024
 *      Author: Abhineet
 */


#include "battery_gui.h"
#include "cell_interface.h"
#include "charger.h"
#include "utils.h"

static const uint8_t TIMEOUT = 100;

static const uint8_t ESCAPE_CHAR = 0xAA;
static const uint8_t PACK_FRAME_START = 0xBB;
static const uint8_t FRAME_END = 0x0A;

static uint8_t buf[600] __ALIGNED(32); // same length as buffer specified in Battery GUI
static uint16_t idx = 0;

volatile uint8_t UART_TxCplt = 1;

extern volatile BAT_PACK_t bat_pack;

extern const Charge_Profile_t ESDC;
extern const Charge_Profile_t COMP;

extern Charge_Profile_t selected_profile;

// Write a byte to the buffer
static void write_byte(uint8_t byte) {
	buf[idx] = byte;
	idx++;
}

// Writes extra escape byte to buffer if byte is an escape byte
static void write_byte_with_escape(uint8_t byte) {
	if (byte == ESCAPE_CHAR) write_byte(ESCAPE_CHAR);
	write_byte(byte);
}

static void flush_buffer(UART_HandleTypeDef* const huart) {
	SCB_CleanDCache_by_Addr((uint32_t*)buf, idx + 1);
	HAL_UART_Transmit_DMA(huart, buf, idx + 1);

	UART_TxCplt = 0;
}

void uart_send_GUI_Data(UART_HandleTypeDef* const huart) {
	idx = 0;

	uint8_t i = 0;
	uint8_t j = 0;

	for(i = 0; i < N_OF_SUBPACK; i++){

		write_byte(ESCAPE_CHAR);
		write_byte(i);

		for(j = 0; j < CELLS_PER_SUBPACK; j++){
			uint16_t v = (uint16_t)bat_pack.subpacks[i].cells[j].voltage_raw;
			write_byte_with_escape(HI8(v));
			write_byte_with_escape(LO8(v));
		}
		for(j = 0; j < CELL_TEMPS_PER_SUBPACK; j++){
			uint8_t t = (uint8_t)bat_pack.subpacks[i].cell_temps[j].temp_c;
			write_byte_with_escape(t);
		}

		write_byte(ESCAPE_CHAR);
		write_byte(FRAME_END);
	}

	write_byte(ESCAPE_CHAR);
	write_byte(PACK_FRAME_START);

	uint16_t v = (uint16_t)bat_pack.pack_voltage_raw;
	write_byte_with_escape(HI8(v));
	write_byte_with_escape(LO8(v));

	v = (uint16_t)bat_pack.LO_voltage_raw;
	write_byte_with_escape(HI8(v));
	write_byte_with_escape(LO8(v));

	v = (uint16_t)bat_pack.HI_voltage_raw;
	write_byte_with_escape(HI8(v));
	write_byte_with_escape(LO8(v));

	uint8_t t = (uint8_t)bat_pack.HI_temp_c;
	write_byte_with_escape(t);

	t = (uint8_t)bat_pack.LO_temp_c;
	write_byte_with_escape(t);

	t = (uint8_t)bat_pack.AVG_temp_c;
	write_byte_with_escape(t);

	write_byte_with_escape(bat_pack.SOC_percent);

	write_byte_with_escape(bat_pack.status);

	write_byte(ESCAPE_CHAR);
	write_byte(FRAME_END);

	flush_buffer(huart);
}

// Returns if a charge profile choice was received from the GUI
uint8_t uart_receive_charge_profile(UART_HandleTypeDef* const huart) {
	static uint8_t rx_buff[1];

	if (HAL_UART_Receive(huart, rx_buff, 1, 100) == HAL_OK) {
		if (rx_buff[0] == 1) selected_profile = COMP;
		else selected_profile = ESDC;

		return 1;
	}

	return 0;
}
