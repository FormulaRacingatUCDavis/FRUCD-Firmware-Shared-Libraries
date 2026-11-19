/*
 * display.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Abhineet
 */


#include <stdint.h>
#include <stdio.h>

#include "display.h"
#include "LCD.h"
#include "fsm.h"
#include "cell_interface.h"

extern volatile BAT_PACK_t bat_pack;
extern uint8_t pei_status;
extern uint8_t pei_state;
extern volatile uint8_t vcu_attached;
extern volatile uint8_t charger_attached;

void update_display(TIM_HandleTypeDef* const htim_ptr) {
	static uint8_t spi_fault = 0;
	char str[20];

	LCD_ReturnHome(htim_ptr);

	if (bat_pack.status & (PACK_TEMP_OVER | PACK_TEMP_UNDER)) {
		LCD_PrintString(htim_ptr, "BMS TEMP        ");
	}
	else if (bat_pack.status & SPI_FAULT) {
		spi_fault = 1;

		LCD_PrintString(htim_ptr, "SPI FAULT       ");
		LCD_Position(htim_ptr, 1, 0);

		// Print out ADBMS6830 SPI error addresses
		uint16_t mask = 0;
		char bit_str[6];
		for (uint8_t i = 9; i >= 1; i--) {
			mask = 0x01 << i;
			sprintf(bit_str, "%u", (bat_pack.spi_fault_addresses & mask) >> i);
			LCD_PrintString(htim_ptr, bit_str);
		}

		sprintf(bit_str, "%u", bat_pack.spi_fault_addresses & 0x01);
		LCD_PrintString(htim_ptr, bit_str);

		// Print out spi error counter for whichever ADBMS6830 is faulting
//		char counter_str[8];
//		sprintf(counter_str, "%u", bat_pack.spi_error_counters[0]);
//		LCD_PrintString(counter_str, htim_ptr);

		// Clear rest of the bottom row of the display
		LCD_PrintString(htim_ptr, "       ");
	}
	else if (bat_pack.status & CELL_VOLT_OVER) {
		LCD_PrintString(htim_ptr, "OVERVOLT        ");
	}
	else if (bat_pack.status & CELL_VOLT_UNDER) {
		LCD_PrintString(htim_ptr, "UNDERVOLT       ");
	}
	else if (bat_pack.status & OPEN_WIRE) {
		LCD_PrintString(htim_ptr, "OPEN WIRE       ");
	}
	else if (bat_pack.status & MISMATCH) {
		LCD_PrintString(htim_ptr, "MISMATCH        ");
	}
	else if (pei_status & MC_FAULT) {
		LCD_PrintString(htim_ptr, "MC FAULT        ");
	}
	else if (pei_status & MC_TIMEOUT) {
		LCD_PrintString(htim_ptr, "MC TIMEOUT      ");
	}
	else if (pei_status & VCU_FAULT) {
		LCD_PrintString(htim_ptr, "VCU FAULT       ");
	}
	else if (pei_status & VCU_TIMEOUT) {
		LCD_PrintString(htim_ptr, "VCU TIMEOUT     ");
	}
	else if (pei_status & CHARGER_FAULT) {
		LCD_PrintString(htim_ptr, "CHARGER FAULT   ");
	}
	else if (pei_status & CHARGER_TIMEOUT) {
		LCD_PrintString(htim_ptr, "CHARGER TIMEOUT ");
	}
	else if (!vcu_attached && !charger_attached) {
		LCD_PrintString(htim_ptr, "I am so alone :(");
	}
	else if (pei_status & MC_DISCHARGING) {
		LCD_PrintString(htim_ptr, "MC DISCHARGE    ");
	}
	else if (pei_status & SHUTDOWN) {
		LCD_PrintString(htim_ptr, "SHUTDOWN OPEN   ");
	}
	else if (pei_state == PEI_LV) {
		LCD_PrintString(htim_ptr, "LV              ");
	}
	else if (pei_state == PEI_PRECHARGE) {
		LCD_PrintString(htim_ptr, "PRECHARGE       ");
	}
	else if (pei_state == PEI_HV) {
		LCD_PrintString(htim_ptr, "HV              ");
	}
	else {
		LCD_PrintString(htim_ptr, "YO WTF?         ");
	}

	if (!spi_fault) {
		LCD_Position(htim_ptr, 1, 0);
		sprintf(str, "%u%% ", bat_pack.SOC_percent);
		LCD_PrintString(htim_ptr, str);

		LCD_Position(htim_ptr, 1, 5);
		sprintf(str, "%uC", (uint8_t)bat_pack.HI_temp_c);
		LCD_PrintString(htim_ptr, str);

		LCD_Position(htim_ptr, 1, 10);
		sprintf(str, "%dA   ", (int8_t)bat_pack.current);
		LCD_PrintString(htim_ptr, str);
	}
}
