/*
 * cell_interface.h
 *
 *  Created on: Aug 31, 2024
 *      Author: Abhineet
 */

#ifndef INC_CELL_INTERFACE_H_
#define INC_CELL_INTERFACE_H_

#include <stdint.h>
#include "stm32f7xx_hal.h"

#include "data.h"

// BMS Status Macros
#define NO_ERROR 0x00
#define CHARGEMODE 0x01
#define PACK_TEMP_OVER 0x02
#define PACK_TEMP_UNDER 0x04
#define CELL_VOLT_OVER 0x08
#define CELL_VOLT_UNDER 0x10
#define OPEN_WIRE 0x20
#define MISMATCH 0x40
#define SPI_FAULT 0x80

enum CellErrorType_t {
	OVERVOLT, UNDERVOLT, FUSE_BLOWN, RD_FAIL
};

enum TempErrorType_t {
	OVERTEMP, UNDERTEMP
};

typedef enum {
	BMS_FAULT, BMS_NORMAL
} BMS_MODE_t;

typedef struct {
	int16_t voltage_raw;
	float voltage;
	float filt_voltage;
	uint8_t bad_counters[4];
} BAT_CELL_t;

typedef struct {
	float temp_c;
	float filt_temp_c;
	uint8_t bad_counters[2];
} BAT_TEMP_t;

typedef struct {
	BAT_CELL_t cells[CELLS_PER_SUBPACK];
	BAT_TEMP_t cell_temps[CELL_TEMPS_PER_SUBPACK];

	float filt_subpack_voltage;
	float filt_subpack_balance;
} BAT_SUBPACK_t;

typedef struct {
	BAT_SUBPACK_t subpacks[N_OF_SUBPACK];

	int16_t pack_voltage_raw;
	float filt_pack_voltage;
	int16_t HI_voltage_raw;
	int16_t LO_voltage_raw;
	float LO_voltage;
	float filt_pack_balance;

	uint16_t current_ref_raw;
	uint16_t current_raw;
	float current;

	float HI_temp_c;
	uint8_t AVG_temp_c;
	uint8_t LO_temp_c;

	uint8_t SOC_percent;

	uint8_t status;

	uint16_t spi_fault_addresses; // Stores the SPI fault flags for all ICs in a bit string
	uint8_t spi_error_counters[N_OF_ADBMS]; // Stores the number of SPI communication errors for each IC
} BAT_PACK_t;

void cell_interface_init(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void set_voltage(uint8_t subpack_num, uint8_t cell_num, int16_t voltage_raw, int16_t filt_voltage_raw);
float get_voltage(uint8_t subpack_num, uint8_t cell_num);
float get_filtered_voltage(uint8_t subpack_num, uint8_t cell_num);
int16_t get_voltage_raw(uint8_t subpack_num, uint8_t cell_num);

void set_cell_temp(uint8_t subpack_num, uint8_t temp_num, int16_t temp_raw);
float get_cell_temp(uint8_t subpack_num, uint8_t temp_num);
float get_filtered_cell_temp(uint8_t subpack_num, uint8_t temp_num);

void update_voltages(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void update_temps(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void cell_disconnect_check(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void process_voltages();
void process_temps();

void disable_cell_balancing(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void balance_cells(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void update_max_fault_data();
BMS_MODE_t bat_health_check();

#endif /* INC_CELL_INTERFACE_H_ */
