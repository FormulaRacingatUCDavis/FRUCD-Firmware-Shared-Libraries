/*
 * ADBMS6830.h
 *
 *  Created on: Aug 23, 2024
 *      Author: Abhineet
 */

#ifndef INC_ADBMS6830_H_
#define INC_ADBMS6830_H_

#include <stdint.h>
#include "stm32f7xx_hal.h"

#include "data.h"

/*
  |CH    | Dec  |Channels to convert   |
  |------|------|----------------------|
  |00000 | 0    | Everything           |
  |00001 | 1    | GPIO 1 			   |
  |00010 | 2    | GPIO 2               |
  |00011 | 3    | GPIO 3 			   |
  |00100 | 4    | GPIO 4 			   |
  |00101 | 5    | GPIO 5 			   |
  |00110 | 6    | GPIO 6  			   |
  |00111 | 7    | GPIO 7               |
  |01000 | 8    | GPIO 8               |
  |01001 | 9    | GPIO 9               |
  |01010 | 10   | GPIO 10              |
  |10000 | 16   | VREF2                |
  |10001 | 17   | VD                   |
  |10010 | 18   | VA                   |
  |10011 | 19   | ITEMP                |
  |10100 | 20   | VPV                  |
  |10101 | 21   | VMV                  |
  |10110 | 22   | VRES                 |
  |10111 | 23   | Reserved             |
*/
#define AUX_CH_ALL 0u
#define AUX_CH_GPIO1 1u
#define AUX_CH_GPIO2 2u
#define AUX_CH_GPIO3 3u
#define AUX_CH_GPIO4 4u
#define AUX_CH_GPIO5 5u
#define AUX_CH_GPIO6 6u
#define AUX_CH_GPIO7 7u
#define AUX_CH_GPIO8 8u
#define AUX_CH_GPIO9 9u
#define AUX_CH_GPIO10 10u
#define AUX_CH_VREF2 16u
#define AUX_CH_VD 17u
#define AUX_CH_VA 18u
#define AUX_CH_ITEMP 19u
#define AUX_CH_VPV 20u
#define AUX_CH_VMV 21u
#define AUX_CH_VRES 22u
#define AUX_CH_RESERVED 23u

/*
  |OW | Dec | Cell Open Wire Detection State |
  |---|-----|--------------------------------|
  |00 | 0   | Disabled                       |
  |01 | 1   | Enabled for even channels      |
  |10 | 2   | Enabled for odd channels       |
  |11 | 3   | Enabled for all channels       |
*/
#define CELL_OW_DISABLED 0u
#define CELL_OW_CH_EVEN 1u
#define CELL_OW_CH_ODD 2u
#define CELL_OW_CH_ALL 3u

typedef enum {
    GPIO1,
    GPIO2,
    GPIO3,
    GPIO4,
    GPIO5,
	GPIO6,
	GPIO7,
	GPIO8,
	GPIO9,
	GPIO10,
    VREF2,
	VD,
	VA,
	ITEMP,
	VPV,
	VMV,
	VRES,
	RESERVED
} AuxPin_t;

typedef enum {
	A, B, C, D, E, F
} RegGroup_t;

void ADBMS6830_initialize(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void ADBMS6830_set_C_ADC(uint8_t RD, uint8_t CONT, uint8_t DCP, uint8_t RSTF, uint8_t OW);
void ADBMS6830_set_S_ADC(uint8_t CONT, uint8_t DCP, uint8_t RSTF, uint8_t OW);
void ADBMS6830_set_Aux_ADC(uint8_t OW, uint8_t PUP, uint8_t CH);

void ADBMS6830_set_discharge(uint8_t ic_num, uint8_t cell_num);
void ADBMS6830_reset_discharge();

void ADBMS6830_enable_comm_bk();

uint8_t ADBMS6830_wakeup_necessary();

void ADBMS6830_wakeup(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void ADBMS6830_wrcfga(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void ADBMS6830_wrcfgb(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void ADBMS6830_wrpwma(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void ADBMS6830_wrpwmb(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void ADBMS6830_adcv(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void ADBMS6830_adsv(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);
void ADBMS6830_adax(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr);

void ADBMS6830_rdfc_all(SPI_HandleTypeDef* const hspi_ptr,
						 TIM_HandleTypeDef* const htim_ptr,
						 int16_t voltages[N_OF_ADBMS][CELLS_PER_ADBMS],
						 uint8_t spi_errors[N_OF_ADBMS]);
void ADBMS6830_rdcv_all(SPI_HandleTypeDef* const hspi_ptr,
						 TIM_HandleTypeDef* const htim_ptr,
						 int16_t voltages[N_OF_ADBMS][CELLS_PER_ADBMS],
						 uint8_t spi_errors[N_OF_ADBMS]);
void ADBMS6830_rdsv_all(SPI_HandleTypeDef* const hspi_ptr,
						 TIM_HandleTypeDef* const htim_ptr,
						 int16_t s_voltages[N_OF_ADBMS][CELLS_PER_ADBMS],
						 uint8_t spi_errors[N_OF_ADBMS]);
void ADBMS6830_rdaux_pin(SPI_HandleTypeDef* const hspi_ptr,
						 TIM_HandleTypeDef* const htim_ptr,
						 AuxPin_t pin,
						 int16_t aux[N_OF_ADBMS],
						 uint8_t spi_errors[N_OF_ADBMS]);
void ADBMS6830_rdaux_raw_temp_voltages(SPI_HandleTypeDef* const hspi_ptr,
						 TIM_HandleTypeDef* const htim_ptr,
						 int16_t raw_temps[N_OF_ADBMS][CELL_TEMPS_PER_ADBMS],
						 uint8_t spi_errors[N_OF_ADBMS]);
void ADBMS6830_rdstatc_mismatch(SPI_HandleTypeDef* const hspi_ptr,
						 TIM_HandleTypeDef* const htim_ptr,
						 uint8_t mismatches[N_OF_ADBMS][CELLS_PER_ADBMS],
						 uint8_t spi_errors[N_OF_ADBMS]);

#endif /* INC_ADBMS6830_H_ */
