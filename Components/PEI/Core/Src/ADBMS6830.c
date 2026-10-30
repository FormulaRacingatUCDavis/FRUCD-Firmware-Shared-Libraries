/*
 * ADBMS6830.c
 *
 *  Created on: Aug 24, 2024
 *      Author: Abhineet
 */


#include "ADBMS6830.h"
#include "delays.h"
#include "utils.h"
#include "main.h"

static const uint8_t WAKE_UP_DELAY_US = 10 * N_OF_ADBMS; // number of microseconds to wait after sending wake up signal

// Default values for CFGA registers
static const uint8_t CFGA0 = 0x82; // REFON enabled, C-ADC vs S-ADC comparison voltage threshold set to 9 mV
static const uint8_t CFGA1 = 0x00;
static const uint8_t CFGA2 = 0x80; // Soak time enabled for Aux GPIO
static const uint8_t CFGA3 = 0xFF; // Pull-down resistor disabled for Aux GPIO 1-8
static const uint8_t CFGA4 = 0x03; // Pull-down resistor disabled for Aux GPIO 9-10
static const uint8_t CFGA5 = 0x06; // Set cell voltage ADC IIR filter corner frequency to 1.25 Hz

// Default values for CFGB registers
static const uint8_t CFGB0 = 0x00;
static const uint8_t CFGB1 = 0x00;
static const uint8_t CFGB2 = 0x00;
static const uint8_t CFGB3 = 0x01; // Discharge timer set to 1 minute
static const uint8_t CFGB4 = 0x00;
static const uint8_t CFGB5 = 0x00;

// Default values for PWMA registers
static const uint8_t PWMA0 = 0x00;
static const uint8_t PWMA1 = 0x00;
static const uint8_t PWMA2 = 0x00;
static const uint8_t PWMA3 = 0x00;
static const uint8_t PWMA4 = 0x00;
static const uint8_t PWMA5 = 0x00;

// Default values for PWMB registers (see memory map for reference)
static const uint8_t PWMB0 = 0x00;
static const uint8_t PWMB1 = 0x00;
static const uint8_t PWMB2 = 0xFF;
static const uint8_t PWMB3 = 0xFF;
static const uint8_t PWMB4 = 0xFF;
static const uint8_t PWMB5 = 0xFF;

static uint8_t ADCV[2]; // Cell voltage conversion command
static uint8_t ADSV[2]; // Redundant cell voltage conversion command
static uint8_t ADAX[2]; // Auxiliary ADC conversion command

static uint8_t tx_cfga[N_OF_ADBMS][6]; // Stores CFGA data to be written to each IC
static uint8_t tx_cfgb[N_OF_ADBMS][6]; // Stores CFGB data to be written to each IC
static uint8_t tx_pwma[N_OF_ADBMS][6]; // Stores PWMA data to be written to each IC
static uint8_t tx_pwmb[N_OF_ADBMS][6]; // Stores PWMB data to be written to each IC

static uint8_t cmd_counter[N_OF_ADBMS];

extern int8_t comm_bk_id;

/*
 \brief Calculates and returns the CRC10 of a given register group that is being read

 @param[in] uint8_t data[]: the array of data that the PEC will be generated from (6 bytes)
 @param[in] uint8_t cmd_count: the number of commands that increment the counter
 	 	 	 	 	 	 	   that have been sent to the chip

 @returns The calculated pec10 as an unsigned int16_t
*/
static uint16_t data_pec_calc(uint8_t data[], uint8_t cmd_count) {

	static const uint16_t crc10Table[256] = {0x0, 0x8f, 0x11e, 0x191, 0x23c, 0x2b3, 0x322, 0x3ad, 0xf7, 0x78, 0x1e9, 0x166,       // pre-computed CRC10 table
	0x2cb, 0x244, 0x3d5, 0x35a, 0x1ee, 0x161, 0xf0, 0x7f, 0x3d2, 0x35d, 0x2cc, 0x243, 0x119,
	0x196, 0x7, 0x88, 0x325, 0x3aa, 0x23b, 0x2b4, 0x3dc, 0x353, 0x2c2, 0x24d, 0x1e0, 0x16f,
	0xfe, 0x71, 0x32b, 0x3a4, 0x235, 0x2ba, 0x117, 0x198, 0x9, 0x86, 0x232, 0x2bd, 0x32c,
	0x3a3, 0xe, 0x81, 0x110, 0x19f, 0x2c5, 0x24a, 0x3db, 0x354, 0xf9, 0x76, 0x1e7, 0x168,
	0x337, 0x3b8, 0x229, 0x2a6, 0x10b, 0x184, 0x15, 0x9a, 0x3c0, 0x34f, 0x2de, 0x251, 0x1fc,
	0x173, 0xe2, 0x6d, 0x2d9, 0x256, 0x3c7, 0x348, 0xe5, 0x6a, 0x1fb, 0x174, 0x22e, 0x2a1,
	0x330, 0x3bf, 0x12, 0x9d, 0x10c, 0x183, 0xeb, 0x64, 0x1f5, 0x17a, 0x2d7, 0x258, 0x3c9,
	0x346, 0x1c, 0x93, 0x102, 0x18d, 0x220, 0x2af, 0x33e, 0x3b1, 0x105, 0x18a, 0x1b, 0x94,
	0x339, 0x3b6, 0x227, 0x2a8, 0x1f2, 0x17d, 0xec, 0x63, 0x3ce, 0x341, 0x2d0, 0x25f, 0x2e1,
	0x26e, 0x3ff, 0x370, 0xdd, 0x52, 0x1c3, 0x14c, 0x216, 0x299, 0x308, 0x387, 0x2a, 0xa5,
	0x134, 0x1bb, 0x30f, 0x380, 0x211, 0x29e, 0x133, 0x1bc, 0x2d, 0xa2, 0x3f8, 0x377, 0x2e6,
	0x269, 0x1c4, 0x14b, 0xda, 0x55, 0x13d, 0x1b2, 0x23, 0xac, 0x301, 0x38e, 0x21f, 0x290,
	0x1ca, 0x145, 0xd4, 0x5b, 0x3f6, 0x379, 0x2e8, 0x267, 0xd3, 0x5c, 0x1cd, 0x142, 0x2ef,
	0x260, 0x3f1, 0x37e, 0x24, 0xab, 0x13a, 0x1b5, 0x218, 0x297, 0x306, 0x389, 0x1d6, 0x159,
	0xc8, 0x47, 0x3ea, 0x365, 0x2f4, 0x27b, 0x121, 0x1ae, 0x3f, 0xb0, 0x31d, 0x392, 0x203,
	0x28c, 0x38, 0xb7, 0x126, 0x1a9, 0x204, 0x28b, 0x31a, 0x395, 0xcf, 0x40, 0x1d1, 0x15e,
	0x2f3, 0x27c, 0x3ed, 0x362, 0x20a, 0x285, 0x314, 0x39b, 0x36, 0xb9, 0x128, 0x1a7, 0x2fd,
	0x272, 0x3e3, 0x36c, 0xc1, 0x4e, 0x1df, 0x150, 0x3e4, 0x36b, 0x2fa, 0x275, 0x1d8, 0x157,
	0xc6, 0x49, 0x313, 0x39c, 0x20d, 0x282, 0x12f, 0x1a0, 0x31, 0xbe};

	uint16_t remainder = 16; // initialize the PEC
	uint16_t poly = 0x48F;
	uint8_t addr = 0;

	for (uint8_t i = 0; i < 6; i++) { // loops for each byte in data array (there are 6 bytes in a register group)
		addr = (uint8_t)(((remainder >> 2) ^ data[i]) & 0xFF); // calculate PEC table address
		remainder = (remainder << 8) ^ crc10Table[addr];
	}
	remainder ^= ((cmd_count << 2) & 0xFC) << 2;

	for (uint8_t i = 0; i < 6; i++) {
		if (remainder & 0x200) {
			remainder = remainder << 1;
			remainder = remainder ^ poly;
		}
		else {
			remainder = remainder << 1;
		}
	}

	return remainder & 0x3FF;
}

/*
 \brief Calculates and returns the CRC15 of a given command

 @param[in] uint8_t cmd[2]: the command array that the PEC will be generated from (2 bytes)

 @returns The calculated pec15 as an unsigned int16_t
*/
static uint16_t cmd_pec_calc(uint8_t cmd[2]) {

	static const uint16_t crc15Table[256] = {0x0, 0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  // pre-computed CRC15 table
	0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
	0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
	0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
	0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
	0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
	0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
	0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
	0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
	0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
	0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
	0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
	0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
	0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
	0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
	0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
	0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
	0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
	0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
	0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
	0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
	0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
	0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095};

	uint16_t remainder = 16; // initialize the PEC
	uint8_t addr = 0;

	for (uint8_t i = 0; i < 2; i++) { // loops for each byte in command array
		addr = (uint8_t)(((remainder >> 7) ^ cmd[i]) & 0xFF); // calculate PEC table address
		remainder = (remainder << 8) ^ crc15Table[addr];
	}
	return remainder * 2; // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

// SPI write for when we are only using the transceiver in the forward direction
static void single_spi_write(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
							 TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
							 uint8_t cmd[2],                    // Array of command bytes to be written on the SPI port
							 uint8_t data[N_OF_ADBMS][6]        // 2D array storing data bytes for each IC (can be set to NULL if not a write command)
							 ) {
	uint8_t tx_len = 4; // 2 command bytes + 2 PEC bytes
	if (data != NULL) tx_len += N_OF_ADBMS * 8; // For write commands, there will always be 8 bytes per IC (6 register bytes + 2 PEC bytes)
	uint8_t tx_data[tx_len];

	tx_data[0] = cmd[0];
	tx_data[1] = cmd[1];

	uint16_t cmd_pec = cmd_pec_calc(cmd);
	tx_data[2] = HI8(cmd_pec);
	tx_data[3] = LO8(cmd_pec);

	if (data != NULL) {
		uint8_t idx = 4;

		// Data written to last IC in the chain first and first IC in the chain last (see bus protocols in datasheet)
		for (int8_t ic = N_OF_ADBMS - 1; ic >= 0; ic--) {
			uint16_t data_pec;

			for (uint8_t i = 0; i < 6; i++) {
				tx_data[idx] = data[ic][i];
				idx++;
			}

			data_pec = data_pec_calc(data[ic], 0);
			tx_data[idx] = HI8(data_pec);
			tx_data[idx + 1] = LO8(data_pec);
			idx += 2;
		}
	}

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_RESET);
	delay_500_ns(htim_ptr);

	HAL_SPI_Transmit(hspi_ptr, tx_data, tx_len, 100);
	delay_500_ns(htim_ptr);

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_SET);
	delay_us(htim_ptr, 2);
}

// SPI write for when we are using transceivers in the forward and reverse directions
static void dual_spi_write(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
						   TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
						   uint8_t cmd[2],                    // Array of command bytes to be written on the SPI port
						   uint8_t data[N_OF_ADBMS][6]        // 2D array storing data bytes for each IC (can be set to NULL if not a write command)
						   ) {
	uint8_t tx_len_fwd = 4; // 2 command bytes + 2 PEC bytes
	uint8_t tx_len_rev = 4;

	if (data != NULL) {
		tx_len_fwd += (comm_bk_id + 1) * 8; // For write commands, there will always be 8 bytes per IC (6 register bytes + 2 PEC bytes)
		tx_len_rev += (N_OF_ADBMS - comm_bk_id - 1) * 8;
	}
	uint8_t tx_data_fwd[tx_len_fwd];
	uint8_t tx_data_rev[tx_len_rev];

	tx_data_fwd[0] = cmd[0];
	tx_data_fwd[1] = cmd[1];

	tx_data_rev[0] = cmd[0];
	tx_data_rev[1] = cmd[1];

	uint16_t cmd_pec = cmd_pec_calc(cmd);

	tx_data_fwd[2] = HI8(cmd_pec);
	tx_data_fwd[3] = LO8(cmd_pec);

	tx_data_rev[2] = HI8(cmd_pec);
	tx_data_rev[3] = LO8(cmd_pec);

	if (data != NULL) {
		uint16_t data_pec;
		uint8_t idx = 4;

		// Data packaging in the forward direction
		for (int8_t ic = comm_bk_id; ic >= 0; ic--) {
			for (uint8_t i = 0; i < 6; i++) {
				tx_data_fwd[idx] = data[ic][i];
				idx++;
			}

			data_pec = data_pec_calc(data[ic], 0);
			tx_data_fwd[idx] = HI8(data_pec);
			tx_data_fwd[idx + 1] = LO8(data_pec);
			idx += 2;
		}

		idx = 4; // reset index counter

		// Data packaging in the reverse direction
		for (uint8_t ic = comm_bk_id + 1; ic < N_OF_ADBMS; ic++) {
			for (uint8_t i = 0; i < 6; i++) {
				tx_data_rev[idx] = data[ic][i];
				idx++;
			}

			data_pec = data_pec_calc(data[ic], 0);
			tx_data_rev[idx] = HI8(data_pec);
			tx_data_rev[idx + 1] = LO8(data_pec);
			idx += 2;
		}
	}

	HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
	delay_500_ns(htim_ptr);

	HAL_SPI_Transmit(hspi_ptr, tx_data_fwd, tx_len_fwd, 100);
	delay_500_ns(htim_ptr);

	HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_RESET);
	delay_500_ns(htim_ptr);

	HAL_SPI_Transmit(hspi_ptr, tx_data_rev, tx_len_rev, 100);
	delay_500_ns(htim_ptr);

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_SET);
	delay_us(htim_ptr, 1);
}

/*
 \brief Writes an array of bytes out of the SPI port

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle
 @param[in] uint8_t cmd[2] the command array to be written on the SPI port (2 bytes)
 @param[in] uint8_t* data[N_OF_ADBMS][6] the 2D array storing the data arrays to be written on the SPI port
*/
static void spi_write(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
				      TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
					  uint8_t cmd[2],                    // Array of command bytes to be written on the SPI port
					  uint8_t data[N_OF_ADBMS][6]        // 2D array storing data bytes for each IC (can be set to NULL if not a write command)
				      )
{
	if (comm_bk_id == -1) {
		single_spi_write(hspi_ptr, htim_ptr, cmd, data);
	}
	else {
		dual_spi_write(hspi_ptr, htim_ptr, cmd, data);
	}

	for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
		cmd_counter[ic]++;
		if (cmd_counter[ic] > 63) cmd_counter[ic] = 1;
	}
}

// SPI read for when we are only using the transceiver in the forward direction
static uint8_t single_spi_write_read(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
									 TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
									 uint8_t cmd[2],                    // Array of command bytes to be written on SPI port
									 uint8_t rx_data[N_OF_ADBMS][6],    // Input: 2D array that will store the data read by the SPI port
									 uint8_t pec_mismatches[N_OF_ADBMS] // Input: Array containing flags indicating which nodes had PEC mismatches
									 ) {
	uint8_t tx_data[4];
	uint8_t no_errors = 1;

	tx_data[0] = cmd[0];
	tx_data[1] = cmd[1];

	uint16_t cmd_pec = cmd_pec_calc(cmd);
	tx_data[2] = HI8(cmd_pec);
	tx_data[3] = LO8(cmd_pec);

	uint8_t rx_len = N_OF_ADBMS * 8; // 6 register bytes + 2 PEC bytes = 8 bytes per IC
	uint8_t rx_data_flattened[rx_len]; // array to store data from all ICs

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_RESET);
	delay_500_ns(htim_ptr);

	HAL_SPI_Transmit(hspi_ptr, tx_data, 4, 100);
	HAL_SPI_Receive(hspi_ptr, rx_data_flattened, rx_len, 100);
	delay_500_ns(htim_ptr);

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_SET);
	delay_us(htim_ptr, 2);

	// Package data (excluding PEC) into 2D array and process PECs
	// Data read from first IC in the chain first and last IC in the chain last (see bus protocols in datasheet)
	for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
		for (uint8_t i = 0; i < 6; i++) {
			rx_data[ic][i] = rx_data_flattened[(ic * 8) + i];
		}

		uint16_t data_pec = data_pec_calc(rx_data[ic], cmd_counter[ic]);
		uint16_t received_pec = (rx_data_flattened[(ic * 8) + 6] & 0x03) << 8;
		received_pec += rx_data_flattened[(ic * 8) + 7];
		uint8_t received_cmd_count = (rx_data_flattened[(ic * 8) + 6] & 0xFC) >> 2;

		if (data_pec != received_pec) {
			no_errors = 0;
			pec_mismatches[ic] = 1;
		}
		else {
			pec_mismatches[ic] = 0;
		}

		cmd_counter[ic] = received_cmd_count;
	}

	return no_errors;
}

// SPI read for when we are using transceivers in the forward and reverse directions
static uint8_t dual_spi_write_read(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
								   TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
								   uint8_t cmd[2],                    // Array of command bytes to be written on SPI port
								   uint8_t rx_data[N_OF_ADBMS][6],    // Input: 2D array that will store the data read by the SPI port
								   uint8_t pec_mismatches[N_OF_ADBMS] // Input: Array containing flags indicating which nodes had PEC mismatches
								   ) {
	uint8_t tx_data[4];
	uint8_t no_errors = 1;

	tx_data[0] = cmd[0];
	tx_data[1] = cmd[1];

	uint16_t cmd_pec = cmd_pec_calc(cmd);
	tx_data[2] = HI8(cmd_pec);
	tx_data[3] = LO8(cmd_pec);

	uint8_t rx_len_fwd = (comm_bk_id + 1) * 8; // 6 register bytes + 2 PEC bytes = 8 bytes per IC
	uint8_t rx_len_rev = (N_OF_ADBMS - comm_bk_id - 1) * 8;

	// Arrays to story data from all ICs
	uint8_t rx_data_flattened_fwd[rx_len_fwd];
	uint8_t rx_data_flattened_rev[rx_len_rev];

	HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET);
	delay_500_ns(htim_ptr);

	HAL_SPI_Transmit(hspi_ptr, tx_data, 4, 100);
	HAL_SPI_Receive(hspi_ptr, rx_data_flattened_fwd, rx_len_fwd, 100);
	delay_500_ns(htim_ptr);

	HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_RESET);
	delay_500_ns(htim_ptr);

	HAL_SPI_Transmit(hspi_ptr, tx_data, 4, 100);
	HAL_SPI_Receive(hspi_ptr, rx_data_flattened_rev, rx_len_rev, 100);
	delay_500_ns(htim_ptr);

	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_SET);
	delay_us(htim_ptr, 1);

	// Package data (excluding PEC) into 2D array and process PECs

	for (uint8_t ic = 0; ic < comm_bk_id; ic++) {
		for (uint8_t i = 0; i < 6; i++) {
			rx_data[ic][i] = rx_data_flattened_fwd[(ic * 8) + i];
		}

		uint16_t data_pec = data_pec_calc(rx_data[ic], cmd_counter[ic]);
		uint16_t received_pec = rx_data_flattened_fwd[(ic * 8) + 6] << 8;
		received_pec += rx_data_flattened_fwd[(ic * 8) + 7];
		uint8_t received_cmd_count = (rx_data_flattened_fwd[(ic * 8) + 6] & 0xFC) >> 2;

		if (data_pec != received_pec) {
			no_errors = 0;
			pec_mismatches[ic] = 1;
		}
		else {
			pec_mismatches[ic] = 0;
		}

		cmd_counter[ic] = received_cmd_count;
	}

	uint8_t packet = 0;
	for (int8_t ic = N_OF_ADBMS - 1; ic > comm_bk_id; ic--, packet++) {
		for (uint8_t i = 0; i < 6; i++) {
			rx_data[ic][i] = rx_data_flattened_rev[(packet * 8) + i];
		}

		uint16_t data_pec = data_pec_calc(rx_data[ic], cmd_counter[ic]);
		uint16_t received_pec = rx_data_flattened_rev[(packet * 8) + 6] << 8;
		received_pec += rx_data_flattened_rev[(packet * 8) + 7];
		uint8_t received_cmd_count = (rx_data_flattened_rev[(ic * 8) + 6] & 0xFC) >> 2;

		if (data_pec != received_pec) {
			no_errors = 0;
			pec_mismatches[ic] = 1;
		}
		else {
			pec_mismatches[ic] = 0;
		}

		cmd_counter[ic] = received_cmd_count;
	}

	return no_errors;
}


/*
 \brief Writes and reads a set number of bytes using the SPI port.

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle
 @param[in] uint8_t cmd[2] the command array to be written on the SPI port (2 bytes)

 @param[out] uint8_t rx_data[N_OF_ADBMS][6] 2D array that read data will be written to (6 bytes per IC)
 @param[out] uint8_t pec_mismatches Array containing flags indicating which nodes had PEC mismatches

 @returns Whether the transaction occurred without any PEC mismatches
*/
static uint8_t spi_write_read(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
					   	   	  TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
							  uint8_t cmd[2],                    // Array of command bytes to be written on SPI port
							  uint8_t rx_data[N_OF_ADBMS][6],    // Input: 2D array that will store the data read by the SPI port
							  uint8_t pec_mismatches[N_OF_ADBMS] // Input: Array containing flags indicating which nodes had PEC mismatches
					   	   	  )
{
	if (comm_bk_id == -1) {
		return single_spi_write_read(hspi_ptr, htim_ptr, cmd, rx_data, pec_mismatches);
	}
	else {
		return dual_spi_write_read(hspi_ptr, htim_ptr, cmd, rx_data, pec_mismatches);
	}
}

// Set default configs
void ADBMS6830_initialize(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
		tx_cfga[ic][0] = CFGA0;
		tx_cfga[ic][1] = CFGA1;
		tx_cfga[ic][2] = CFGA2;
		tx_cfga[ic][3] = CFGA3;
		tx_cfga[ic][4] = CFGA4;
		tx_cfga[ic][5] = CFGA5;

		tx_cfgb[ic][0] = CFGB0;
		tx_cfgb[ic][1] = CFGB1;
		tx_cfgb[ic][2] = CFGB2;
		tx_cfgb[ic][3] = CFGB3;
		tx_cfgb[ic][4] = CFGB4;
		tx_cfgb[ic][5] = CFGB5;

		tx_pwma[ic][0] = PWMA0;
		tx_pwma[ic][1] = PWMA1;
		tx_pwma[ic][2] = PWMA2;
		tx_pwma[ic][3] = PWMA3;
		tx_pwma[ic][4] = PWMA4;
		tx_pwma[ic][5] = PWMA5;

		tx_pwmb[ic][0] = PWMB0;
		tx_pwmb[ic][1] = PWMB1;
		tx_pwmb[ic][2] = PWMB2;
		tx_pwmb[ic][3] = PWMB3;
		tx_pwmb[ic][4] = PWMB4;
		tx_pwmb[ic][5] = PWMB5;

		cmd_counter[ic] = 0;
	}

	ADBMS6830_set_Aux_ADC(0, 0, AUX_CH_ALL);

	ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	HAL_Delay(5); // Waking up from sleep state requires more time

	ADBMS6830_wrcfga(hspi_ptr, htim_ptr);
	ADBMS6830_wrcfgb(hspi_ptr, htim_ptr);

	// See Cell Discharge With Cell Measurements and Cell Diagnostics section of datasheet
	ADBMS6830_set_C_ADC(1, 1, 0, 0, CELL_OW_DISABLED); // RD = 1, DCP = 0, CONT = 1
	ADBMS6830_adcv(hspi_ptr, htim_ptr);
}

/*
 \brief Sets the configuration for the cell ADCs

 @param[in] uint8_t RD Whether or not to run the conversion with redundancy
 @param[in] uint8_t CONT Whether or not the ADC should run continuously
 @param[in] uint8_t DCP Whether or not PWM discharge is permitted during ADC conversion
 @param[in] uint8_t RSTF Whether or not to reset the digital filter
 @param[in] uint8_t OW Determines open wire detection mode
*/
void ADBMS6830_set_C_ADC(uint8_t RD, uint8_t CONT, uint8_t DCP, uint8_t RSTF, uint8_t OW) {
	ADCV[0] = 0x02;
	ADCV[0] += RD;

	ADCV[1] = 0x60;
	ADCV[1] += CONT << 7;
	ADCV[1] += DCP << 4;
	ADCV[1] += RSTF << 2;
	ADCV[1] += OW;
}

/*
 \brief Sets the configuration for the redundant cell ADCs

 @param[in] uint8_t CONT Whether or not the ADC should run continuously
 @param[in] uint8_t DCP Whether or not PWM discharge is permitted during ADC conversion
 @param[in] uint8_t RSTF Whether or not to reset the IIR filter
 @param[in] uint8_t OW Determines open wire detection mode
*/
void ADBMS6830_set_S_ADC(uint8_t CONT, uint8_t DCP, uint8_t RSTF, uint8_t OW) {
	ADSV[0] = 0x01;

	ADSV[1] = 0x68;
	ADSV[1] += CONT << 7;
	ADSV[1] += DCP << 4;
	ADSV[1] += OW;
}

/*
 \brief Sets the configuration for the redundant cell ADCs

 @param[in] uint8_t OW Determines whether conversion runs with open wire detection or not
 @param[in] uint8_t PUP Determines whether pull-up or pull-down current sources are applied during conversion
 @param[in] uint8_t CH Determines which channels are converted
*/
void ADBMS6830_set_Aux_ADC(uint8_t OW, uint8_t PUP, uint8_t CH) {
	ADAX[0] = 0x04;
	ADAX[0] += OW;

	ADAX[1] = 0x10;
	ADAX[1] += PUP << 7;
	ADAX[1] += (CH >> 4) << 6;
	ADAX[1] += CH & 0x0F;
}

void ADBMS6830_set_discharge(uint8_t ic_num, uint8_t cell_num) {
	uint8_t reg_idx = 0;
	uint8_t config_val = 0b1001; // Set to 60% duty cycle
	cell_num++; // Cell numbering in PWM register group is 1-based
	if ((cell_num % 2) == 0) config_val <<= 4;

	if (cell_num < 13) {
		reg_idx = (cell_num - 1) / 2;

		if ((cell_num % 2) == 0) config_val += LO4(tx_pwma[ic_num][reg_idx]);
		else config_val += HI4(tx_pwma[ic_num][reg_idx]) << 4;

		tx_pwma[ic_num][reg_idx] = config_val;
	}
	else {
		reg_idx = (cell_num - 13) / 2;

		if ((cell_num % 2) == 0) config_val += LO4(tx_pwmb[ic_num][reg_idx]);
		else config_val += HI4(tx_pwmb[ic_num][reg_idx]) << 4;

		tx_pwmb[ic_num][reg_idx] = config_val;
	}
}

// Resets PWM registers to default values
void ADBMS6830_reset_discharge() {
	for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
		tx_pwma[ic][0] = PWMA0;
		tx_pwma[ic][1] = PWMA1;
		tx_pwma[ic][2] = PWMA2;
		tx_pwma[ic][3] = PWMA3;
		tx_pwma[ic][4] = PWMA4;
		tx_pwma[ic][5] = PWMA5;

		tx_pwmb[ic][0] = PWMB0;
		tx_pwmb[ic][1] = PWMB1;
		tx_pwmb[ic][2] = PWMB2;
		tx_pwmb[ic][3] = PWMB3;
		tx_pwmb[ic][4] = PWMB4;
		tx_pwmb[ic][5] = PWMB5;
	}
}

void ADBMS6830_enable_comm_bk() {
	tx_cfga[comm_bk_id][5] |= 0x08;
	tx_cfga[comm_bk_id + 1][5] |= 0x08;
}

uint8_t ADBMS6830_wakeup_necessary() {
	if (comm_bk_id == -1) {
		return !HAL_GPIO_ReadPin(Wake_6822_GPIO_Port, Wake_6822_Pin);
	}
	else {
		return !HAL_GPIO_ReadPin(Wake_6822_GPIO_Port, Wake_6822_Pin) ||
			   !HAL_GPIO_ReadPin(Wake2_6822_GPIO_Port, Wake2_6822_Pin);
	}
}

void ADBMS6830_wakeup(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	// Send dummy RCFGA command to wake up ICs

	uint8_t cmd[2] = {0x00, 0x02};
	uint8_t rx_data[N_OF_ADBMS][6];
	uint8_t pec_mismatches[N_OF_ADBMS];

	spi_write_read(hspi_ptr, htim_ptr, cmd, rx_data, pec_mismatches);
	delay_us(htim_ptr, WAKE_UP_DELAY_US);
}

// Write to configuration register group A
void ADBMS6830_wrcfga(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	uint8_t cmd[2] = {0x00, 0x01};
	spi_write(hspi_ptr, htim_ptr, cmd, tx_cfga);
}

// Write to configuration register group B
void ADBMS6830_wrcfgb(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	uint8_t cmd[2] = {0x00, 0x04};
	spi_write(hspi_ptr, htim_ptr, cmd, tx_cfgb);
}

// Write to PWM register group A
void ADBMS6830_wrpwma(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	uint8_t cmd[2] = {0x00, 0x20};
	spi_write(hspi_ptr, htim_ptr, cmd, tx_pwma);
}

// Write to PWM register group B
void ADBMS6830_wrpwmb(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	uint8_t cmd[2] = {0x00, 0x21};
	spi_write(hspi_ptr, htim_ptr, cmd, tx_pwmb);
}

// Start cell voltage ADC conversions
void ADBMS6830_adcv(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	spi_write(hspi_ptr, htim_ptr, ADCV, NULL);
	HAL_Delay(5);
}

// Start redundant cell voltage ADC conversions
void ADBMS6830_adsv(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	spi_write(hspi_ptr, htim_ptr, ADSV, NULL);
	HAL_Delay(8);
}

// Start auxiliary ADC conversions
void ADBMS6830_adax(SPI_HandleTypeDef* const hspi_ptr, TIM_HandleTypeDef* const htim_ptr) {
	spi_write(hspi_ptr, htim_ptr, ADAX, NULL);
	HAL_Delay(5);
}

// Freeze result registers
static void ADBMS6830_freeze_results(SPI_HandleTypeDef* const hspi_ptr,
									 TIM_HandleTypeDef* const htim_ptr
									 )
{
	uint8_t cmd[2] = {0x00, 0x2D};
	spi_write(hspi_ptr, htim_ptr, cmd, NULL);
}

// Allow result registers to be updated again
static void ADBMS6830_unfreeze_results(SPI_HandleTypeDef* const hspi_ptr,
									   TIM_HandleTypeDef* const htim_ptr
									   )
{
	uint8_t cmd[2] = {0x00, 0x2F};
	spi_write(hspi_ptr, htim_ptr, cmd, NULL);
}

/*
 \brief Reads back filtered cell voltages from one register group for each IC

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle
 @param[in] RegGroup_t reg filtered cell voltage register group to read from

 @param[out] uint8_t data[N_OF_ADBMS][6] 2D array containing the data read back (6 bytes per register)
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
static void ADBMS6830_rdfc_reg(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
							   TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
							   RegGroup_t reg_num,                // Option: filtered cell voltage register group to read from
							   uint8_t data[N_OF_ADBMS][6],       // Input: 2D array containing the data read back
							   uint8_t spi_errors[N_OF_ADBMS]     // Input: Array containing flags indicating which nodes had SPI errors
							   )
{
	uint8_t cmd[2];
	cmd[0] = 0x00;
	cmd[1] = 0x12 + reg_num;

	uint8_t num_tries = 0;
	uint8_t try_again = 0;

	do {
		try_again = !spi_write_read(hspi_ptr, htim_ptr, cmd, data, spi_errors);

		num_tries++;
		if ((num_tries > 2) && try_again) return;
	} while (try_again);
}

/*
 \brief Reads back filtered cell voltages from all register groups from all ICs

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle

 @param[out] int16_t voltages[N_OF_ADBMS][CELLS_PER_ADBMS] 2D array containing the voltages
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
void ADBMS6830_rdfc_all(SPI_HandleTypeDef* const hspi_ptr,             // Pointer to the SPI handle
						TIM_HandleTypeDef* const htim_ptr,             // Pointer to a timer handle
						int16_t filt_voltages[N_OF_ADBMS][CELLS_PER_ADBMS], // Input: 2D array containing voltages
						uint8_t spi_errors[N_OF_ADBMS]                 // Input: Array containing flags indicating which nodes had SPI errors
						)
{
	const uint8_t CELL_IN_REG = 3u; // 6 bytes per register / 2 bytes per cell = 3 cell voltages per register

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	// Freeze all result registers for data coherence
	ADBMS6830_freeze_results(hspi_ptr, htim_ptr);

	for (uint8_t reg = 0; reg < 4; reg++) {

		// Wake up ICs if necessary
		if (ADBMS6830_wakeup_necessary()) {
			ADBMS6830_wakeup(hspi_ptr, htim_ptr);
		}

		uint8_t data[N_OF_ADBMS][6];
		ADBMS6830_rdfc_reg(hspi_ptr, htim_ptr, reg, data, spi_errors);

		// Parse voltages and package them into 2D array
		for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
			if (!spi_errors[ic]) {
				for (uint8_t cell = 0; cell < CELL_IN_REG; cell++) {
					int16_t parsed_voltage = (int16_t)((data[ic][(cell * 2) + 1] << 8) +
														data[ic][cell * 2]);
					filt_voltages[ic][(reg * CELL_IN_REG) + cell] = parsed_voltage;
				}
			}
			else {
				for (uint8_t cell = 0; cell < CELL_IN_REG; cell++) {
					filt_voltages[ic][(reg * CELL_IN_REG) + cell] = 0;
				}
			}
		}
	}

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	ADBMS6830_unfreeze_results(hspi_ptr, htim_ptr);
}

/*
 \brief Reads back un-filtered cell voltages from one register group for each IC

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle
 @param[in] RegGroup_t reg filtered cell voltage register group to read from

 @param[out] uint8_t data[N_OF_ADBMS][6] 2D array containing the data read back (6 bytes per register)
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
*/
static void ADBMS6830_rdcv_reg(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
							   TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
							   RegGroup_t reg_num,                // Option: filtered cell voltage register group to read from
							   uint8_t data[N_OF_ADBMS][6],       // Input: 2D array containing the data read back
							   uint8_t spi_errors[N_OF_ADBMS]     // Input: Array containing flags indicating which nodes had SPI errors
							   )
{
	uint8_t cmd[2];
	cmd[0] = 0x00;
	if (reg_num < 4) cmd[1] = 0x04 + (reg_num * 2);
	else cmd[1] = 0x09 + ((reg_num - 4) * 2);

	uint8_t num_tries = 0;
	uint8_t try_again = 0;

	do {
		try_again = !spi_write_read(hspi_ptr, htim_ptr, cmd, data, spi_errors);

		num_tries++;
		if ((num_tries > 2) && try_again) return;
	} while (try_again);
}

/*
 \brief Reads back un-filtered cell voltages from all register groups from all ICs

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle

 @param[out] int16_t voltages[N_OF_ADBMS][CELLS_PER_ADBMS] 2D array containing the voltages
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
*/
void ADBMS6830_rdcv_all(SPI_HandleTypeDef* const hspi_ptr,             // Pointer to the SPI handle
						TIM_HandleTypeDef* const htim_ptr,             // Pointer to a timer handle
						int16_t voltages[N_OF_ADBMS][CELLS_PER_ADBMS], // Input: 2D array containing voltages
						uint8_t spi_errors[N_OF_ADBMS]                 // Input: Array containing flags indicating which nodes had SPI errors
					    )
{
	const uint8_t CELL_IN_REG = 3u; // 6 bytes per register / 2 bytes per cell = 3 cell voltages per register

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	// Freeze all result registers for data coherence
	ADBMS6830_freeze_results(hspi_ptr, htim_ptr);

	for (uint8_t reg = 0; reg < 4; reg++) {

		if (ADBMS6830_wakeup_necessary()) {
			ADBMS6830_wakeup(hspi_ptr, htim_ptr);
		}

		uint8_t data[N_OF_ADBMS][6];
		ADBMS6830_rdcv_reg(hspi_ptr, htim_ptr, reg, data, spi_errors);

		// Parse voltages and package them into 2D array
		for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
			if (!spi_errors[ic]) {
				for (uint8_t cell = 0; cell < CELL_IN_REG; cell++) {
					int16_t parsed_voltage = (int16_t)((data[ic][(cell * 2) + 1] << 8) +
														data[ic][cell * 2]);
					voltages[ic][(reg * CELL_IN_REG) + cell] = parsed_voltage;
				}
			}
			else {
				for (uint8_t cell = 0; cell < CELL_IN_REG; cell++) {
					voltages[ic][(reg * CELL_IN_REG) + cell] = 0;
				}
			}
		}
	}

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	ADBMS6830_unfreeze_results(hspi_ptr, htim_ptr);
}
/*
 \brief Reads back S voltages from one register group for each IC

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle
 @param[in] RegGroup_t reg filtered cell voltage register group to read from

 @param[out] uint8_t data[N_OF_ADBMS][6] 2D array containing the data read back (6 bytes per register)
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
static void ADBMS6830_rdsv_reg(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
							   TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
							   RegGroup_t reg_num,                // Option: filtered cell voltage register group to read from
							   uint8_t data[N_OF_ADBMS][6],       // Input: 2D array containing the data read back
							   uint8_t spi_errors[N_OF_ADBMS]     // Input: Array containing flags indicating which nodes had SPI errors
							   )
{
	uint8_t cmd[2];
	cmd[0] = 0x00;
	if (reg_num < 3) cmd[1] = 0x03 + (reg_num * 2);
	else cmd[1] = 0x0D + (reg_num - 3);

	uint8_t num_tries = 0;
	uint8_t try_again = 0;

	do {
		try_again = !spi_write_read(hspi_ptr, htim_ptr, cmd, data, spi_errors);

		num_tries++;
		if ((num_tries > 2) && try_again) return;
	} while (try_again);
}

/*
 \brief Reads back S voltages from all register groups from all ICs

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle

 @param[out] int16_t s_voltages[N_OF_ADBMS][CELLS_PER_ADBMS] 2D array containing the S voltages
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
void ADBMS6830_rdsv_all(SPI_HandleTypeDef* const hspi_ptr,               // Pointer to the SPI handle
						TIM_HandleTypeDef* const htim_ptr,               // Pointer to a timer handle
						int16_t s_voltages[N_OF_ADBMS][CELLS_PER_ADBMS], // Input: 2D array containing S voltages
						uint8_t spi_errors[N_OF_ADBMS]                   // Input: Array containing flags indicating which nodes had SPI errors
						)
{
	const uint8_t CELL_IN_REG = 3u; // 6 bytes per register / 2 bytes per cell = 3 cell voltages per register

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	// Freeze all result registers for data coherence
	ADBMS6830_freeze_results(hspi_ptr, htim_ptr);

	for (uint8_t reg = 0; reg < 4; reg++) {

		if (ADBMS6830_wakeup_necessary()) {
			ADBMS6830_wakeup(hspi_ptr, htim_ptr);
		}

		uint8_t data[N_OF_ADBMS][6];
		ADBMS6830_rdsv_reg(hspi_ptr, htim_ptr, reg, data, spi_errors);

		// Parse voltages and package them into 2D array
		for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
			if (!spi_errors[ic]) {
				for (uint8_t cell = 0; cell < CELL_IN_REG; cell++) {
					int16_t parsed_voltage = (int16_t)((data[ic][(cell * 2) + 1] << 8) +
														data[ic][cell * 2]);
					s_voltages[ic][(reg * CELL_IN_REG) + cell] = parsed_voltage;
				}
			}
			else {
				for (uint8_t cell = 0; cell < CELL_IN_REG; cell++) {
					s_voltages[ic][(reg * CELL_IN_REG) + cell] = 0;
				}
			}
		}
	}

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	ADBMS6830_unfreeze_results(hspi_ptr, htim_ptr);
}

/*
 \brief Reads back auxiliary voltages from one register group for each IC

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle
 @param[in] RegGroup_t reg auxiliary register group to read from

 @param[out] uint8_t data[N_OF_ADBMS][6] 2D array containing the data read back (6 bytes per register)
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
static void ADBMS6830_rdaux_reg(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
						 	 	TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
								RegGroup_t reg_num,                // Option: filtered cell voltage register group to read from
								uint8_t data[N_OF_ADBMS][6],       // Input: 2D array containing the data read back
								uint8_t spi_errors[N_OF_ADBMS]     // Input: Array containing flags indicating which nodes had SPI errors
						 	 	)
{
	uint8_t cmd[2];
	cmd[0] = 0x00;
	cmd[1] = 0x19 + reg_num;

	uint8_t num_tries = 0;
	uint8_t try_again = 0;

	do {
		try_again = !spi_write_read(hspi_ptr, htim_ptr, cmd, data, spi_errors);

		num_tries++;
		if ((num_tries > 2) && try_again) return;
	} while (try_again);
}

/*
 \brief Reads ADC reading from one aux pin for each IC

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandletypeDef* htim_ptr pointer to a timer handle
 @param[in] AuxPin_t pin the auxiliary pin to read from

 @param[out] int16_t aux[N_OF_ADBMS] Array to read ADC conversions into
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
void ADBMS6830_rdaux_pin(SPI_HandleTypeDef* const hspi_ptr, // Pointer to the SPI handle
						 TIM_HandleTypeDef* const htim_ptr, // Pointer to a timer handle
						 AuxPin_t pin,                      // Option: Auxiliary pin to read from
						 int16_t aux[N_OF_ADBMS],           // Input: Array to read ADC conversions into
						 uint8_t spi_errors[N_OF_ADBMS]     // Input: Array containing flags indicating which nodes had SPI errors
						 )
{
	uint8_t cmd[2];
	cmd[0] = 0x00;

	uint8_t data[N_OF_ADBMS][6]; // 6 bytes per register
	uint8_t lsb_idx = 0;
	uint8_t try_again = 0;
	uint8_t num_tries = 0;

	// Check if pin is a GPIO pin
	if (pin < VREF2) {
		uint8_t reg = pin / 3;
		cmd[1] = 0x19 + reg;

		uint8_t offset = reg * 3;
		lsb_idx = (pin - offset) * 2;
	}
	else if ((pin == VREF2) || (pin == ITEMP) || (pin == RESERVED)) {
		cmd[1] = 0x30;

		if (pin == VREF2) lsb_idx = 0;
		else if (pin == ITEMP) lsb_idx = 2;
		else lsb_idx = 4;
	}
	else if ((pin == VD) || (pin == VA) || (pin == VRES)) {
		cmd[1] = 0x31;

		if (pin == VD) lsb_idx = 0;
		else if (pin == VA) lsb_idx = 2;
		else lsb_idx = 4;
	}

	else if ((pin == VPV) || (pin == VMV)) {
		cmd[1] = 0x1F;

		if (pin == VMV) lsb_idx = 2;
		else lsb_idx = 4;
	}

	do {
		try_again = !spi_write_read(hspi_ptr, htim_ptr, cmd, data, spi_errors);

		num_tries++;
		if ((num_tries > 2) && try_again) break;
	} while (try_again);

	for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
		if (!spi_errors[ic]) {
			aux[ic] = (int16_t)((data[ic][lsb_idx + 1] << 8) + data[ic][lsb_idx]);
		}
	}
}

/*
 \brief Reads back raw ADC readings from GPIOs, which are connected to thermistors

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle

 @param[out] int16_t raw_temp_voltages[N_OF_ADBMS][CELL_TEMPS_PER_ADBMS] 2D array containing the ADC readings
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
void ADBMS6830_rdaux_raw_temp_voltages(SPI_HandleTypeDef* const hspi_ptr,                           // Pointer to the SPI handle
							           TIM_HandleTypeDef* const htim_ptr,                           // Pointer to a timer handle
							           int16_t raw_temp_voltages[N_OF_ADBMS][CELL_TEMPS_PER_ADBMS], // Input: 2D array containing the ADC readings
							           uint8_t spi_errors[N_OF_ADBMS]                               // Input: Array containing flags indicating which nodes had SPI errors
							           )
{
	const uint8_t TEMP_IN_REG = 3u; // 6 register bytes / 2 bytes per voltage = 3 temp voltages per register

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	// Freeze all result registers for data coherence
	ADBMS6830_freeze_results(hspi_ptr, htim_ptr);

	for (uint8_t reg = 0; reg < 3; reg++) {

		// Wake up ICs if necessary
		if (ADBMS6830_wakeup_necessary()) {
			ADBMS6830_wakeup(hspi_ptr, htim_ptr);
		}

		uint8_t data[N_OF_ADBMS][6];
		ADBMS6830_rdaux_reg(hspi_ptr, htim_ptr, reg, data, spi_errors);

		// Parse voltages and package them into 2D array
		for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
			if (!spi_errors[ic]) {
				for (uint8_t temp = 0; temp < TEMP_IN_REG; temp++) {
					if ((reg == 2) && (temp == 2)) break; // Ignore G9V since there are only 8 cell temps per IC
					int16_t parsed_voltage = (int16_t)((data[ic][(temp * 2) + 1] << 8) +
														data[ic][temp * 2]);
					raw_temp_voltages[ic][(reg * TEMP_IN_REG) + temp] = parsed_voltage;
				}
			}
			else {
				for (uint8_t temp = 0; temp < TEMP_IN_REG; temp++) {
					if ((reg == 2) && (temp == 2)) break;
					raw_temp_voltages[ic][(reg * TEMP_IN_REG) + temp] = 0;
				}
			}
		}
	}

	// Wake up ICs if necessary
	if (ADBMS6830_wakeup_necessary()) {
		ADBMS6830_wakeup(hspi_ptr, htim_ptr);
	}

	ADBMS6830_unfreeze_results(hspi_ptr, htim_ptr);
}

/*
 \brief Reads back all cell voltage mismatches detected during the redundancy check

 @param[in] SPI_HandleTypeDef* hspi_ptr pointer to the SPI handle
 @param[in] TIM_HandleTypeDef* htim_ptr pointer to a timer handle

 @param[out] uint8_t mismatches[N_OF_ADBMS][CELLS_PER_ADBMS] 2D array containing the mismatch flags for each IC
 @param[out] uint8_t spi_errors[N_OF_ADBMS] Array containing flags indicating which nodes had SPI errors
 */
void ADBMS6830_rdstatc_mismatch(SPI_HandleTypeDef* const hspi_ptr,               // Pointer to the SPI handle
								TIM_HandleTypeDef* const htim_ptr,               // Pointer to a timer handle
								uint8_t mismatches[N_OF_ADBMS][CELLS_PER_ADBMS], // Input: 2D array containing the mismatch flags for each IC
								uint8_t spi_errors[N_OF_ADBMS]                   // Input: Array containing flags indicating which nodes had SPI errors
								)
{
	uint8_t data[N_OF_ADBMS][6]; // 6 bytes per register
	uint8_t cmd[2] = {0x00, 0x32};
	uint8_t try_again = 0;
	uint8_t num_tries = 0;

	do {
		try_again = !spi_write_read(hspi_ptr, htim_ptr, cmd, data, spi_errors);

		num_tries++;
		if ((num_tries > 2) && try_again) break;
	} while (try_again);

	for (uint8_t ic = 0; ic < N_OF_ADBMS; ic++) {
		if (!spi_errors[ic]) {
			if (data[ic][5] & 0x20) { // Check if C-ADC/S-ADC comparisons are enabled
				for (uint8_t reg_idx = 0; reg_idx < 2; reg_idx++) {
					for (uint8_t bit = 0; bit < 8; bit++) {
						if ((reg_idx == 1) && (bit == 4)) break; // Ignore flags past CS12FLT since we only have 12 cells per IC
						mismatches[ic][(reg_idx * 8) + bit] = (data[ic][reg_idx] >> bit) & 0x01;
					}
				}
			}
		}
	}
}
