#ifndef TELEM_NODE_H
#define TELEM_NODE_H


#include "main.h"
#include "config.h"

typedef struct {
	uint32_t channel_number;
	uint8_t flag;
	uint16_t adc_value;
} ADC_Channel_t;

#define SHOCK_ANGLE1_CHANNEL ADC_CHANNEL_2
#define SHOCK_ANGLE2_CHANNEL ADC_CHANNEL_7
#define BRAKE_TEMP_CHANNEL ADC_CHANNEL_8

#define NUM_ADC_CHANNELS 3
extern volatile ADC_Channel_t adc_channels[NUM_ADC_CHANNELS];
extern uint8_t wheel_speed_flag;

// PUBLIC FUNCTION PROTOTYPES
void TelemNode_Init();
void TelemNode_Update();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

uint16_t get_strain_gauge_val(uint64_t adc_val);

uint16_t get_shock_angle(uint64_t adc_val);

uint16_t get_ir_brake_temp(uint64_t adc_val);

uint16_t get_k_brake_temp(uint64_t adc_val);

#endif
