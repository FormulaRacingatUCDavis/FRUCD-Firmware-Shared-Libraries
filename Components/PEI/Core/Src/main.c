/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>

#include "fsm.h"
#include "cell_interface.h"
#include "can_manager.h"
#include "charger.h"
#include "SOC.h"
#include "battery_gui.h"
#include "relays.h"
#include "LCD.h"
#include "display.h"
#include "delays.h"
#include "data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
// PEI parameters
PEI_STATE_t pei_state = PEI_LV;
uint8_t pei_status = NORMAL;

// Relay parameters
extern uint8_t relay_flags;

// VCU parameters
volatile VCU_STATE_t vcu_state = LV;
volatile uint8_t hv_requested = 0;
volatile uint8_t vcu_attached = 0;

// MC parameters
volatile int16_t mc_voltage = 0;
volatile MC_VSM_STATE_t mc_vsm_state = VSM_START;
volatile MC_DISCHARGE_STATE_t mc_discharge_state = DISCHARGE_DISABLED;
volatile uint32_t mc_post_faults = 0;
volatile uint32_t mc_run_faults = 0;

// BMS parameters
extern volatile BAT_PACK_t bat_pack;

// Charger parameters
extern volatile uint8_t charger_attached;
extern uint8_t charge_control;
extern uint8_t charge_profile_received;
extern uint16_t charger_max_current;

extern volatile UART_TxCplt; // flag for whether UART transmission finished

// Tick counters
volatile uint32_t ticks_since_vcu_message = 0;
volatile uint32_t ticks_since_mc_message = 0;
volatile uint32_t ticks_since_charger_message = 0;

// Buffer to store raw current measurements from ADC
// Buffer length in bytes needs to be divisible by 32
static volatile uint16_t current_meas_buf[16] __ALIGNED(32); // cache lines are 32 bytes long

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI5_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void BMS_Init();
static float raw_to_mvolts(uint16_t adc_raw);
static float mvolts_to_amps(float mVolts, float mVolt_ref);
static void set_back_fans(float duty_cycle);
static void set_side_fans(float duty_cycle);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_ptr);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void BMS_Init() {
	// Pull SPI chip selects high (chip select is active low)
	HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI5_CS2_GPIO_Port, SPI5_CS2_Pin, GPIO_PIN_SET);

	cell_interface_init(&hspi5, &htim10);
	init_SOC_vars();
}

static float raw_to_mvolts(uint16_t adc_raw) {
	return ((float)adc_raw / 4095) * 3.3 * 1000;
}

static float mvolts_to_amps(float mVolts, float mVolt_ref) {
	return ((mVolts * 7.4 / 4.7) - mVolt_ref) / 6.667;
}

static void enable_fans() {
	HAL_GPIO_WritePin(SCF1_EN_GPIO_Port, SCF1_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SCF2_EN_GPIO_Port, SCF2_EN_Pin, GPIO_PIN_SET);
}

static void disable_fans() {
	HAL_GPIO_WritePin(SCF1_EN_GPIO_Port, SCF1_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SCF2_EN_GPIO_Port, SCF2_EN_Pin, GPIO_PIN_RESET);
}

static void set_back_fans(float duty_cycle) {
	htim2.Instance->CCR1 = (uint32_t)((1 - duty_cycle) * htim2.Instance->ARR);
}

static void set_side_fans(float duty_cycle) {
	htim2.Instance->CCR2 = (uint32_t)((1 - duty_cycle) * htim2.Instance->ARR);
}

// Timer triggers ADC conversion every 250 us (4 kHz)
// Called when ADC conversion and DMA transfer is complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	SCB_InvalidateDCache_by_Addr((uint32_t*)current_meas_buf, 4);

	// Move raw measurements into bat_pack struct
	bat_pack.current_raw = current_meas_buf[0];
	bat_pack.current_ref_raw = current_meas_buf[1];

	// Calculate current in Amps
	float mVolts = raw_to_mvolts(current_meas_buf[0]);
	float mVolt_ref = raw_to_mvolts(current_meas_buf[1]);

	// Move calculated current into bat_pack struct
	bat_pack.current = mvolts_to_amps(mVolts, mVolt_ref);
	if (!charger_attached) can_send_PEI_Current();
}

// Called every 100 ms (10 Hz)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (!charger_attached) {
		can_send_BMS_High_Level_Data();

		for (uint8_t subpack = 0; subpack < N_OF_SUBPACK; subpack++) {
			can_send_BMS_Subpack_Data(subpack);
			delay_us(&htim10, 250); // delay so we don't spam the bus
		}

		for (uint8_t subpack = 0; subpack < N_OF_SUBPACK; subpack++) {
			for (uint8_t group = 0; group < 6; group++) {
				can_send_BMS_Temps(subpack, group);
				delay_us(&htim10, 250);
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
	UART_TxCplt = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_SPI5_Init();
  MX_IWDG_Init();
  MX_CAN2_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim10);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim7);

  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)current_meas_buf, 2);

  LCD_Init(&htim10);
  BMS_Init();

  uint8_t shutdown_flags = 0;

  BMS_MODE_t bms_status = BMS_NORMAL;
  uint32_t cell_disconnect_check_tickstart = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // BMS

	  update_voltages(&hspi5, &htim10);
	  update_temps(&hspi5, &htim10);

	  process_voltages();
	  process_temps();

	  if ((HAL_GetTick() - cell_disconnect_check_tickstart) > 1000) {
		  cell_disconnect_check(&hspi5, &htim10);
		  cell_disconnect_check_tickstart = HAL_GetTick();
	  }

	  switch (bms_status) {
		  case BMS_NORMAL:
			  HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_SET);
			  bms_status = bat_health_check();

			  // Check if we're charging accumulator
			  if(charger_attached && (pei_state == PEI_HV)){
				  balance_cells(&hspi5, &htim10);
			  } else {
				  disable_cell_balancing(&hspi5, &htim10);
			  }

			  break;

		  case BMS_FAULT:
			  HAL_GPIO_WritePin(BMS_OK_GPIO_Port, BMS_OK_Pin, GPIO_PIN_RESET);
			  disable_cell_balancing(&hspi5, &htim10); // make sure cell balancing is disabled

			  break;
	  }

	  if (bat_pack.HI_temp_c > 40) {
		  enable_fans();

		  // Set cooling fans to 40% duty cycle
		  set_back_fans(0.4);
		  set_side_fans(0.4);
	  }
	  else {
		  // Stop cooling fans
		  set_back_fans(0);
		  set_side_fans(0);
		  disable_fans();
	  }

	  // Update SOC estimate
	  update_SOC_input();
	  EKF(); // run extended Kalman filter and update pack struct SOC estimate

	  // -------------------- BMS ----------------------

	  // PEI

	  charge_control = CHARGE_STOP;

	  shutdown_flags = relay_flags & 0x07;
	  if (!HAL_GPIO_ReadPin(IMD_Fault_GPIO_Port, IMD_Fault_Pin)) shutdown_flags |= (1 << 5);
	  if (!HAL_GPIO_ReadPin(BMS_Fault_GPIO_Port, BMS_Fault_Pin)) shutdown_flags |= (1 << 4);
	  if (HAL_GPIO_ReadPin(SDC_Final_GPIO_Port, SDC_Final_Pin)) shutdown_flags |= (1 << 3);

	  // Interlock state machine

	  update_status();

	  if (!hv_allowed()) {
		  pei_state = PEI_FAULT;
	  }

	  switch (pei_state) {
		  case PEI_LV:
			  clear_interlock(); // clears interlock, sends a message to open AIRs

			  if (hv_request() && hv_allowed()) {
				  if (vcu_attached && precharge_ready()) {
					  pei_state = PEI_PRECHARGE;
				  }
				  else if (charger_attached) {
					  pei_state = PEI_HV; // charger doesn't need precharge
				  }
			  }

			  break;

		  case PEI_PRECHARGE:
			  start_precharge();

			  if (precharge_complete()) {
				  pei_state = PEI_HV;
			  }

			  if (!hv_request()) {
				  pei_state = PEI_LV;
			  }

			  break;

		  case PEI_HV:
			  finish_precharge();

			  if (charger_attached) {
				  charge_profile_received = uart_receive_charge_profile(&huart3);
				  if (charge_profile_received) {
					  update_max_charge_current();
				  }
				  if (charger_max_current != 0) {
					  charge_control = CHARGE_START;
				  }
			  }

			  if (!hv_request()) {
				  pei_state = PEI_LV;
			  }

			  break;

		  default:
			  clear_interlock();
			  pei_state = PEI_FAULT;

			  if (!hv_request() && hv_allowed()) {
				  pei_state = PEI_LV;
			  }
	  }

	  // -------------------- PEI ----------------------

	  update_can_status();

	  update_display(&htim10);
	  HAL_GPIO_TogglePin(Heartbeat_GPIO_Port, Heartbeat_Pin);

	  // TODO: check if USB is plugged in when we get the hardware functionality
	  // Send battery GUI data
	  // Need to wait for previous UART transmission to complete if not already, but only allow 30 ms delay
	  uint32_t timeout_tickstart = HAL_GetTick();
	  uint8_t timed_out = 0;
	  while(!UART_TxCplt && !timed_out) timed_out = (HAL_GetTick() - timeout_tickstart) > 30;
	  if (!timed_out) uart_send_GUI_Data(&huart3);

	  HAL_IWDG_Refresh(&hiwdg);

	  if (charger_attached) {
		  can_send_Charger();
	  }
	  else {
		  can_send_PEI_Status(shutdown_flags);

		  can_send_BMS_Status();
		  can_send_BMS_Diagnostics();
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  // Workaround for ADC missing external timer triggers: enable DAC clock
  // Source: https://community.st.com/stm32-mcus-products-25/adc-trigger-with-timer-not-working-51318
  uint32_t* RCC_APB1ENR = (uint32_t*)(0x40023800 + 0x40);
  *RCC_APB1ENR |= 0x20000000; // set DAC_EN (bit 29)

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // Filter vehicle state messages into FIFO0
  CAN_FilterTypeDef can1_filter;
  can1_filter.FilterActivation = CAN_FILTER_ENABLE;
  can1_filter.SlaveStartFilterBank = 18;
  can1_filter.FilterBank = 0;
  can1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can1_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can1_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can1_filter.FilterIdHigh = 0x766 << 5;
  can1_filter.FilterIdLow = 0x0000;
  can1_filter.FilterMaskIdHigh = 0x766 << 5;
  can1_filter.FilterMaskIdLow = 0x0000;
  if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter) != HAL_OK) {
	  Error_Handler();
  }

  // Filter motor controller messages into FIFO1
  can1_filter.FilterBank = 1;
  can1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  can1_filter.FilterIdHigh = 0x0A0 << 5;
  can1_filter.FilterMaskIdHigh = 0x0A0 << 5;
  if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter) != HAL_OK) {
	  Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
	  Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  	  Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
	  Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 18;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  // Filter charger messages into FIFO0
  const uint32_t charger_id = 0x18FF50E5 << 3;

  CAN_FilterTypeDef can2_filter;
  can2_filter.FilterActivation = CAN_FILTER_ENABLE;
  can2_filter.SlaveStartFilterBank = 18;
  can2_filter.FilterBank = 18;
  can2_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can2_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can2_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can2_filter.FilterIdHigh = charger_id >> 16;
  can2_filter.FilterIdLow = charger_id & 0x0000FFFF;
  can2_filter.FilterMaskIdHigh = charger_id >> 16;
  can2_filter.FilterMaskIdLow = charger_id & 0x0000FFFF;
  if (HAL_CAN_ConfigFilter(&hcan2, &can2_filter) != HAL_OK) {
	  Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
	  Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	  Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4000-1;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  uint32_t* DBGMCU_APB1_FZ = (uint32_t*)(0xE0042008);
  *DBGMCU_APB1_FZ |= 0x00001000; // set DBG_IWDG_STOP to disable IWDG in debug mode

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 27000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 54000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 27-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_RS_Pin|SCF1_EN_Pin|SPI5_CS2_Pin|SPI5_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Heartbeat_Pin|SCF2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_DB4_Pin|LCD_DB5_Pin|LCD_DB6_Pin|LCD_DB7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_DB0_Pin|LCD_DB1_Pin|LCD_DB2_Pin|LCD_DB3_Pin
                          |LCD_Backlight_Pin|LCD_RW_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BMS_OK_Pin|AIR_Neg_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIR_Pos_Pin|Precharge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Wake_6822_Pin Wake2_6822_Pin */
  GPIO_InitStruct.Pin = Wake_6822_Pin|Wake2_6822_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin SCF1_EN_Pin SPI5_CS2_Pin SPI5_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|SCF1_EN_Pin|SPI5_CS2_Pin|SPI5_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Heartbeat_Pin SCF2_EN_Pin */
  GPIO_InitStruct.Pin = Heartbeat_Pin|SCF2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB4_Pin LCD_DB5_Pin LCD_DB6_Pin LCD_DB7_Pin */
  GPIO_InitStruct.Pin = LCD_DB4_Pin|LCD_DB5_Pin|LCD_DB6_Pin|LCD_DB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB0_Pin LCD_DB1_Pin LCD_DB2_Pin LCD_DB3_Pin
                           LCD_Backlight_Pin LCD_RW_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_DB0_Pin|LCD_DB1_Pin|LCD_DB2_Pin|LCD_DB3_Pin
                          |LCD_Backlight_Pin|LCD_RW_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SDC_Acc_Out_Pin SDC_Final_Pin SDC_Out_Pin SDC_In_Pin */
  GPIO_InitStruct.Pin = SDC_Acc_Out_Pin|SDC_Final_Pin|SDC_Out_Pin|SDC_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_Fault_Pin IMD_Fault_Pin */
  GPIO_InitStruct.Pin = BMS_Fault_Pin|IMD_Fault_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_OK_Pin AIR_Neg_Pin */
  GPIO_InitStruct.Pin = BMS_OK_Pin|AIR_Neg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : AIR_Pos_Pin Precharge_Pin */
  GPIO_InitStruct.Pin = AIR_Pos_Pin|Precharge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
