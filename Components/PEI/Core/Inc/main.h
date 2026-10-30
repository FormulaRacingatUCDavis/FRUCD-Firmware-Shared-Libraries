/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Wake_6822_Pin GPIO_PIN_2
#define Wake_6822_GPIO_Port GPIOE
#define Wake2_6822_Pin GPIO_PIN_3
#define Wake2_6822_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOF
#define SCF1_EN_Pin GPIO_PIN_4
#define SCF1_EN_GPIO_Port GPIOF
#define SPI5_CS2_Pin GPIO_PIN_5
#define SPI5_CS2_GPIO_Port GPIOF
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define Heartbeat_Pin GPIO_PIN_3
#define Heartbeat_GPIO_Port GPIOA
#define SCF2_EN_Pin GPIO_PIN_6
#define SCF2_EN_GPIO_Port GPIOA
#define LCD_DB4_Pin GPIO_PIN_7
#define LCD_DB4_GPIO_Port GPIOE
#define LCD_DB5_Pin GPIO_PIN_8
#define LCD_DB5_GPIO_Port GPIOE
#define LCD_DB6_Pin GPIO_PIN_9
#define LCD_DB6_GPIO_Port GPIOE
#define LCD_DB7_Pin GPIO_PIN_10
#define LCD_DB7_GPIO_Port GPIOE
#define LCD_DB0_Pin GPIO_PIN_14
#define LCD_DB0_GPIO_Port GPIOD
#define LCD_DB1_Pin GPIO_PIN_15
#define LCD_DB1_GPIO_Port GPIOD
#define SDC_Acc_Out_Pin GPIO_PIN_6
#define SDC_Acc_Out_GPIO_Port GPIOC
#define SDC_Final_Pin GPIO_PIN_7
#define SDC_Final_GPIO_Port GPIOC
#define SDC_Out_Pin GPIO_PIN_8
#define SDC_Out_GPIO_Port GPIOC
#define SDC_In_Pin GPIO_PIN_9
#define SDC_In_GPIO_Port GPIOC
#define LCD_DB2_Pin GPIO_PIN_0
#define LCD_DB2_GPIO_Port GPIOD
#define LCD_DB3_Pin GPIO_PIN_1
#define LCD_DB3_GPIO_Port GPIOD
#define LCD_Backlight_Pin GPIO_PIN_2
#define LCD_Backlight_GPIO_Port GPIOD
#define LCD_RW_Pin GPIO_PIN_5
#define LCD_RW_GPIO_Port GPIOD
#define LCD_E_Pin GPIO_PIN_7
#define LCD_E_GPIO_Port GPIOD
#define BMS_Fault_Pin GPIO_PIN_10
#define BMS_Fault_GPIO_Port GPIOG
#define BMS_OK_Pin GPIO_PIN_11
#define BMS_OK_GPIO_Port GPIOG
#define IMD_Fault_Pin GPIO_PIN_12
#define IMD_Fault_GPIO_Port GPIOG
#define AIR_Neg_Pin GPIO_PIN_14
#define AIR_Neg_GPIO_Port GPIOG
#define AIR_Pos_Pin GPIO_PIN_6
#define AIR_Pos_GPIO_Port GPIOB
#define Precharge_Pin GPIO_PIN_7
#define Precharge_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
