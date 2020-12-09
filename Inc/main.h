/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_ON_OFF_IND_Pin GPIO_PIN_3
#define POWER_ON_OFF_IND_GPIO_Port GPIOE
#define BAT_LED1_Pin GPIO_PIN_4
#define BAT_LED1_GPIO_Port GPIOE
#define BAT_LED2_Pin GPIO_PIN_5
#define BAT_LED2_GPIO_Port GPIOE
#define BAT_LED3_Pin GPIO_PIN_6
#define BAT_LED3_GPIO_Port GPIOE
#define RS232_DETECT_Pin GPIO_PIN_1
#define RS232_DETECT_GPIO_Port GPIOA
#define EDA_SPI1_MISO_Pin GPIO_PIN_6
#define EDA_SPI1_MISO_GPIO_Port GPIOA
#define EDA_SPI1_MOSI_Pin GPIO_PIN_7
#define EDA_SPI1_MOSI_GPIO_Port GPIOA
#define EDA_SEL2_Pin GPIO_PIN_4
#define EDA_SEL2_GPIO_Port GPIOC
#define EDA_SEL_1_Pin GPIO_PIN_5
#define EDA_SEL_1_GPIO_Port GPIOC
#define EDA_SPI1_CS_Pin GPIO_PIN_1
#define EDA_SPI1_CS_GPIO_Port GPIOB
#define ECG_SPI2_CS_Pin GPIO_PIN_7
#define ECG_SPI2_CS_GPIO_Port GPIOE
#define ECG_START_Pin GPIO_PIN_8
#define ECG_START_GPIO_Port GPIOE
#define ECG_RESET_Pin GPIO_PIN_9
#define ECG_RESET_GPIO_Port GPIOE
#define ECG_CLK_SEL_Pin GPIO_PIN_10
#define ECG_CLK_SEL_GPIO_Port GPIOE
#define EDA_RESET_Pin GPIO_PIN_11
#define EDA_RESET_GPIO_Port GPIOE
#define EDA_DATA_INT_Pin GPIO_PIN_12
#define EDA_DATA_INT_GPIO_Port GPIOE
#define EDA_SPI1_SCLK_Pin GPIO_PIN_13
#define EDA_SPI1_SCLK_GPIO_Port GPIOE
#define EDA_SEL1_Pin GPIO_PIN_14
#define EDA_SEL1_GPIO_Port GPIOE
#define TEMP_POWER_CON_Pin GPIO_PIN_15
#define TEMP_POWER_CON_GPIO_Port GPIOE
#define TEMP_I2C2_SCL_Pin GPIO_PIN_10
#define TEMP_I2C2_SCL_GPIO_Port GPIOB
#define TEMP_I2C2_SDA_Pin GPIO_PIN_11
#define TEMP_I2C2_SDA_GPIO_Port GPIOB
#define ECG_SPI2_SCLK_Pin GPIO_PIN_13
#define ECG_SPI2_SCLK_GPIO_Port GPIOB
#define ECG_SPI2_MISO_Pin GPIO_PIN_14
#define ECG_SPI2_MISO_GPIO_Port GPIOB
#define ECG_SPI2_MOSI_Pin GPIO_PIN_15
#define ECG_SPI2_MOSI_GPIO_Port GPIOB
#define ECG_DRDY_Pin GPIO_PIN_10
#define ECG_DRDY_GPIO_Port GPIOD
#define PD3_SEL_Pin GPIO_PIN_12
#define PD3_SEL_GPIO_Port GPIOD
#define LD3_SEL_Pin GPIO_PIN_13
#define LD3_SEL_GPIO_Port GPIOD
#define PD2_SEL_Pin GPIO_PIN_14
#define PD2_SEL_GPIO_Port GPIOD
#define LED2_SEL_Pin GPIO_PIN_15
#define LED2_SEL_GPIO_Port GPIOD
#define PD1_SEL_Pin GPIO_PIN_6
#define PD1_SEL_GPIO_Port GPIOC
#define LED1_SEL_Pin GPIO_PIN_7
#define LED1_SEL_GPIO_Port GPIOC
#define FLASH_PWR_CON_Pin GPIO_PIN_15
#define FLASH_PWR_CON_GPIO_Port GPIOA
#define MEM_SPI3_SCK_Pin GPIO_PIN_10
#define MEM_SPI3_SCK_GPIO_Port GPIOC
#define MEM_SPI3_MISO_Pin GPIO_PIN_11
#define MEM_SPI3_MISO_GPIO_Port GPIOC
#define MEM_SPI3_MOSI_Pin GPIO_PIN_12
#define MEM_SPI3_MOSI_GPIO_Port GPIOC
#define MEM_SPI3_CS_Pin GPIO_PIN_0
#define MEM_SPI3_CS_GPIO_Port GPIOD
#define RS232_DETECTD1_Pin GPIO_PIN_1
#define RS232_DETECTD1_GPIO_Port GPIOD
#define M_KEY_Pin GPIO_PIN_3
#define M_KEY_GPIO_Port GPIOD
#define ECG_PWR_CON_Pin GPIO_PIN_4
#define ECG_PWR_CON_GPIO_Port GPIOD
#define PPG_PWR_CON_Pin GPIO_PIN_4
#define PPG_PWR_CON_GPIO_Port GPIOB
#define PPG_INT_Pin GPIO_PIN_5
#define PPG_INT_GPIO_Port GPIOB
#define PPG_I2C1_SCL_Pin GPIO_PIN_6
#define PPG_I2C1_SCL_GPIO_Port GPIOB
#define PPG_I2C1_SDA_Pin GPIO_PIN_7
#define PPG_I2C1_SDA_GPIO_Port GPIOB
#define USB_DET_Pin GPIO_PIN_8
#define USB_DET_GPIO_Port GPIOB
#define CHARG_STAT2_Pin GPIO_PIN_0
#define CHARG_STAT2_GPIO_Port GPIOE
#define CHARG_STAT1_Pin GPIO_PIN_1
#define CHARG_STAT1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
