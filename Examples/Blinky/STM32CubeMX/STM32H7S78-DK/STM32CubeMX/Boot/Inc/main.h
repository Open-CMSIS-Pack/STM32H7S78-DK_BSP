/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7rsxx_hal.h"

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
#define DCMI_D2_Pin GPIO_PIN_0
#define DCMI_D2_GPIO_Port GPIOE
#define DCMI_HSYNC_Pin GPIO_PIN_3
#define DCMI_HSYNC_GPIO_Port GPIOG
#define USB2_FS_N_Pin GPIO_PIN_12
#define USB2_FS_N_GPIO_Port GPIOM
#define USB1_HS_P_Pin GPIO_PIN_6
#define USB1_HS_P_GPIO_Port GPIOM
#define DCMI_D5_Pin GPIO_PIN_3
#define DCMI_D5_GPIO_Port GPIOD
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define DCMI_D6_Pin GPIO_PIN_8
#define DCMI_D6_GPIO_Port GPIOB
#define MIC_CK_Pin GPIO_PIN_2
#define MIC_CK_GPIO_Port GPIOE
#define I2S_SDO_Pin GPIO_PIN_5
#define I2S_SDO_GPIO_Port GPIOB
#define LCD_RGB_R2_Pin GPIO_PIN_0
#define LCD_RGB_R2_GPIO_Port GPIOF
#define RMII_REF_CLK_Pin GPIO_PIN_7
#define RMII_REF_CLK_GPIO_Port GPIOD
#define USB2_FS_P_Pin GPIO_PIN_11
#define USB2_FS_P_GPIO_Port GPIOM
#define USB1_HS_N_Pin GPIO_PIN_5
#define USB1_HS_N_GPIO_Port GPIOM
#define LCD_RGB_R7_Pin GPIO_PIN_0
#define LCD_RGB_R7_GPIO_Port GPIOG
#define SPI_MISO_Pin GPIO_PIN_13
#define SPI_MISO_GPIO_Port GPIOE
#define VCP_TX_Pin GPIO_PIN_1
#define VCP_TX_GPIO_Port GPIOD
#define SD_D2_Pin GPIO_PIN_10
#define SD_D2_GPIO_Port GPIOC
#define SD_D3_Pin GPIO_PIN_11
#define SD_D3_GPIO_Port GPIOC
#define I2C1_I3C_SDA_Pin GPIO_PIN_9
#define I2C1_I3C_SDA_GPIO_Port GPIOB
#define DCMI_D3_Pin GPIO_PIN_1
#define DCMI_D3_GPIO_Port GPIOE
#define LCD_HSYNC_Pin GPIO_PIN_2
#define LCD_HSYNC_GPIO_Port GPIOG
#define UCPD1_CC2_Pin GPIO_PIN_1
#define UCPD1_CC2_GPIO_Port GPIOM
#define LCD_VSYNC_Pin GPIO_PIN_11
#define LCD_VSYNC_GPIO_Port GPIOE
#define VCP_RX_Pin GPIO_PIN_0
#define VCP_RX_GPIO_Port GPIOD
#define LCD_RGB_B2_Pin GPIO_PIN_12
#define LCD_RGB_B2_GPIO_Port GPIOA
#define LCD_RGB_B3_Pin GPIO_PIN_11
#define LCD_RGB_B3_GPIO_Port GPIOA
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define DCMI_VSYNC_Pin GPIO_PIN_7
#define DCMI_VSYNC_GPIO_Port GPIOB
#define LCD_RGB_R3_Pin GPIO_PIN_4
#define LCD_RGB_R3_GPIO_Port GPIOB
#define DCMI_PIXCLK_Pin GPIO_PIN_5
#define DCMI_PIXCLK_GPIO_Port GPIOD
#define UCPD1_CC1_Pin GPIO_PIN_0
#define UCPD1_CC1_GPIO_Port GPIOM
#define SD_CK_Pin GPIO_PIN_12
#define SD_CK_GPIO_Port GPIOC
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LCD_RGB_B5_Pin GPIO_PIN_9
#define LCD_RGB_B5_GPIO_Port GPIOA
#define SD_D0_Pin GPIO_PIN_8
#define SD_D0_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_6
#define SPI_MOSI_GPIO_Port GPIOE
#define DCMI_D4_Pin GPIO_PIN_4
#define DCMI_D4_GPIO_Port GPIOE
#define I2C1_I3C_SCL_Pin GPIO_PIN_6
#define I2C1_I3C_SCL_GPIO_Port GPIOB
#define LCD_RGB_R6_Pin GPIO_PIN_1
#define LCD_RGB_R6_GPIO_Port GPIOG
#define SPI_CLK_Pin GPIO_PIN_12
#define SPI_CLK_GPIO_Port GPIOE
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define LCD_RGB_B4_Pin GPIO_PIN_10
#define LCD_RGB_B4_GPIO_Port GPIOA
#define DCMI_D0_Pin GPIO_PIN_6
#define DCMI_D0_GPIO_Port GPIOC
#define LCD_RGB_B1_Pin GPIO_PIN_14
#define LCD_RGB_B1_GPIO_Port GPIOG
#define I2S_SDI_Pin GPIO_PIN_12
#define I2S_SDI_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define DCMI_D1_Pin GPIO_PIN_7
#define DCMI_D1_GPIO_Port GPIOC
#define LCD_CLK_Pin GPIO_PIN_13
#define LCD_CLK_GPIO_Port GPIOG
#define LCD_RGB_R5_Pin GPIO_PIN_15
#define LCD_RGB_R5_GPIO_Port GPIOA
#define OCTOSPI_IO1_Pin GPIO_PIN_3
#define OCTOSPI_IO1_GPIO_Port GPION
#define OCTOSPI_DQS_Pin GPIO_PIN_0
#define OCTOSPI_DQS_GPIO_Port GPION
#define OCTOSPI_IO7_Pin GPIO_PIN_11
#define OCTOSPI_IO7_GPIO_Port GPION
#define LCD_RGB_G0_Pin GPIO_PIN_7
#define LCD_RGB_G0_GPIO_Port GPIOF
#define OCTOSPI_IO6_Pin GPIO_PIN_10
#define OCTOSPI_IO6_GPIO_Port GPION
#define OCTOSPI_IO5_Pin GPIO_PIN_9
#define OCTOSPI_IO5_GPIO_Port GPION
#define OCTOSPI_IO0_Pin GPIO_PIN_2
#define OCTOSPI_IO0_GPIO_Port GPION
#define LCD_RGB_R0_Pin GPIO_PIN_9
#define LCD_RGB_R0_GPIO_Port GPIOF
#define LCD_RGB_R1_Pin GPIO_PIN_10
#define LCD_RGB_R1_GPIO_Port GPIOF
#define OCTOSPI_CLK_Pin GPIO_PIN_6
#define OCTOSPI_CLK_GPIO_Port GPION
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define MIC_DET_Pin GPIO_PIN_1
#define MIC_DET_GPIO_Port GPIOH
#define OCTOSPI_IO4_Pin GPIO_PIN_8
#define OCTOSPI_IO4_GPIO_Port GPION
#define OCTOSPI_IO2_Pin GPIO_PIN_4
#define OCTOSPI_IO2_GPIO_Port GPION
#define OCTOSPI_IO3_Pin GPIO_PIN_5
#define OCTOSPI_IO3_GPIO_Port GPION
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define LCD_RGB_G5_Pin GPIO_PIN_12
#define LCD_RGB_G5_GPIO_Port GPIOB
#define DCMI_D7_Pin GPIO_PIN_14
#define DCMI_D7_GPIO_Port GPIOD
#define LCD_RGB_G3_Pin GPIO_PIN_0
#define LCD_RGB_G3_GPIO_Port GPIOA
#define LCD_RGB_G2_Pin GPIO_PIN_1
#define LCD_RGB_G2_GPIO_Port GPIOA
#define I2S_WS_Pin GPIO_PIN_4
#define I2S_WS_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define LCD_RGB_B0_Pin GPIO_PIN_11
#define LCD_RGB_B0_GPIO_Port GPIOF
#define HEXASPI_DQS1_Pin GPIO_PIN_3
#define HEXASPI_DQS1_GPIO_Port GPIOO
#define HEXASPI_IO10_Pin GPIO_PIN_10
#define HEXASPI_IO10_GPIO_Port GPIOP
#define LCD_DE_Pin GPIO_PIN_14
#define LCD_DE_GPIO_Port GPIOB
#define LCD_RGB_G7_Pin GPIO_PIN_15
#define LCD_RGB_G7_GPIO_Port GPIOB
#define I2S_MCK_Pin GPIO_PIN_3
#define I2S_MCK_GPIO_Port GPIOA
#define I2S_CK_Pin GPIO_PIN_5
#define I2S_CK_GPIO_Port GPIOA
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LCD_RGB_G1_Pin GPIO_PIN_15
#define LCD_RGB_G1_GPIO_Port GPIOF
#define HEXASPI_IO12_Pin GPIO_PIN_12
#define HEXASPI_IO12_GPIO_Port GPIOP
#define HEXASPI_IO14_Pin GPIO_PIN_14
#define HEXASPI_IO14_GPIO_Port GPIOP
#define HEXASPI_IO2_Pin GPIO_PIN_2
#define HEXASPI_IO2_GPIO_Port GPIOP
#define HEXASPI_IO5_Pin GPIO_PIN_5
#define HEXASPI_IO5_GPIO_Port GPIOP
#define HEXASPI_DQS0_Pin GPIO_PIN_2
#define HEXASPI_DQS0_GPIO_Port GPIOO
#define HEXASPI_IO1_Pin GPIO_PIN_1
#define HEXASPI_IO1_GPIO_Port GPIOP
#define LCD_RGB_G6_Pin GPIO_PIN_11
#define LCD_RGB_G6_GPIO_Port GPIOB
#define LCD_RGB_G4_Pin GPIO_PIN_13
#define LCD_RGB_G4_GPIO_Port GPIOB
#define LCD_RGB_B7_Pin GPIO_PIN_6
#define LCD_RGB_B7_GPIO_Port GPIOA
#define RMII_TXD0_Pin GPIO_PIN_0
#define RMII_TXD0_GPIO_Port GPIOB
#define UCPD1_VSENSE_Pin GPIO_PIN_14
#define UCPD1_VSENSE_GPIO_Port GPIOF
#define HEXASPI_IO11_Pin GPIO_PIN_11
#define HEXASPI_IO11_GPIO_Port GPIOP
#define HEXASPI_IO15_Pin GPIO_PIN_15
#define HEXASPI_IO15_GPIO_Port GPIOP
#define HEXASPI_IO3_Pin GPIO_PIN_3
#define HEXASPI_IO3_GPIO_Port GPIOP
#define HEXASPI_IO0_Pin GPIO_PIN_0
#define HEXASPI_IO0_GPIO_Port GPIOP
#define HEXASPI_IO7_Pin GPIO_PIN_7
#define HEXASPI_IO7_GPIO_Port GPIOP
#define HEXASPI_IO8_Pin GPIO_PIN_8
#define HEXASPI_IO8_GPIO_Port GPIOP
#define RMII_RX_ER_Pin GPIO_PIN_10
#define RMII_RX_ER_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_1
#define RMII_TXD1_GPIO_Port GPIOB
#define UCPD1_ISENSE_Pin GPIO_PIN_12
#define UCPD1_ISENSE_GPIO_Port GPIOF
#define HEXASPI_IO13_Pin GPIO_PIN_13
#define HEXASPI_IO13_GPIO_Port GPIOP
#define HEXASPI_IO4_Pin GPIO_PIN_4
#define HEXASPI_IO4_GPIO_Port GPIOP
#define HEXASPI_CLK_Pin GPIO_PIN_4
#define HEXASPI_CLK_GPIO_Port GPIOO
#define HEXASPI_IO6_Pin GPIO_PIN_6
#define HEXASPI_IO6_GPIO_Port GPIOP
#define HEXASPI_IO9_Pin GPIO_PIN_9
#define HEXASPI_IO9_GPIO_Port GPIOP

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
