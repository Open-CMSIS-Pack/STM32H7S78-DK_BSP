/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 20/06/2024 18:09:42
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated with a generator out of the
 *               STM32CubeMX project and its generated files (DO NOT EDIT!)
 ******************************************************************************/

#ifndef MX_DEVICE_H__
#define MX_DEVICE_H__

/* MX_Device.h version */
#define MX_DEVICE_VERSION                       0x01000000


/*------------------------------ ETH            -----------------------------*/
#define MX_ETH                                  1

/* Pins */

/* ETH_MDC */
#define MX_ETH_MDC_Pin                          PC1
#define MX_ETH_MDC_GPIO_Pin                     GPIO_PIN_1
#define MX_ETH_MDC_GPIOx                        GPIOC
#define MX_ETH_MDC_GPIO_Mode                    GPIO_MODE_AF_PP
#define MX_ETH_MDC_GPIO_PuPd                    GPIO_NOPULL
#define MX_ETH_MDC_GPIO_Speed                   GPIO_SPEED_FREQ_LOW
#define MX_ETH_MDC_GPIO_AF                      GPIO_AF11_ETH

/* ETH_MDIO */
#define MX_ETH_MDIO_Pin                         PA2
#define MX_ETH_MDIO_GPIO_Pin                    GPIO_PIN_2
#define MX_ETH_MDIO_GPIOx                       GPIOA
#define MX_ETH_MDIO_GPIO_Mode                   GPIO_MODE_AF_PP
#define MX_ETH_MDIO_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_MDIO_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_ETH_MDIO_GPIO_AF                     GPIO_AF11_ETH

/* ETH_MII_RX_ER */
#define MX_ETH_MII_RX_ER_Pin                    PB10
#define MX_ETH_MII_RX_ER_GPIO_Pin               GPIO_PIN_10
#define MX_ETH_MII_RX_ER_GPIOx                  GPIOB
#define MX_ETH_MII_RX_ER_GPIO_Mode              GPIO_MODE_AF_PP
#define MX_ETH_MII_RX_ER_GPIO_PuPd              GPIO_NOPULL
#define MX_ETH_MII_RX_ER_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_ETH_MII_RX_ER_GPIO_AF                GPIO_AF11_ETH

/* ETH_RMII_CRS_DV */
#define MX_ETH_RMII_CRS_DV_Pin                  PA7
#define MX_ETH_RMII_CRS_DV_GPIO_Pin             GPIO_PIN_7
#define MX_ETH_RMII_CRS_DV_GPIOx                GPIOA
#define MX_ETH_RMII_CRS_DV_GPIO_Mode            GPIO_MODE_AF_PP
#define MX_ETH_RMII_CRS_DV_GPIO_PuPd            GPIO_NOPULL
#define MX_ETH_RMII_CRS_DV_GPIO_Speed           GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_CRS_DV_GPIO_AF              GPIO_AF11_ETH

/* ETH_RMII_REF_CLK */
#define MX_ETH_RMII_REF_CLK_Pin                 PD7
#define MX_ETH_RMII_REF_CLK_GPIO_Pin            GPIO_PIN_7
#define MX_ETH_RMII_REF_CLK_GPIOx               GPIOD
#define MX_ETH_RMII_REF_CLK_GPIO_Mode           GPIO_MODE_AF_PP
#define MX_ETH_RMII_REF_CLK_GPIO_PuPd           GPIO_NOPULL
#define MX_ETH_RMII_REF_CLK_GPIO_Speed          GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_REF_CLK_GPIO_AF             GPIO_AF4_ETH

/* ETH_RMII_RXD0 */
#define MX_ETH_RMII_RXD0_Pin                    PC4
#define MX_ETH_RMII_RXD0_GPIO_Pin               GPIO_PIN_4
#define MX_ETH_RMII_RXD0_GPIOx                  GPIOC
#define MX_ETH_RMII_RXD0_GPIO_Mode              GPIO_MODE_AF_PP
#define MX_ETH_RMII_RXD0_GPIO_PuPd              GPIO_NOPULL
#define MX_ETH_RMII_RXD0_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_RXD0_GPIO_AF                GPIO_AF11_ETH

/* ETH_RMII_RXD1 */
#define MX_ETH_RMII_RXD1_Pin                    PC5
#define MX_ETH_RMII_RXD1_GPIO_Pin               GPIO_PIN_5
#define MX_ETH_RMII_RXD1_GPIOx                  GPIOC
#define MX_ETH_RMII_RXD1_GPIO_Mode              GPIO_MODE_AF_PP
#define MX_ETH_RMII_RXD1_GPIO_PuPd              GPIO_NOPULL
#define MX_ETH_RMII_RXD1_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_RXD1_GPIO_AF                GPIO_AF11_ETH

/* ETH_RMII_TXD0 */
#define MX_ETH_RMII_TXD0_Pin                    PB0
#define MX_ETH_RMII_TXD0_GPIO_Pin               GPIO_PIN_0
#define MX_ETH_RMII_TXD0_GPIOx                  GPIOB
#define MX_ETH_RMII_TXD0_GPIO_Mode              GPIO_MODE_AF_PP
#define MX_ETH_RMII_TXD0_GPIO_PuPd              GPIO_NOPULL
#define MX_ETH_RMII_TXD0_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_TXD0_GPIO_AF                GPIO_AF11_ETH

/* ETH_RMII_TXD1 */
#define MX_ETH_RMII_TXD1_Pin                    PB1
#define MX_ETH_RMII_TXD1_GPIO_Pin               GPIO_PIN_1
#define MX_ETH_RMII_TXD1_GPIOx                  GPIOB
#define MX_ETH_RMII_TXD1_GPIO_Mode              GPIO_MODE_AF_PP
#define MX_ETH_RMII_TXD1_GPIO_PuPd              GPIO_NOPULL
#define MX_ETH_RMII_TXD1_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_TXD1_GPIO_AF                GPIO_AF11_ETH

/* ETH_RMII_TX_EN */
#define MX_ETH_RMII_TX_EN_Pin                   PG11
#define MX_ETH_RMII_TX_EN_GPIO_Pin              GPIO_PIN_11
#define MX_ETH_RMII_TX_EN_GPIOx                 GPIOG
#define MX_ETH_RMII_TX_EN_GPIO_Mode             GPIO_MODE_AF_PP
#define MX_ETH_RMII_TX_EN_GPIO_PuPd             GPIO_NOPULL
#define MX_ETH_RMII_TX_EN_GPIO_Speed            GPIO_SPEED_FREQ_LOW
#define MX_ETH_RMII_TX_EN_GPIO_AF               GPIO_AF11_ETH

/*------------------------------ I2C1           -----------------------------*/
#define MX_I2C1                                 1

/* Filter Settings */
#define MX_I2C1_ANF_ENABLE                      1
#define MX_I2C1_DNF                             0

/* Pins */

/* I2C1_SCL */
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SCL_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1

/* I2C1_SDA */
#define MX_I2C1_SDA_Pin                         PB9
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_9
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SDA_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1

/*------------------------------ SDMMC1         -----------------------------*/
#define MX_SDMMC1                               1

/* Pins */

/* SDMMC1_CK */
#define MX_SDMMC1_CK_Pin                        PC12
#define MX_SDMMC1_CK_GPIO_Pin                   GPIO_PIN_12
#define MX_SDMMC1_CK_GPIOx                      GPIOC
#define MX_SDMMC1_CK_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_CK_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_CK_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_CK_GPIO_AF                    GPIO_AF11_SDMMC1

/* SDMMC1_CMD */
#define MX_SDMMC1_CMD_Pin                       PD2
#define MX_SDMMC1_CMD_GPIO_Pin                  GPIO_PIN_2
#define MX_SDMMC1_CMD_GPIOx                     GPIOD
#define MX_SDMMC1_CMD_GPIO_Mode                 GPIO_MODE_AF_PP
#define MX_SDMMC1_CMD_GPIO_PuPd                 GPIO_NOPULL
#define MX_SDMMC1_CMD_GPIO_Speed                GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_CMD_GPIO_AF                   GPIO_AF11_SDMMC1

/* SDMMC1_D0 */
#define MX_SDMMC1_D0_Pin                        PC8
#define MX_SDMMC1_D0_GPIO_Pin                   GPIO_PIN_8
#define MX_SDMMC1_D0_GPIOx                      GPIOC
#define MX_SDMMC1_D0_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D0_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D0_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D0_GPIO_AF                    GPIO_AF11_SDMMC1

/* SDMMC1_D1 */
#define MX_SDMMC1_D1_Pin                        PC9
#define MX_SDMMC1_D1_GPIO_Pin                   GPIO_PIN_9
#define MX_SDMMC1_D1_GPIOx                      GPIOC
#define MX_SDMMC1_D1_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D1_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D1_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D1_GPIO_AF                    GPIO_AF11_SDMMC1

/* SDMMC1_D2 */
#define MX_SDMMC1_D2_Pin                        PC10
#define MX_SDMMC1_D2_GPIO_Pin                   GPIO_PIN_10
#define MX_SDMMC1_D2_GPIOx                      GPIOC
#define MX_SDMMC1_D2_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D2_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D2_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D2_GPIO_AF                    GPIO_AF12_SDMMC1

/* SDMMC1_D3 */
#define MX_SDMMC1_D3_Pin                        PC11
#define MX_SDMMC1_D3_GPIO_Pin                   GPIO_PIN_11
#define MX_SDMMC1_D3_GPIOx                      GPIOC
#define MX_SDMMC1_D3_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SDMMC1_D3_GPIO_PuPd                  GPIO_NOPULL
#define MX_SDMMC1_D3_GPIO_Speed                 GPIO_SPEED_FREQ_HIGH
#define MX_SDMMC1_D3_GPIO_AF                    GPIO_AF11_SDMMC1

/*------------------------------ SPI4           -----------------------------*/
#define MX_SPI4                                 1

/* Pins */

/* SPI4_MISO */
#define MX_SPI4_MISO_Pin                        PE13
#define MX_SPI4_MISO_GPIO_Pin                   GPIO_PIN_13
#define MX_SPI4_MISO_GPIOx                      GPIOE
#define MX_SPI4_MISO_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SPI4_MISO_GPIO_PuPd                  GPIO_NOPULL
#define MX_SPI4_MISO_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_SPI4_MISO_GPIO_AF                    GPIO_AF5_SPI4

/* SPI4_MOSI */
#define MX_SPI4_MOSI_Pin                        PE6
#define MX_SPI4_MOSI_GPIO_Pin                   GPIO_PIN_6
#define MX_SPI4_MOSI_GPIOx                      GPIOE
#define MX_SPI4_MOSI_GPIO_Mode                  GPIO_MODE_AF_PP
#define MX_SPI4_MOSI_GPIO_PuPd                  GPIO_NOPULL
#define MX_SPI4_MOSI_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_SPI4_MOSI_GPIO_AF                    GPIO_AF5_SPI4

/* SPI4_SCK */
#define MX_SPI4_SCK_Pin                         PE12
#define MX_SPI4_SCK_GPIO_Pin                    GPIO_PIN_12
#define MX_SPI4_SCK_GPIOx                       GPIOE
#define MX_SPI4_SCK_GPIO_Mode                   GPIO_MODE_AF_PP
#define MX_SPI4_SCK_GPIO_PuPd                   GPIO_NOPULL
#define MX_SPI4_SCK_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_SPI4_SCK_GPIO_AF                     GPIO_AF5_SPI4

/*------------------------------ UART4          -----------------------------*/
#define MX_UART4                                1

/* Pins */

/* UART4_RX */
#define MX_UART4_RX_Pin                         PD0
#define MX_UART4_RX_GPIO_Pin                    GPIO_PIN_0
#define MX_UART4_RX_GPIOx                       GPIOD
#define MX_UART4_RX_GPIO_Mode                   GPIO_MODE_AF_PP
#define MX_UART4_RX_GPIO_PuPd                   GPIO_NOPULL
#define MX_UART4_RX_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_UART4_RX_GPIO_AF                     GPIO_AF8_UART4

/* UART4_TX */
#define MX_UART4_TX_Pin                         PD1
#define MX_UART4_TX_GPIO_Pin                    GPIO_PIN_1
#define MX_UART4_TX_GPIOx                       GPIOD
#define MX_UART4_TX_GPIO_Mode                   GPIO_MODE_AF_PP
#define MX_UART4_TX_GPIO_PuPd                   GPIO_NOPULL
#define MX_UART4_TX_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_UART4_TX_GPIO_AF                     GPIO_AF8_UART4

/*------------------------------ USB_OTG_FS     -----------------------------*/
#define MX_USB_OTG_FS                           1

/* Handle */
#define MX_USB_OTG_FS_HANDLE                    hpcd_USB_OTG_FS

/* Virtual mode */
#define MX_USB_OTG_FS_VM                        Device_Only
#define MX_USB_OTG_FS_Device_Only               1


/*------------------------------ USB_OTG_HS     -----------------------------*/
#define MX_USB_OTG_HS                           1

/* Virtual mode */
#define MX_USB_OTG_HS_VM                        Host_HS
#define MX_USB_OTG_HS_Host_HS                   1


#endif  /* MX_DEVICE_H__ */
