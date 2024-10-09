/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 17/03/2025 12:35:28
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated with a generator out of the
 *               STM32CubeMX project and its generated files (DO NOT EDIT!)
 ******************************************************************************/

#ifndef MX_DEVICE_H__
#define MX_DEVICE_H__

/* MX_Device.h version */
#define MX_DEVICE_VERSION                       0x01000000


/*------------------------------ SPI4           -----------------------------*/
#define MX_SPI4                                 1

/* Peripheral Clock Frequency */
#define MX_SPI4_PERIPH_CLOCK_FREQ               150000000

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

#endif  /* MX_DEVICE_H__ */
