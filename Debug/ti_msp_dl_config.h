/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (49999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                            1000000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART1
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART1_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOB
#define GPIO_UART_0_TX_PORT                                                GPIOB
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)




/* Defines for SPI_LSM6DS3TRC */
#define SPI_LSM6DS3TRC_INST                                                SPI1
#define SPI_LSM6DS3TRC_INST_IRQHandler                          SPI1_IRQHandler
#define SPI_LSM6DS3TRC_INST_INT_IRQN                              SPI1_INT_IRQn
#define GPIO_SPI_LSM6DS3TRC_PICO_PORT                                     GPIOA
#define GPIO_SPI_LSM6DS3TRC_PICO_PIN                             DL_GPIO_PIN_18
#define GPIO_SPI_LSM6DS3TRC_IOMUX_PICO                          (IOMUX_PINCM40)
#define GPIO_SPI_LSM6DS3TRC_IOMUX_PICO_FUNC          IOMUX_PINCM40_PF_SPI1_PICO
#define GPIO_SPI_LSM6DS3TRC_POCI_PORT                                     GPIOA
#define GPIO_SPI_LSM6DS3TRC_POCI_PIN                             DL_GPIO_PIN_16
#define GPIO_SPI_LSM6DS3TRC_IOMUX_POCI                          (IOMUX_PINCM38)
#define GPIO_SPI_LSM6DS3TRC_IOMUX_POCI_FUNC          IOMUX_PINCM38_PF_SPI1_POCI
/* GPIO configuration for SPI_LSM6DS3TRC */
#define GPIO_SPI_LSM6DS3TRC_SCLK_PORT                                     GPIOA
#define GPIO_SPI_LSM6DS3TRC_SCLK_PIN                             DL_GPIO_PIN_17
#define GPIO_SPI_LSM6DS3TRC_IOMUX_SCLK                          (IOMUX_PINCM39)
#define GPIO_SPI_LSM6DS3TRC_IOMUX_SCLK_FUNC          IOMUX_PINCM39_PF_SPI1_SCLK
/* Defines for SPI_LED_DRV */
#define SPI_LED_DRV_INST                                                   SPI0
#define SPI_LED_DRV_INST_IRQHandler                             SPI0_IRQHandler
#define SPI_LED_DRV_INST_INT_IRQN                                 SPI0_INT_IRQn
#define GPIO_SPI_LED_DRV_PICO_PORT                                        GPIOA
#define GPIO_SPI_LED_DRV_PICO_PIN                                 DL_GPIO_PIN_5
#define GPIO_SPI_LED_DRV_IOMUX_PICO                             (IOMUX_PINCM10)
#define GPIO_SPI_LED_DRV_IOMUX_PICO_FUNC             IOMUX_PINCM10_PF_SPI0_PICO
#define GPIO_SPI_LED_DRV_POCI_PORT                                        GPIOA
#define GPIO_SPI_LED_DRV_POCI_PIN                                DL_GPIO_PIN_10
#define GPIO_SPI_LED_DRV_IOMUX_POCI                             (IOMUX_PINCM21)
#define GPIO_SPI_LED_DRV_IOMUX_POCI_FUNC             IOMUX_PINCM21_PF_SPI0_POCI
/* GPIO configuration for SPI_LED_DRV */
#define GPIO_SPI_LED_DRV_SCLK_PORT                                        GPIOA
#define GPIO_SPI_LED_DRV_SCLK_PIN                                 DL_GPIO_PIN_6
#define GPIO_SPI_LED_DRV_IOMUX_SCLK                             (IOMUX_PINCM11)
#define GPIO_SPI_LED_DRV_IOMUX_SCLK_FUNC             IOMUX_PINCM11_PF_SPI0_SCLK



/* Defines for BAT_ADC */
#define BAT_ADC_INST                                                        ADC0
#define BAT_ADC_INST_IRQHandler                                  ADC0_IRQHandler
#define BAT_ADC_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define BAT_ADC_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define BAT_ADC_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define BAT_ADC_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_BAT_ADC_C0_PORT                                               GPIOA
#define GPIO_BAT_ADC_C0_PIN                                       DL_GPIO_PIN_27
#define GPIO_BAT_ADC_IOMUX_C0                                    (IOMUX_PINCM60)
#define GPIO_BAT_ADC_IOMUX_C0_FUNC                (IOMUX_PINCM60_PF_UNCONNECTED)



/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (1)
#define UART_0_INST_DMA_TRIGGER_0                            (DMA_UART1_TX_TRIG)
/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define UART_0_INST_DMA_TRIGGER_1                            (DMA_UART1_RX_TRIG)


/* Port definition for Pin Group GPIO_LSM6DS3TRC */
#define GPIO_LSM6DS3TRC_PORT                                             (GPIOA)

/* Defines for PIN_LSM6DS3TRC_CS: GPIOA.15 with pinCMx 37 on package pin 30 */
#define GPIO_LSM6DS3TRC_PIN_LSM6DS3TRC_CS_PIN                   (DL_GPIO_PIN_15)
#define GPIO_LSM6DS3TRC_PIN_LSM6DS3TRC_CS_IOMUX                  (IOMUX_PINCM37)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN_LED: GPIOB.8 with pinCMx 25 on package pin 22 */
#define LED_PIN_LED_PIN                                          (DL_GPIO_PIN_8)
#define LED_PIN_LED_IOMUX                                        (IOMUX_PINCM25)
/* Port definition for Pin Group GPIO_LED_DRV */
#define GPIO_LED_DRV_PORT                                                (GPIOA)

/* Defines for PIN_LED_CSB: GPIOA.4 with pinCMx 9 on package pin 10 */
#define GPIO_LED_DRV_PIN_LED_CSB_PIN                             (DL_GPIO_PIN_4)
#define GPIO_LED_DRV_PIN_LED_CSB_IOMUX                            (IOMUX_PINCM9)
/* Port definition for Pin Group GPIO_PWR */
#define GPIO_PWR_PORT                                                    (GPIOA)

/* Defines for PIN_PWR_HOLD: GPIOA.22 with pinCMx 47 on package pin 40 */
#define GPIO_PWR_PIN_PWR_HOLD_PIN                               (DL_GPIO_PIN_22)
#define GPIO_PWR_PIN_PWR_HOLD_IOMUX                              (IOMUX_PINCM47)
/* Port definition for Pin Group GPIO_USERKEY */
#define GPIO_USERKEY_PORT                                                (GPIOA)

/* Defines for PIN_USER3: GPIOA.23 with pinCMx 53 on package pin 43 */
#define GPIO_USERKEY_PIN_USER3_PIN                              (DL_GPIO_PIN_23)
#define GPIO_USERKEY_PIN_USER3_IOMUX                             (IOMUX_PINCM53)
/* Defines for PIN_USER2: GPIOA.24 with pinCMx 54 on package pin 44 */
#define GPIO_USERKEY_PIN_USER2_PIN                              (DL_GPIO_PIN_24)
#define GPIO_USERKEY_PIN_USER2_IOMUX                             (IOMUX_PINCM54)
/* Defines for PIN_USER1: GPIOA.25 with pinCMx 55 on package pin 45 */
#define GPIO_USERKEY_PIN_USER1_PIN                              (DL_GPIO_PIN_25)
#define GPIO_USERKEY_PIN_USER1_IOMUX                             (IOMUX_PINCM55)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_LSM6DS3TRC_init(void);
void SYSCFG_DL_SPI_LED_DRV_init(void);
void SYSCFG_DL_BAT_ADC_init(void);
void SYSCFG_DL_DMA_init(void);



bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
