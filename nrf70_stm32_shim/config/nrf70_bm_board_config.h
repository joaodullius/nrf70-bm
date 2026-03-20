/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Board-specific pin and peripheral assignments for STM32U073RCTX.
 *        Edit this file to match your actual hardware connections.
 *        Replaces devicetree (DTS) node configuration used by the Zephyr shim.
 *
 * Connection example (adjust to your schematic):
 *   SPI1 SCK  → PA5
 *   SPI1 MISO → PA6
 *   SPI1 MOSI → PA7
 *   CS        → PA4  (GPIO output, active-low)
 *   BUCKEN    → PC0  (GPIO output, active-high)
 *   IOVDD_CTL → PC1  (GPIO output, active-high)
 *   HOST_IRQ  → PB0  (GPIO input, rising-edge EXTI)
 */

#ifndef __NRF70_BM_BOARD_CONFIG_H__
#define __NRF70_BM_BOARD_CONFIG_H__

#include "stm32u0xx_hal.h"

/* -------------------------------------------------------------------------
 * SPI peripheral
 * The extern handle must be defined (and initialised by CubeMX) in main.c.
 * -------------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
#define NRF70_SPI_HANDLE   hspi1

/* SPI timeout for HAL_SPI_TransmitReceive (milliseconds) */
#define NRF70_SPI_TIMEOUT_MS   1000U

/* -------------------------------------------------------------------------
 * Chip-select GPIO  (active LOW — nRF70 CS is asserted low)
 * -------------------------------------------------------------------------*/
#define NRF70_CS_PORT      GPIOA
#define NRF70_CS_PIN       GPIO_PIN_4

/* -------------------------------------------------------------------------
 * HOST_IRQ GPIO  (rising-edge interrupt from nRF70)
 * Must be connected to a pin that supports EXTI on the STM32U073.
 * -------------------------------------------------------------------------*/
#define NRF70_IRQ_PORT     GPIOB
#define NRF70_IRQ_PIN      GPIO_PIN_0
/* EXTI line number matching the pin (PB0 → EXTI0) */
#define NRF70_IRQ_EXTI_LINE EXTI_LINE_0

/* -------------------------------------------------------------------------
 * BUCKEN GPIO  (active HIGH — enables the nRF70 buck regulator)
 * -------------------------------------------------------------------------*/
#define NRF70_BUCKEN_PORT  GPIOC
#define NRF70_BUCKEN_PIN   GPIO_PIN_0

/* -------------------------------------------------------------------------
 * IOVDD_CTRL GPIO  (active HIGH — enables nRF70 IO voltage)
 * -------------------------------------------------------------------------*/
#define NRF70_IOVDD_CTRL_PORT  GPIOC
#define NRF70_IOVDD_CTRL_PIN   GPIO_PIN_1

/* -------------------------------------------------------------------------
 * Power-on timing (milliseconds)
 * -------------------------------------------------------------------------*/
#define NRF70_BUCKEN_SETTLE_MS    10U
#define NRF70_IOVDD_SETTLE_MS     10U

#endif /* __NRF70_BM_BOARD_CONFIG_H__ */
