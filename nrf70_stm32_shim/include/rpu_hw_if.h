/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief RPU hardware interface prototypes for STM32 bare-metal shim.
 *        Mirrors nrf70_zephyr_shim/include/rpu_hw_if.h.
 */

#ifndef __STM32_RPU_HW_IF_H__
#define __STM32_RPU_HW_IF_H__

#include <stdint.h>

/* Memory block identifiers (mirrors Zephyr shim enum) */
enum {
    SYSBUS = 0,
    EXT_SYS_BUS,
    PBUS,
    PKTRAM,
    GRAM,
    LMAC_ROM,
    LMAC_RET_RAM,
    LMAC_SRC_RAM,
    UMAC_ROM,
    UMAC_RET_RAM,
    UMAC_SRC_RAM,
    NUM_MEM_BLOCKS
};

extern char blk_name[][15];
extern uint32_t rpu_7002_memmap[][3];

/* Initialise GPIO outputs (BUCKEN, IOVDD_CTRL, CS) and configure EXTI for HOST_IRQ */
int rpu_init(void);

/* Power on the nRF70: assert BUCKEN → IOVDD_CTRL → wakeup → clocks */
int rpu_enable(void);

/* Power off the nRF70 */
int rpu_disable(void);

/* Bulk memory read / write via SPI */
int rpu_read(unsigned int addr, void *data, int len);
int rpu_write(unsigned int addr, const void *data, int len);

/* Sleep / wakeup helpers */
int rpu_sleep(void);
int rpu_wakeup(void);
int rpu_sleep_status(void);

/* Individual RDSR/WRSR2 operations */
int rpu_wrsr2(uint8_t data);
int rpu_rdsr2(void);
int rpu_rdsr1(void);

/* Enable RPU clocks */
int rpu_clks_on(void);

/**
 * @brief Configure HOST_IRQ EXTI and store driver callback.
 *
 * @param callbk_data  Opaque pointer passed back to callbk_fn.
 * @param callbk_fn    Driver interrupt handler; called from EXTI ISR context.
 * @return 0 on success, negative error code on failure.
 */
int rpu_irq_config(void *callbk_data, int (*callbk_fn)(void *callbk_data));

/** Disable the HOST_IRQ EXTI */
int rpu_irq_remove(void);

/* Register read/write via SPI opcode */
int rpu_qspi_read_reg(uint8_t reg_addr, uint8_t *reg_value);
int rpu_qspi_write_reg(uint8_t reg_addr, uint8_t reg_value);

#endif /* __STM32_RPU_HW_IF_H__ */
