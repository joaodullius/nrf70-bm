/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief STM32 HAL SPI driver for the nRF70 BM shim.
 *        Mirrors nrf70_zephyr_shim/source/bus/spi_if.c.
 *
 * Protocol summary
 * ----------------
 *  Write : opcode 0x02 + addr[23:16]|0x80 + addr[15:8] + addr[7:0] + data
 *  Read  : opcode 0x0B + addr[23:16]      + addr[15:8] + addr[7:0] + dummy + data
 *  HL read (addr < 0x0C0000): same as Read but with extra discard words equal to
 *          g_spi_slave_latency * 4 bytes appended before the payload.
 *  Reg   : opcode + value; response in byte[1] of a 6-byte read-back.
 *
 * All transfers are full-duplex via HAL_SPI_TransmitReceive().
 * CS is bit-banged (active LOW) using HAL_GPIO_WritePin().
 */

#include <string.h>
#include <stdio.h>

#include "autoconf.h"
#include "nrf70_bm_board_config.h"
#include "spi_if.h"
#include "qspi_if.h"  /* RPU_AWAKE_BIT, RPU_WAKEUP_NOW */

/* Slave latency: 0 = 8 MHz bus (no extra dummy words), 1 = 16+ MHz (one extra word) */
unsigned char g_spi_slave_latency = 0;

/* -------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

static inline void cs_assert(void)
{
    HAL_GPIO_WritePin(NRF70_CS_PORT, NRF70_CS_PIN, GPIO_PIN_RESET);
}

static inline void cs_deassert(void)
{
    HAL_GPIO_WritePin(NRF70_CS_PORT, NRF70_CS_PIN, GPIO_PIN_SET);
}

/**
 * Full-duplex SPI transaction helper.
 * Sends @p tx_len bytes from @p tx_buf, simultaneously receives into @p rx_buf.
 * If @p rx_buf is NULL the received bytes are discarded.
 */
static int spi_transceive(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (rx_buf == NULL) {
        /* Transmit-only — provide a dummy RX scratch buffer on stack */
        uint8_t scratch[4];
        uint16_t remaining = len;

        while (remaining > 0) {
            uint16_t chunk = (remaining > sizeof(scratch)) ? sizeof(scratch) : remaining;
            status = HAL_SPI_TransmitReceive(&NRF70_SPI_HANDLE,
                                             (uint8_t *)tx_buf,
                                             scratch, chunk,
                                             NRF70_SPI_TIMEOUT_MS);
            if (status != HAL_OK) {
                return -1;
            }
            tx_buf    += chunk;
            remaining -= chunk;
        }
    } else {
        status = HAL_SPI_TransmitReceive(&NRF70_SPI_HANDLE,
                                         (uint8_t *)tx_buf,
                                         rx_buf, len,
                                         NRF70_SPI_TIMEOUT_MS);
        if (status != HAL_OK) {
            return -1;
        }
    }

    return 0;
}

/** Alignment / size check (mirrors Zephyr spim_addr_check) */
static void spim_addr_check(unsigned int addr, const void *data, unsigned int len)
{
    if ((addr % 4 != 0) || (((unsigned int)data) % 4 != 0) || (len % 4 != 0)) {
        printf("[nrf70 spi] Unaligned: addr=0x%x data=%p len=%u\r\n",
               addr, data, len);
    }
}

/* -------------------------------------------------------------------------
 * Write path  (opcode 0x02 PP — Page Program)
 * Header: {0x02, addr[23:16]|0x80, addr[15:8], addr[7:0]}
 * -------------------------------------------------------------------------*/
static int spim_xfer_tx(unsigned int addr, const void *data, unsigned int len)
{
    uint8_t hdr[4] = {
        0x02,
        (uint8_t)(((addr >> 16) & 0xFF) | 0x80),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)
    };
    int ret;

    cs_assert();

    ret = spi_transceive(hdr, NULL, sizeof(hdr));
    if (ret == 0) {
        ret = spi_transceive((const uint8_t *)data, NULL, len);
    }

    cs_deassert();
    return ret;
}

/* -------------------------------------------------------------------------
 * Read path  (opcode 0x0B FAST READ)
 * Header: {0x0B, addr[23:16], addr[15:8], addr[7:0], dummy}
 * @p discard_bytes: extra bytes to throw away before payload (HL read)
 * -------------------------------------------------------------------------*/
static int spim_xfer_rx(unsigned int addr, void *data, unsigned int len,
                         unsigned int discard_bytes)
{
    /* Maximum discard: 5-byte header + 2 extra words (8 bytes) */
    uint8_t discard[5 + 2 * 4];
    uint8_t hdr[5] = {
        0x0B,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF),
        0x00  /* dummy byte */
    };
    unsigned int hdr_and_discard = sizeof(hdr) + discard_bytes;
    int ret;

    if (hdr_and_discard > sizeof(discard)) {
        printf("[nrf70 spi] Discard too large: %u\r\n", hdr_and_discard);
        return -1;
    }

    cs_assert();

    /* Send the header while discarding the same number of received bytes */
    ret = spi_transceive(hdr, discard, sizeof(hdr));
    if (ret != 0) {
        goto out;
    }

    /* Discard extra bytes (HL latency words): transmit zeros, throw away RX */
    if (discard_bytes > 0) {
        uint8_t zeros[8] = {0};
        ret = spi_transceive(zeros, discard, discard_bytes);
        if (ret != 0) {
            goto out;
        }
    }

    /* Now receive the actual payload; TX dummy zeros */
    {
        uint8_t zeros[4] = {0};
        uint8_t *dst = (uint8_t *)data;
        unsigned int remaining = len;

        while (remaining > 0) {
            uint16_t chunk = (remaining > sizeof(zeros)) ? sizeof(zeros) : remaining;
            ret = spi_transceive(zeros, dst, chunk);
            if (ret != 0) {
                goto out;
            }
            dst       += chunk;
            remaining -= chunk;
        }
    }

out:
    cs_deassert();
    return ret;
}

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

int spim_init(void)
{
    /* CS starts deasserted */
    cs_deassert();

    /* Determine slave latency from SPI clock.
     * CubeMX configures prescaler; assume ≥16 MHz needs latency=1.
     * Adjust the threshold to match your CubeMX SPI1 prescaler setting. */
    g_spi_slave_latency = 0;  /* set to 1 if SPI clock ≥ 16 MHz */

    printf("[nrf70 spi] Initialised (latency=%d)\r\n", g_spi_slave_latency);
    return 0;
}

int spim_deinit(void)
{
    return 0;
}

int spim_write(unsigned int addr, const void *data, int len)
{
    spim_addr_check(addr, data, len);
    return spim_xfer_tx(addr, data, len);
}

int spim_read(unsigned int addr, void *data, int len)
{
    spim_addr_check(addr, data, len);
    return spim_xfer_rx(addr, data, len, 0);
}

static int spim_hl_readw(unsigned int addr, void *data)
{
    return spim_xfer_rx(addr, data, 4, 4 * g_spi_slave_latency);
}

int spim_hl_read(unsigned int addr, void *data, int len)
{
    int count = 0;

    spim_addr_check(addr, data, len);

    while (count < (len / 4)) {
        int ret = spim_hl_readw(addr + (4 * count),
                                (char *)data + (4 * count));
        if (ret) {
            return ret;
        }
        count++;
    }

    return 0;
}

int spim_read_reg(uint8_t reg_addr, uint8_t *reg_value)
{
    uint8_t tx[6] = { reg_addr, 0, 0, 0, 0, 0 };
    uint8_t rx[6] = { 0 };
    int ret;

    cs_assert();
    ret = spi_transceive(tx, rx, sizeof(tx));
    cs_deassert();

    if (ret == 0) {
        *reg_value = rx[1];
    }

    return ret;
}

int spim_write_reg(uint8_t reg_addr, uint8_t reg_value)
{
    uint8_t tx[2] = { reg_addr, reg_value };
    int ret;

    cs_assert();
    ret = spi_transceive(tx, NULL, sizeof(tx));
    cs_deassert();

    return ret;
}

int spim_cmd_rpu_wakeup_fn(uint32_t data)
{
    /* WRSR2 opcode = 0x3F, data byte triggers wakeup */
    return spim_write_reg(0x3F, (uint8_t)data);
}

int spim_cmd_sleep_rpu_fn(void)
{
    /* WRSR2 = 0x3F, value = 0x00 → sleep */
    return spim_write_reg(0x3F, 0x00);
}

int spim_wait_while_rpu_awake(void)
{
    uint8_t val = 0;
    int ret;

    for (int i = 0; i < 10; i++) {
        ret = spim_read_reg(0x1F, &val);  /* RDSR1 opcode */
        if (ret == 0 && (val & RPU_AWAKE_BIT)) {
            return val;
        }
        HAL_Delay(1);
    }

    printf("[nrf70 spi] RPU not awake after 10ms (RDSR1=0x%02x)\r\n", val);
    return -1;
}

int spi_validate_rpu_wake_writecmd(void)
{
    uint8_t val = 0;
    int ret;

    for (int i = 0; i < 10; i++) {
        ret = spim_read_reg(0x2F, &val);  /* RDSR2 opcode */
        if (ret == 0 && (val & RPU_WAKEUP_NOW)) {
            return 0;
        }
        HAL_Delay(1);
    }

    printf("[nrf70 spi] RDSR2 wakeup ACK failed (val=0x%02x)\r\n", val);
    return -1;
}
