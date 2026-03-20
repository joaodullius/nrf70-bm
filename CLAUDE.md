# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

nRF70 BM (Bare Metal) is a portable Wi-Fi driver library for the nRF70 Series chipsets, designed to run without a full RTOS. It provides a hardware abstraction that can be ported to any platform. The `nrf70_zephyr_shim` is the reference implementation using Zephyr RTOS.

## Build Commands

This project uses CMake + West (Zephyr meta-tool). Zephyr v4.1.0 is pinned via `west.yml`.

**Initial setup (west-based, for nRF boards):**
```sh
west init -m https://github.com/nrfconnect/nrf70-bm
cd nrf70-bm
west update
```

**Build a sample:**
```sh
west build -b <board_name> samples/scan_bm -- -DCONFIG_NRF70_BOARD_TYPE_DK=y
west build -b <board_name> samples/radio_test_bm -- -DCONFIG_NRF70_BOARD_TYPE_DK=y
west build -b <board_name> samples/scan_rt_bm -- -DCONFIG_NRF70_BOARD_TYPE_DK=y
```

Board type options: `CONFIG_NRF70_BOARD_TYPE_DK` (default), `CONFIG_NRF70_BOARD_TYPE_EK`, `CONFIG_NRF70_BOARD_TYPE_CUSTOM`

**Flash:**
```sh
west flash
```

**Run tests:**
```sh
west build -b <board_name> tests/bustest
west flash
# Tests run automatically on boot and report via UART console
# bustest uses the Ztest framework; covers RDSR0/1/2, WRSR2, system bus, peripheral bus, and data RAM
```

**Build docs (Linux only):**
```sh
pip install -r nrf70_bm_lib/docs/requirements.txt
sudo apt install doxygen
./build-docs.sh
# Output: nrf70_bm_lib/docs/build/html/
```

## Architecture

The driver is structured in distinct layers:

```
Application (samples/)
    ↓
nRF70 BM Library (nrf70_bm_lib/)
    ↓
nRF Wi-Fi OSAL — OS-agnostic driver core (nrf_wifi/ submodule)
    ↓
Platform Shim (nrf70_zephyr_shim/ for Zephyr; implement your own for other platforms)
    ↓
Zephyr RTOS + sdk-nrfxlib (firmware blobs)
```

**`nrf70_bm_lib/`** — The main portable library (v1.1.0-rc1). Contains two operation modes selected at build time via Kconfig; the mode controls which source files compile, which firmware binary links, and which header files expose the API:
- `CONFIG_NRF70_BM_SCAN_ONLY` → uses `source/system/` for Wi-Fi scanning
- `CONFIG_NRF70_BM_RADIO_TEST` → uses `source/radio_test/` for RF testing
- `source/common/nrf70_bm_core.c` handles shared initialization (firmware loading, TX power config)

**`nrf70_zephyr_shim/`** — Reference platform shim for Zephyr. When porting to a new platform, this is the directory to replicate. Key abstractions: QSPI/SPI bus (`source/bus/`), OS primitives like timers and work queues (`source/os/`), and RPU hardware interface (`source/platform/`). Bus type is auto-selected via devicetree; QSPI is preferred.

**`samples/scan_rt_bm/`** — Combines both scan and radio test modes with runtime switching between them.

**`nrf_wifi/`** (git submodule) — OS-agnostic Wi-Fi driver core from Nordic. Do not modify directly; update via west manifest.

**`sdk-nrfxlib/`** (git submodule) — Contains nRF70 firmware patch blobs under `nrf_wifi/`. Do not modify directly.

## Key Kconfig Options

| Option | Description |
|--------|-------------|
| `CONFIG_NRF70_BM_SCAN_ONLY` | Enables scan/system mode |
| `CONFIG_NRF70_BM_RADIO_TEST` | Enables radio test mode |
| `CONFIG_NRF70_ON_QSPI` | Use QSPI bus (preferred, auto-selected from DT) |
| `CONFIG_NRF70_ON_SPI` | Use SPI bus (auto-selected from DT) |
| `CONFIG_NRF70_BOARD_TYPE_DK/EK/CUSTOM` | Board type selection |
| `CONFIG_NRF70_OTP_MAC_ADDRESS` | Use OTP MAC address (default) |
| `CONFIG_NRF70_FIXED_MAC_ADDRESS` | Override with fixed MAC |
| `CONFIG_NRF70_RANDOM_MAC_ADDRESS` | Use randomly generated MAC |
| `CONFIG_NRF_WIFI_LOW_POWER` | Low power mode (default: enabled) |
| `CONFIG_HEAP_MEM_POOL_SIZE` | Heap for shim (default: 30000 bytes) |
| `CONFIG_NRF70_WORKQ_STACK_SIZE` | General workqueue stack (default: 4096) |

## Porting to a New Platform

Use `nrf70_zephyr_shim/` as the reference. You need to implement:
1. Bus interface (QSPI or SPI) — see `nrf70_zephyr_shim/source/bus/`
2. OS primitives (timers, work queues, memory) — see `nrf70_zephyr_shim/source/os/`
3. RPU hardware interface — see `nrf70_zephyr_shim/source/platform/`
4. TX power ceiling header for custom boards — reference `nrf70_bm_lib/include/nrf70_tx_pwr_ceil_dk.h`

Set `CONFIG_NRF70_BOARD_TYPE_CUSTOM` when using a custom board.

## IDE

VS Code with the nRF Connect extension is pre-configured (`.vscode/settings.json` points to `samples/scan_bm` as the default application). Use the nRF Connect extension to build, flash, and debug without manually invoking west.
