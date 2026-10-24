/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP IO expander: CH422
 *
 * CH422 (WCH) I2C GPIO expander in IO-expansion mode.
 *
 * Hardware summary:
 *  - 4x open-drain/push-pull outputs: OC0..OC3 (output-only)
 *  - 8x quasi-bidirectional IO: IO0..IO7 (input or output, write 1 = input)
 *  - No hardware direction register; direction is emulated in software.
 *  - CH422 uses 4 fixed 7-bit I2C addresses internally:
 *      0x24 = system setup, 0x23 = OC write, 0x38 = IO write, 0x26 = IO read
 *
 * Exposed pin mapping (io_count = 12):
 *  - expander pins 0..3  -> OC0..OC3 (output-only)
 *  - expander pins 4..11 -> IO0..IO7  (input or output)
 *
 * Use with the generic API:
 *  `esp_io_expander_set_dir()`, `esp_io_expander_set_level()`,
 *  `esp_io_expander_get_level()`, `esp_io_expander_print_state()`, ...
 * which is the same API used by TCA9554, so LCD / touch drivers built on
 * `esp_io_expander` work unchanged.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_IO_EXPANDER_CH422_IO_COUNT       (12)

/* Bit masks in the 12-bit expander value */
#define ESP_IO_EXPANDER_CH422_OC_MASK        (0x00F) /*!< pins 0..3  (OC0..OC3) */
#define ESP_IO_EXPANDER_CH422_IO_MASK        (0xFF0) /*!< pins 4..11 (IO0..IO7) */

/**
 * @brief Create a CH422 IO expander object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from `i2c_new_master_bus()`
 * @param[in]  dev_addr   Unused, kept for API symmetry with other expanders
 *                        (e.g. `esp_io_expander_new_i2c_tca9554`). CH422 uses
 *                        fixed internal addresses (0x23/0x24/0x38/0x26), so
 *                        any value passed here is ignored.
 * @param[out] handle_ret Handle to created IO expander object
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_new_i2c_ch422(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
                                        esp_io_expander_handle_t *handle_ret);

#ifdef __cplusplus
}
#endif
