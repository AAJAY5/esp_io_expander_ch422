/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_io_expander.h"
#include "esp_io_expander_ch422.h"

/* I2C communication related */
#define I2C_TIMEOUT_MS      (1000)
#define I2C_CLK_SPEED       (400000)

/* CH422 fixed 7-bit addresses (command byte >> 1). See WCH datasheet. */
#define CH422_ADDR_SYS_SETUP    (0x24)  /* write system parameter */
#define CH422_ADDR_OC           (0x23)  /* write OC3..OC0 (low nibble) */
#define CH422_ADDR_IO_OUT       (0x38)  /* write IO7..IO0 */
#define CH422_ADDR_IO_IN        (0x26)  /* read  IO7..IO0 */

/* System parameter bits (data byte for CH422_ADDR_SYS_SETUP) */
#define CH422_SYS_IO_OE     (1 << 0)    /* IO7~IO0 output enable: 1 = allow output */
#define CH422_SYS_A_SCAN    (1 << 2)    /* 1 = 7-seg scan mode, 0 = IO expand mode (we use 0) */
#define CH422_SYS_OD_EN     (1 << 4)    /* 1 = OC pins open-drain, 0 = push-pull */
#define CH422_SYS_SLEEP     (1 << 7)

#define IO_COUNT                ESP_IO_EXPANDER_CH422_IO_COUNT  /* 12 */

/* direction: 0 = output, 1 = input (matches TCA9554 convention, dir_out_bit_zero = 1) */
#define DIR_REG_DEFAULT_VAL     (0xFF0) /* OC0-3 = output, IO0-7 = input */
#define OUT_REG_DEFAULT_VAL     (0xFFF) /* all high: OC high-Z/high, IO input w/ pull-up */
#define HIGHZ_REG_DEFAULT_VAL   (0x000) /* push-pull */

/**
 * @brief Device Structure Type
 */
typedef struct {
    esp_io_expander_t base;
    i2c_master_dev_handle_t h_sys;
    i2c_master_dev_handle_t h_oc;
    i2c_master_dev_handle_t h_io_out;
    i2c_master_dev_handle_t h_io_in;
    struct {
        uint32_t direction;
        uint32_t output;
        uint32_t highz;
    } regs;
    uint8_t sys_param;
} esp_io_expander_ch422_t;

static const char *TAG = "ch422";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_highz_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_highz_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

static esp_err_t ch422_add_dev(i2c_master_bus_handle_t bus, uint8_t addr, i2c_master_dev_handle_t *dev_handle)
{
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, dev_handle);
}

esp_err_t esp_io_expander_new_i2c_ch422(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
                                        esp_io_expander_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle_ret");
    ESP_RETURN_ON_FALSE(i2c_bus != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid i2c_bus");
    (void)dev_addr; /* CH422 addresses are fixed; kept only for API symmetry */

    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)calloc(1, sizeof(esp_io_expander_ch422_t));
    ESP_RETURN_ON_FALSE(ch422 != NULL, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    esp_err_t ret = ESP_OK;
    ch422->h_sys = NULL;
    ch422->h_oc = NULL;
    ch422->h_io_out = NULL;
    ch422->h_io_in = NULL;

    ESP_GOTO_ON_ERROR(ch422_add_dev(i2c_bus, CH422_ADDR_SYS_SETUP, &ch422->h_sys), err, TAG, "Add sys dev failed");
    ESP_GOTO_ON_ERROR(ch422_add_dev(i2c_bus, CH422_ADDR_OC, &ch422->h_oc), err, TAG, "Add OC dev failed");
    ESP_GOTO_ON_ERROR(ch422_add_dev(i2c_bus, CH422_ADDR_IO_OUT, &ch422->h_io_out), err, TAG, "Add IO-out dev failed");
    ESP_GOTO_ON_ERROR(ch422_add_dev(i2c_bus, CH422_ADDR_IO_IN, &ch422->h_io_in), err, TAG, "Add IO-in dev failed");

    ch422->base.config.io_count = IO_COUNT;
    ch422->base.config.flags.dir_out_bit_zero = 1;    /* dir bit 0 = output, 1 = input */
    ch422->base.config.flags.input_high_bit_zero = 0;
    ch422->base.config.flags.output_high_bit_zero = 0;
    ch422->base.config.flags.pullup_high_bit_zero = 0;
    ch422->base.read_input_reg = read_input_reg;
    ch422->base.write_output_reg = write_output_reg;
    ch422->base.read_output_reg = read_output_reg;
    ch422->base.write_direction_reg = write_direction_reg;
    ch422->base.read_direction_reg = read_direction_reg;
    ch422->base.write_highz_reg = write_highz_reg;
    ch422->base.read_highz_reg = read_highz_reg;
    ch422->base.del = del;
    ch422->base.reset = reset;

    ESP_GOTO_ON_ERROR(reset(&ch422->base), err, TAG, "Reset failed");

    *handle_ret = &ch422->base;
    return ESP_OK;

err:
    del(&ch422->base);
    return ret;
}

static esp_err_t write_sys_param(esp_io_expander_ch422_t *ch422)
{
    /* Always stay in IO-expand mode */
    ch422->sys_param &= ~CH422_SYS_A_SCAN;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422->h_sys, &ch422->sys_param, 1, I2C_TIMEOUT_MS),
                        TAG, "Write sys param failed");
    return ESP_OK;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid value pointer");

    uint8_t io_in = 0xFF;
    ESP_RETURN_ON_ERROR(i2c_master_receive(ch422->h_io_in, &io_in, 1, I2C_TIMEOUT_MS),
                        TAG, "Read IO input failed");

    /* OC pins are output-only: report cached output level for low nibble so
     * esp_io_expander_get_level() / print_state() stay meaningful there. */
    *value = (((uint32_t)io_in << 4) & ESP_IO_EXPANDER_CH422_IO_MASK) |
             (ch422->regs.output & ESP_IO_EXPANDER_CH422_OC_MASK);
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    value &= 0xFFF;

    uint8_t oc = (uint8_t)(value & 0x0F);
    uint8_t io = (uint8_t)((value >> 4) & 0xFF);

    /* IO outputs require IO_OE */
    if (!(ch422->sys_param & CH422_SYS_IO_OE)) {
        ch422->sys_param |= CH422_SYS_IO_OE;
        ESP_RETURN_ON_ERROR(write_sys_param(ch422), TAG, "Enable IO_OE failed");
    }

    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422->h_oc, &oc, 1, I2C_TIMEOUT_MS),
                        TAG, "Write OC failed");
    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422->h_io_out, &io, 1, I2C_TIMEOUT_MS),
                        TAG, "Write IO output failed");
    ch422->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid value pointer");
    *value = ch422->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    value &= 0xFFF;

    /* OC0-3 are output-only */
    ESP_RETURN_ON_FALSE((value & ESP_IO_EXPANDER_CH422_OC_MASK) == 0, ESP_ERR_INVALID_ARG, TAG,
                        "OC0-OC3 (pins 0-3) are output-only");

    /* Quasi-bidirectional IO: input = output latch driven high.
     * Force latch-high on every pin newly set to input. */
    uint32_t new_output = ch422->regs.output;
    uint32_t input_pins = value & ESP_IO_EXPANDER_CH422_IO_MASK;
    new_output |= input_pins;
    if (new_output != ch422->regs.output) {
        ESP_RETURN_ON_ERROR(write_output_reg(handle, new_output), TAG, "Apply direction to latch failed");
    }
    ch422->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid value pointer");
    *value = ch422->regs.direction;
    return ESP_OK;
}

static esp_err_t write_highz_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    /* Only OC pins support open-drain/push-pull, and OD_EN is chip-global.
     * If any OC pin requests open-drain, enable OD_EN for all OC pins. */
    uint32_t oc_highz = value & ESP_IO_EXPANDER_CH422_OC_MASK;
    /* Reject High-Z requests on IO pins: quasi-bidirectional only */
    ESP_RETURN_ON_FALSE((value & ESP_IO_EXPANDER_CH422_IO_MASK) == 0, ESP_ERR_NOT_SUPPORTED, TAG,
                        "IO0-IO7 do not support open-drain mode");

    if (oc_highz) {
        ch422->sys_param |= CH422_SYS_OD_EN;
    } else {
        ch422->sys_param &= ~CH422_SYS_OD_EN;
    }
    ESP_RETURN_ON_ERROR(write_sys_param(ch422), TAG, "Write OD_EN failed");
    ch422->regs.highz = oc_highz;
    return ESP_OK;
}

static esp_err_t read_highz_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid value pointer");
    *value = ch422->regs.highz;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);

    /* IO-expand mode, IO outputs enabled, push-pull OC */
    ch422->sys_param = CH422_SYS_IO_OE;
    ESP_RETURN_ON_ERROR(write_sys_param(ch422), TAG, "Write sys param failed");

    ch422->regs.highz = HIGHZ_REG_DEFAULT_VAL;
    /* Bypass write_output_reg's IO_OE handling (already set above) with raw writes */
    uint8_t oc = (uint8_t)(OUT_REG_DEFAULT_VAL & 0x0F);
    uint8_t io = (uint8_t)((OUT_REG_DEFAULT_VAL >> 4) & 0xFF);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422->h_oc, &oc, 1, I2C_TIMEOUT_MS),
                        TAG, "Reset OC failed");
    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422->h_io_out, &io, 1, I2C_TIMEOUT_MS),
                        TAG, "Reset IO output failed");
    ch422->regs.output = OUT_REG_DEFAULT_VAL;
    /* Direction last: latches already high, so IO reads as input */
    ch422->regs.direction = DIR_REG_DEFAULT_VAL;
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_ch422_t *ch422 = (esp_io_expander_ch422_t *)__containerof(handle, esp_io_expander_ch422_t, base);

    if (ch422->h_sys) {
        i2c_master_bus_rm_device(ch422->h_sys);
    }
    if (ch422->h_oc) {
        i2c_master_bus_rm_device(ch422->h_oc);
    }
    if (ch422->h_io_out) {
        i2c_master_bus_rm_device(ch422->h_io_out);
    }
    if (ch422->h_io_in) {
        i2c_master_bus_rm_device(ch422->h_io_in);
    }
    free(ch422);
    return ESP_OK;
}
