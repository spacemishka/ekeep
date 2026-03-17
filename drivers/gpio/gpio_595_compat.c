/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_gpio_595

/**
 * @file Driver for 74HC595 SPI-based GPIO expander.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_595_compat);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct reg_595_config {
    /* gpio_driver_config needs to be first */
    struct gpio_driver_config common;
    struct spi_dt_spec bus;
    uint8_t ngpios;
};

struct reg_595_drv_data {
    /* gpio_driver_data needs to be first */
    struct gpio_driver_data common;
    struct k_sem lock;
    uint32_t gpio_cache;
};

static int reg_595_write_registers(const struct device *dev, uint32_t value) {
    const struct reg_595_config *config = dev->config;
    struct reg_595_drv_data *const drv_data = (struct reg_595_drv_data *const)dev->data;
    uint8_t nwrite = config->ngpios / 8;
    uint32_t reg_data = sys_cpu_to_be32(value);

    const struct spi_buf tx_buf[1] = {{
        .buf = ((uint8_t *)&reg_data) + (4 - nwrite),
        .len = nwrite,
    }};

    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = ARRAY_SIZE(tx_buf),
    };

    int ret = spi_write_dt(&config->bus, &tx);
    if (ret < 0) {
        LOG_ERR("spi_write failed: %d", ret);
        return ret;
    }

    drv_data->gpio_cache = value;
    return 0;
}

static int setup_pin_dir(const struct device *dev, uint32_t pin, int flags) {
    ARG_UNUSED(dev);
    ARG_UNUSED(pin);

    if ((flags & GPIO_OUTPUT) == 0U) {
        return -ENOTSUP;
    }

    return 0;
}

static int reg_595_port_get_raw(const struct device *dev, uint32_t *value) {
    ARG_UNUSED(dev);
    ARG_UNUSED(value);
    return -ENOTSUP;
}

static int reg_595_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value) {
    struct reg_595_drv_data *const drv_data = (struct reg_595_drv_data *const)dev->data;
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }

    k_sem_take(&drv_data->lock, K_FOREVER);

    uint32_t buf = drv_data->gpio_cache;
    buf = (buf & ~mask) | (mask & value);

    int ret = reg_595_write_registers(dev, buf);

    k_sem_give(&drv_data->lock);
    return ret;
}

static int reg_595_port_set_bits_raw(const struct device *dev, uint32_t mask) {
    return reg_595_port_set_masked_raw(dev, mask, mask);
}

static int reg_595_port_clear_bits_raw(const struct device *dev, uint32_t mask) {
    return reg_595_port_set_masked_raw(dev, mask, 0);
}

static int reg_595_port_toggle_bits(const struct device *dev, uint32_t mask) {
    struct reg_595_drv_data *const drv_data = (struct reg_595_drv_data *const)dev->data;
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }

    k_sem_take(&drv_data->lock, K_FOREVER);

    uint32_t buf = drv_data->gpio_cache;
    buf ^= mask;

    int ret = reg_595_write_registers(dev, buf);

    k_sem_give(&drv_data->lock);
    return ret;
}

static int reg_595_pin_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags) {
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }

    if ((flags & GPIO_OPEN_DRAIN) != 0U) {
        return -ENOTSUP;
    }

    int ret = setup_pin_dir(dev, pin, flags);
    if (ret) {
        LOG_ERR("595: error setting pin direction (%d)", ret);
        return ret;
    }

    if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
        return reg_595_port_clear_bits_raw(dev, BIT(pin));
    } else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
        return reg_595_port_set_bits_raw(dev, BIT(pin));
    }

    return 0;
}

static int reg_595_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
                                           enum gpio_int_mode mode, enum gpio_int_trig trig) {
    ARG_UNUSED(dev);
    ARG_UNUSED(pin);
    ARG_UNUSED(mode);
    ARG_UNUSED(trig);
    return -ENOTSUP;
}

static const struct gpio_driver_api api_table = {
    .pin_configure = reg_595_pin_config,
    .port_get_raw = reg_595_port_get_raw,
    .port_set_masked_raw = reg_595_port_set_masked_raw,
    .port_set_bits_raw = reg_595_port_set_bits_raw,
    .port_clear_bits_raw = reg_595_port_clear_bits_raw,
    .port_toggle_bits = reg_595_port_toggle_bits,
    .pin_interrupt_configure = reg_595_pin_interrupt_configure,
};

static int reg_595_init(const struct device *dev) {
    const struct reg_595_config *const config = dev->config;
    struct reg_595_drv_data *const drv_data = (struct reg_595_drv_data *const)dev->data;
    if (!spi_is_ready_dt(&config->bus)) {
        LOG_ERR("SPI bus not ready for 595");
        return -ENODEV;
    }

    k_sem_init(&drv_data->lock, 1, 1);

    return 0;
}

#define GPIO_PORT_PIN_MASK_FROM_NGPIOS(ngpios) ((gpio_port_pins_t)(((uint64_t)1 << (ngpios)) - 1U))
#define GPIO_PORT_PIN_MASK_FROM_DT_INST(inst) \
    GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_PROP(inst, ngpios))

#define REG_595_INIT(n) \
    static const struct reg_595_config reg_595_##n##_config = { \
        .common = \
            { \
                .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n), \
            }, \
        .bus = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), \
                                    0), \
        .ngpios = DT_INST_PROP(n, ngpios), \
    }; \
    static struct reg_595_drv_data reg_595_##n##_drvdata = {}; \
    DEVICE_DT_INST_DEFINE(n, reg_595_init, NULL, &reg_595_##n##_drvdata, &reg_595_##n##_config, \
                          POST_KERNEL, CONFIG_EKEEP_GPIO_595_INIT_PRIORITY, &api_table);

DT_INST_FOREACH_STATUS_OKAY(REG_595_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
