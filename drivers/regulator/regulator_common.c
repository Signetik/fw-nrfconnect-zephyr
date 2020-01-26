/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <string.h>
#include <drivers/gpio.h>

#include "regulator.h"

int regulator_init(struct device *dev)
{
    struct regulator_data *data = dev->driver_data;
    const struct regulator_config *config = dev->config->config_info;

    data->use_count = 0;

    data->gpio = device_get_binding(config->en_cont);
    if (data->gpio) {
        gpio_pin_configure(data->gpio, config->en_pin, GPIO_DIR_OUT);
        gpio_pin_write(data->gpio, config->en_pin, 1-config->flags);
    }

    return 0;
}

void z_impl_regulator_enable(struct device *dev)
{
    struct regulator_data *data = dev->driver_data;

    data->use_count++;
    if (data->gpio) {
        gpio_pin_write(data->gpio, config->en_pin, config->flags);
    }
}

void z_impl_regulator_disable(struct device *dev)
{
    struct regulator_data *data = dev->driver_data;

    data->use_count--;
    if (data->use_count <= 0) {
        if (data->gpio) {
            gpio_pin_write(data->gpio, config->en_pin, 1-config->flags);
        }
        data->use_count = 0;
    }
}

