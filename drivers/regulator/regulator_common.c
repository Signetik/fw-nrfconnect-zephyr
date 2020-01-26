/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <string.h>
#include <drivers/gpio.h>

#include "regulator.h"

static void regulator_on_off(struct device *dev, int on_off);

int regulator_init(struct device *dev)
{
    struct regulator_data *data = dev->driver_data;
    const struct regulator_config *config = dev->config->config_info;

    data->use_count = config->use_init;

    data->gpio = device_get_binding(config->en_cont);
    if (data->gpio) {
        gpio_pin_configure(data->gpio, config->en_pin, GPIO_DIR_OUT);
        regulator_on_off(dev, data->use_count > 0 ? 1 : 0);
    }

    return 0;
}

void z_impl_regulator_enable(struct device *dev)
{
    struct regulator_data *data = dev->driver_data;

    data->use_count++;
    if (data->use_count == 1) {
        regulator_on_off(dev, 1);
    }
}

void z_impl_regulator_disable(struct device *dev)
{
    struct regulator_data *data = dev->driver_data;

    data->use_count--;
    if (data->use_count <= 0) {
        regulator_on_off(dev, 0);
        data->use_count = 0;
    }
}

static void regulator_on_off(struct device *dev, int on_off)
{
    struct regulator_data *data = dev->driver_data;
    const struct regulator_config *config = dev->config->config_info;

    if (data->gpio) {
        gpio_pin_write(data->gpio, config->en_pin, on_off ? config->flags : (1-config->flags));
    }
}

