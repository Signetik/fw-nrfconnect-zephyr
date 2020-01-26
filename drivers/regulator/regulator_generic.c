/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief I2S bus (SSP) driver for Intel CAVS.
 *
 * Limitations:
 * - DMA is used in simple single block transfer mode (with linked list
 *   enabled) and "interrupt on full transfer completion" mode.
 */

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <soc.h>

#include "regulator.h"

DEVICE_DECLARE(lsensor_reg);
DEVICE_DECLARE(accel_reg);

static struct regulator_data lsensor_reg_data;
static struct regulator_config lsensor_reg_config = {
    .en_cont = DT_SIGNETIK_REGULATORS_LSENSE_ADDON_GPIOS_CONTROLLER,
    .en_pin =DT_SIGNETIK_REGULATORS_LSENSE_ADDON_GPIOS_PIN,
    .flags = DT_SIGNETIK_REGULATORS_LSENSE_ADDON_GPIOS_FLAGS,
};

static struct regulator_data accel_reg_data;
static struct regulator_config accel_reg_config = {
    .en_cont = DT_SIGNETIK_REGULATORS_ACCEL_ADDON_GPIOS_CONTROLLER,
    .en_pin =DT_SIGNETIK_REGULATORS_ACCEL_ADDON_GPIOS_PIN,
    .flags = DT_SIGNETIK_REGULATORS_LSENSE_ADDON_GPIOS_FLAGS,
};

#if(0)
DEVICE_INIT(lsensor_reg, "lsensor_reg", regulator_init, &lsensor_reg_data,
		    &lsensor_reg_config, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

DEVICE_INIT(accel_reg, "accel_reg", regulator_init, &accel_reg_data,
		    &accel_reg_config, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif