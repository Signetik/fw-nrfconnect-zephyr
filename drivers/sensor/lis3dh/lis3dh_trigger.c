/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/util.h>
#include <kernel.h>

#define START_TRIG_INT1			0
#define START_TRIG_INT2			1
#define TRIGGED_INT1			4
#define TRIGGED_INT2			5

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_DECLARE(lis3dh);
#include "lis3dh.h"

static int lis3dh_trigger_drdy_set(struct device *dev, enum sensor_channel chan,
				   sensor_trigger_handler_t handler)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	int status;

	gpio_pin_disable_callback(lis3dh->gpio_int1, DT_LIS3DH_INT1_GPIO_PIN);

	/* cancel potentially pending trigger */
	atomic_clear_bit(&lis3dh->trig_flags, TRIGGED_INT1);

	status = lis3dh_reg_field_update(dev, LIS3DH_REG_CTRL3,
					 LIS3DH_EN_DRDY1_INT1_SHIFT,
					 LIS3DH_EN_DRDY1_INT1, 0);

	lis3dh->handler_drdy = handler;
	if ((handler == NULL) || (status < 0)) {
		return status;
	}

	lis3dh->chan_drdy = chan;

	/* serialize start of int1 in thread to synchronize output sampling
	 * and first interrupt. this avoids concurrent bus context access.
	 */
	atomic_set_bit(&lis3dh->trig_flags, START_TRIG_INT1);
#if defined(CONFIG_LIS3DH_TRIGGER_OWN_THREAD)
	k_sem_give(&lis3dh->gpio_sem);
#elif defined(CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lis3dh->work);
#endif

	return 0;
}

static int lis3dh_start_trigger_int1(struct device *dev)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	int status;
	u8_t raw[LIS3DH_BUF_SZ];
	u8_t ctrl1 = 0U;

	/* power down temporarly to align interrupt & data output sampling */
	status = lis3dh_reg_read_byte(dev, LIS3DH_REG_CTRL1, &ctrl1);
	if (unlikely(status < 0)) {
		return status;
	}
	status = lis3dh_reg_write_byte(dev, LIS3DH_REG_CTRL1,
				       ctrl1 & ~LIS3DH_ODR_MASK);
	if (unlikely(status < 0)) {
		return status;
	}

	LOG_DBG("ctrl1=0x%x @tick=%u", ctrl1, k_cycle_get_32());

	/* empty output data */
	status = lis3dh_burst_read(dev, LIS3DH_REG_STATUS, raw, sizeof(raw));
	if (unlikely(status < 0)) {
		return status;
	}

	gpio_pin_enable_callback(lis3dh->gpio_int1, DT_LIS3DH_INT1_GPIO_PIN);

	/* re-enable output sampling */
	status = lis3dh_reg_write_byte(dev, LIS3DH_REG_CTRL1, ctrl1);
	if (unlikely(status < 0)) {
		return status;
	}

	return lis3dh_reg_field_update(dev, LIS3DH_REG_CTRL3,
				       LIS3DH_EN_DRDY1_INT1_SHIFT,
				       LIS3DH_EN_DRDY1_INT1, 1);
}

#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
#define LIS3DH_ANYM_CFG (LIS3DH_INT_CFG_ZHIE_ZUPE | LIS3DH_INT_CFG_YHIE_YUPE |\
			 LIS3DH_INT_CFG_XHIE_XUPE)

static int lis3dh_trigger_anym_set(struct device *dev,
				   sensor_trigger_handler_t handler)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	int status;
	u8_t reg_val;

	gpio_pin_disable_callback(lis3dh->gpio_int2, DT_LIS3DH_INT2_GPIO_PIN);

	/* cancel potentially pending trigger */
	atomic_clear_bit(&lis3dh->trig_flags, TRIGGED_INT2);

	/* disable all interrupt 2 events */
	status = lis3dh_reg_write_byte(dev, LIS3DH_REG_INT2_CFG, 0);

	/* make sure any pending interrupt is cleared */
	status = lis3dh_reg_read_byte(dev, LIS3DH_REG_INT2_SRC, &reg_val);

	lis3dh->handler_anymotion = handler;
	if ((handler == NULL) || (status < 0)) {
		return status;
	}

	/* serialize start of int2 in thread to synchronize output sampling
	 * and first interrupt. this avoids concurrent bus context access.
	 */
	atomic_set_bit(&lis3dh->trig_flags, START_TRIG_INT2);
#if defined(CONFIG_LIS3DH_TRIGGER_OWN_THREAD)
	k_sem_give(&lis3dh->gpio_sem);
#elif defined(CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lis3dh->work);
#endif
	return 0;
}

static int lis3dh_start_trigger_int2(struct device *dev)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	int status;

	status = gpio_pin_enable_callback(lis3dh->gpio_int2,
					  DT_LIS3DH_INT2_GPIO_PIN);
	if (unlikely(status < 0)) {
		LOG_ERR("enable callback failed err=%d", status);
	}

	return lis3dh_reg_write_byte(dev, LIS3DH_REG_INT2_CFG,
				     LIS3DH_ANYM_CFG);
}
#endif /* DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1 */

int lis3dh_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	if (trig->type == SENSOR_TRIG_DATA_READY &&
	    trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		return lis3dh_trigger_drdy_set(dev, trig->chan, handler);
#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
	} else if (trig->type == SENSOR_TRIG_DELTA) {
		return lis3dh_trigger_anym_set(dev, handler);
#endif /* DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1 */
	}

	return -ENOTSUP;
}

int lis3dh_acc_slope_config(struct device *dev, enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	int status;

	if (attr == SENSOR_ATTR_SLOPE_TH) {
		u8_t range_g, reg_val;
		u32_t slope_th_ums2;

		status = lis3dh_reg_read_byte(dev, LIS3DH_REG_CTRL4, &reg_val);
		if (status < 0) {
			return status;
		}

		/* fs reg value is in the range 0 (2g) - 3 (16g) */
		range_g = 2 * (1 << ((LIS3DH_FS_MASK & reg_val)
				      >> LIS3DH_FS_SHIFT));

		slope_th_ums2 = val->val1 * 1000000 + val->val2;

		/* make sure the provided threshold does not exceed range */
		if ((slope_th_ums2 - 1) > (range_g * SENSOR_G)) {
			return -EINVAL;
		}

		/* 7 bit full range value */
		reg_val = 128 / range_g * (slope_th_ums2 - 1) / SENSOR_G;

		LOG_INF("int2_ths=0x%x range_g=%d ums2=%u", reg_val,
			    range_g, slope_th_ums2 - 1);

		status = lis3dh_reg_write_byte(dev, LIS3DH_REG_INT2_THS,
					       reg_val);
	} else { /* SENSOR_ATTR_SLOPE_DUR */
		/*
		 * slope duration is measured in number of samples:
		 * N/ODR where N is the register value
		 */
		if (val->val1 < 0 || val->val1 > 127) {
			return -ENOTSUP;
		}

		LOG_INF("int2_dur=0x%x", val->val1);

		status = lis3dh_reg_write_byte(dev, LIS3DH_REG_INT2_DUR,
					       val->val1);
	}

	return status;
}

static void lis3dh_gpio_int1_callback(struct device *dev,
				      struct gpio_callback *cb, u32_t pins)
{
	struct lis3dh_data *lis3dh =
		CONTAINER_OF(cb, struct lis3dh_data, gpio_int1_cb);

	ARG_UNUSED(pins);

	atomic_set_bit(&lis3dh->trig_flags, TRIGGED_INT1);

#if defined(CONFIG_LIS3DH_TRIGGER_OWN_THREAD)
	k_sem_give(&lis3dh->gpio_sem);
#elif defined(CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lis3dh->work);
#endif
}

#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
static void lis3dh_gpio_int2_callback(struct device *dev,
				      struct gpio_callback *cb, u32_t pins)
{
	struct lis3dh_data *lis3dh =
		CONTAINER_OF(cb, struct lis3dh_data, gpio_int2_cb);

	ARG_UNUSED(pins);

	atomic_set_bit(&lis3dh->trig_flags, TRIGGED_INT2);

#if defined(CONFIG_LIS3DH_TRIGGER_OWN_THREAD)
	k_sem_give(&lis3dh->gpio_sem);
#elif defined(CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&lis3dh->work);
#endif
}
#endif /* DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1 */

static void lis3dh_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct lis3dh_data *lis3dh = dev->driver_data;
	int status;

	if (unlikely(atomic_test_and_clear_bit(&lis3dh->trig_flags,
		     START_TRIG_INT1))) {
		status = lis3dh_start_trigger_int1(dev);

		if (unlikely(status < 0)) {
			LOG_ERR("lis3dh_start_trigger_int1: %d", status);
		}
		return;
	}

#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
	if (unlikely(atomic_test_and_clear_bit(&lis3dh->trig_flags,
		     START_TRIG_INT2))) {
		status = lis3dh_start_trigger_int2(dev);

		if (unlikely(status < 0)) {
			LOG_ERR("lis3dh_start_trigger_int2: %d", status);
		}
		return;
	}
#endif /* DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1 */

	if (atomic_test_and_clear_bit(&lis3dh->trig_flags,
				      TRIGGED_INT1)) {
		struct sensor_trigger drdy_trigger = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = lis3dh->chan_drdy,
		};

		if (likely(lis3dh->handler_drdy != NULL)) {
			lis3dh->handler_drdy(dev, &drdy_trigger);
		}

		return;
	}

#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
	if (atomic_test_and_clear_bit(&lis3dh->trig_flags,
				      TRIGGED_INT2)) {
		struct sensor_trigger anym_trigger = {
			.type = SENSOR_TRIG_DELTA,
			.chan = lis3dh->chan_drdy,
		};
		u8_t reg_val;

		/* clear interrupt 2 to de-assert int2 line */
		status = lis3dh_reg_read_byte(dev, LIS3DH_REG_INT2_SRC,
					      &reg_val);
		if (status < 0) {
			LOG_ERR("clearing interrupt 2 failed: %d", status);
			return;
		}

		if (likely(lis3dh->handler_anymotion != NULL)) {
			lis3dh->handler_anymotion(dev, &anym_trigger);
		}

		LOG_DBG("@tick=%u int2_src=0x%x", k_cycle_get_32(),
			    reg_val);

		return;
	}
#endif /* DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1 */
}

#ifdef CONFIG_LIS3DH_TRIGGER_OWN_THREAD
static void lis3dh_thread(void *arg1, void *unused2, void *unused3)
{
	struct device *dev = arg1;
	struct lis3dh_data *lis3dh = dev->driver_data;

	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	while (1) {
		k_sem_take(&lis3dh->gpio_sem, K_FOREVER);
		lis3dh_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD
static void lis3dh_work_cb(struct k_work *work)
{
	struct lis3dh_data *lis3dh =
		CONTAINER_OF(work, struct lis3dh_data, work);

	lis3dh_thread_cb(lis3dh->dev);
}
#endif

#define LIS3DH_INT1_CFG			(GPIO_DIR_IN | GPIO_INT |\
					 GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

#define LIS3DH_INT2_CFG			(GPIO_DIR_IN | GPIO_INT |\
					 GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

int lis3dh_init_interrupt(struct device *dev)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	int status;
	u8_t raw[LIS3DH_DATA_OFS + 2];

	/* setup data ready gpio interrupt */
	lis3dh->gpio_int1 = device_get_binding(DT_LIS3DH_INT1_GPIO_DEV_NAME);
	if (lis3dh->gpio_int1 == NULL) {
		LOG_ERR("Cannot get pointer to %s device",
			    DT_LIS3DH_INT1_GPIO_DEV_NAME);
		return -EINVAL;
	}

	/* data ready int1 gpio configuration */
	status = gpio_pin_configure(lis3dh->gpio_int1, DT_LIS3DH_INT1_GPIO_PIN,
				    LIS3DH_INT1_CFG);
	if (status < 0) {
		LOG_ERR("Could not configure gpio %d",
			     DT_LIS3DH_INT1_GPIO_PIN);
		return status;
	}

	gpio_init_callback(&lis3dh->gpio_int1_cb,
			   lis3dh_gpio_int1_callback,
			   BIT(DT_LIS3DH_INT1_GPIO_PIN));

	status = gpio_add_callback(lis3dh->gpio_int1, &lis3dh->gpio_int1_cb);
	if (status < 0) {
		LOG_ERR("Could not add gpio int1 callback");
		return status;
	}

	LOG_INF("int1 on pin=%d cfg=0x%x",
		    DT_LIS3DH_INT1_GPIO_PIN, LIS3DH_INT1_CFG);

#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
	/* setup any motion gpio interrupt */
	lis3dh->gpio_int2 = device_get_binding(DT_LIS3DH_INT2_GPIO_DEV_NAME);
	if (lis3dh->gpio_int2 == NULL) {
		LOG_ERR("Cannot get pointer to %s device",
			    DT_LIS3DH_INT2_GPIO_DEV_NAME);
		return -EINVAL;
	}

	/* any motion int2 gpio configuration */
	status = gpio_pin_configure(lis3dh->gpio_int2, DT_LIS3DH_INT2_GPIO_PIN,
				    LIS3DH_INT2_CFG);
	if (status < 0) {
		LOG_ERR("Could not configure gpio %d",
			     DT_LIS3DH_INT2_GPIO_PIN);
		return status;
	}

	gpio_init_callback(&lis3dh->gpio_int2_cb,
			   lis3dh_gpio_int2_callback,
			   BIT(DT_LIS3DH_INT2_GPIO_PIN));

	/* callback is going to be enabled by trigger setting function */
	status = gpio_add_callback(lis3dh->gpio_int2, &lis3dh->gpio_int2_cb);
	if (status < 0) {
		LOG_ERR("Could not add gpio int2 callback (%d)", status);
		return status;
	}

	LOG_INF("int2 on pin=%d cfg=0x%x",
		    DT_LIS3DH_INT2_GPIO_PIN, LIS3DH_INT2_CFG);
#endif /* DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1 */

#if defined(CONFIG_LIS3DH_TRIGGER_OWN_THREAD)
	k_sem_init(&lis3dh->gpio_sem, 0, UINT_MAX);

	k_thread_create(&lis3dh->thread, lis3dh->thread_stack,
			CONFIG_LIS3DH_THREAD_STACK_SIZE,
			(k_thread_entry_t)lis3dh_thread, dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_LIS3DH_THREAD_PRIORITY), 0, 0);
#elif defined(CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD)
	lis3dh->work.handler = lis3dh_work_cb;
	lis3dh->dev = dev;
#endif
	/* disable interrupt 2 in case of warm (re)boot */
	status = lis3dh_reg_write_byte(dev, LIS3DH_REG_INT2_CFG, 0);
	if (status < 0) {
		LOG_ERR("Interrupt 2 disable reg write failed (%d)",
			    status);
		return status;
	}

	(void)memset(raw, 0, sizeof(raw));
	status = lis3dh_burst_write(dev, LIS3DH_REG_INT2_THS, raw, sizeof(raw));
	if (status < 0) {
		LOG_ERR("Burst write to INT2 THS failed (%d)", status);
		return status;
	}

	/* latch int2 line interrupt */
	status = lis3dh_reg_write_byte(dev, LIS3DH_REG_CTRL5,
				       LIS3DH_EN_LIR_INT2);
	if (status < 0) {
		LOG_ERR("INT2 latch enable reg write failed (%d)", status);
		return status;
	}

	/* enable interrupt 2 on int2 line */
	return lis3dh_reg_write_byte(dev, LIS3DH_REG_CTRL6,
				     LIS3DH_EN_INT2_INT2);
}
