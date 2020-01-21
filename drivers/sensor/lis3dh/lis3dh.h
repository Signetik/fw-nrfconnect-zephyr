/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <device.h>
#include <i2c.h>
#include <misc/byteorder.h>
#include <misc/util.h>
#include <kernel.h>
#include <sensor.h>
#include <stdint.h>
#include <gpio.h>
#include <string.h>


#define LIS3DH_BUS_ADDRESS		CONFIG_LIS3DH_I2C_ADDR



#define LIS3DH_AUTOINCREMENT_ADDR	BIT(7)

#define LIS3DH_REG_CTRL1		0x20
#define LIS3DH_ACCEL_XYZ_SHIFT		0
#define LIS3DH_ACCEL_X_EN_BIT		BIT(0)
#define LIS3DH_ACCEL_Y_EN_BIT		BIT(1)
#define LIS3DH_ACCEL_Z_EN_BIT		BIT(2)
#define LIS3DH_ACCEL_EN_BITS		(LIS3DH_ACCEL_X_EN_BIT | \
					LIS3DH_ACCEL_Y_EN_BIT | \
					LIS3DH_ACCEL_Z_EN_BIT)
#define LIS3DH_ACCEL_XYZ_MASK		BIT_MASK(3)

#define LIS3DH_LP_EN_BIT_MASK		BIT(3)
#if defined(CONFIG_LIS3DH_POWER_MODE_LOW)
	#define LIS3DH_LP_EN_BIT	BIT(3)
#elif defined(CONFIG_LIS3DH_POWER_MODE_NORMAL)
	#define LIS3DH_LP_EN_BIT	0
#endif

#define LIS3DH_ODR_1			1
#define LIS3DH_ODR_2			2
#define LIS3DH_ODR_3			3
#define LIS3DH_ODR_4			4
#define LIS3DH_ODR_5			5
#define LIS3DH_ODR_6			6
#define LIS3DH_ODR_7			7
#define LIS3DH_ODR_8			8
#define LIS3DH_ODR_9			9

#if defined(CONFIG_LIS3DH_ODR_1)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_1
#elif defined(CONFIG_LIS3DH_ODR_2)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_2
#elif defined(CONFIG_LIS3DH_ODR_3)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_3
#elif defined(CONFIG_LIS3DH_ODR_4) || defined(CONFIG_LIS3DH_ODR_RUNTIME)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_4
#elif defined(CONFIG_LIS3DH_ODR_5)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_5
#elif defined(CONFIG_LIS3DH_ODR_6)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_6
#elif defined(CONFIG_LIS3DH_ODR_7)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_7
#elif defined(CONFIG_LIS3DH_ODR_8)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_8
#elif defined(CONFIG_LIS3DH_ODR_9_NORMAL) || defined(CONFIG_LIS3DH_ODR_9_LOW)
	#define LIS3DH_ODR_IDX		LIS3DH_ODR_9
#endif

#define LIS3DH_ODR_SHIFT		4
#define LIS3DH_ODR_RATE(r)		((r) << LIS3DH_ODR_SHIFT)
#define LIS3DH_ODR_BITS			(LIS3DH_ODR_RATE(LIS3DH_ODR_IDX))
#define LIS3DH_ODR_MASK			(BIT_MASK(4) << LIS3DH_ODR_SHIFT)

#define LIS3DH_REG_CTRL2		0x21
#define LIS3DH_HPIS2_EN_BIT		BIT(1)
#define LIS3DH_FDS_EN_BIT		BIT(3)

#define LIS3DH_REG_CTRL3		0x22
#define LIS3DH_EN_DRDY1_INT1_SHIFT	4
#define LIS3DH_EN_DRDY1_INT1		BIT(LIS3DH_EN_DRDY1_INT1_SHIFT)

#define LIS3DH_REG_CTRL4		0x23
#define LIS3DH_FS_SHIFT			4
#define LIS3DH_FS_MASK			(BIT_MASK(2) << LIS3DH_FS_SHIFT)

#if defined(CONFIG_LIS3DH_ACCEL_RANGE_2G) ||\
	defined(CONFIG_LIS3DH_ACCEL_RANGE_RUNTIME)
	#define LIS3DH_FS_IDX		0
#elif defined(CONFIG_LIS3DH_ACCEL_RANGE_4G)
	#define LIS3DH_FS_IDX		1
#elif defined(CONFIG_LIS3DH_ACCEL_RANGE_8G)
	#define LIS3DH_FS_IDX		2
#elif defined(CONFIG_LIS3DH_ACCEL_RANGE_16G)
	#define LIS3DH_FS_IDX		3
#endif

#define LIS3DH_FS_SELECT(fs)		((fs) << LIS3DH_FS_SHIFT)
#define LIS3DH_FS_BITS			(LIS3DH_FS_SELECT(LIS3DH_FS_IDX))
#define LIS3DH_ACCEL_SCALE(range_g)	((SENSOR_G * 2 * (range_g)) / 65636LL)

#define LIS3DH_REG_CTRL5		0x24
#define LIS3DH_LIR_INT2_SHIFT		1
#define LIS3DH_EN_LIR_INT2		BIT(LIS3DH_LIR_INT2_SHIFT)

#define LIS3DH_REG_CTRL6		0x25
#define LIS3DH_EN_INT2_INT2_SHIFT	5
#define LIS3DH_EN_INT2_INT2		BIT(LIS3DH_EN_INT2_INT2_SHIFT)

#define LIS3DH_REG_REFERENCE		0x26

#define LIS3DH_REG_STATUS		0x27
#define LIS3DH_STATUS_ZYZ_OVR		BIT(7)
#define LIS3DH_STATUS_Z_OVR		BIT(6)
#define LIS3DH_STATUS_Y_OVR		BIT(5)
#define LIS3DH_STATUS_X_OVR		BIT(4)
#define LIS3DH_STATUS_OVR_MASK		(BIT_MASK(4) << 4)
#define LIS3DH_STATUS_ZYX_DRDY		BIT(3)
#define LIS3DH_STATUS_Z_DRDY		BIT(2)
#define LIS3DH_STATUS_Y_DRDY		BIT(1)
#define LIS3DH_STATUS_X_DRDY		BIT(0)
#define LIS3DH_STATUS_DRDY_MASK		BIT_MASK(4)

#define LIS3DH_REG_ACCEL_X_LSB		0x28
#define LIS3DH_REG_ACCEL_Y_LSB		0x2A
#define LIS3DH_REG_ACCEL_Z_LSB		0x2C
#define LIS3DH_REG_ACCEL_X_MSB		0x29
#define LIS3DH_REG_ACCEL_Y_MSB		0x2B
#define LIS3DH_REG_ACCEL_Z_MSB		0x2D

#define LIS3DH_REG_INT1_CFG		0x30
#define LIS3DH_REG_INT2_CFG		0x34
#define LIS3DH_AOI_CFG			BIT(7)
#define LIS3DH_INT_CFG_ZHIE_ZUPE	BIT(5)
#define LIS3DH_INT_CFG_ZLIE_ZDOWNE	BIT(4)
#define LIS3DH_INT_CFG_YHIE_YUPE	BIT(3)
#define LIS3DH_INT_CFG_YLIE_YDOWNE	BIT(2)
#define LIS3DH_INT_CFG_XHIE_XUPE	BIT(1)
#define LIS3DH_INT_CFG_XLIE_XDOWNE	BIT(0)

#define LIS3DH_REG_INT2_SRC		0x35

#define LIS3DH_REG_INT2_THS		0x36

#define LIS3DH_REG_INT2_DUR		0x37

/* sample buffer size includes status register */

#define LIS3DH_BUF_SZ			7
#define LIS3DH_DATA_OFS			0


#if defined(DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1)
/* INT1 and INT2 are configured */
#define DT_LIS3DH_INT1_GPIO_PIN		DT_ST_LIS3DH_0_IRQ_GPIOS_PIN_0
#define DT_LIS3DH_INT1_GPIO_DEV_NAME	DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_0
#define DT_LIS3DH_INT2_GPIO_PIN		DT_ST_LIS3DH_0_IRQ_GPIOS_PIN_1
#define DT_LIS3DH_INT2_GPIO_DEV_NAME	DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER_1
#else
/* INT1 only */
#define DT_LIS3DH_INT1_GPIO_PIN		DT_ST_LIS3DH_0_IRQ_GPIOS_PIN
#define DT_LIS3DH_INT1_GPIO_DEV_NAME	DT_ST_LIS3DH_0_IRQ_GPIOS_CONTROLLER
#endif

union lis3dh_sample {
	u8_t raw[LIS3DH_BUF_SZ];
	struct {
		u8_t status;
		s16_t xyz[3];
	} __packed;
};

struct lis3dh_data {
	struct device *bus;
	union lis3dh_sample sample;
	/* current scaling factor, in micro m/s^2 / lsb */
	u16_t scale;

#ifdef CONFIG_LIS3DH_TRIGGER
	struct device *gpio_int1;
	struct device *gpio_int2;
	struct gpio_callback gpio_int1_cb;
	struct gpio_callback gpio_int2_cb;

	sensor_trigger_handler_t handler_drdy;
	sensor_trigger_handler_t handler_anymotion;
	atomic_t trig_flags;
	enum sensor_channel chan_drdy;

#if defined(CONFIG_LIS3DH_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_LIS3DH_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_LIS3DH_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_LIS3DH_TRIGGER */
};


static inline int lis3dh_bus_configure(struct device *dev)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	lis3dh->bus = device_get_binding(CONFIG_LIS3DH_I2C_MASTER_DEV_NAME);
	if (lis3dh->bus == NULL) {
		printk("Could not get pointer to %s device",
			    CONFIG_LIS3DH_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	return 0;
}

static inline int lis3dh_burst_read(struct device *dev, u8_t start_addr,
				    u8_t *buf, u8_t num_bytes)
{
	struct lis3dh_data *lis3dh = dev->driver_data;

	return i2c_burst_read(lis3dh->bus, LIS3DH_BUS_ADDRESS,
			      start_addr | LIS3DH_AUTOINCREMENT_ADDR,
			      buf, num_bytes);
}

static inline int lis3dh_reg_read_byte(struct device *dev, u8_t reg_addr,
				       u8_t *value)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	return i2c_reg_read_byte(lis3dh->bus, LIS3DH_BUS_ADDRESS,
				 reg_addr, value);
}

static inline int lis3dh_burst_write(struct device *dev, u8_t start_addr,
				     u8_t *buf, u8_t num_bytes)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	return i2c_burst_write(lis3dh->bus, LIS3DH_BUS_ADDRESS,
			       start_addr | LIS3DH_AUTOINCREMENT_ADDR,
			       buf, num_bytes);
}

static inline int lis3dh_reg_write_byte(struct device *dev, u8_t reg_addr,
					u8_t value)
{
	struct lis3dh_data *lis3dh = dev->driver_data;
	u8_t tx_buf[2] = {reg_addr, value};

	return i2c_write(lis3dh->bus, tx_buf, sizeof(tx_buf),
			 LIS3DH_BUS_ADDRESS);
}

#ifdef CONFIG_LIS3DH_TRIGGER
int lis3dh_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int lis3dh_init_interrupt(struct device *dev);

int lis3dh_reg_field_update(struct device *dev, u8_t reg_addr,
			    u8_t pos, u8_t mask, u8_t val);

int lis3dh_acc_slope_config(struct device *dev, enum sensor_attribute attr,
			    const struct sensor_value *val);
#endif

/* __SENSOR_LIS3DH__ */
