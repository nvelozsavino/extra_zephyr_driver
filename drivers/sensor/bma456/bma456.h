/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMA456_BMA456_H_
#define ZEPHYR_DRIVERS_SENSOR_BMA456_BMA456_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>

#include "bosch/bma4_defs.h"


#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

/* sample buffer size includes status register */
#define BMA456_BUF_SZ			7

union bma456_sample {
	uint8_t raw[BMA456_BUF_SZ];
	struct {
		uint8_t status;
		int16_t xyz[3];
	} __packed;
};

union bma456_bus_cfg {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct temperature {
	uint8_t cfg_addr;
	uint8_t enable_mask;
	uint8_t dout_addr;
	uint8_t fractional_bits;
};

struct bma456_config {
	int (*bus_init)(const struct device *dev);
	const union bma456_bus_cfg bus_cfg;
#ifdef CONFIG_BMA456_TRIGGER
	const struct gpio_dt_spec gpio_drdy;
	const struct gpio_dt_spec gpio_int;
#endif /* CONFIG_BMA456_TRIGGER */
	struct {
		bool disc_pull_up : 1;
		bool anym_on_int1 : 1;
		bool anym_latch : 1;
		uint8_t anym_mode : 2;
	} hw;
#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
	const struct temperature temperature;
#endif
};

struct bma456_transfer_function {
	int (*read_data)(const struct device *dev, uint8_t reg_addr,
			 uint8_t *value, uint32_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr,
			  uint8_t *value, uint32_t len);
};

struct bma456_data {
	const struct device *bus;
	struct bma4_dev bma;
	const struct bma456_transfer_function *hw_tf;

	union bma456_sample sample;
	/* current scaling factor, in micro m/s^2 / lsb */
	uint32_t scale;

#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
	struct sensor_value temperature;
#endif

#ifdef CONFIG_PM_DEVICE
	uint8_t reg_ctrl1_active_val;
#endif

#ifdef CONFIG_BMA456_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_int1_cb;
	struct gpio_callback gpio_int2_cb;

	sensor_trigger_handler_t handler_drdy;
	sensor_trigger_handler_t handler_anymotion;
	atomic_t trig_flags;
	enum sensor_channel chan_drdy;

#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_BMA456_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_BMA456_TRIGGER */
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
int bma456_spi_access(struct bma456_data *ctx, uint8_t cmd,
		      void *data, size_t length);
#endif

#ifdef CONFIG_BMA456_TRIGGER
int bma456_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int bma456_init_interrupt(const struct device *dev);

int bma456_acc_slope_config(const struct device *dev,
			    enum sensor_attribute attr,
			    const struct sensor_value *val);
#endif

int bma456_spi_init(const struct device *dev);
int bma456_i2c_init(const struct device *dev);


#endif /* __SENSOR_BMA456__ */
