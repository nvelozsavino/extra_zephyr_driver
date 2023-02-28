/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma456

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "bma456.h"
LOG_MODULE_REGISTER(BMA456, LOG_LEVEL_DBG);

static int bma456_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bma456_data *drv_data = dev->data;
	const struct bma456_config *config = dev->config;
	uint8_t buf[6];
	uint8_t lsb;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	/*
	 * since all accel data register addresses are consecutive,
	 * a burst read can be used to read all the samples
	 */
	if (i2c_burst_read_dt(&config->i2c,
			      BMA456_REG_ACCEL_X_LSB, buf, 6) < 0) {
		LOG_DBG("Could not read accel axis data");
		return -EIO;
	}

	lsb = (buf[0] & BMA456_ACCEL_LSB_MASK) >> BMA456_ACCEL_LSB_SHIFT;
	drv_data->x_sample = (((int8_t)buf[1]) << BMA456_ACCEL_LSB_BITS) | lsb;

	lsb = (buf[2] & BMA456_ACCEL_LSB_MASK) >> BMA456_ACCEL_LSB_SHIFT;
	drv_data->y_sample = (((int8_t)buf[3]) << BMA456_ACCEL_LSB_BITS) | lsb;

	lsb = (buf[4] & BMA456_ACCEL_LSB_MASK) >> BMA456_ACCEL_LSB_SHIFT;
	drv_data->z_sample = (((int8_t)buf[5]) << BMA456_ACCEL_LSB_BITS) | lsb;

	if (i2c_reg_read_byte_dt(&config->i2c,
				 BMA456_REG_TEMP,
				 (uint8_t *)&drv_data->temp_sample) < 0) {
		LOG_DBG("Could not read temperature data");
		return -EIO;
	}

	return 0;
}

static void bma456_channel_accel_convert(struct sensor_value *val,
					int64_t raw_val)
{
	/*
	 * accel_val = (sample * BMA456_PMU_FULL_RAGE) /
	 *             (2^data_width * 10^6)
	 */
	raw_val = (raw_val * BMA456_PMU_FULL_RANGE) /
		  (1 << (8 + BMA456_ACCEL_LSB_BITS));
	val->val1 = raw_val / 1000000;
	val->val2 = raw_val % 1000000;

	/* normalize val to make sure val->val2 is positive */
	if (val->val2 < 0) {
		val->val1 -= 1;
		val->val2 += 1000000;
	}
}

static int bma456_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bma456_data *drv_data = dev->data;

	/*
	 * See datasheet "Sensor data" section for
	 * more details on processing sample data.
	 */
	if (chan == SENSOR_CHAN_ACCEL_X) {
		bma456_channel_accel_convert(val, drv_data->x_sample);
	} else if (chan == SENSOR_CHAN_ACCEL_Y) {
		bma456_channel_accel_convert(val, drv_data->y_sample);
	} else if (chan == SENSOR_CHAN_ACCEL_Z) {
		bma456_channel_accel_convert(val, drv_data->z_sample);
	} else if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		bma456_channel_accel_convert(val, drv_data->x_sample);
		bma456_channel_accel_convert(val + 1, drv_data->y_sample);
		bma456_channel_accel_convert(val + 2, drv_data->z_sample);
	} else if (chan == SENSOR_CHAN_DIE_TEMP) {
		/* temperature_val = 23 + sample / 2 */
		val->val1 = (drv_data->temp_sample >> 1) + 23;
		val->val2 = 500000 * (drv_data->temp_sample & 1);
		return 0;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api bma456_driver_api = {
#if CONFIG_BMA456_TRIGGER
	.attr_set = bma456_attr_set,
	.trigger_set = bma456_trigger_set,
#endif
	.sample_fetch = bma456_sample_fetch,
	.channel_get = bma456_channel_get,
};

int bma456_init(const struct device *dev)
{
	const struct bma456_config *config = dev->config;
	uint8_t id = 0U;
	LOG_INF("BMA456 init %s", dev->name);
	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* read device ID */
	if (i2c_reg_read_byte_dt(&config->i2c,
				 BMA456_REG_CHIP_ID, &id) < 0) {
		LOG_ERR("Could not read chip id");
		return -EIO;
	}

	if (id != BMA456_CHIP_ID) {
		LOG_DBG("Unexpected chip id (%x)", id);
		return -EIO;
	}

	if (i2c_reg_write_byte_dt(&config->i2c,
				  BMA456_REG_PMU_BW, BMA456_PMU_BW) < 0) {
		LOG_DBG("Could not set data filter bandwidth");
		return -EIO;
	}

	/* set g-range */
	if (i2c_reg_write_byte_dt(&config->i2c,
				  BMA456_REG_PMU_RANGE, BMA456_PMU_RANGE) < 0) {
		LOG_DBG("Could not set data g-range");
		return -EIO;
	}

#ifdef CONFIG_BMA456_TRIGGER
	if (config->int1_gpio.port) {
		if (bma456_init_interrupt(dev) < 0) {
			LOG_DBG("Could not initialize interrupts");
			return -EIO;
		}
	}
#endif

	return 0;
}

#define BMA456_DEFINE(inst)									\
	static struct bma456_data bma456_data_##inst;						\
												\
	static const struct bma456_config bma456_config##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		IF_ENABLED(CONFIG_BMA456_TRIGGER,						\
			   (.int1_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),	\
			   .int2_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 1 }),))	\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, bma456_init, NULL, &bma456_data_##inst,		\
			      &bma456_config##inst, POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY, &bma456_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(BMA456_DEFINE)
