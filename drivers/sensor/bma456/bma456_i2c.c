/* ST Microelectronics BMA456 3-axis accelerometer driver
 *
 * Copyright (c) 2020 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma456-ds000.pdf
 */

#define DT_DRV_COMPAT bosch_bma456

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "bma456.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(bma456, CONFIG_SENSOR_LOG_LEVEL);

static int bma456_i2c_read_data(const struct device *dev, uint8_t reg_addr,
				 uint8_t *value, uint32_t len)
{
	const struct bma456_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus_cfg.i2c, reg_addr, value, len);
}

static int bma456_i2c_write_data(const struct device *dev, uint8_t reg_addr,
				  const uint8_t *value, uint32_t len)
{
	const struct bma456_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus_cfg.i2c, reg_addr, value, len);
}

static const struct bma456_transfer_function bma456_i2c_transfer_fn = {
	.read_data = bma456_i2c_read_data,
	.write_data = bma456_i2c_write_data,
};

int bma456_i2c_init(const struct device *dev)
{
	struct bma456_data *data = dev->data;
	const struct bma456_config *cfg = dev->config;
	struct bma4_dev *bma = &data->bma;
	data->hw_tf = &bma456_i2c_transfer_fn;
	bma->intf = BMA4_I2C_INTF;
	if (!device_is_ready(cfg->bus_cfg.i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
