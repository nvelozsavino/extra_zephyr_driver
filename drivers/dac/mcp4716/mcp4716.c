/*
 * Copyright (c) 2021 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT microchip_mcp4716

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>
#include "mcp4716.h"

LOG_MODULE_REGISTER(mcp4716, LOG_LEVEL_DBG);

/* Information in this file comes from MCP4716 datasheet revision D
 * found at https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22272C.pdf
 */

/* Defines for field values in MCP4716 DAC register */
#define MCP4716_DAC_MAX_VAL			(1<<MCP4716_RESOLUTION)-1

#define MCP4716_FAST_MODE_POWER_DOWN_POS	4U
#define MCP4716_FAST_MODE_DAC_UPPER_VAL_POS	8U
#define MCP4716_FAST_MODE_DAC_UPPER_VAL_MASK	0xF
#define MCP4716_FAST_MODE_DAC_LOWER_VAL_MASK	0xFF

#define MCP4716_READ_RDY_POS			7U
#define MCP4716_READ_RDY_MASK			(0x1 << MCP4716_READ_RDY_POS)

/* After writing eeprom, the MCP4716 can be in a busy state for 25 - 50ms
 * See section 1.0 of MCP4716 datasheet, 'Electrical Characteristics'
 */
#define MCP4716_BUSY_TIMEOUT_MS			60U

struct mcp4716_config {
	struct i2c_dt_spec i2c;
};

static inline int mcp4716_bus_config(const struct device *dev)
{
	const struct mcp4716_config *dev_cfg = dev->config;
	uint32_t i2c_cfg;

	i2c_cfg = I2C_MODE_CONTROLLER | I2C_SPEED_SET(MCP4716_BUS_SPEED);

	return i2c_configure(dev_cfg->i2c.bus, i2c_cfg);
}


/* Read mcp4716 and check RDY status bit */
static int mcp4716_wait_until_ready(const struct device *dev)
{
	const struct mcp4716_config *config = dev->config;
	uint8_t rx_data[5];
	bool mcp4716_ready = false;
	int ret;
	int32_t timeout = k_uptime_get_32() + MCP4716_BUSY_TIMEOUT_MS;
	ret = mcp4716_bus_config(dev);
	if (ret!=0){
		LOG_ERR("Error configuring I2C Bus %d",ret);
		return ret;
	}

	/* Wait until RDY bit is set or return error if timer exceeds MCP4716_BUSY_TIMEOUT_MS */
	while (!mcp4716_ready) {
		ret = i2c_read_dt(&config->i2c, rx_data, sizeof(rx_data));

		if (ret == 0) {
			mcp4716_ready = rx_data[0] & MCP4716_READ_RDY_MASK;
		} else {
			/* I2C error */
			return ret;
		}

		if (k_uptime_get_32() > timeout) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

/* MCP4716 is a single channel 12 bit DAC */
static int mcp4716_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	if (channel_cfg->channel_id != 0) {
		return -EINVAL;
	}

	if (channel_cfg->resolution != MCP4716_RESOLUTION) {
		return -ENOTSUP;
	}

	return 0;
}

static int mcp4716_write_value(const struct device *dev, uint8_t channel,
				uint32_t value)
{

	int ret = mcp4716_bus_config(dev);
	if (ret!=0){
		LOG_ERR("Error configuring I2C Bus %d",ret);
		return ret;
	}

	const struct mcp4716_config *config = dev->config;
	uint8_t tx_data[2];


	if (channel != 0) {
		return -EINVAL;
	}

	/* Check value isn't over 12 bits */
	if (value > MCP4716_DAC_MAX_VAL) {
		return -ENOTSUP;
	}

	/* WRITE_MODE_FAST message format (2 bytes):
	 *
	 * ||     15 14     |        13 12        |    11 10 9 8    || 7 6 5 4 3 2 1 0 ||
	 * || Fast mode (0) | Power-down bits (0) | DAC value[11:8] || DAC value[7:0]  ||
	 */
	tx_data[0] = ((value >> MCP4716_FAST_MODE_DAC_UPPER_VAL_POS) &
		MCP4716_FAST_MODE_DAC_UPPER_VAL_MASK);
	tx_data[1] = (value & MCP4716_FAST_MODE_DAC_LOWER_VAL_MASK);
	ret = i2c_write_dt(&config->i2c, tx_data, sizeof(tx_data));

	return ret;
}

static int dac_mcp4716_init(const struct device *dev)
{
	const struct mcp4716_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C device not found");
		return -EINVAL;
	}

	/* Check we can read a 'RDY' bit from this device */
	int err = mcp4716_wait_until_ready(dev);
	if (err) {
		LOG_ERR("Check we can read a RDY %d",err);
		return -EBUSY;
	}

	return 0;
}

static const struct dac_driver_api mcp4716_driver_api = {
	.channel_setup = mcp4716_channel_setup,
	.write_value = mcp4716_write_value,
};


#define INST_DT_MCP4716(index)						\
	static const struct mcp4716_config mcp4716_config_##index = {	\
		.i2c = I2C_DT_SPEC_INST_GET(index),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(index, dac_mcp4716_init,			\
			    NULL,					\
			    NULL,					\
			    &mcp4716_config_##index, POST_KERNEL,	\
			    CONFIG_DAC_MCP4716_INIT_PRIORITY,		\
			    &mcp4716_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INST_DT_MCP4716);
