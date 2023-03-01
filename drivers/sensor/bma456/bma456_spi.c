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
#include "bma456.h"
#include <zephyr/logging/log.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

LOG_MODULE_DECLARE(bma456, CONFIG_SENSOR_LOG_LEVEL);


#define BMA456_SPI_READ_BIT		BIT(7)
#define BMA456_SPI_AUTOINC		BIT(6)
#define BMA456_SPI_ADDR_MASK		BIT_MASK(6)

static int bma456_spi_read_data(const struct device *dev, uint8_t reg_addr,
			    uint8_t *value, uint32_t len)
{
	const struct bma456_config *cfg = dev->config;
	uint8_t buffer_tx[2] = { reg_addr | BMA456_SPI_READ_BIT, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};


	if (len > 64) {
		return -EIO;
	}

	if (len > 1) {
		buffer_tx[0] |= BMA456_SPI_AUTOINC;
	}

	if (spi_transceive_dt(&cfg->bus_cfg.spi, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

static int bma456_spi_write_data(const struct device *dev, uint8_t reg_addr,
			     uint8_t *value, uint32_t len)
{
	const struct bma456_config *cfg = dev->config;
	uint8_t buffer_tx[1] = { reg_addr & ~BMA456_SPI_READ_BIT };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};


	if (len > 64) {
		return -EIO;
	}

	if (len > 1) {
		buffer_tx[0] |= BMA456_SPI_AUTOINC;
	}

	if (spi_write_dt(&cfg->bus_cfg.spi, &tx)) {
		return -EIO;
	}

	return 0;
}

static const struct bma456_transfer_function bma456_spi_transfer_fn = {
	.read_data = bma456_spi_read_data,
	.write_data = bma456_spi_write_data,
};

int bma456_spi_init(const struct device *dev)
{
	struct bma456_data *data = dev->data;
	const struct bma456_config *cfg = dev->config;
	struct bma4_dev *bma = &data->bma;
	data->hw_tf = &bma456_spi_transfer_fn;
	bma->intf = BMA4_SPI_INTF;
	if (!spi_is_ready(&cfg->bus_cfg.spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
