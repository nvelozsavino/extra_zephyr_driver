/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma456


#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "bosch/bma4.h"

#ifdef CONFIG_BMA456_VARIANT_MM
#include "bosch/bma456mm.h"
#endif
#ifdef CONFIG_BMA456_VARIANT_H
#include "bosch/bma456h.h"
#endif
#ifdef CONFIG_BMA456_VARIANT_W
#include "bosch/bma456w.h"
#endif
#ifdef CONFIG_BMA456_VARIANT_AN
#include "bosch/bma456an.h"
#endif


LOG_MODULE_REGISTER(bma456, CONFIG_SENSOR_LOG_LEVEL);
#include "bma456.h"


#define BMA4_READ_WRITE_LEN  UINT8_C(46)


#define ACCEL_SCALE(sensitivity)			\
	((SENSOR_G * (sensitivity) >> 14) / 100)

/*
 * Use values for low-power mode in DS "Mechanical (Sensor) characteristics",
 * multiplied by 100.
 */
static uint32_t bma456_reg_val_to_scale[] = {
	ACCEL_SCALE(1600),
	ACCEL_SCALE(3200),
	ACCEL_SCALE(6400),
	ACCEL_SCALE(19200),
};

static void bma456_convert(int16_t raw_val, uint32_t scale,
			   struct sensor_value *val)
{
	int32_t converted_val;

	/*
	 * maximum converted value we can get is: max(raw_val) * max(scale)
	 *	max(raw_val >> 4) = +/- 2^11
	 *	max(scale) = 114921
	 *	max(converted_val) = 235358208 which is less than 2^31
	 */
	converted_val = (raw_val >> 4) * scale;
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

static int bma456_sample_fetch_temp(const struct device *dev)
{
	int ret = -ENOTSUP;

#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;
	uint8_t raw[sizeof(uint16_t)];

	ret = bma456->hw_tf->read_data(dev, cfg->temperature.dout_addr, raw,
				       sizeof(raw));

	if (ret < 0) {
		LOG_WRN("Failed to fetch raw temperature sample");
		ret = -EIO;
	} else {
		/*
		 * The result contains a delta value for the
		 * temperature that must be added to the reference temperature set
		 * for your board to return an absolute temperature in Celsius.
		 *
		 * The data is left aligned.  Fixed point after first 8 bits.
		 */
		bma456->temperature.val1 = (int32_t)((int8_t)raw[1]);
		if (cfg->temperature.fractional_bits == 0) {
			bma456->temperature.val2 = 0;
		} else {
			bma456->temperature.val2 =
				(raw[0] >> (8 - cfg->temperature.fractional_bits));
			bma456->temperature.val2 = (bma456->temperature.val2 * 1000000);
			bma456->temperature.val2 >>= cfg->temperature.fractional_bits;
			if (bma456->temperature.val1 < 0) {
				bma456->temperature.val2 *= -1;
			}
		}
	}
#else
	LOG_WRN("Temperature measurement disabled");
#endif

	return ret;
}

static int bma456_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bma456_data *bma456 = dev->data;
	int ofs_start;
	int ofs_end;
	int i;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ofs_start = ofs_end = 0;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ofs_start = ofs_end = 1;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ofs_start = ofs_end = 2;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		ofs_start = 0;
		ofs_end = 2;
		break;
#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
	case SENSOR_CHAN_DIE_TEMP:
		memcpy(val, &bma456->temperature, sizeof(*val));
		return 0;
#endif
	default:
		return -ENOTSUP;
	}

	for (i = ofs_start; i <= ofs_end; i++, val++) {
		bma456_convert(bma456->sample.xyz[i], bma456->scale, val);
	}

	return 0;
}

static int bma456_fetch_xyz(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bma456_data *bma456 = dev->data;
	int status = -ENODATA;
	size_t i;
	/*
	 * since status and all accel data register addresses are consecutive,
	 * a burst read can be used to read all the samples
	 */
	status = bma456->hw_tf->read_data(dev, BMA456_REG_STATUS,
					  bma456->sample.raw,
					  sizeof(bma456->sample.raw));
	if (status < 0) {
		LOG_WRN("Could not read accel axis data");
		return status;
	}

	for (i = 0; i < (3 * sizeof(int16_t)); i += sizeof(int16_t)) {
		int16_t *sample =
			(int16_t *)&bma456->sample.raw[1 + i];

		*sample = sys_le16_to_cpu(*sample);
	}

	if (bma456->sample.status & BMA456_STATUS_DRDY_MASK) {
		status = 0;
	}

	return status;
}

static int bma456_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	int status = -ENODATA;

	if (chan == SENSOR_CHAN_ALL) {
		status = bma456_fetch_xyz(dev, chan);
#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
		if (status == 0) {
			status = bma456_sample_fetch_temp(dev);
		}
#endif
	} else if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		status = bma456_fetch_xyz(dev, chan);
	} else if (chan == SENSOR_CHAN_DIE_TEMP) {
		status = bma456_sample_fetch_temp(dev);
	} else {
		__ASSERT(false, "Invalid sensor channel in fetch");
	}

	return status;
}

#ifdef CONFIG_BMA456_ODR_RUNTIME
/* 1620 & 5376 are low power only */
static const uint16_t bma456_odr_map[] = {0, 1, 10, 25, 50, 100, 200, 400, 1620,
				       1344, 5376};

static int bma456_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(bma456_odr_map); i++) {
		if (freq == bma456_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int bma456_acc_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;
	int status;
	uint8_t value;
	struct bma456_data *data = dev->data;

	odr = bma456_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	status = data->hw_tf->read_reg(dev, BMA456_REG_CTRL1, &value);
	if (status < 0) {
		return status;
	}

	/* some odr values cannot be set in certain power modes */
	if ((value & BMA456_LP_EN_BIT_MASK) == 0U && odr == BMA456_ODR_8) {
		return -ENOTSUP;
	}

	/* adjust odr index for LP enabled mode, see table above */
	if (((value & BMA456_LP_EN_BIT_MASK) == BMA456_LP_EN_BIT_MASK) &&
		(odr == BMA456_ODR_9 + 1)) {
		odr--;
	}

	return data->hw_tf->write_reg(dev, BMA456_REG_CTRL1,
				      (value & ~BMA456_ODR_MASK) |
				      BMA456_ODR_RATE(odr));
}
#endif

#ifdef CONFIG_BMA456_ACCEL_RANGE_RUNTIME

#define BMA456_RANGE_IDX_TO_VALUE(idx)		(1 << ((idx) + 1))
#define BMA456_NUM_RANGES			4

static int bma456_range_to_reg_val(uint16_t range)
{
	int i;

	for (i = 0; i < BMA456_NUM_RANGES; i++) {
		if (range == BMA456_RANGE_IDX_TO_VALUE(i)) {
			return i;
		}
	}

	return -EINVAL;
}

static int bma456_acc_range_set(const struct device *dev, int32_t range)
{
	struct bma456_data *bma456 = dev->data;
	int fs;

	fs = bma456_range_to_reg_val(range);
	if (fs < 0) {
		return fs;
	}

	bma456->scale = bma456_reg_val_to_scale[fs];

	return bma456->hw_tf->update_reg(dev, BMA456_REG_CTRL4,
					 BMA456_FS_MASK,
					 (fs << BMA456_FS_SHIFT));
}
#endif

static int bma456_acc_config(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	switch (attr) {
#ifdef CONFIG_BMA456_ACCEL_RANGE_RUNTIME
	case SENSOR_ATTR_FULL_SCALE:
		return bma456_acc_range_set(dev, sensor_ms2_to_g(val));
#endif
#ifdef CONFIG_BMA456_ODR_RUNTIME
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return bma456_acc_odr_set(dev, val->val1);
#endif
#if defined(CONFIG_BMA456_TRIGGER)
	case SENSOR_ATTR_SLOPE_TH:
	case SENSOR_ATTR_SLOPE_DUR:
		return bma456_acc_slope_config(dev, attr, val);
#endif
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int bma456_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return bma456_acc_config(dev, chan, attr, val);
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api bma456_driver_api = {
	.attr_set = bma456_attr_set,
#if CONFIG_BMA456_TRIGGER
	.trigger_set = bma456_trigger_set,
#endif
	.sample_fetch = bma456_sample_fetch,
	.channel_get = bma456_channel_get,
};


static BMA4_INTF_RET_TYPE bma4_bus_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr){
	const struct device *dev = intf_ptr;
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;

	
	int err = bma456->hw_tf->read_data(dev,reg_addr,read_data,len);
	if (err!=0){
		return BMA4_E_COM_FAIL;
	}
	return BMA4_INTF_RET_SUCCESS;
}

static BMA4_INTF_RET_TYPE bma4_bus_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr){
	const struct device *dev = intf_ptr;
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;

	
	int err = bma456->hw_tf->write_data(dev,reg_addr,read_data,len);
	if (err!=0){
		return BMA4_E_COM_FAIL;
	}
	return BMA4_INTF_RET_SUCCESS;
}

static void bma4_delay_us(uint32_t period, void *intf_ptr){
	k_sleep(K_USEC(period));
}

int bma456_init(const struct device *dev)
{
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	const struct bma456_config *cfg = dev->config;
	int status;
	uint8_t id;
	uint8_t raw[6];
	bma->intf_ptr = dev;
	bma->bus_read=bma4_bus_read;
	bma->bus_write=bma4_bus_write;
	bma->delay_us=bma4_delay_us;
	bma->variant = BMA45X_VARIANT;
	bma->read_write_len = BMA4_READ_WRITE_LEN;
	status = cfg->bus_init(dev);
	if (status < 0) {
		return status;
	}
	
	int8_t rslt=0;
#ifdef CONFIG_BMA456_VARIANT_MM
	rslt = bma456mm_init(bma);
	if (rslt<0){
		LOG_ERR("Error init bma456mm %d",rslt);
		return -EIO;
	}
	rslt = bma456mm_write_config_file(bma);
	if (rslt<0){
		LOG_ERR("Error writing bma456mm config %d",rslt);
		return -EIO;
	}
#endif
#ifdef CONFIG_BMA456_VARIANT_H
	rslt = bma456h_init(bma);
	if (rslt<0){
		LOG_ERR("Error init bma456h %d",rslt);
		return -EIO;
	}
	rslt = bma456h_write_config_file(bma);
	if (rslt<0){
		LOG_ERR("Error writing bma456h config %d",rslt);
		return -EIO;
	}
#endif

#ifdef CONFIG_BMA456_VARIANT_W
	rslt = bma456w_init(bma);
	if (rslt<0){
		LOG_ERR("Error init bma456w %d",rslt);
		return -EIO;
	}
	rslt = bma456w_write_config_file(bma);
	if (rslt<0){
		LOG_ERR("Error writing bma456w config %d",rslt);
		return -EIO;
	}
#endif
#ifdef CONFIG_BMA456_VARIANT_AN
	rslt = bma456an_init(bma);
	if (rslt<0){
		LOG_ERR("Error init bma456an %d",rslt);
		return -EIO;
	}
	rslt = bma456an_write_config_file(bma);
	if (rslt<0){
		LOG_ERR("Error writing bma456an config %d",rslt);
		return -EIO;
	}
#endif


	rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
	if (rslt<0){
		LOG_ERR("Error enabling accelerometer %d",rslt);
		return -EIO;
	}


	
	status = bma456->hw_tf->read_reg(dev, BMA456_REG_WAI, &id);
	if (status < 0) {
		LOG_ERR("Failed to read chip id.");
		return status;
	}

	if (id != BMA456_CHIP_ID) {
		LOG_ERR("Invalid chip ID: %02x\n", id);
		return -EINVAL;
	}

	/* Fix LSM303AGR_ACCEL device scale values */
	if (cfg->hw.is_lsm303agr_dev) {
		bma456_reg_val_to_scale[0] = ACCEL_SCALE(1563);
		bma456_reg_val_to_scale[1] = ACCEL_SCALE(3126);
		bma456_reg_val_to_scale[2] = ACCEL_SCALE(6252);
		bma456_reg_val_to_scale[3] = ACCEL_SCALE(18758);
	}

	if (cfg->hw.disc_pull_up) {
		status = bma456->hw_tf->update_reg(dev, BMA456_REG_CTRL0,
						   BMA456_SDO_PU_DISC_MASK,
						   BMA456_SDO_PU_DISC_MASK);
		if (status < 0) {
			LOG_ERR("Failed to disconnect SDO/SA0 pull-up.");
			return status;
		}
	}

	/* Initialize control register ctrl1 to ctrl 6 to default boot values
	 * to avoid warm start/reset issues as the accelerometer has no reset
	 * pin. Register values are retained if power is not removed.
	 * Default values see BMA456 documentation page 30, chapter 6.
	 */
	(void)memset(raw, 0, sizeof(raw));
	raw[0] = BMA456_ACCEL_EN_BITS;

	status = bma456->hw_tf->write_data(dev, BMA456_REG_CTRL1, raw,
					   sizeof(raw));

	if (status < 0) {
		LOG_ERR("Failed to reset ctrl registers.");
		return status;
	}

	/* set full scale range and store it for later conversion */
	bma456->scale = bma456_reg_val_to_scale[BMA456_FS_IDX];
#ifdef CONFIG_BMA456_BLOCK_DATA_UPDATE
	status = bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL4,
					  BMA456_FS_BITS | BMA456_HR_BIT | BMA456_CTRL4_BDU_BIT);
#else
	status = bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL4, BMA456_FS_BITS | BMA456_HR_BIT);
#endif

	if (status < 0) {
		LOG_ERR("Failed to set full scale ctrl register.");
		return status;
	}

#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
	status = bma456->hw_tf->update_reg(dev, cfg->temperature.cfg_addr,
					   cfg->temperature.enable_mask,
					   cfg->temperature.enable_mask);

	if (status < 0) {
		LOG_ERR("Failed to enable temperature measurement");
		return status;
	}
#endif

#ifdef CONFIG_BMA456_TRIGGER
	if (cfg->gpio_drdy.port != NULL || cfg->gpio_int.port != NULL) {
		status = bma456_init_interrupt(dev);
		if (status < 0) {
			LOG_ERR("Failed to initialize interrupts.");
			return status;
		}
	}
#endif

	LOG_INF("fs=%d, odr=0x%x lp_en=0x%x scale=%d", 1 << (BMA456_FS_IDX + 1), BMA456_ODR_IDX,
		(uint8_t)BMA456_LP_EN_BIT, bma456->scale);

	/* enable accel measurements and set power mode and data rate */
	return bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL1,
					BMA456_ACCEL_EN_BITS | BMA456_LP_EN_BIT |
					BMA456_ODR_BITS);
}

#ifdef CONFIG_PM_DEVICE
static int bma456_pm_action(const struct device *dev,
			    enum pm_device_action action)
{
	int status;
	struct bma456_data *bma456 = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Resume previous mode. */
		status = bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL1,
						  bma456->reg_ctrl1_active_val);
		if (status < 0) {
			LOG_ERR("failed to write reg_crtl1");
			return status;
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Store current mode, suspend. */
		status = bma456->hw_tf->read_reg(dev, BMA456_REG_CTRL1,
						 &bma456->reg_ctrl1_active_val);
		if (status < 0) {
			LOG_ERR("failed to read reg_crtl1");
			return status;
		}
		status = bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL1,
						  BMA456_SUSPEND);
		if (status < 0) {
			LOG_ERR("failed to write reg_crtl1");
			return status;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "BMA456 driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by BMA456_DEFINE_SPI() and
 * BMA456_DEFINE_I2C().
 */

#define BMA456_DEVICE_INIT(inst)					\
	PM_DEVICE_DT_INST_DEFINE(inst, bma456_pm_action);		\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			    bma456_init,				\
			    PM_DEVICE_DT_INST_GET(inst),		\
			    &bma456_data_##inst,			\
			    &bma456_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &bma456_driver_api);


#define ANYM_ON_INT1(inst) DT_INST_PROP(inst, anym_on_int1)

#define INT_LATCHED(inst) DT_INST_PROP(inst, int_latched)

#define ANYM_MODE(inst) DT_INST_PROP(inst, anym_mode)



#ifdef CONFIG_BMA456_TRIGGER
#define GPIO_DT_SPEC_INST_GET_BY_IDX_COND(id, prop, idx)		\
	COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx),		\
		    (GPIO_DT_SPEC_INST_GET_BY_IDX(id, prop, idx)),	\
		    ({.port = NULL, .pin = 0, .dt_flags = 0}))

#define BMA456_CFG_INT(inst)				\
	.gpio_int1 =							\
		GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, int_gpios, 0),	\
	.gpio_int2 =								\
		GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, int_gpios, 1),
#else
#define BMA456_CFG_INT(inst)
#endif /* CONFIG_BMA456_TRIGGER */

#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
/* The first 8 bits are the integer portion of the temperature.
 * The result is left justified.  The remainder of the bits are
 * the fractional part.
 *
 * BMA456 has 8 total bits.
 * BMA45612/LIS3DH have 10 bits unless they are in lower power mode.
 * compat(bma456) cannot be used here because it is the base part.
 */
#define FRACTIONAL_BITS(inst)	\
	(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), st_bma45612) ||				\
	 DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), st_lis3dh)) ?				\
		      (IS_ENABLED(CONFIG_BMA456_OPER_MODE_LOW_POWER) ? 0 : 2) : \
		      0

#define BMA456_CFG_TEMPERATURE(inst)	\
	.temperature = { .cfg_addr = 0x1F,	\
			 .enable_mask = 0xC0,		\
			 .dout_addr = 0x0C,			\
			 .fractional_bits = FRACTIONAL_BITS(inst) },
#else
#define BMA456_CFG_TEMPERATURE(inst)
#endif /* CONFIG_BMA456_MEASURE_TEMPERATURE */

#define BMA456_CONFIG_SPI(inst)						\
	{								\
		.bus_init = bma456_spi_init,				\
		.bus_cfg = { .spi = SPI_DT_SPEC_INST_GET(inst,		\
					SPI_WORD_SET(8) |		\
					SPI_OP_MODE_MASTER |		\
					SPI_MODE_CPOL |			\
					SPI_MODE_CPHA,			\
					0) },				\
		.hw = { \
			.int_latched = INT_LATCHED(inst), 	\
			.int_latched = INT_LATCHED(inst), },		\
		BMA456_CFG_TEMPERATURE(inst)				\
		BMA456_CFG_INT(inst)					\
	}

#define BMA456_DEFINE_SPI(inst)						\
	static struct bma456_data bma456_data_##inst;			\
	static const struct bma456_config bma456_config_##inst =	\
		BMA456_CONFIG_SPI(inst);				\
	BMA456_DEVICE_INIT(inst)

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define BMA456_CONFIG_I2C(inst)						\
	{								\
		.bus_init = bma456_i2c_init,				\
		.bus_cfg = { .i2c = I2C_DT_SPEC_INST_GET(inst), },	\
		.hw = { \
			.disc_pull_up = DISC_PULL_UP(inst),		\
			.anym_on_int1 = ANYM_ON_INT1(inst),		\
			.anym_latch = ANYM_LATCH(inst),			\
			.anym_mode = ANYM_MODE(inst), },		\
		BMA456_CFG_TEMPERATURE(inst)				\
		BMA456_CFG_INT(inst)					\
	}

#define BMA456_DEFINE_I2C(inst)						\
	static struct bma456_data bma456_data_##inst;			\
	static const struct bma456_config bma456_config_##inst =	\
		BMA456_CONFIG_I2C(inst);				\
	BMA456_DEVICE_INIT(inst)
/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define BMA456_DEFINE(inst)						\
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),				\
		    (BMA456_DEFINE_SPI(inst)),				\
		    (BMA456_DEFINE_I2C(inst)))

DT_INST_FOREACH_STATUS_OKAY(BMA456_DEFINE)
