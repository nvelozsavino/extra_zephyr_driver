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


#define ACCEL_SCALE(sensitivity,res)			\
	((SENSOR_G * (sensitivity))/ ((1<<(res-1))-1))


static void bma456_convert(int16_t raw_val, int32_t scale,
			   struct sensor_value *val)
{
	int32_t converted_val;

	/*
	 * maximum converted value we can get is: max(raw_val) * max(scale)
	 *	max(raw_val) = +/- 2^15
	 *	max(scale) = 9806650*16/(2^15 -1) =4788
	 *	max(converted_val) = 156893184 which is less than 2^31
	 */
	converted_val = raw_val * scale;
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

static int bma456_sample_fetch_temp(const struct device *dev)
{
	int ret = -ENOTSUP;

#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma  = &bma456->bma;

	int32_t temp;
	int rslt = bma4_get_temperature(&temp,BMA4_DEG, &bma);
	if (rslt!=BMA4_OK){
		LOG_WRN("Failed to fetch raw temperature sample");
		ret = -EIO;
	} else {
		bma456->temperature.val1 = temp/1000;
		bma456->temperature.val2 = temp%1000;
		ret=0;
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

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		bma456_convert(bma456->sens_data.x,bma456->scale,val);		
		break;
	case SENSOR_CHAN_ACCEL_Y:
		bma456_convert(bma456->sens_data.y,bma456->scale,val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		bma456_convert(bma456->sens_data.z,bma456->scale,val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		bma456_convert(bma456->sens_data.x,bma456->scale,&val[0]);
		bma456_convert(bma456->sens_data.y,bma456->scale,&val[1]);
		bma456_convert(bma456->sens_data.z,bma456->scale,&val[2]);
		break;
#ifdef CONFIG_BMA456_MEASURE_TEMPERATURE
	case SENSOR_CHAN_DIE_TEMP:
		memcpy(val, &bma456->temperature, sizeof(*val));
		return 0;
#endif
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int bma456_fetch_xyz(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma  = &bma456->bma;


	int rslt = bma4_read_accel_xyz(&bma456->sens_data, bma);
	if (rslt!=BMA4_OK){
		LOG_WRN("Could not read accel axis data");
		return -EIO;
	}

	return 0;


	// int status = -ENODATA;
	// size_t i;
	// /*
	//  * since status and all accel data register addresses are consecutive,
	//  * a burst read can be used to read all the samples
	//  */
	// status = bma456->hw_tf->read_data(dev, BMA456_REG_STATUS,
	// 				  bma456->sample.raw,
	// 				  sizeof(bma456->sample.raw));
	// if (status < 0) {
	// 	LOG_WRN("Could not read accel axis data");
	// 	return status;
	// }

	// for (i = 0; i < (3 * sizeof(int16_t)); i += sizeof(int16_t)) {
	// 	int16_t *sample =
	// 		(int16_t *)&bma456->sample.raw[1 + i];

	// 	*sample = sys_le16_to_cpu(*sample);
	// }

	// if (bma456->sample.status & BMA456_STATUS_DRDY_MASK) {
	// 	status = 0;
	// }

	// return status;
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


#define FLOAT_TO_MEG(val) (int32_t)(val*1e6)
#define SENSOR_TO_MEG(val) (int32_t)(val->val1*1000000 + val->val2)
typedef struct {
	int32_t freq;
	uint8_t code;
} bma456_freq_code_t;
static const bma456_freq_code_t bma456_odr_map[] = {
		{.freq =780000, .code= BMA4_OUTPUT_DATA_RATE_0_78HZ},
		{.freq =1560000,.code= BMA4_OUTPUT_DATA_RATE_1_56HZ},
		{.freq =3125000, .code= BMA4_OUTPUT_DATA_RATE_3_12HZ},
		{.freq =6250000, .code= BMA4_OUTPUT_DATA_RATE_6_25HZ},
		{.freq =12000000, .code= BMA4_OUTPUT_DATA_RATE_12_5HZ},
		{.freq =25000000, .code= BMA4_OUTPUT_DATA_RATE_25HZ},
		{.freq =50000000, .code= BMA4_OUTPUT_DATA_RATE_50HZ},
		{.freq =100000000, .code= BMA4_OUTPUT_DATA_RATE_100HZ},
		{.freq =200000000, .code= BMA4_OUTPUT_DATA_RATE_200HZ},
		{.freq =400000000, .code= BMA4_OUTPUT_DATA_RATE_400HZ},
		{.freq =800000000, .code= BMA4_OUTPUT_DATA_RATE_800HZ},
		{.freq =1600000000, .code= BMA4_OUTPUT_DATA_RATE_1600HZ}};

static int bma456_freq_to_odr_val(const struct sensor_value *val)
{
	size_t i;
	int32_t freq = SENSOR_TO_MEG(val);
	for (i = 0; i < ARRAY_SIZE(bma456_odr_map); i++) {
		const bma456_freq_code_t* freq_code = &bma456_odr_map[i];
		if (freq == freq_code->freq) {
			
			return i;
		}
	}
	LOG_ERR("Unsuported freq %d*10^(-6)",freq);
	return -EINVAL;
}

static int bma456_acc_odr_set(const struct device *dev, const struct sensor_value *val)
{
	int odr;
	struct bma456_data *data = dev->data;
	struct bma4_dev *bma = &data->bma;

	
	odr = bma456_freq_to_odr_val(val);
	if (odr < 0) {
		return odr;
	}
	struct bma4_accel_config accel_conf = { 0 };
	int rslt = bma4_get_accel_config(&accel_conf,bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to get accel config %d",rslt);
		return -EIO;
	}
	
	accel_conf.odr = bma456_odr_map[odr].code;

	rslt = bma4_set_accel_config(&accel_conf, bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to set accel config %d",rslt);
		return -EIO;
	}
	return 0;
}
#endif

#ifdef CONFIG_BMA456_ACCEL_RANGE_RUNTIME


/*
 * Use values for low-power mode in DS "Mechanical (Sensor) characteristics",
 * multiplied by 100.
 */
static const uint8_t bma456_ranges[] = {
	BMA4_ACCEL_RANGE_2G,
	BMA4_ACCEL_RANGE_4G,
	BMA4_ACCEL_RANGE_8G,
	BMA4_ACCEL_RANGE_16G,
};
#define BMA456_RANGE_IDX_TO_VALUE(idx)		(1 << ((idx) + 1))
#define BMA456_NUM_RANGES			ARRAY_SIZE(bma456_ranges)

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
	struct bma4_dev *bma = &bma456->bma;
	int fs;

	fs = bma456_range_to_reg_val(range);
	if (fs < 0) {
		LOG_ERR("Unsupported range %d",range);
		return fs;
	}
	bma456->scale=ACCEL_SCALE(range,bma->resolution);
	struct bma4_accel_config accel_conf = { 0 };
	int rslt = bma4_get_accel_config(&accel_conf,bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to get accel config %d",rslt);
		return -EIO;
	}
	
	accel_conf.range = bma456_ranges[fs];

	rslt = bma4_set_accel_config(&accel_conf, bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to set accel config %d",rslt);
		return -EIO;
	}
	return 0;
}
#endif


/*
 * Use values for low-power mode in DS "Mechanical (Sensor) characteristics",
 * multiplied by 100.
 */
static const uint8_t bma456_bandwidths[] = {
	BMA4_ACCEL_OSR4_AVG1,
	BMA4_ACCEL_OSR2_AVG2,
	BMA4_ACCEL_NORMAL_AVG4,
	BMA4_ACCEL_CIC_AVG8,
	BMA4_ACCEL_RES_AVG16,
	BMA4_ACCEL_RES_AVG32,
	BMA4_ACCEL_RES_AVG64,
	BMA4_ACCEL_RES_AVG128,
};
#define BMA456_NUM_BW			ARRAY_SIZE(bma456_bandwidths)

static int bma456_bw_to_reg_val(int32_t bw)
{
	int i;

	for (i = 0; i < BMA456_NUM_BW; i++) {
		if (bw == bma456_bandwidths[i]) {
			return i;
		}
	}

	return -EINVAL;
}



static int bma456_acc_bandwidth_set(const struct device *dev, int32_t bw)
{
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	int fs;

	fs = bma456_bw_to_reg_val(bw);
	if (fs < 0) {
		LOG_ERR("Unsupported bw %d",bw);
		return fs;
	}
	struct bma4_accel_config accel_conf = { 0 };
	int rslt = bma4_get_accel_config(&accel_conf,bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to get accel config %d",rslt);
		return -EIO;
	}
	
	accel_conf.bandwidth = bma456_bandwidths[fs];

	rslt = bma4_set_accel_config(&accel_conf, bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to set accel config %d",rslt);
		return -EIO;
	}
	return 0;
}



static int bma456_acc_perf_mode_set(const struct device *dev, int32_t perf_mode)
{
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	uint8_t pm = (uint8_t)perf_mode;
	if ((pm!=BMA4_CIC_AVG_MODE) && (pm!=BMA4_CONTINUOUS_MODE)){
		LOG_ERR("Unsupported perf_mode %d",perf_mode);
		return -EINVAL;
	}
	struct bma4_accel_config accel_conf = { 0 };
	int rslt = bma4_get_accel_config(&accel_conf,bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to get accel config %d",rslt);
		return -EIO;
	}
	
	accel_conf.perf_mode = pm;

	rslt = bma4_set_accel_config(&accel_conf, bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to set accel config %d",rslt);
		return -EIO;
	}
	return 0;
}


static int bma456_acc_config(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	switch ((int)attr) {
#ifdef CONFIG_BMA456_ACCEL_RANGE_RUNTIME
	case SENSOR_ATTR_FULL_SCALE:
		return bma456_acc_range_set(dev, sensor_ms2_to_g(val));
#endif
#ifdef CONFIG_BMA456_ODR_RUNTIME
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return bma456_acc_odr_set(dev, val);
#endif
	case BMA456_ATTR_BANDWIDTH:
		return bma456_acc_bandwidth_set(dev,val->val1);
	case BMA456_ATTR_PERF_MODE:
		return bma456_acc_perf_mode_set(dev,val->val1);

// #if defined(CONFIG_BMA456_TRIGGER)
// 	case SENSOR_ATTR_SLOPE_TH:
// 	case SENSOR_ATTR_SLOPE_DUR:
// 		return bma456_acc_slope_config(dev, attr, val);
// #endif
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
	// const struct bma456_config *cfg = dev->config;

	
	int err = bma456->hw_tf->read_data(dev,reg_addr,read_data,len);
	if (err!=0){
		return BMA4_E_COM_FAIL;
	}
	return BMA4_INTF_RET_SUCCESS;
}

static BMA4_INTF_RET_TYPE bma4_bus_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr){
	const struct device *dev = intf_ptr;
	struct bma456_data *bma456 = dev->data;
	// const struct bma456_config *cfg = dev->config;

	
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
	bma->intf_ptr = (void*) dev;
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

	
#ifdef CONFIG_BMA456_TRIGGER
	if (cfg->gpio_int1.port != NULL || cfg->gpio_int2.port != NULL) {
		status = bma456_init_interrupt(dev);
		if (status < 0) {
			LOG_ERR("Failed to initialize interrupts.");
			return status;
		}
	}
#endif
	struct bma4_accel_config accel_conf = { 0 };
	rslt = bma4_get_accel_config(&accel_conf,bma);
	if (rslt!=BMA4_OK){
		LOG_ERR("Failed to get accel config %d",rslt);
		return -EIO;
	}
#ifndef CONFIG_BMA456_ACCEL_RANGE_RUNTIME

#ifdef BMA456_ACCEL_RANGE_2G
	accel_conf.range = BMA4_ACCEL_RANGE_2G;
	bma456->ACCEL_SCALE(2,bma->resolution);
#elif BMA456_ACCEL_RANGE_4G
    accel_conf.range = BMA4_ACCEL_RANGE_4G;
	bma456->scale=ACCEL_SCALE(4,bma->resolution);
#elif BMA456_ACCEL_RANGE_8G
    accel_conf.range = BMA4_ACCEL_RANGE_8G;
	bma456->scale=ACCEL_SCALE(8,bma->resolution);
#elif BMA456_ACCEL_RANGE_16G
    accel_conf.range = BMA4_ACCEL_RANGE_16G;
	bma456->scale=ACCEL_SCALE(16,bma->resolution);
#endif

#endif

#ifndef CONFIG_BMA456_ODR_RUNTIME
#ifdef BMA456_ODR_0_78
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_0_78HZ;
#elif BMA456_ODR_1_56
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_1_56HZ;
#elif BMA456_ODR_3_125
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_3_12HZ;

#elif BMA456_ODR_6_25
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_6_25HZ;
#elif BMA456_ODR_12_5
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_12_5HZ;
#elif BMA456_ODR_25
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_25HZ;
#elif BMA456_ODR_50
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
#elif BMA456_ODR_100
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
#elif BMA456_ODR_200
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_200HZ;
#elif BMA456_ODR_400
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_400HZ;
#elif BMA456_ODR_800
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_800HZ;
#elif BMA456_ODR_1600
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_1600HZ;
#endif
#endif


    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheet.
     */
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;


    rslt = bma4_set_accel_config(&accel_conf, bma);
	if (rslt<0){
		LOG_ERR("Error setting accelerometer config %d",rslt);
		return -EIO;
	}

	rslt = bma4_set_accel_enable(BMA4_ENABLE, bma);
	if (rslt<0){
		LOG_ERR("Error enabling accelerometer %d",rslt);
		return -EIO;
	}


	return 0;

}

#ifdef CONFIG_PM_DEVICE
static int bma456_pm_action(const struct device *dev,
			    enum pm_device_action action)
{
	
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	int8_t rslt = BMA4_OK;
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Resume previous mode. */		
		rslt = bma4_set_advance_power_save(bma456->adv_pwr_save,bma);
		if (rslt != BMA4_OK) {
			LOG_ERR("failed to read reg_crtl1");
			return -EIO;
		}

		rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
		if (rslt != BMA4_OK) {
			LOG_ERR("failed to disable accelerometer");
			return -EIO;
		}
		
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Store current mode, suspend. */
		rslt = bma4_get_advance_power_save(&bma456->adv_pwr_save,bma);
		if (rslt != BMA4_OK) {
			LOG_ERR("failed to read reg_crtl1");
			return -EIO;
		}

		rslt = bma4_set_accel_enable(BMA4_DISABLE, &bma);
		if (rslt != BMA4_OK) {
			LOG_ERR("failed to disable accelerometer");
			return -EIO;
		}

		rslt = bma4_set_advance_power_save(BMA4_ADVANCE_POWER_SAVE_MSK,bma);
		if (rslt != BMA4_OK) {
			LOG_ERR("failed to write reg_crtl1");
			return -EIO;
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

#define INT_NON_LATCHED(inst) DT_INST_PROP(inst, int_non_latched)

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
			.int_non_latched = INT_NON_LATCHED(inst), },		\
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
			.int_non_latched = INT_NON_LATCHED(inst), },		\
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
