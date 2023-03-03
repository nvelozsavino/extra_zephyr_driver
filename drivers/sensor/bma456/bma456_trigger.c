/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma456

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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


#define START_TRIG_INT1			0
#define START_TRIG_INT2			1
#define TRIGGED_INT1			4
#define TRIGGED_INT2			5

LOG_MODULE_DECLARE(bma456, CONFIG_SENSOR_LOG_LEVEL);
#include "bma456.h"


static const uint8_t int1_triggers_array[] =  { 
	0,  /* Both int1 and int2 disabled */ 
	(BMA456_TRIGGER_DATA_READY|BMA456_TRIGGER_ANY_MOTION|BMA456_TRIGGER_NO_MOTION|BMA456_TRIGGER_TAP|BMA456_TRIGGER_DOUBLE_TAP), /* only int1 enabled */ 
	0,  /* only int2 enabled */ 
	BMA456_TRIGGER_DATA_READY  /* Both enabled */ 
};

static const uint8_t int2_triggers_array[] =  { 
	0, 	/* Both int1 and int2 disabled*/ 
	0, /* only int1 enabled */ 
	(BMA456_TRIGGER_DATA_READY|BMA456_TRIGGER_ANY_MOTION|BMA456_TRIGGER_NO_MOTION|BMA456_TRIGGER_TAP|BMA456_TRIGGER_DOUBLE_TAP), /* only int2 enabled */ 
	(BMA456_TRIGGER_ANY_MOTION|BMA456_TRIGGER_NO_MOTION|BMA456_TRIGGER_TAP|BMA456_TRIGGER_DOUBLE_TAP) /* Both enabled */ 
};

static inline void setup_int1(const struct device *dev,
			      bool enable)
{
	const struct bma456_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int1,
					enable
					? GPIO_INT_LEVEL_ACTIVE
					: GPIO_INT_DISABLE);
}

static inline void setup_int2(const struct device *dev,
			      bool enable)
{
	const struct bma456_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int2,
					enable
					? GPIO_INT_LEVEL_ACTIVE
					: GPIO_INT_DISABLE);
}

static int bma456_trigger_drdy_set(const struct device *dev,
				   enum sensor_channel chan,
				   sensor_trigger_handler_t handler)
{
	const struct bma456_config *cfg = dev->config;
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	int status;
	if (bma456->int1_triggers & BMA456_TRIGGER_DATA_READY){
		if (cfg->gpio_int1.port == NULL) {
			LOG_ERR("trigger_set DRDY int not supported");
			return -ENOTSUP;
		}
		setup_int1(dev, false);
	}

	
    struct bma4_accel_config accel_conf = { 0 };

	/* Accelerometer configuration settings */
    /* Output data Rate */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA4_ACCEL_RANGE_2G;

    /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
     * if it is set to 2, then 2^(bandwidth parameter) samples
     * are averaged, resulting in 4 averaged samples
     * Note1 : For more information, refer the datasheet.
     * Note2 : A higher number of averaged samples will result in a less noisier signal, but
     * this has an adverse effect on the power consumed.
     */
    accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Averaging samples (Default)
     *  1 -> No averaging
     * For more info on No Averaging mode refer datasheet.
     */
    accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

    /* Set the accel configurations */
    rslt = bma4_set_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    /* NOTE : Enable accel after set of configurations */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    /* Mapping data ready interrupt with interrupt pin 1 to get interrupt status once getting new accel data */
    rslt = bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma456mm_map_interrupt status", rslt);


	/* cancel potentially pending trigger */
	atomic_clear_bit(&bma456->trig_flags, TRIGGED_INT1);
    int8_t rslt = bma4_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT, BMA4_DISABLE, &bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Error clearing data rdy on int1 (%d)", rslt);
		return -EIO;
	}

	bma456->handler_drdy = handler;
	if ((handler == NULL) || (status < 0)) {
		return status;
	}

	bma456->chan_drdy = chan;

	/* serialize start of int1 in thread to synchronize output sampling
	 * and first interrupt. this avoids concurrent bus context access.
	 */
	atomic_set_bit(&bma456->trig_flags, START_TRIG_INT1);
#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_give(&bma456->gpio_sem);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bma456->work);
#endif

	return 0;
}

static int bma456_start_trigger_int1(const struct device *dev)
{
	int status;
	uint8_t raw[BMA456_BUF_SZ];
	uint8_t ctrl1 = 0U;
	struct bma456_data *bma456 = dev->data;

	/* power down temporarily to align interrupt & data output sampling */
	status = bma456->hw_tf->read_reg(dev, BMA456_REG_CTRL1, &ctrl1);
	if (unlikely(status < 0)) {
		return status;
	}
	status = bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL1,
					  ctrl1 & ~BMA456_ODR_MASK);

	if (unlikely(status < 0)) {
		return status;
	}

	LOG_DBG("ctrl1=0x%x @tick=%u", ctrl1, k_cycle_get_32());

	/* empty output data */
	status = bma456->hw_tf->read_data(dev, BMA456_REG_STATUS,
					  raw, sizeof(raw));
	if (unlikely(status < 0)) {
		return status;
	}

	setup_int1(dev, true);

	/* re-enable output sampling */
	status = bma456->hw_tf->write_reg(dev, BMA456_REG_CTRL1, ctrl1);
	if (unlikely(status < 0)) {
		return status;
	}

	return bma456->hw_tf->update_reg(dev, BMA456_REG_CTRL3,
					 BMA456_EN_DRDY1_INT1,
					 BMA456_EN_DRDY1_INT1);
}

#define BMA456_ANYM_CFG (BMA456_INT_CFG_ZHIE_ZUPE | BMA456_INT_CFG_YHIE_YUPE |\
			 BMA456_INT_CFG_XHIE_XUPE)

static inline void setup_int2(const struct device *dev,
			      bool enable)
{
	const struct bma456_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int,
					enable
					? GPIO_INT_LEVEL_ACTIVE
					: GPIO_INT_DISABLE);
}

static int bma456_trigger_anym_set(const struct device *dev,
				   sensor_trigger_handler_t handler)
{
	const struct bma456_config *cfg = dev->config;
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;

	int status;
	uint8_t reg_val;

	if (cfg->gpio_int.port == NULL) {
		LOG_ERR("trigger_set AnyMotion int not supported");
		return -ENOTSUP;
	}

	setup_int2(dev, false);

	/* cancel potentially pending trigger */
	atomic_clear_bit(&bma456->trig_flags, TRIGGED_INT2);

	int8_t rslt;
	if (cfg->hw.anym_on_int1) {
		rslt = bma4_map_interrupt(BMA4_INTR1_MAP, (BMA456MM_ANY_MOT_INT | BMA456MM_NO_MOT_INT), BMA4_DISABLE, &bma);

		// status = bma456->hw_tf->update_reg(dev, BMA456_REG_CTRL3,
		// 				   BMA456_EN_DRDY1_INT1, 0);
	}

	/* disable any movement interrupt events */
	status = bma456->hw_tf->write_reg(
		dev,
		cfg->hw.anym_on_int1 ? BMA456_REG_INT1_CFG : BMA456_REG_INT2_CFG,
		0);

	/* make sure any pending interrupt is cleared */
	status = bma456->hw_tf->read_reg(
		dev,
		cfg->hw.anym_on_int1 ? BMA456_REG_INT1_SRC : BMA456_REG_INT2_SRC,
		&reg_val);

	bma456->handler_anymotion = handler;
	if ((handler == NULL) || (status < 0)) {
		return status;
	}

	/* serialize start of int2 in thread to synchronize output sampling
	 * and first interrupt. this avoids concurrent bus context access.
	 */
	atomic_set_bit(&bma456->trig_flags, START_TRIG_INT2);
#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_give(&bma456->gpio_sem);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bma456->work);
#endif
	return 0;
}

static int bma456_start_trigger_int2(const struct device *dev)
{
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;

	setup_int2(dev, true);

	return bma456->hw_tf->write_reg(
		dev,
		cfg->hw.anym_on_int1 ? BMA456_REG_INT1_CFG : BMA456_REG_INT2_CFG,
		(cfg->hw.anym_mode << BMA456_INT_CFG_MODE_SHIFT) | BMA456_ANYM_CFG);
}

int bma456_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		return bma456_trigger_drdy_set(dev, trig->chan, handler);
	} else if (trig->type == SENSOR_TRIG_DELTA) {
		return bma456_trigger_anym_set(dev, handler);
	}

	return -ENOTSUP;
}

// int bma456_acc_slope_config(const struct device *dev,
// 			    enum sensor_attribute attr,
// 			    const struct sensor_value *val)
// {
// 	struct bma456_data *bma456 = dev->data;
// 	const struct bma456_config *cfg = dev->config;
// 	int status;

// 	if (attr == SENSOR_ATTR_SLOPE_TH) {
// 		uint8_t range_g, reg_val;
// 		uint32_t slope_th_ums2;

// 		status = bma456->hw_tf->read_reg(dev, BMA456_REG_CTRL4,
// 						 &reg_val);
// 		if (status < 0) {
// 			return status;
// 		}

// 		/* fs reg value is in the range 0 (2g) - 3 (16g) */
// 		range_g = 2 * (1 << ((BMA456_FS_MASK & reg_val)
// 				      >> BMA456_FS_SHIFT));

// 		slope_th_ums2 = val->val1 * 1000000 + val->val2;

// 		/* make sure the provided threshold does not exceed range */
// 		if ((slope_th_ums2 - 1) > (range_g * SENSOR_G)) {
// 			return -EINVAL;
// 		}

// 		/* 7 bit full range value */
// 		reg_val = 128 / range_g * (slope_th_ums2 - 1) / SENSOR_G;

// 		LOG_INF("int2_ths=0x%x range_g=%d ums2=%u", reg_val,
// 			    range_g, slope_th_ums2 - 1);

// 		status = bma456->hw_tf->write_reg(dev,
// 						  cfg->hw.anym_on_int1 ?
// 								BMA456_REG_INT1_THS :
// 								BMA456_REG_INT2_THS,
// 						  reg_val);
// 	} else { /* SENSOR_ATTR_SLOPE_DUR */
// 		/*
// 		 * slope duration is measured in number of samples:
// 		 * N/ODR where N is the register value
// 		 */
// 		if (val->val1 < 0 || val->val1 > 127) {
// 			return -ENOTSUP;
// 		}

// 		LOG_INF("int2_dur=0x%x", val->val1);

// 		status = bma456->hw_tf->write_reg(dev,
// 						  cfg->hw.anym_on_int1 ?
// 								BMA456_REG_INT1_DUR :
// 								BMA456_REG_INT2_DUR,
// 						  val->val1);
// 	}

// 	return status;
// }

static void bma456_gpio_int1_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct bma456_data *bma456 =
		CONTAINER_OF(cb, struct bma456_data, gpio_int1_cb);

	ARG_UNUSED(pins);

	atomic_set_bit(&bma456->trig_flags, TRIGGED_INT1);

	/* int is level triggered so disable until processed */
	setup_int1(bma456->dev, false);

#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_give(&bma456->gpio_sem);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bma456->work);
#endif
}

static void bma456_gpio_int2_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct bma456_data *bma456 =
		CONTAINER_OF(cb, struct bma456_data, gpio_int2_cb);

	ARG_UNUSED(pins);

	atomic_set_bit(&bma456->trig_flags, TRIGGED_INT2);

	/* int is level triggered so disable until processed */
	setup_int2(bma456->dev, false);

#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_give(&bma456->gpio_sem);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bma456->work);
#endif
}

static void bma456_thread_cb(const struct device *dev)
{
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;
	int status;

	if (cfg->gpio_drdy.port &&
			unlikely(atomic_test_and_clear_bit(&bma456->trig_flags,
			START_TRIG_INT1))) {
		status = bma456_start_trigger_int1(dev);

		if (unlikely(status < 0)) {
			LOG_ERR("bma456_start_trigger_int1: %d", status);
		}
		return;
	}

	if (cfg->gpio_int.port &&
			unlikely(atomic_test_and_clear_bit(&bma456->trig_flags,
			START_TRIG_INT2))) {
		status = bma456_start_trigger_int2(dev);

		if (unlikely(status < 0)) {
			LOG_ERR("bma456_start_trigger_int2: %d", status);
		}
		return;
	}

	if (cfg->gpio_drdy.port &&
			atomic_test_and_clear_bit(&bma456->trig_flags,
			TRIGGED_INT1)) {
		struct sensor_trigger drdy_trigger = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = bma456->chan_drdy,
		};

		if (likely(bma456->handler_drdy != NULL)) {
			bma456->handler_drdy(dev, &drdy_trigger);

		}

		/* Reactivate level triggered interrupt if handler did not
		 * disable itself
		 */
		if (likely(bma456->handler_drdy != NULL)) {
			setup_int1(dev, true);
		}

		return;
	}

	if (cfg->gpio_int.port &&
			atomic_test_and_clear_bit(&bma456->trig_flags,
			TRIGGED_INT2)) {
		struct sensor_trigger anym_trigger = {
			.type = SENSOR_TRIG_DELTA,
			.chan = bma456->chan_drdy,
		};
		uint8_t reg_val;

		if (cfg->hw.anym_latch) {
			/* clear interrupt to de-assert int line */
			status = bma456->hw_tf->read_reg(dev,
							 cfg->hw.anym_on_int1 ?
								BMA456_REG_INT1_SRC :
								BMA456_REG_INT2_SRC,
							 &reg_val);
			if (status < 0) {
				LOG_ERR("clearing interrupt 2 failed: %d", status);
				return;
			}
		}

		if (likely(bma456->handler_anymotion != NULL)) {
			bma456->handler_anymotion(dev, &anym_trigger);
		}

		/* Reactivate level triggered interrupt if handler did not
		 * disable itself
		 */
		if (bma456->handler_anymotion != NULL) {
			setup_int2(dev, true);
		}

		LOG_DBG("@tick=%u int2_src=0x%x", k_cycle_get_32(),
			    reg_val);

		return;
	}
}

#ifdef CONFIG_BMA456_TRIGGER_OWN_THREAD
static void bma456_thread(struct bma456_data *bma456)
{
	while (1) {
		k_sem_take(&bma456->gpio_sem, K_FOREVER);
		bma456_thread_cb(bma456->dev);
	}
}
#endif

#ifdef CONFIG_BMA456_TRIGGER_GLOBAL_THREAD
static void bma456_work_cb(struct k_work *work)
{
	struct bma456_data *bma456 =
		CONTAINER_OF(work, struct bma456_data, work);

	bma456_thread_cb(bma456->dev);
}
#endif

int bma456_init_interrupt(const struct device *dev)
{
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;
	struct bma4_dev *bma = &bma456->bma;
	int status;
	uint8_t raw[2];

	/* disable interrupt in case of warm (re)boot */
	int8_t rslt = bma4_map_interrupt(BMA4_INTR1_MAP, 0xFFFF, BMA4_DISABLE, &bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Interrupt disable reg write failed (%d)", rslt);
		return -EIO;
	}

	rslt = bma4_set_interrupt_mode(cfg->hw.int_non_latched?BMA4_NON_LATCH_MODE:BMA4_LATCH_MODE,&bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Setting Latch mode %d failed (%d)", cfg->hw.int_latched, rslt);
		return -EIO;
	}




	/*
	 * Setup INT1 (for DRDY) if defined in DT
	 */
	uint8_t int_pins=0;
	/* setup data ready gpio interrupt */
	if (!device_is_ready(cfg->gpio_int1.port)) {
		/* API may return false even when ptr is NULL */
		if (cfg->gpio_int1.port != NULL) {
			LOG_ERR("device %s is not ready", cfg->gpio_int1.port->name);
			return -ENODEV;
		}
		LOG_DBG("gpio_int1 not defined in DT");
	} else {
		
		/* data ready int1 gpio configuration */
		status = gpio_pin_configure_dt(&cfg->gpio_int1, GPIO_INPUT);
		if (status < 0) {
			LOG_ERR("Could not configure %s.%02u",
				cfg->gpio_drdy.port->name, cfg->gpio_int1.pin);
			return status;
		}

		gpio_init_callback(&bma456->gpio_int1_cb,
				bma456_gpio_int1_callback,
				BIT(cfg->gpio_int1.pin));

		status = gpio_add_callback(cfg->gpio_int1.port, &bma456->gpio_int1_cb);
		if (status < 0) {
			LOG_ERR("Could not add gpio int1 callback");
			return status;
		}

		LOG_INF("%s: int1 on %s.%02u", dev->name,
						cfg->gpio_int1.port->name,
						cfg->gpio_int1.pin);
		int_pins|=1;		
	}

	if (!device_is_ready(cfg->gpio_int2.port)) {
		/* API may return false even when ptr is NULL */
		if (cfg->gpio_int2.port != NULL) {
			LOG_ERR("device %s is not ready", cfg->gpio_int2.port->name);
			return -ENODEV;
		}
		LOG_DBG("gpio_int2 not defined in DT");
	} else {
		status = gpio_pin_configure_dt(&cfg->gpio_int2, GPIO_INPUT);
		if (status < 0) {
			LOG_ERR("Could not configure %s.%02u",
				cfg->gpio_drdy.port->name, cfg->gpio_int2.pin);
			return status;
		}

		gpio_init_callback(&bma456->gpio_int2_cb,
				bma456_gpio_int2_callback,
				BIT(cfg->gpio_int2.pin));

		status = gpio_add_callback(cfg->gpio_int2.port, &bma456->gpio_int2_cb);
		if (status < 0) {
			LOG_ERR("Could not add gpio int2 callback");
			return status;
		}

		LOG_INF("%s: int2 on %s.%02u", dev->name,
						cfg->gpio_int2.port->name,
						cfg->gpio_int2.pin);
		int_pins|=2;		
	}


	if (int_pins==0){
		LOG_ERR("No int pin enabled");
		return -ENODEV;
	}

	bma456->int1_triggers = int1_triggers_array[int_pins];
	bma456->int2_triggers = int2_triggers_array[int_pins];

#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_init(&bma456->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&bma456->thread, bma456->thread_stack, CONFIG_BMA456_THREAD_STACK_SIZE,
			(k_thread_entry_t)bma456_thread, bma456, NULL, NULL,
			K_PRIO_COOP(CONFIG_BMA456_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	bma456->work.handler = bma456_work_cb;
#endif

	return status;
}
