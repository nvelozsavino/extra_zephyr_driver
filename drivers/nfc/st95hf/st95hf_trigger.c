/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_st95hf

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define START_TRIG_INT1			0
#define START_TRIG_INT2			1
#define TRIGGED_INT1			4
#define TRIGGED_INT2			5

LOG_MODULE_DECLARE(st95hf, CONFIG_SENSOR_LOG_LEVEL);
#include "st95hf.h"

static inline void setup_int1(const struct device *dev,
			      bool enable)
{
	const struct st95hf_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy,
					enable
					? GPIO_INT_LEVEL_ACTIVE
					: GPIO_INT_DISABLE);
}

static int st95hf_trigger_drdy_set(const struct device *dev,
				   enum sensor_channel chan,
				   sensor_trigger_handler_t handler)
{
	const struct st95hf_config *cfg = dev->config;
	struct st95hf_data *st95hf = dev->data;
	int status;

	if (cfg->gpio_drdy.port == NULL) {
		LOG_ERR("trigger_set DRDY int not supported");
		return -ENOTSUP;
	}

	setup_int1(dev, false);

	/* cancel potentially pending trigger */
	atomic_clear_bit(&st95hf->trig_flags, TRIGGED_INT1);

	status = st95hf->hw_tf->update_reg(dev, ST95HF_REG_CTRL3,
					   ST95HF_EN_DRDY1_INT1, 0);

	st95hf->handler_drdy = handler;
	if ((handler == NULL) || (status < 0)) {
		return status;
	}

	st95hf->chan_drdy = chan;

	/* serialize start of int1 in thread to synchronize output sampling
	 * and first interrupt. this avoids concurrent bus context access.
	 */
	atomic_set_bit(&st95hf->trig_flags, START_TRIG_INT1);
#if defined(CONFIG_ST95HF_TRIGGER_OWN_THREAD)
	k_sem_give(&st95hf->gpio_sem);
#elif defined(CONFIG_ST95HF_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&st95hf->work);
#endif

	return 0;
}

static int st95hf_start_trigger_int1(const struct device *dev)
{
	int status;
	uint8_t raw[ST95HF_BUF_SZ];
	uint8_t ctrl1 = 0U;
	struct st95hf_data *st95hf = dev->data;

	/* power down temporarily to align interrupt & data output sampling */
	status = st95hf->hw_tf->read_reg(dev, ST95HF_REG_CTRL1, &ctrl1);
	if (unlikely(status < 0)) {
		return status;
	}
	status = st95hf->hw_tf->write_reg(dev, ST95HF_REG_CTRL1,
					  ctrl1 & ~ST95HF_ODR_MASK);

	if (unlikely(status < 0)) {
		return status;
	}

	LOG_DBG("ctrl1=0x%x @tick=%u", ctrl1, k_cycle_get_32());

	/* empty output data */
	status = st95hf->hw_tf->read_data(dev, ST95HF_REG_STATUS,
					  raw, sizeof(raw));
	if (unlikely(status < 0)) {
		return status;
	}

	setup_int1(dev, true);

	/* re-enable output sampling */
	status = st95hf->hw_tf->write_reg(dev, ST95HF_REG_CTRL1, ctrl1);
	if (unlikely(status < 0)) {
		return status;
	}

	return st95hf->hw_tf->update_reg(dev, ST95HF_REG_CTRL3,
					 ST95HF_EN_DRDY1_INT1,
					 ST95HF_EN_DRDY1_INT1);
}

#define ST95HF_ANYM_CFG (ST95HF_INT_CFG_ZHIE_ZUPE | ST95HF_INT_CFG_YHIE_YUPE |\
			 ST95HF_INT_CFG_XHIE_XUPE)

static inline void setup_int2(const struct device *dev,
			      bool enable)
{
	const struct st95hf_config *cfg = dev->config;

	gpio_pin_interrupt_configure_dt(&cfg->gpio_int,
					enable
					? GPIO_INT_LEVEL_ACTIVE
					: GPIO_INT_DISABLE);
}

static int st95hf_trigger_anym_set(const struct device *dev,
				   sensor_trigger_handler_t handler)
{
	const struct st95hf_config *cfg = dev->config;
	struct st95hf_data *st95hf = dev->data;
	int status;
	uint8_t reg_val;

	if (cfg->gpio_int.port == NULL) {
		LOG_ERR("trigger_set AnyMotion int not supported");
		return -ENOTSUP;
	}

	setup_int2(dev, false);

	/* cancel potentially pending trigger */
	atomic_clear_bit(&st95hf->trig_flags, TRIGGED_INT2);

	if (cfg->hw.anym_on_int1) {
		status = st95hf->hw_tf->update_reg(dev, ST95HF_REG_CTRL3,
						   ST95HF_EN_DRDY1_INT1, 0);
	}

	/* disable any movement interrupt events */
	status = st95hf->hw_tf->write_reg(
		dev,
		cfg->hw.anym_on_int1 ? ST95HF_REG_INT1_CFG : ST95HF_REG_INT2_CFG,
		0);

	/* make sure any pending interrupt is cleared */
	status = st95hf->hw_tf->read_reg(
		dev,
		cfg->hw.anym_on_int1 ? ST95HF_REG_INT1_SRC : ST95HF_REG_INT2_SRC,
		&reg_val);

	st95hf->handler_anymotion = handler;
	if ((handler == NULL) || (status < 0)) {
		return status;
	}

	/* serialize start of int2 in thread to synchronize output sampling
	 * and first interrupt. this avoids concurrent bus context access.
	 */
	atomic_set_bit(&st95hf->trig_flags, START_TRIG_INT2);
#if defined(CONFIG_ST95HF_TRIGGER_OWN_THREAD)
	k_sem_give(&st95hf->gpio_sem);
#elif defined(CONFIG_ST95HF_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&st95hf->work);
#endif
	return 0;
}

static int st95hf_start_trigger_int2(const struct device *dev)
{
	struct st95hf_data *st95hf = dev->data;
	const struct st95hf_config *cfg = dev->config;

	setup_int2(dev, true);

	return st95hf->hw_tf->write_reg(
		dev,
		cfg->hw.anym_on_int1 ? ST95HF_REG_INT1_CFG : ST95HF_REG_INT2_CFG,
		(cfg->hw.anym_mode << ST95HF_INT_CFG_MODE_SHIFT) | ST95HF_ANYM_CFG);
}

int st95hf_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	if (trig->type == SENSOR_TRIG_DATA_READY &&
	    trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		return st95hf_trigger_drdy_set(dev, trig->chan, handler);
	} else if (trig->type == SENSOR_TRIG_DELTA) {
		return st95hf_trigger_anym_set(dev, handler);
	}

	return -ENOTSUP;
}

int st95hf_acc_slope_config(const struct device *dev,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	struct st95hf_data *st95hf = dev->data;
	const struct st95hf_config *cfg = dev->config;
	int status;

	if (attr == SENSOR_ATTR_SLOPE_TH) {
		uint8_t range_g, reg_val;
		uint32_t slope_th_ums2;

		status = st95hf->hw_tf->read_reg(dev, ST95HF_REG_CTRL4,
						 &reg_val);
		if (status < 0) {
			return status;
		}

		/* fs reg value is in the range 0 (2g) - 3 (16g) */
		range_g = 2 * (1 << ((ST95HF_FS_MASK & reg_val)
				      >> ST95HF_FS_SHIFT));

		slope_th_ums2 = val->val1 * 1000000 + val->val2;

		/* make sure the provided threshold does not exceed range */
		if ((slope_th_ums2 - 1) > (range_g * SENSOR_G)) {
			return -EINVAL;
		}

		/* 7 bit full range value */
		reg_val = 128 / range_g * (slope_th_ums2 - 1) / SENSOR_G;

		LOG_INF("int2_ths=0x%x range_g=%d ums2=%u", reg_val,
			    range_g, slope_th_ums2 - 1);

		status = st95hf->hw_tf->write_reg(dev,
						  cfg->hw.anym_on_int1 ?
								ST95HF_REG_INT1_THS :
								ST95HF_REG_INT2_THS,
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

		status = st95hf->hw_tf->write_reg(dev,
						  cfg->hw.anym_on_int1 ?
								ST95HF_REG_INT1_DUR :
								ST95HF_REG_INT2_DUR,
						  val->val1);
	}

	return status;
}

static void st95hf_gpio_int1_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct st95hf_data *st95hf =
		CONTAINER_OF(cb, struct st95hf_data, gpio_int1_cb);

	ARG_UNUSED(pins);

	atomic_set_bit(&st95hf->trig_flags, TRIGGED_INT1);

	/* int is level triggered so disable until processed */
	setup_int1(st95hf->dev, false);

#if defined(CONFIG_ST95HF_TRIGGER_OWN_THREAD)
	k_sem_give(&st95hf->gpio_sem);
#elif defined(CONFIG_ST95HF_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&st95hf->work);
#endif
}

static void st95hf_gpio_int2_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct st95hf_data *st95hf =
		CONTAINER_OF(cb, struct st95hf_data, gpio_int2_cb);

	ARG_UNUSED(pins);

	atomic_set_bit(&st95hf->trig_flags, TRIGGED_INT2);

	/* int is level triggered so disable until processed */
	setup_int2(st95hf->dev, false);

#if defined(CONFIG_ST95HF_TRIGGER_OWN_THREAD)
	k_sem_give(&st95hf->gpio_sem);
#elif defined(CONFIG_ST95HF_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&st95hf->work);
#endif
}

static void st95hf_thread_cb(const struct device *dev)
{
	struct st95hf_data *st95hf = dev->data;
	const struct st95hf_config *cfg = dev->config;
	int status;

	if (cfg->gpio_drdy.port &&
			unlikely(atomic_test_and_clear_bit(&st95hf->trig_flags,
			START_TRIG_INT1))) {
		status = st95hf_start_trigger_int1(dev);

		if (unlikely(status < 0)) {
			LOG_ERR("st95hf_start_trigger_int1: %d", status);
		}
		return;
	}

	if (cfg->gpio_int.port &&
			unlikely(atomic_test_and_clear_bit(&st95hf->trig_flags,
			START_TRIG_INT2))) {
		status = st95hf_start_trigger_int2(dev);

		if (unlikely(status < 0)) {
			LOG_ERR("st95hf_start_trigger_int2: %d", status);
		}
		return;
	}

	if (cfg->gpio_drdy.port &&
			atomic_test_and_clear_bit(&st95hf->trig_flags,
			TRIGGED_INT1)) {
		struct sensor_trigger drdy_trigger = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = st95hf->chan_drdy,
		};

		if (likely(st95hf->handler_drdy != NULL)) {
			st95hf->handler_drdy(dev, &drdy_trigger);

		}

		/* Reactivate level triggered interrupt if handler did not
		 * disable itself
		 */
		if (likely(st95hf->handler_drdy != NULL)) {
			setup_int1(dev, true);
		}

		return;
	}

	if (cfg->gpio_int.port &&
			atomic_test_and_clear_bit(&st95hf->trig_flags,
			TRIGGED_INT2)) {
		struct sensor_trigger anym_trigger = {
			.type = SENSOR_TRIG_DELTA,
			.chan = st95hf->chan_drdy,
		};
		uint8_t reg_val;

		if (cfg->hw.anym_latch) {
			/* clear interrupt to de-assert int line */
			status = st95hf->hw_tf->read_reg(dev,
							 cfg->hw.anym_on_int1 ?
								ST95HF_REG_INT1_SRC :
								ST95HF_REG_INT2_SRC,
							 &reg_val);
			if (status < 0) {
				LOG_ERR("clearing interrupt 2 failed: %d", status);
				return;
			}
		}

		if (likely(st95hf->handler_anymotion != NULL)) {
			st95hf->handler_anymotion(dev, &anym_trigger);
		}

		/* Reactivate level triggered interrupt if handler did not
		 * disable itself
		 */
		if (st95hf->handler_anymotion != NULL) {
			setup_int2(dev, true);
		}

		LOG_DBG("@tick=%u int2_src=0x%x", k_cycle_get_32(),
			    reg_val);

		return;
	}
}

#ifdef CONFIG_ST95HF_TRIGGER_OWN_THREAD
static void st95hf_thread(struct st95hf_data *st95hf)
{
	while (1) {
		k_sem_take(&st95hf->gpio_sem, K_FOREVER);
		st95hf_thread_cb(st95hf->dev);
	}
}
#endif

#ifdef CONFIG_ST95HF_TRIGGER_GLOBAL_THREAD
static void st95hf_work_cb(struct k_work *work)
{
	struct st95hf_data *st95hf =
		CONTAINER_OF(work, struct st95hf_data, work);

	st95hf_thread_cb(st95hf->dev);
}
#endif

int st95hf_init_interrupt(const struct device *dev)
{
	struct st95hf_data *st95hf = dev->data;
	const struct st95hf_config *cfg = dev->config;
	int status;
	uint8_t raw[2];

	st95hf->dev = dev;

#if defined(CONFIG_ST95HF_TRIGGER_OWN_THREAD)
	k_sem_init(&st95hf->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&st95hf->thread, st95hf->thread_stack, CONFIG_ST95HF_THREAD_STACK_SIZE,
			(k_thread_entry_t)st95hf_thread, st95hf, NULL, NULL,
			K_PRIO_COOP(CONFIG_ST95HF_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_ST95HF_TRIGGER_GLOBAL_THREAD)
	st95hf->work.handler = st95hf_work_cb;
#endif

	/*
	 * Setup INT1 (for DRDY) if defined in DT
	 */

	/* setup data ready gpio interrupt */
	if (!device_is_ready(cfg->gpio_drdy.port)) {
		/* API may return false even when ptr is NULL */
		if (cfg->gpio_drdy.port != NULL) {
			LOG_ERR("device %s is not ready", cfg->gpio_drdy.port->name);
			return -ENODEV;
		}

		LOG_DBG("gpio_drdy not defined in DT");
		status = 0;
		goto check_gpio_int;
	}

	/* data ready int1 gpio configuration */
	status = gpio_pin_configure_dt(&cfg->gpio_drdy, GPIO_INPUT);
	if (status < 0) {
		LOG_ERR("Could not configure %s.%02u",
			cfg->gpio_drdy.port->name, cfg->gpio_drdy.pin);
		return status;
	}

	gpio_init_callback(&st95hf->gpio_int1_cb,
			   st95hf_gpio_int1_callback,
			   BIT(cfg->gpio_drdy.pin));

	status = gpio_add_callback(cfg->gpio_drdy.port, &st95hf->gpio_int1_cb);
	if (status < 0) {
		LOG_ERR("Could not add gpio int1 callback");
		return status;
	}

	LOG_INF("%s: int1 on %s.%02u", dev->name,
				       cfg->gpio_drdy.port->name,
				       cfg->gpio_drdy.pin);

check_gpio_int:
	/*
	 * Setup Interrupt (for Any Motion) if defined in DT
	 */

	/* setup any motion gpio interrupt */
	if (!device_is_ready(cfg->gpio_int.port)) {
		/* API may return false even when ptr is NULL */
		if (cfg->gpio_int.port != NULL) {
			LOG_ERR("device %s is not ready", cfg->gpio_int.port->name);
			return -ENODEV;
		}

		LOG_DBG("gpio_int not defined in DT");
		status = 0;
		goto end;
	}

	/* any motion int2 gpio configuration */
	status = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
	if (status < 0) {
		LOG_ERR("Could not configure %s.%02u",
			cfg->gpio_int.port->name, cfg->gpio_int.pin);
		return status;
	}

	gpio_init_callback(&st95hf->gpio_int2_cb,
			   st95hf_gpio_int2_callback,
			   BIT(cfg->gpio_int.pin));

	/* callback is going to be enabled by trigger setting function */
	status = gpio_add_callback(cfg->gpio_int.port, &st95hf->gpio_int2_cb);
	if (status < 0) {
		LOG_ERR("Could not add gpio int2 callback (%d)", status);
		return status;
	}

	LOG_INF("%s: int2 on %s.%02u", dev->name,
				       cfg->gpio_int.port->name,
				       cfg->gpio_int.pin);

	/* disable interrupt in case of warm (re)boot */
	status = st95hf->hw_tf->write_reg(
		dev,
		cfg->hw.anym_on_int1 ? ST95HF_REG_INT1_CFG : ST95HF_REG_INT2_CFG,
		0);
	if (status < 0) {
		LOG_ERR("Interrupt disable reg write failed (%d)", status);
		return status;
	}

	(void)memset(raw, 0, sizeof(raw));
	status = st95hf->hw_tf->write_data(
		dev,
		cfg->hw.anym_on_int1 ? ST95HF_REG_INT1_THS : ST95HF_REG_INT2_THS,
		raw, sizeof(raw));
	if (status < 0) {
		LOG_ERR("Burst write to THS failed (%d)", status);
		return status;
	}

	if (cfg->hw.anym_on_int1) {
		/* enable interrupt 1 on int1 line */
		status = st95hf->hw_tf->update_reg(dev, ST95HF_REG_CTRL3,
						   ST95HF_EN_INT1_INT1,
						   ST95HF_EN_INT1_INT1);
		if (cfg->hw.anym_latch) {
			/* latch int1 line interrupt */
			status = st95hf->hw_tf->write_reg(dev, ST95HF_REG_CTRL5,
							  ST95HF_EN_LIR_INT1);
		}
	} else {
		/* enable interrupt 2 on int2 line */
		status = st95hf->hw_tf->update_reg(dev, ST95HF_REG_CTRL6,
						   ST95HF_EN_INT2_INT2,
						   ST95HF_EN_INT2_INT2);
		if (cfg->hw.anym_latch) {
			/* latch int2 line interrupt */
			status = st95hf->hw_tf->write_reg(dev, ST95HF_REG_CTRL5,
							  ST95HF_EN_LIR_INT2);
		}
	}

	if (status < 0) {
		LOG_ERR("enable reg write failed (%d)", status);
		return status;
	}

end:
	return status;
}
