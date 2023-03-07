/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_st95hf

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define START_TRIG_IRQ_OUT		0
#define START_TRIG_INT2			1
#define TRIGGED_DATA_READY		4
#define TRIGGED_DATA_EXPECTED	5


LOG_MODULE_DECLARE(st95hf, CONFIG_NFC_LOG_LEVEL);
#include "st95hf.h"


static void st95hf_gpio_irq_out_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	st95hf_data_t *st95hf = CONTAINER_OF(cb, st95hf_data_t, gpio_irq_out_cb);
	// const st95hf_config_t *cfg = dev->config;

	ARG_UNUSED(pins);
	// int  irq_out = gpio_pin_get_dt(&cfg->gpio_irq_out);
	// if (irq_out){
	// 	// IRQ_OUT went to HIGH => Data was read
	// 	// k_sem_give(&st95hf->tx_sem);
	// } else {
	// 	// IRQ_OUT went to LOW => Data is ready to be read
	
		k_sem_give(&st95hf->rx_sem);
	// }

}


int st95hf_init_interrupt(const struct device *dev)
{
	st95hf_data_t *st95hf = dev->data;
	const st95hf_config_t *cfg = dev->config;
	int status;

	st95hf->dev = dev;
	LOG_ERR("Initializing interrupts");
	k_sem_init(&st95hf->rx_sem, 0, 1);

	/* setup data ready gpio interrupt */ 
	if (!device_is_ready(cfg->gpio_irq_out.port)) {
		LOG_DBG("gpio_irq_out not defined in DT");
		return -ENODEV;
	}

	/* API may return false even when ptr is NULL */
	if (cfg->gpio_irq_out.port == NULL) {
		LOG_ERR("device %s is not ready", cfg->gpio_irq_out.port->name);
		return -ENODEV;
	}


	/* data ready int1 gpio configuration */
	status = gpio_pin_configure_dt(&cfg->gpio_irq_out, GPIO_PULL_UP| GPIO_INPUT);
	if (status < 0) {
		LOG_ERR("Could not configure %s.%02u",
			cfg->gpio_irq_out.port->name, cfg->gpio_irq_out.pin);
		return status;
	}

	int ret = gpio_pin_get_dt(&cfg->gpio_irq_out);
	if (ret < 0) {
		return ret;
	}
	if (ret==0){
		uint8_t result_code;
		uint8_t dummy_data[528];
		uint16_t size = sizeof(dummy_data);
		st95hf_receive(dev, &result_code, dummy_data, &size);
	}




	gpio_init_callback(&st95hf->gpio_irq_out_cb,
			   st95hf_gpio_irq_out_callback,
			   BIT(cfg->gpio_irq_out.pin));

	status = gpio_add_callback(cfg->gpio_irq_out.port, &st95hf->gpio_irq_out_cb);
	if (status < 0) {
		LOG_ERR("Could not add gpio int1 callback");
		return status;
	}

	gpio_pin_interrupt_configure_dt(&cfg->gpio_irq_out,
					GPIO_INT_EDGE_FALLING);

	LOG_INF("%s: irq_out on %s.%02u", dev->name,
				       cfg->gpio_irq_out.port->name,
				       cfg->gpio_irq_out.pin);
	return status;
}
