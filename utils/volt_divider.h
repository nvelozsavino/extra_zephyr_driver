#pragma once

#include <zephyr/types.h>
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

#define VOLT_DIVIDER_CFG_INIT(node,_gain) { \
	.adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(node)), \
	.io_channel = { DT_IO_CHANNELS_INPUT(node)}, \
	.power_gpios = GPIO_DT_SPEC_GET_OR(node, power_gpios, {}), \
	.output_ohm = DT_PROP(node, output_ohms), \
	.full_ohm = DT_PROP(node, full_ohms),	\
	.gain = (_gain) \
}


typedef struct divider_config {
	const struct device *adc;
	uint8_t io_channel;
	enum adc_gain gain;
	struct gpio_dt_spec power_gpios;
	/* output_ohm is used as a flag value: if it is nonzero then
	 * the battery is measured through a voltage divider;
	 * otherwise it is assumed to be directly connected to Vdd.
	 */
	uint32_t output_ohm;
	uint32_t full_ohm;
} volt_divider_cfg_t;

typedef struct divider_data {
	struct adc_channel_cfg adc_cfg;
	struct adc_sequence adc_seq;
	int16_t raw;
	const volt_divider_cfg_t* cfg;
	bool ok;
} volt_divider_t;


int volt_divider_setup(volt_divider_t* divider, const volt_divider_cfg_t* cfg);
int volt_divider_measure_enable(volt_divider_t* divider, bool enable);
int volt_divider_sample(volt_divider_t* divider);
