#include "volt_divider.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(vdivider, CONFIG_ADC_LOG_LEVEL);

int volt_divider_setup(volt_divider_t* divider, const volt_divider_cfg_t* cfg)
{   
    if (divider==NULL || cfg==NULL){
        return -EINVAL;
    }
    divider->cfg=cfg;
	const struct gpio_dt_spec *gcp = &cfg->power_gpios;
	struct adc_sequence *asp = &divider->adc_seq;
	struct adc_channel_cfg *accp = &divider->adc_cfg;
	int rc;

	if (!device_is_ready(cfg->adc)) {
		LOG_ERR("ADC device is not ready %s", cfg->adc->name);
		return -ENOENT;
	}

	if (gcp->port) {
		if (!device_is_ready(gcp->port)) {
			LOG_ERR("%s: device not ready", gcp->port->name);
			return -ENOENT;
		}
		rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
		if (rc != 0) {
			LOG_ERR("Failed to control feed %s.%u: %d",
				gcp->port->name, gcp->pin, rc);
			return rc;
		}
	}

	*asp = (struct adc_sequence){
		.channels = BIT(cfg->io_channel),
		.buffer = &divider->raw,
		.buffer_size = sizeof(divider->raw),
		.oversampling = 4,
		.calibrate = true,
	};

#ifdef CONFIG_ADC_NRFX_SAADC
	*accp = (struct adc_channel_cfg){
		.gain = cfg->gain,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	};

	if (cfg->output_ohm != 0) {
		accp->input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0
			+ cfg->io_channel;
	} else {
		accp->input_positive = SAADC_CH_PSELP_PSELP_VDD;
	}

	asp->resolution = 14;
#else /* CONFIG_ADC_var */
#error Unsupported ADC
#endif /* CONFIG_ADC_var */

	rc = adc_channel_setup(cfg->adc, accp);
	LOG_DBG("Setup AIN%u got %d", cfg->io_channel, rc);
    divider->ok=(rc==0);
	return rc;
}


int volt_divider_measure_enable(volt_divider_t* divider, bool enable)
{

    if (divider==NULL){
        return -EINVAL;
    }
	int rc = -ENOENT;

	if (divider->ok) {
		const struct gpio_dt_spec *gcp = &divider->cfg->power_gpios;

		rc = 0;
		if (gcp->port) {
			// LOG_INF("Setting pin %d",enable);
			rc = gpio_pin_set_dt(gcp, enable);
		}
	}
	return rc;
}


int volt_divider_sample(volt_divider_t* divider)
{
    if (divider==NULL){
        return -EINVAL;
    }

	int rc = -ENOENT;

	if (divider->ok) {
		const volt_divider_cfg_t *cfg = divider->cfg;
		struct adc_sequence *sp = &divider->adc_seq;
		rc = adc_read(cfg->adc, sp);
		sp->calibrate = false;
		if (rc == 0) {
			int32_t val = divider->raw;

			rc = adc_raw_to_millivolts(adc_ref_internal(cfg->adc),
					      divider->adc_cfg.gain,
					      sp->resolution,
					      &val);
            if (rc!=0){
                return rc;
            }
			if (cfg->output_ohm != 0) {
				rc = val * (uint64_t)cfg->full_ohm / cfg->output_ohm;
				// LOG_INF("raw %u ~ %u mV => %d mV\n", divider->raw, val, rc);
			} else {
				rc = val;
				// LOG_INF("raw %u ~ %u mV\n", divider->raw, val);
			}
		}
	}

	return rc;
}
