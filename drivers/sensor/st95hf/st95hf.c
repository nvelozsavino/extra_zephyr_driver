/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_st95hf


#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(st95hf, CONFIG_SENSOR_LOG_LEVEL);
#include "st95hf.h"


int st95hf_poll(const struct device *dev, uint8_t mask, k_timeout_t timeout){
	const st95hf_config_t *cfg = dev->config;
	uint8_t ctrl_byte = 0x03;
	const struct spi_buf tx_buf = {
			.buf = &ctrl_byte,
			.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	bool timeout_expired=true;
	int err = 0;
	do {
		err = spi_write_dt(&cfg->bus, &tx);	
		if (err!=0){
			err= -EIO;
			break;
		}

		uint8_t flags=0;

		const struct spi_buf rx_buf = {
			.buf = &flags,
			.len = 1,
		};
		const struct spi_buf_set rx = {
			.buffers = &rx_buf,
			.count = 1
		};
		uint64_t end = sys_clock_timeout_end_calc(timeout);

    	while (end > k_uptime_ticks()) {
			err = spi_transceive_dt(&cfg->bus,&tx,&rx);
			if (err!=0){
				err=-EIO;
				break;
			}
			if ((flags & mask) == mask){
				err=0;
				timeout_expired=false;
				break;
			}
		}


	} while (0);
	/* Our device is flagged with SPI_HOLD_ON_CS|SPI_LOCK_ON, release */
	spi_release_dt(&cfg->bus);

	if (err==0 && timeout_expired){
		return -ETIMEDOUT;
	}

	return err;

}


int st95hf_send(const struct device *dev, uint8_t cmd, uint8_t len, const void* data){

	if (data==NULL && len>0){
		return -EINVAL;
	}

	const st95hf_config_t *cfg = dev->config;
	uint8_t buffer[3] = {0x00,cmd,len};

	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer,
			.len = sizeof(buffer),
		},
		{
			.buf = (void*)data,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};
	int err = spi_write_dt(&cfg->bus, &tx);

	/* Our device is flagged with SPI_HOLD_ON_CS|SPI_LOCK_ON, release */
	spi_release_dt(&cfg->bus);

	if (err) {
		return -EIO;
	}
	return 0;
}

int st95hf_receive(const struct device *dev, uint8_t* result_code, void* data, uint16_t* size){


	const st95hf_config_t *cfg = dev->config;
	uint8_t buffer[1]={0x02};
	const struct spi_buf tx_buf = {
		.buf = buffer,
		.len = sizeof(buffer),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	st95hf_rsp_header_t header;
	const struct spi_buf rx_header_buf = {
		.buf = (uint8_t*)&header,
		.len = sizeof(st95hf_rsp_header_t),
	};
	const struct spi_buf_set rx_header = {
		.buffers = &rx_header_buf,
		.count = 1
	};
	uint16_t max_buffer_size = 0;
	if (size!=NULL){
		max_buffer_size=*size;
	}
	int err =0;
	do {
		err = spi_write_dt(&cfg->bus, &tx);
		if (err){
			err=-EIO;
			break;
		}
		err = spi_read_dt(&cfg->bus, &rx_header);
		if (err){
			err=-EIO;
			break;
		}
		uint16_t len=0;
		if ((header.result_code & 0x9F) == 0x80){			
			len = ((0x60 & header.result_code)<<3) | header.len;
		}
		if (result_code!=NULL){
			*result_code = header.result_code;
		}
		if (size!=NULL){
			*size = len;
		}
		if (data==NULL || size==NULL){
			break;
		}
		if (len>max_buffer_size){
			err= -ENOMEM;
			break;
		}
		
		const struct spi_buf rx_data_buf = {
			.buf = data,
			.len = max_buffer_size,
		};
		const struct spi_buf_set rx_data = {
			.buffers = &rx_data_buf,
			.count = 1
		};
		err = spi_read_dt(&cfg->bus, &rx_data);
		if (err){
			err=-EIO;
			break;
		}

	} while(0);
	
	/* Our device is flagged with SPI_HOLD_ON_CS|SPI_LOCK_ON, release */
	spi_release_dt(&cfg->bus);
	return err;
}


int st95hf_reset(const struct device *dev){
	const st95hf_config_t *cfg = dev->config;
	uint8_t ctrl_byte = {0x01};
	const struct spi_buf tx_buf = {
		.buf = &ctrl_byte,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	int err = spi_write_dt(&cfg->bus, &tx);

	/* Our device is flagged with SPI_HOLD_ON_CS|SPI_LOCK_ON, release */
	spi_release_dt(&cfg->bus);

	if (err) {
		return -EIO;
	}
	return 0;
}



static int st95hf_wakeup(const struct device* dev){
	const st95hf_config_t *cfg = dev->config;
	gpio_pin_set_dt(&cfg->gpio_irq_in, 0);
	k_sleep(K_USEC(10));
	gpio_pin_set_dt(&cfg->gpio_irq_in, 1);
	return 0;
}


static int st95hf_idn(const struct device *dev, st95hf_idn_rsp_t* rsp){
	if (rsp==NULL){
		return -EINVAL;
	}
	int err = st95hf_send(dev,ST95HF_CMD_IDN,0x00,NULL);
	if (err!=0){
		return err;
	}
	#ifdef CONFIG_ST95HF_TRIGGER
		//wait for response
	#else
		// poll command
		err  = st95hf_poll(dev,ST95HF_POLL_MASK_READ_READY,K_MSEC(10));
		if (err!=0){
			return err;
		}
	#endif
	
	uint8_t result_code;	
	uint16_t size = sizeof(st95hf_idn_rsp_t);	
	return st95hf_receive(dev,&result_code,rsp,&size); 
}

int st95hf_init(const struct device *dev)
{

// const st95hf_config_t st95hf_config	={
// 	.bus = SPI_DT_SPEC_INST_GET(0,
	
// 					SPI_WORD_SET(8) |		
// 					SPI_OP_MODE_MASTER |		
// 					SPI_MODE_CPOL |			
// 					SPI_MODE_CPHA |			
// 					SPI_HOLD_ON_CS |	
// 					SPI_LOCK_ON,			
// 					0),
// 					.gpio_irq_in =GPIO_DT_SPEC_INST_GET(0, irq_in_gpios),
// };

	// st95hf_data_t *st95hf = dev->data;
	const st95hf_config_t *cfg = dev->config;
	int status;	

	if (!spi_is_ready(&cfg->bus)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->gpio_irq_in.port)) {
		LOG_ERR("IRQ_IN port device not ready");
		return -ENODEV;
	}
	LOG_INF("Configuring IRQ_IN pin to OUTPUT_HIGH");
	gpio_pin_configure_dt(&cfg->gpio_irq_in, GPIO_OUTPUT_HIGH);

	// Set irq-in-gpio low for 10us

	st95hf_wakeup(dev);


	st95hf_idn_rsp_t idn;
	status = st95hf_idn(dev,&idn);
	if (status < 0) {
		LOG_ERR("Failed to read chip id.");
		return status;
	}
	LOG_INF("ST95HF Device ID: %s",idn.device_id);

#ifdef CONFIG_ST95HF_TRIGGER
	if (cfg->gpio_drdy.port != NULL || cfg->gpio_int.port != NULL) {
		status = st95hf_init_interrupt(dev);
		if (status < 0) {
			LOG_ERR("Failed to initialize interrupts.");
			return status;
		}
	}
#endif
	return 0;
}


#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ST95HF driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by ST95HF_DEFINE() 
 */

#ifdef CONFIG_ST95HF_TRIGGER

#define GPIO_DT_SPEC_INST_GET_COND(id, prop)		\
	COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop,0),		\
		    (GPIO_DT_SPEC_INST_GET(id, prop)),	\
		    ({.port = NULL, .pin = 0, .dt_flags = 0}))

#define ST95HF_CFG_INT(inst)				\
	.gpio_irq_out =							\
		GPIO_DT_SPEC_INST_GET_COND(inst, irq_out_gpios),		


#else

#define ST95HF_CFG_INT(inst) 
#endif /* CONFIG_ST95HF_TRIGGER */

#define ST95HF_CONFIG(inst)						\
	{								\
		.bus = SPI_DT_SPEC_INST_GET(inst,		\
					SPI_WORD_SET(8) |		\
					SPI_OP_MODE_MASTER |		\
					SPI_MODE_CPOL |			\
					SPI_MODE_CPHA |			\
					SPI_HOLD_ON_CS |	\
					SPI_LOCK_ON,			\
					0),				\
		.gpio_irq_in =	GPIO_DT_SPEC_INST_GET(inst, irq_in_gpios), \
		ST95HF_CFG_INT(inst) \
	}
#define ST95HF_DEFINE(inst)	  					\
	static const st95hf_config_t st95hf_config_##inst =	 ST95HF_CONFIG(inst);  \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
	 	st95hf_init,				\
	 	NULL,		\
	 	NULL,			\
	 	&st95hf_config_##inst,			\
	 	POST_KERNEL,				\
	 	CONFIG_SENSOR_INIT_PRIORITY,		\
	 	NULL);

DT_INST_FOREACH_STATUS_OKAY(ST95HF_DEFINE)


