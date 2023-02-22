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

LOG_MODULE_REGISTER(st95hf, CONFIG_NFC_LOG_LEVEL);
#include "st95hf.h"


int st95hf_poll(const struct device *dev,k_timeout_t timeout){

	st95hf_data_t *st95hf = dev->data;
 	int err = 0;
	
	#ifdef CONFIG_ST95HF_TRIGGER		
		//wait for response
		err = k_sem_take(&st95hf->rx_sem,timeout);
	#else	
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
			if ((flags & ST95HF_POLL_MASK_READ_READY) == ST95HF_POLL_MASK_READ_READY){
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
	#endif

	return err;

}



static int st95hf_send_echo(const struct device *dev){

	const st95hf_config_t *cfg = dev->config;
	uint8_t echo_cmd[] = {0x00, ST95HF_CMD_ECHO};
	const struct spi_buf tx_buf[1] = {
		{
			.buf = echo_cmd,
			.len = sizeof(echo_cmd),
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count =1
	};
	int err = spi_write_dt(&cfg->bus, &tx);

	/* Our device is flagged with SPI_HOLD_ON_CS|SPI_LOCK_ON, release */
	spi_release_dt(&cfg->bus);

	if (err) {
		return -EIO;
	}
	return 0;
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
	uint8_t header[2];
	const struct spi_buf rx_header_buf = {
		.buf = header,
		.len = sizeof(header),
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
			LOG_ERR("Error SPI sending CMD %d", err);
			err=-EIO;
			break;
		}

		err = spi_read_dt(&cfg->bus, &rx_header);
		if (err){
			LOG_ERR("Error SPI reading header %d", err);
			err=-EIO;
			break;
		}


		

		if (header[0]==ST95HF_CMD_ECHO || header[0]==0xFF){
			//if header[0]=ECHO while listening need to retrieve 0x85 (EUserStop) already in header[1] and 0x00 (Len)
			//if header[0]=0xFF retrieve 2 more bytes (first is already in header[1])
			uint8_t dummy;
			const struct spi_buf dummy_data_buf = {
				.buf = &dummy,
				.len = 1,
			};
			const struct spi_buf_set dummy_data = {
				.buffers = &dummy_data_buf,
				.count = 1
			};

			err = spi_read_dt(&cfg->bus, &dummy_data);
			if (err){
				err=-EIO;
				break;
			}
			if (result_code!=NULL){
				*result_code = header[0];
			}
			if (size!=NULL){
				*size = 0;
			}
			break;
		}
	

		uint16_t len=header[1];
		if ((header[0] & 0x9F) == 0x80){			

			len = ((0x60 & header[0])<<3) | header[1];
		}
		if (result_code!=NULL){
			*result_code = header[0];
		}
		if (size!=NULL){
			*size = len;
		}
		if (data==NULL || size==NULL){
			break;
		}
		if (len>max_buffer_size){
			LOG_ERR("Len %d, max %d",len, max_buffer_size);
			LOG_ERR("Header %02x %02x",header[0], header[1]);
			err= -ENOMEM;
			break;
		}
		
		const struct spi_buf rx_data_buf = {
			.buf = data,
			.len = len,
		};
		const struct spi_buf_set rx_data = {
			.buffers = &rx_data_buf,
			.count = 1
		};

		err = spi_read_dt(&cfg->bus, &rx_data);
		if (err){
			LOG_ERR("Error SPI reading data %d", err);
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
		.len = sizeof(ctrl_byte),
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



int st95hf_wakeup(const struct device* dev){
	const st95hf_config_t *cfg = dev->config;
	gpio_pin_set_dt(&cfg->gpio_irq_in, 0);
	k_sleep(K_USEC(20));
	gpio_pin_set_dt(&cfg->gpio_irq_in, 1);
	return 0;
}

int st95hf_idn_cmd(const struct device* dev, st95hf_rsp_t *rsp, st95hf_idn_data_t* data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_IDN,
		.len = 0,
		.data = NULL
	};
	if (rsp==NULL || data==NULL){
		return -EINVAL;
	}
	rsp->len = sizeof(st95hf_idn_data_t);
	return st95hf_req_rsp(dev,&request, rsp, data ,timeout);
}

int st95hf_protocol_select_cmd(const struct device* dev, const st95hf_protocol_selection_req_t * req, st95hf_rsp_t *rsp,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_PROTOCOL_SELECTION,
		.len = 0,
		.data = NULL
	};
	if (req == NULL || rsp==NULL){
		return -EINVAL;
	}

	switch (req->protocol){
		case ST95HF_PROTOCOL_CODE_READER_FIELD_OFF:
			request.len=sizeof(st95hf_protocol_reader_field_off_t);
			request.data=&req->parameters.field_off;
			break;
		case ST95HF_PROTOCOL_CODE_READER_IEC15693:
			request.len=sizeof(st95hf_protocol_reader_iec15693_t);
			request.data=&req->parameters.reader_iec15693;
			break;
		case ST95HF_PROTOCOL_CODE_READER_IEC14443A:
			request.len=sizeof(st95hf_protocol_reader_iec14443a_t);
			request.data=&req->parameters.reader_iec14443a;
			break;
		case ST95HF_PROTOCOL_CODE_READER_IEC14443B:
			request.len=sizeof(st95hf_protocol_reader_iec14443b_t);
			request.data=&req->parameters.reader_iec14443b;
			break;
		case ST95HF_PROTOCOL_CODE_READER_IEC18092:
			request.len=sizeof(st95hf_protocol_reader_iec18092_t);
			request.data=&req->parameters.reader_iec18092;
			break;
		case ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC14443A:
			request.len=sizeof(st95hf_protocol_emulation_iec14443a_t);
			request.data=&req->parameters.card_emulation_iec14443a;
			break;
		case ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC14443B:
			return -EINVAL;
		case ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC18092:
			return -EINVAL;
		default:
			return -EINVAL;
	}
	rsp->len = 0;
	return st95hf_req_rsp(dev,&request, rsp, NULL ,timeout);
}

int st95hf_pollfield_check_cmd(const struct device* dev, st95hf_rsp_t *rsp, st95hf_pollfield_data_t* data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_POLLFIELD,
		.len = 0,
		.data = NULL
	};

	if ( rsp==NULL || data==NULL){
		return -EINVAL;
	}
	rsp->len = sizeof(st95hf_pollfield_data_t);
	return st95hf_req_rsp(dev,&request, rsp, data ,timeout);
}

int st95hf_pollfield_wait_cmd(const struct device* dev, const st95hf_pollfield_req_t* req, st95hf_rsp_t *rsp, st95hf_pollfield_data_t* data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_POLLFIELD,
		.len = sizeof(st95hf_pollfield_req_t),
		.data = req
	};

	if (req == NULL || rsp==NULL || data==NULL){
		return -EINVAL;
	}

	rsp->len = sizeof(st95hf_pollfield_data_t);
	return st95hf_req_rsp(dev,&request, rsp, data ,timeout);

}

int st95hf_send_receive_cmd(const struct device* dev, uint8_t send_size, const void* send_data, st95hf_rsp_t *rsp, void* recv_data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_SEND_RECV,
		.len = send_size,
		.data = send_data,
	};

	if ((send_size!=0 && send_data == NULL) ||  rsp == NULL || (rsp->len!=0 && recv_data == NULL)){
		return -EINVAL;
	}

	return st95hf_req_rsp(dev,&request, rsp, recv_data ,timeout);
}

int st95hf_listen_set_cmd(const struct device* dev, st95hf_rsp_t *rsp,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_LISTEN,
		.len = 0,
		.data = NULL,
	};
	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=0;
	return st95hf_req_rsp(dev,&request, rsp, NULL ,timeout);
}

int st95hf_listen_wait_cmd(const struct device* dev, st95hf_rsp_t *rsp, void* listen_data,k_timeout_t timeout){
	if (rsp==NULL){
		return -EINVAL;
	}
	int err = st95hf_poll(dev,timeout);
	if (err!=0){
		return err;
	}
	return st95hf_receive(dev,&rsp->result_code,listen_data,&rsp->len);

}

int st95hf_send_cmd(const struct device* dev, uint8_t send_size, const void* send_data, st95hf_rsp_t *rsp,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_SEND,
		.len = send_size,
		.data = send_data,
	};
	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=0;
	return st95hf_req_rsp(dev,&request, rsp, NULL ,timeout);
}

int st95hf_idle_cmd(const struct device* dev, const st95hf_idle_req_t* req, st95hf_rsp_t* rsp, st95hf_idle_data_t* data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_IDLE,
		.len = sizeof(st95hf_idle_req_t),
		.data = req,
	};

	if (rsp == NULL || data == NULL){
		return -EINVAL;
	}
	rsp->len=sizeof(st95hf_idle_data_t);

	return st95hf_req_rsp(dev,&request, rsp,data ,timeout);
}

int st95hf_read_reg_cmd(const struct device* dev, const st95hf_read_req_t* req, st95hf_rsp_t* rsp, uint8_t* data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_RDREG,
		.len = sizeof(st95hf_read_req_t),
		.data = req,
	};

	if (rsp == NULL || data == NULL){
		return -EINVAL;
	}
	rsp->len=1;

	return st95hf_req_rsp(dev,&request, rsp,data ,timeout);
}

int st95hf_write_reg_cmd(const struct device* dev, const st95hf_write_req_t* req, st95hf_rsp_t* rsp,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_WRREG,
		.len = sizeof(st95hf_write_req_t),
		.data = req,
	};

	if (rsp == NULL || req == NULL){
		return -EINVAL;
	}
	rsp->len=0;

	return st95hf_req_rsp(dev,&request, rsp,NULL ,timeout);
}
int st95hf_subcarrier_frequency_cmd(const struct device* dev, st95hf_rsp_t* rsp, st95hf_sub_freq_data_t* data,k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_SUB_FREQ_RES,
		.len = 0,
		.data = NULL,
	};

	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=sizeof(st95hf_sub_freq_data_t);

	return st95hf_req_rsp(dev,&request, rsp, data ,timeout);
}


int st95hf_ac_filter_deactivate_cmd(const struct device* dev, st95hf_rsp_t * rsp, k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_ACFILTER,
		.len = 0,
		.data = NULL,
	};

	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=0;
	return st95hf_req_rsp(dev,&request, rsp, NULL ,timeout);
}
int st95hf_ac_filter_set_state_cmd(const struct device* dev, uint8_t state, st95hf_rsp_t * rsp, k_timeout_t timeout){
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_ACFILTER,
		.len = 1,
		.data = &state,
	};

	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=0;
	return st95hf_req_rsp(dev,&request, rsp, NULL ,timeout);

}
int st95hf_ac_filter_get_state_cmd(const struct device* dev, st95hf_rsp_t * rsp, st95hf_ac_filter_data_t* data,k_timeout_t timeout){
	uint8_t buffer[2]={0x00,0x00};
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_ACFILTER,
		.len = sizeof(buffer),
		.data = buffer,
	};

	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=sizeof(st95hf_ac_filter_data_t);
	return st95hf_req_rsp(dev,&request, rsp, data ,timeout);
}

int st95hf_ac_filter_activate_anti_colision_cmd(const struct device* dev, uint8_t uid_count, const st95hf_ac_filter_req_t* req, st95hf_rsp_t * rsp, k_timeout_t timeout){
	if (uid_count<1 || uid_count>3 || req==NULL){
		return -EINVAL;
	}
	st95hf_req_t request = {
		.cmd = ST95HF_CMD_ACFILTER,
		.len = 3 + (4*uid_count),
		.data = req,
	};

	if (rsp == NULL){
		return -EINVAL;
	}
	rsp->len=0;
	return st95hf_req_rsp(dev,&request, rsp,NULL ,timeout);
}



int st95hf_echo_cmd(const struct device* dev,k_timeout_t timeout){
	int err = st95hf_send_echo(dev);
	if (err!=0){
		return err;
	}

	err  = st95hf_poll(dev,timeout);

	if (err!=0){
		return err;
	}
	uint8_t result_code;
	err = st95hf_receive(dev,&result_code,NULL,NULL);
	if (err!=0){
		return err;
	}
	if (result_code!=ST95HF_CMD_ECHO){
		LOG_ERR("Echo %02x!=0x55", result_code);
		return -EBADMSG;
	}
	return 0;
}

int st95hf_tag_calibration(const struct device* dev, uint8_t wu_period, uint8_t* data_h){
	
	uint8_t steps[6]  = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04};

	 st95hf_idle_req_t idle_req = {
        .wakeup_source=ST95HF_WAKEUP_SOURCE_TIME_OUT | ST95HF_WAKEUP_SOURCE_TAG_DETECTION,
        // .enter_ctrl=ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION,
        // .wakeup_ctrl=ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION,

        // .leave_ctrl=ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION,

        .enter_ctrl_h=ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION>>8,
        .enter_ctrl_l=ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION&0xFF,
        .wakeup_ctrl_h=ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION>>8,
        .wakeup_ctrl_l=ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION&0xFF,

        .leave_ctrl_h=ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION>>8,
        .leave_ctrl_l=ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION&0xFF,
        .wakeup_period=wu_period,
        .osc_start=0x60, // Recomended value
        .dac_start=0x60, // Recomended value
        .dac_data_h=0x00,
        .dac_data_l=0x00,
        .swings_count=0x3F, // Recomended value
        .max_sleep=0x01, // From ST example

    };
    st95hf_rsp_t rsp;
    st95hf_idle_data_t data;
    //>>>0x07 0E 03 A1 00 F8 01 18 00 20 60 60 00 xx 3F 01
    int err = st95hf_idle_cmd(dev,&idle_req,&rsp,&data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error %d. Calibrating",err);
        return err;
    }
    if (rsp.result_code==0x00 && rsp.len==1 && data.byte==0x02){

		idle_req.dac_data_h=0xFC;
		err = st95hf_idle_cmd(dev,&idle_req,&rsp,&data,K_SECONDS(3));
		if (err!=0){
			LOG_ERR("Error %d. Calibrating",err);
			return err;
		}
		if (rsp.result_code==0x00 && rsp.len==1 && data.byte==0x01){
			for(uint8_t i=0; i<6; i++) {					
				switch(data.byte) {
					case 0x01:

						idle_req.dac_data_h-= steps[i];
						break;
				
					case 0x02:
						idle_req.dac_data_h+= steps[i];
						break;
				
					default:
						return -EINVAL;
				}
				
				err = st95hf_idle_cmd(dev,&idle_req,&rsp,&data,K_SECONDS(3));
				if (err!=0){
					LOG_ERR("Error %d. Calibrating",err);
					return err;
				}
			}
		}
		if (rsp.result_code==0x00 && rsp.len==1 && data.byte==0x01){
					*data_h = (idle_req.dac_data_h -0x04) + 0x08;
		}else{
					*data_h = idle_req.dac_data_h + 0x08;
		}
		return 0;
    } else {
        LOG_ERR("Calibration failed %02x",rsp.result_code);
    }
	return -EIO;
}

int st95hf_req_rsp(const struct device* dev, const st95hf_req_t* req, st95hf_rsp_t* rsp, void* data ,k_timeout_t timeout){
	// st95hf_data_t *st95hf = dev->data;
	// const st95hf_config_t *cfg = dev->config;
	if (req ==NULL || rsp==NULL){
		return -EINVAL;
	}
	if (rsp->len!=0 && data==NULL){
		return -EINVAL;
	}
	int err = st95hf_send(dev,req->cmd,req->len,req->data);
	if (err!=0){
		LOG_ERR("Error sending. %d",err);
		return err;
	}
	err  = st95hf_poll(dev,timeout);

	if (err!=0){
		LOG_ERR("Error pollling. %d",err);

		return err;
	}
	err = st95hf_receive(dev,&rsp->result_code,data,&rsp->len);
	if (err!=0){
		LOG_ERR("Error receiving. %d",err);
	}
	return err;
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

	status = st95hf_reset(dev);
	if (status < 0) {
		LOG_ERR("Failed to reset. %d",status);
	}
	// k_sleep(K_MSEC(1));
	st95hf_wakeup(dev);
#ifdef CONFIG_ST95HF_TRIGGER
	status = st95hf_init_interrupt(dev);
	if (status < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return status;
	}
#endif
	uint8_t retries=0;
	do {
		status = st95hf_echo_cmd(dev,K_MSEC(10));
		retries++;
	} while (status!=0 && retries<5);
	if (status < 0) {
		LOG_ERR("Failed to echo. %d",status);
		return status;
	}
	st95hf_rsp_t rsp;
	st95hf_idn_data_t idn;
	status = st95hf_idn_cmd(dev,&rsp,&idn,K_MSEC(10));
	if (status < 0) {
		LOG_ERR("Failed to read chip id. %d",status);
		return status;
	}
	if (rsp.result_code!=0){
		LOG_ERR("Failed to get IDN Err. %02x",rsp.result_code);
		return -EIO;
	}
	LOG_INF("ST95HF Device ID: %s",idn.device_id);


	return 0;
}


#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ST95HF driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by ST95HF_DEFINE() 
 */

#define ST95HF_HAS_SSI0(inst)					\
		DT_INST_PROP_HAS_IDX(inst, ssi_gpios, 0)
#define ST95HF_HAS_SSI1(inst)					\
		DT_INST_PROP_HAS_IDX(inst, ssi_gpios, 1)

#define GPIO_DT_SPEC_INST_GET_COND_BY_IDX(inst, prop,idx)		\
	COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, prop,idx),		\
		    (GPIO_DT_SPEC_INST_GET_BY_IDX(inst, prop,idx)),	\
		    ({.port = NULL, .pin = 0, .dt_flags = 0}))


#ifdef CONFIG_ST95HF_TRIGGER

#define ST95HF_CFG_INT(inst)				\
	.gpio_irq_out =							\
		GPIO_DT_SPEC_INST_GET_COND_BY_IDX(inst, irq_out_gpios,0),		
#else

#define ST95HF_CFG_INT(inst) 
#endif /* CONFIG_ST95HF_TRIGGER */

#define ST95HF_CONFIG(inst)						\
	{								\
		.bus = SPI_DT_SPEC_INST_GET(inst,		\
					SPI_WORD_SET(8) |		\
					SPI_OP_MODE_MASTER |		\
					SPI_HOLD_ON_CS |	\
					SPI_LOCK_ON,			\
					0),				\
		.gpio_irq_in =	GPIO_DT_SPEC_INST_GET(inst, irq_in_gpios), \
		.gpio_ssi[0] = GPIO_DT_SPEC_INST_GET_COND_BY_IDX(inst,ssi_gpios,0), \
		.gpio_ssi[1] = GPIO_DT_SPEC_INST_GET_COND_BY_IDX(inst,ssi_gpios,1), \
		ST95HF_CFG_INT(inst) \
	}
#define ST95HF_DEFINE(inst)	  					\
	static st95hf_data_t st95hf_data_##inst; \
	static const st95hf_config_t st95hf_config_##inst =	 ST95HF_CONFIG(inst);  \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
	 	st95hf_init,				\
	 	NULL,		\
	 	&st95hf_data_##inst,			\
	 	&st95hf_config_##inst,			\
	 	POST_KERNEL,				\
	 	CONFIG_NFC_INIT_PRIORITY,		\
	 	NULL);

DT_INST_FOREACH_STATUS_OKAY(ST95HF_DEFINE)


