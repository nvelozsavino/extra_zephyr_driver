/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ST95HF_ST95HF_H_
#define ZEPHYR_DRIVERS_SENSOR_ST95HF_ST95HF_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <app/drivers/nfc.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include "st95hf_defs.h"


typedef struct {
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_irq_in;
	const struct gpio_dt_spec gpio_ssi[2];
#ifdef CONFIG_ST95HF_TRIGGER
	const struct gpio_dt_spec gpio_irq_out;
#endif /* CONFIG_ST95HF_TRIGGER */
} st95hf_config_t;


typedef struct {

#ifdef CONFIG_PM_DEVICE
	#erro todo
#endif

#ifdef CONFIG_ST95HF_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_irq_out_cb;

	struct k_sem rx_sem;
#else
	bool user_stop;
#endif /* CONFIG_ST95HF_TRIGGER */
	st95hf_ic_version_t ic_version;
	st95hf_protocol_t current_protocol;
	st95hf_device_mode_t device_mode;
	st95hf_tag_type_t tag_type;
} st95hf_data_t;

#ifdef CONFIG_ST95HF_TRIGGER
int st95hf_init_interrupt(const struct device *dev);
#endif





/**
 * Basic functions
*/
int st95hf_poll(const struct device *dev, k_timeout_t timeout);
int st95hf_send(const struct device *dev, uint8_t cmd, uint8_t len, const void* data);
int st95hf_receive(const struct device *dev, uint8_t* result_code, void* data, uint16_t* size);
int st95hf_reset(const struct device *dev);
int st95hf_wakeup(const struct device* dev);

int st95hf_req_rsp(const struct device* dev, const st95hf_req_t* req, st95hf_rsp_t* rsp, void* data ,k_timeout_t timeout);


/**
 * Commands
*/
int st95hf_idn_cmd(const struct device* dev, st95hf_rsp_t *rsp, st95hf_idn_data_t* data,k_timeout_t timeout);
int st95hf_protocol_select_cmd(const struct device* dev, const st95hf_protocol_selection_req_t * req, st95hf_rsp_t *rsp,k_timeout_t timeout);
int st95hf_pollfield_check_cmd(const struct device* dev, st95hf_rsp_t *rsp, st95hf_pollfield_data_t* data,k_timeout_t timeout);
int st95hf_pollfield_wait_cmd(const struct device* dev, const st95hf_pollfield_req_t* req, st95hf_rsp_t *rsp, st95hf_pollfield_data_t* data,k_timeout_t timeout);
int st95hf_send_receive_cmd(const struct device* dev, uint8_t send_size, const void* send_data, st95hf_rsp_t *rsp, void* recv_data,k_timeout_t timeout);
int st95hf_listen_set_cmd(const struct device* dev, st95hf_rsp_t *rsp,k_timeout_t timeout);
int st95hf_listen_wait_cmd(const struct device* dev, st95hf_rsp_t *rsp, void* listen_data,k_timeout_t timeout);
int st95hf_send_cmd(const struct device* dev, uint8_t send_size, const void* send_data, st95hf_rsp_t *rsp,k_timeout_t timeout);
int st95hf_idle_cmd(const struct device* dev, const st95hf_idle_req_t* req, st95hf_rsp_t* rsp, st95hf_idle_data_t* data,k_timeout_t timeout);
int st95hf_read_reg_cmd(const struct device* dev, const st95hf_read_req_t* req, st95hf_rsp_t* rsp, uint8_t* data,k_timeout_t timeout);
int st95hf_write_reg_cmd(const struct device* dev, const st95hf_write_req_t* req, st95hf_rsp_t* rsp,k_timeout_t timeout);
int st95hf_select_reg_index_cmd(const struct device* dev, const st95hf_select_reg_index_req_t* req, st95hf_rsp_t* rsp,k_timeout_t timeout);


int st95hf_subcarrier_frequency_cmd(const struct device* dev, st95hf_rsp_t* rsp, st95hf_sub_freq_data_t* data,k_timeout_t timeout);

int st95hf_ac_filter_deactivate_cmd(const struct device* dev, st95hf_rsp_t * rsp, k_timeout_t timeout);
int st95hf_ac_filter_set_state_cmd(const struct device* dev, uint8_t state, st95hf_rsp_t * rsp, k_timeout_t timeout);
int st95hf_ac_filter_get_state_cmd(const struct device* dev, st95hf_rsp_t * rsp, st95hf_ac_filter_data_t* data,k_timeout_t timeout);
int st95hf_ac_filter_activate_anti_colision_cmd(const struct device* dev, uint8_t uid_count, const st95hf_ac_filter_req_t* req, st95hf_rsp_t * rsp, k_timeout_t timeout);

int st95hf_echo_cmd(const struct device* dev,k_timeout_t timeout);

int st95hf_stop_waiting(const struct device* dev, bool force, bool pulse_irq);
/**
 * Functions
*/
int st95hf_tag_calibration(const struct device* dev, uint8_t wu_period, uint8_t* data_h);
int st95hf_tag_hunting(const struct device* dev, uint8_t* tags_type);

#endif /* __SENSOR_ST95HF__ */
