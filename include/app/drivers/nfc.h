/**
 * @file drivers/nfc.h
 *
 * @brief Public APIs for the nfc driver.
 */

/*
 * Copyright (c) 2023 nico
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_NFC_H_
#define ZEPHYR_INCLUDE_DRIVERS_NFC_H_

/**
 * @brief NFC Interface
 * @defgroup nfc_interface NFC Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif


enum nfc_protocol{
	NFC_PROTOCOL_UNKNOWN = 0,
	NFC_PROTOCOL_IEC14443A,
	NFC_PROTOCOL_IEC14443B,
	NFC_PROTOCOL_IEC15693,
	NFC_PROTOCOL_IEC18092,
    NFC_PROTOCOL_MAX = INT8_MAX,
};


/**
 * @typedef nfc_field_off_t
 * @brief Callback API upon turning OFF the NFC field
 *
 * See nfc_field_off() for argument description
 */
typedef int (*nfc_field_off_t)(const struct device *dev);

/**
 * @typedef nfc_protocol_init_t
 * @brief Callback API for initializing a NFC protocol
 *
 * See nfc_protocol_init() for argument description
 */
typedef int (*nfc_protocol_init_t)(const struct device *dev,
				    enum nfc_protocol protocol);

/**
 * @typedef nfc_protocol_is_present_t
 * @brief Callback API for check if a NFC protocol is present
 *
 * See nfc_protocol_is_present() for argument description
 */
typedef int (*nfc_protocol_is_present_t)(const struct device *dev,
				    enum nfc_protocol protocol);


/**
 * @typedef nfc_ndef_read_t
 * @brief Callback API for reading NDEF
 *
 * See nfc_ndef_read() for argument description
 */
typedef int (*nfc_ndef_read_t)(const struct device *dev,
				    enum nfc_protocol protocol, uint8_t* ndef_buffer, uint16_t* buffer_size);



/**
 * @typedef nfc_ndef_read_t
 * @brief Callback API for reading NDEF
 *
 * See nfc_ndef_read() for argument description
 */
typedef int (*nfc_ndef_read_t)(const struct device *dev,
				    enum nfc_protocol protocol, uint8_t* ndef_buffer, uint16_t* buffer_size);


/**
 * @typedef nfc_send_receive_t
 * @brief Callback API for Sending and Receiving data
 *
 * See nfc_send_receive_t() for argument description
 */
typedef int (*nfc_send_receive_t)(const struct device *dev, const uint8_t* send_data, size_t send_size, uint8_t* rcv_data, size_t* rcv_size);



__subsystem struct nfc_driver_api {
    nfc_field_off_t field_off;
    nfc_protocol_init_t protocol_init;
    nfc_protocol_is_present_t protocol_is_present;
    nfc_ndef_read_t ndef_read;
    nfc_send_receive_t send_receive;
    
};



/**
 * @brief Set nfc field off
 *
 * @param dev Pointer to the sensor device
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int nfc_field_off(const struct device *dev);

static inline int z_impl_nfc_field_off(const struct device *dev)
{
	const struct nfc_driver_api *api =
		(const struct nfc_driver_api *)dev->api;

	if (api->field_off == NULL) {
		return -ENOSYS;
	}

	return api->field_off(dev);
}

/**
 * @brief Get an attribute for a sensor
 *
 * @param dev Pointer to the sensor device
 * @param protocol The NFC protocol to initialize.
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int nfc_protocol_init(const struct device *dev,
				    enum nfc_protocol protocol);

static inline int z_impl_nfc_protocol_init(const struct device *dev,
					 enum nfc_protocol protocol)
{
	const struct nfc_driver_api *api =
		(const struct nfc_driver_api *)dev->api;

	if (api->protocol_init == NULL) {
		return -ENOSYS;
	}

	return api->protocol_init(dev, protocol);
}

/**
 * @brief Verify if a protocol is present in an NFC tag.
 *
 * @param dev Pointer to the sensor device
 * @param protocol The NFC protocol to verify.
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int nfc_protocol_is_present(const struct device *dev,
				    enum nfc_protocol protocol);

static inline int z_impl_nfc_protocol_is_present(const struct device *dev,
				    enum nfc_protocol protocol)
{
	const struct nfc_driver_api *api =
		(const struct nfc_driver_api *)dev->api;

	return api->protocol_is_present(dev, protocol);
}

/**
 * @brief Read NDEF data from NFC tag
 *
 * @param dev Pointer to the sensor device
 * @param protocol The NFC protocol to use.
 * @param ndef_buffer The Buffer where the read NDEF will be stored.
 * @param buffer_size The size of the read NDEF buffer, it must contain 
 *                    the maximum size of ndef_buffer before calling the function.
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int nfc_ndef_read(const struct device *dev,
				    enum nfc_protocol protocol, uint8_t* ndef_buffer, uint16_t* buffer_size);

static inline int z_impl_nfc_ndef_read(const struct device *dev,
				    enum nfc_protocol protocol, uint8_t* ndef_buffer, uint16_t* buffer_size)
{
	const struct nfc_driver_api *api =
		(const struct nfc_driver_api *)dev->api;

	return api->ndef_read(dev, protocol, ndef_buffer,buffer_size);
}

/**
 * @brief Send and Receive data from NFC tag
 *
 * @param dev Pointer to the sensor device
 * @param send_data The buffer with the data to send.
 * @param send_size The size of the data to send.
 * @param rcv_data The buffer for the received data.
 * @param rcv_size 	The size of the received data, 
 * 					before calling the function, this 
 * 					should contain the size of the receive buffer.
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int nfc_send_receive(const struct device *dev, const uint8_t* send_data, size_t send_size, uint8_t* rcv_data, size_t* rcv_size);

static inline int z_impl_nfc_send_receive(const struct device *dev, const uint8_t* send_data, size_t send_size, uint8_t* rcv_data, size_t* rcv_size)
{
	const struct nfc_driver_api *api =
		(const struct nfc_driver_api *)dev->api;

	return api->send_receive(dev, send_data,send_size,rcv_data,rcv_size);
}



#ifdef CONFIG_NFC_INFO

struct nfc_info {
	const struct device *dev;
	const char *vendor;
	const char *model;
	const char *friendly_name;
};

#define NFC_INFO_INITIALIZER(_dev, _vendor, _model, _friendly_name)	\
	{								\
		.dev = _dev,						\
		.vendor = _vendor,					\
		.model = _model,					\
		.friendly_name = _friendly_name,			\
	}

#define NFC_INFO_DEFINE(name, ...)					\
	static const STRUCT_SECTION_ITERABLE(nfc_info, name) =	\
		NFC_INFO_INITIALIZER(__VA_ARGS__)

#define NFC_INFO_DT_NAME(node_id)					\
	_CONCAT(__nfc_info, DEVICE_DT_NAME_GET(node_id))

#define NFC_INFO_DT_DEFINE(node_id)					\
	NFC_INFO_DEFINE(NFC_INFO_DT_NAME(node_id),		\
			   DEVICE_DT_GET(node_id),			\
			   DT_NODE_VENDOR_OR(node_id, NULL),		\
			   DT_NODE_MODEL_OR(node_id, NULL),		\
			   DT_PROP_OR(node_id, friendly_name, NULL))	\

#else

#define NFC_INFO_DEFINE(name, ...)
#define NFC_INFO_DT_DEFINE(node_id)

#endif /* CONFIG_SENSOR_INFO */

/**
 * @brief Like DEVICE_DT_DEFINE() with sensor specifics.
 *
 * @details Defines a device which implements the sensor API. May define an
 * element in the sensor info iterable section used to enumerate all sensor
 * devices.
 *
 * @param node_id The devicetree node identifier.
 *
 * @param init_fn Name of the init function of the driver.
 *
 * @param pm_device PM device resources reference (NULL if device does not use
 * PM).
 *
 * @param data_ptr Pointer to the device's private data.
 *
 * @param cfg_ptr The address to the structure containing the configuration
 * information for this instance of the driver.
 *
 * @param level The initialization level. See SYS_INIT() for details.
 *
 * @param prio Priority within the selected initialization level. See
 * SYS_INIT() for details.
 *
 * @param api_ptr Provides an initial pointer to the API function struct used
 * by the driver. Can be NULL.
 */
#define NFC_DEVICE_DT_DEFINE(node_id, init_fn, pm_device,		\
				data_ptr, cfg_ptr, level, prio,		\
				api_ptr, ...)				\
	DEVICE_DT_DEFINE(node_id, init_fn, pm_device,			\
			 data_ptr, cfg_ptr, level, prio,		\
			 api_ptr, __VA_ARGS__);				\
									\
	NFC_INFO_DT_DEFINE(node_id);

/**
 * @brief Like NFC_DEVICE_DT_DEFINE() for an instance of a DT_DRV_COMPAT
 * compatible
 *
 * @param inst instance number. This is replaced by
 * <tt>DT_DRV_COMPAT(inst)</tt> in the call to NFC_DEVICE_DT_DEFINE().
 *
 * @param ... other parameters as expected by NFC_DEVICE_DT_DEFINE().
 */
#define NFC_DEVICE_DT_INST_DEFINE(inst, ...)				\
	NFC_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/nfc.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_ */