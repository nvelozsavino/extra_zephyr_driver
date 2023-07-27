
#pragma once
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#pragma pack(push,1)
typedef struct {
	uint16_t length;
	uint8_t data[CONFIG_PACKET_MAX_PL_LEN];
} packet_t;
#pragma pack(pop)

typedef bool (*packet_func_t)(const uint8_t* data, uint16_t size, void* context);

typedef struct packet_process_s {
    packet_func_t func;
    void* context;
    struct packet_process_s* next;
} packet_process_t;


int packet_init(const struct device* serial);
int packet_send(const uint8_t *data, uint16_t len, k_timeout_t timeout);

int packet_register(packet_process_t* cb);
int packet_unregister(packet_process_t* cb);