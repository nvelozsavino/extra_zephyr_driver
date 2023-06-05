
#pragma once
#include <stdint.h>


#pragma pack(push,1)
typedef struct {
	uint16_t length;
	uint8_t data[CONFIG_PACKET_MAX_PL_LEN];
} packet_t;
#pragma pack(pop)

typedef void (*packet_func_t)(const uint8_t* data, uint16_t size);

int packet_init(const struct device* serial, packet_func_t process_func);
void packet_send(const uint8_t *data, uint16_t len);