#pragma once
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>

#define PCDNFCT4_OK 										PCDNFC_OK
#define PCDNFCT4_ERROR 									PCDNFC_ERROR
#define PCDNFCT4_ERROR_MEMORY_TAG				PCDNFC_ERROR_MEMORY_TAG
#define PCDNFCT4_ERROR_MEMORY_INTERNAL	PCDNFC_ERROR_MEMORY_INTERNAL
#define PCDNFCT4_ERROR_LOCKED 					PCDNFC_ERROR_LOCKED

/* Command List */
#define PCDNFCT4_SELECT_APPLI				{0xD2,0x76,0x00,0x00,0x85,0x01,0x01,0x00}		
#define PCDNFCT4_CC_ID							{0xE1,0x03}

/* Size */
#define PCDNFCT4_BUFFER_READ				256

#define PCDNFCT4_ACCESS_ALLOWED			0x00

int st95hf_type4_read_ndef(const struct device* dev, uint16_t* ndef_length, uint8_t* ndef);
// uint8_t st95hf_type4_write_ndef(void);