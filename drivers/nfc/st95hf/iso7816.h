#pragma once
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include "iso14443a.h"
#include "iso14443b.h"


/*  status and error code ---------------------------------------------------------------------- */
#define ISO7816_SUCCESSCODE											RESULTOK
#define ISO7816_ERRORCODE_DEFAULT								0x71
#define ISO7816_ERRORCODE_RESPONSE							0x72
#define ISO7816_ERRORCODE_SENDERRORCODE					0x73

/*  Iblock ------------------------------------------------------------------------------------- */
#define ISO7816_IBLOCK02												0x02
#define ISO7816_IBLOCK03												0x03

#define ISO7816_SELECT_FILE     								0xA4
#define ISO7816_UPDATE_BINARY   								0xD6
#define ISO7816_READ_BINARY     								0xB0

#define ISO7816_CLASS_0X00											0x00
#define ISO7816_CLASS_STM												0xA2

// offset of the different field of the APDU
#define ISO7816_ADPUOFFSET_BLOCK		0x00
#define ISO7816_ADPUOFFSET_CLASS		ISO7816_ADPUOFFSET_BLOCK 		+ 1
#define ISO7816_ADPUOFFSET_INS			ISO7816_ADPUOFFSET_BLOCK 		+ 2
#define ISO7816_ADPUOFFSET_P1				ISO7816_ADPUOFFSET_BLOCK		+ 3
#define ISO7816_ADPUOFFSET_P2				ISO7816_ADPUOFFSET_BLOCK 		+ 4
#define ISO7816_ADPUOFFSET_LC				ISO7816_ADPUOFFSET_BLOCK 		+	5
#define ISO7816_ADPUOFFSET_DATA			ISO7816_ADPUOFFSET_BLOCK 		+ 6


int st95hf_iso7816_select_file (const struct device* dev, uint8_t mode, uint8_t option, uint8_t data_length, const uint8_t *data );
int st95hf_iso7816_read_binary (const struct device* dev, uint16_t offset, uint8_t expected_length, uint8_t *data );
int st95hf_iso7816_update_binary (const struct device* dev, uint8_t mode, uint8_t option, uint8_t data_length, const uint8_t *data);
