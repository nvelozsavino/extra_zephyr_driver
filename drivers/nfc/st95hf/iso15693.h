#pragma once

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>


/* ISO15693 */
#define PCD_TYPEV_ARConfigA	0x01
#define PCD_TYPEV_ARConfigB	0xD1



/* number of byte of parameters ------------------------------------------------------------- */
#define ISO15693_NBBYTE_UID	 										0x08
#define ISO15693_NBBYTE_CRC16	 									0x02
#define ISO15693_NBBYTE_DSFID 									0x01
#define ISO15693_NBBYTE_AFI 										0x01
#define ISO15693_NBBYTE_BLOCKSECURITY		   			0x01
#define ISO15693_NBBYTE_REPLYFLAG		   					0x01
#define ISO15693_NBBYTE_INFOFLAG		   					0x01
#define ISO15693_NBBYTE_MEMORYSIZE		   				0x02
#define ISO15693_NBBYTE_ICREF			   						0x01
#define ISO15693_NBBYTE_REQUESTFLAG							0x01

#define ISO15693_NBBIT_INVENTORYSCANCHAIN				2+8+8+64
#define ISO15693_NBBYTE_INVENTORYSCANCHAIN			11  // =(2+8+8+64) /8 +1 
#define ISO15693_NBBITS_MASKPARAMETER   				64
#pragma pack(push,1)
typedef union {
	uint8_t 	uid[ISO15693_NBBYTE_UID];
} iso15963_uid_t;


typedef union {
    uint8_t data[ISO15693_NBBYTE_UID+4];
    struct {        
        uint8_t unknown[2];
        iso15963_uid_t uid;        
        uint8_t crc[2];
        // uint8_t application_field[ISO14443B_MAX_APPLI_SIZE];
	    // uint8_t protocol_info[ISO14443B_MAX_PROTOCOL_SIZE];
	    // uint8_t crc_b[2];
    }fields;
}iso15963_inventory_t;
#pragma pack(pop)

typedef struct{
	iso15963_inventory_t inventory;
	bool 		is_detected;
	// char		log_msg[ISO14443B_MAX_LOG_MSG];
}iso15693_card_t;


int st95hf_iso15693_init(const struct device* dev, iso15693_card_t* card);

int st95hf_iso15693_is_present(const struct device* dev, iso15693_card_t* card);

int st95hf_iso15693_anticollision(const struct device* dev, iso15693_card_t* card);