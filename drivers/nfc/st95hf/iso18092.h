#pragma once

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>


/* Felica */
#define PCD_TYPEF_ARConfigA	0x01
#define PCD_TYPEF_ARConfigB	0x51

/* 	-------------------------------------------------------------------------- */
#define ISO18092_ATQC_SIZE                      0x20
#define ISO18092_UID_SIZE						8



#pragma pack(push,1)
typedef struct {
    uint8_t data[ISO18092_ATQC_SIZE];
} iso18092_atqc_t;
#pragma pack(pop)

typedef struct {
	iso18092_atqc_t atqc;
	uint8_t uid	[ISO18092_UID_SIZE];
	bool 		is_detected;
//	char		log_msg[120];
} iso18092_card_t;




int st95hf_iso18092_init(const struct device* dev);

int st95hf_iso18092_is_present(const struct device* dev,iso18092_card_t* card);

