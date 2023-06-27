#pragma once

#include "iso14443a.h"
#include "iso14443b.h"
#include "iso15693.h"
#include "iso18092.h"

typedef enum {
    NFC_CARD_PROTOCOL_NONE=0,
    NFC_CARD_PROTOCOL_ISO14443A_TYPE1,
    NFC_CARD_PROTOCOL_ISO14443A_TYPE2,
    NFC_CARD_PROTOCOL_ISO18092_TYPE3,
    NFC_CARD_PROTOCOL_ISO14443A_TYPE4A,
    NFC_CARD_PROTOCOL_ISO14443B_TYPE4B,
    NFC_CARD_PROTOCOL_ISO15693_TYPE5,
    NFC_CARD_PROTOCOL_LAST,
    NFC_CARD_PROTOCOL_UNKNOWN=0xFF,
} nfc_card_protocol_t;

typedef struct {
	nfc_card_protocol_t type;
	union {
		iso14443a_uid_t iso14443a;
		iso14443b_pupi_t iso14443b;
		iso15963_uid_t iso15963;
		iso18092_uid_t iso18092;
	} uid;
}nfc_uid_t;


