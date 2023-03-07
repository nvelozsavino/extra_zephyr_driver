#pragma once

#include <zephyr/types.h>
#include <stdbool.h>

/*  status and error code ---------------------------------------------------------------------- */
#define ISO14443A_SUCCESSCODE									RESULTOK
#define ISO14443A_ERRORCODE_DEFAULT						0x61
#define ISO14443A_ERRORCODE_CRC								0x62

/* Anticollison levels (commands)  ------------------------------------------------------------- */
#define SEL_CASCADE_LVL_1											0x93
#define SEL_CASCADE_LVL_2											0x95
#define SEL_CASCADE_LVL_3											0x97
#define COMMAND_REQA													0x26
#define COMMAND_WUPA													0x52
#define COMMAND_SELECT_LV1										0x93
#define COMMAND_SELECT_LV2										0x95
#define COMMAND_SELECT_LV3										0x97
#define COMMAND_HLTA													0x50

#define COMMAND_RATS													0xE0
#define COMMAND_PPS														0xD0
#define COMMAND_DESELECT											0xC2
#define COMMAND_DESELECTCID										0xCA

/* Iblock  ------------------------------------------------------------------------------------- */
#define COMMAND_IBLOCK02												0x02
#define COMMAND_IBLOCK03												0x03
#define COMMAND_SBLOCK													0xC2
#define COMMAND_NACKBLOCK_B2										0xB2
#define COMMAND_NACKBLOCK_B3										0xB3
#define COMMAND_ACKBLOCK												0xA2


/* numbr of the cascade level  ----------------------------------------------------------------- */
#define CASCADE_LVL_1															1
#define CASCADE_LVL_2															2
#define CASCADE_LVL_3															3
		
#define ISO14443A_NVM_10													0x10
#define ISO14443A_NVM_20													0x20
#define ISO14443A_NVM_30													0x30
#define ISO14443A_NVM_40													0x40
#define ISO14443A_NVM_50													0x50
#define ISO14443A_NVM_60													0x60
#define ISO14443A_NVM_70													0x70

/* UID Sizes ---------------------------------------------------------------------------------- */
#define ISO14443A_UIDSIZE_UNDEFINED								-1
#define ISO14443A_UID_PART												3
#define ISO14443A_UID_SINGLE_SIZE									4
#define	ISO14443A_UID_DOUBLE_SIZE									7
#define ISO14443A_UID_TRIPLE_SIZE									10


/* Mask used for ATQA ------------------------------------------------------------------------ */
#define ISO14443A_UID_MASK												0xC0
#define ISO14443A_AC_BIT_FRAME_MASK								0x1F
#define ISO14443A_CID_MASK												0x0F
#define ISO14443A_FSDI_MASK												0xF0

/* Size for ISO14443A variables ------------------------------------------------------------- */
#define ISO14443A_MAX_NAME_SIZE										50
#define ISO14443A_MAX_UID_SIZE			 							10
#define ISO14443A_ATQA_SIZE												2

/* SAK FLAG --------------------------------------------------------------------------------- */
#define SAK_FLAG_ATS_SUPPORTED										0x20
#define SAK_FLAG_UID_NOT_COMPLETE									0x04


/* ATQ FLAG */
#define ATQ_FLAG_UID_SINGLE_SIZE		0
#define	ATQ_FLAG_UID_DOUBLE_SIZE		1
#define ATQ_FLAG_UID_TRIPLE_SIZE		2




/******************  PCD  ******************/
/* ISO14443A */
#define PCD_TYPEA_ARConfigA	0x01
#define PCD_TYPEA_ARConfigB	0xDF

#define PCD_TYPEA_TIMERW    0x5A

/* Mask used for ATQA ------------------------------------------------------------------------ */
#define ISO14443A_UID_MASK												0xC0
#define ISO14443A_AC_BIT_FRAME_MASK								0x1F
#define ISO14443A_CID_MASK												0x0F
#define ISO14443A_FSDI_MASK												0xF0



/* ISO14443B */
#define PCD_TYPEB_ARConfigA	0x01
#define PCD_TYPEB_ARConfigB	0x51

/* Felica */
#define PCD_TYPEF_ARConfigA	0x01
#define PCD_TYPEF_ARConfigB	0x51

/* ISO15693 */
#define PCD_TYPEV_ARConfigA	0x01
#define PCD_TYPEV_ARConfigB	0xD1


#define ISO14443A_MAX_UID_SIZE			 							10
#define ISO14443A_ATQA_SIZE												2

typedef struct {
    uint8_t data[ISO14443A_ATQA_SIZE];
} iec14443a_atqa_t;


typedef struct{
	/* ATQA answer to request of type A*/
	iec14443a_atqa_t 	atqa;
	uint8_t 	cascade_level;
	/* UID : unique Identification*/
	uint8_t 	uid_size;
	uint8_t 	uid[ISO14443A_MAX_UID_SIZE];
	/* SAK : Select acknowledge*/
	uint8_t 	sak;
	bool 		ats_supported;
	bool 		is_detected;
	char		log_msg[120];
	uint8_t 	cid;
	uint8_t 	fsdi;
	uint8_t 	dri;
	uint8_t 	dsi;
}iec14443A_card_t;