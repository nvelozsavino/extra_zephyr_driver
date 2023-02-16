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
#include <string.h>
#include <zephyr/drivers/spi.h>


#define ST95HF_POLL_MASK_WRITE_READY		0x04
#define ST95HF_POLL_MASK_READ_READY			0x08

/**
 * Commands
*/
#define ST95HF_CMD_IDN							0x01
#define ST95HF_CMD_PROTOCOL_SELECTION			0x02
#define ST95HF_CMD_POLLFIELD					0x03
#define ST95HF_CMD_SEND_RECV					0x04
#define ST95HF_CMD_LISTEN						0x05
#define ST95HF_CMD_SEND							0x06
#define ST95HF_CMD_IDLE							0x07
#define ST95HF_CMD_RDREG						0x08
#define ST95HF_CMD_WRREG						0x09
#define ST95HF_CMD_SUB_FREQ_RES					0x0B
#define ST95HF_CMD_ACFILTER						0x0D
#define ST95HF_CMD_ECHO							0x55

/**
 * Protocol Codes
*/
#define ST95HF_PROTOCOL_CODE_READER_FIELD_OFF			0x00
#define ST95HF_PROTOCOL_CODE_READER_IEC15693			0x01
#define ST95HF_PROTOCOL_CODE_READER_IEC14443A			0x02
#define ST95HF_PROTOCOL_CODE_READER_IEC14443B			0x03
#define ST95HF_PROTOCOL_CODE_READER_IEC18092			0x04
#define ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC14443A	0x12
#define ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC14443B	0x13
#define ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC18092	0x14


/**
 * Error codes
*/
#define ST95HF_STATUS_SUCCESS					0x00		
#define ST95HF_ERROR_CODE_FRAME_RECV_OK			0x80		/* EFrameRecvOK		Frame correctly received (additionally see CRC/Parity information) */ 
#define ST95HF_ERROR_CODE_USER_STOP				0x85		/* EUserStop		Stopped by user (used only in Card mode) */ 
#define ST95HF_ERROR_CODE_COMM_ERROR			0x86		/* ECommError		Hardware communication error */ 
#define ST95HF_ERROR_CODE_FRAME_WAIT_TIME_OUT	0x87		/* EFrameWaitTOut	Frame wait time out (no valid reception) */ 
#define ST95HF_ERROR_CODE_INVALID_SOF			0x88		/* EInvalidSof		Invalid SOF */ 
#define ST95HF_ERROR_CODE_BUFFER_OVERFLOW		0x89		/* EBufOverflow		Too many bytes received and data still arriving */ 
#define ST95HF_ERROR_CODE_FRAMING_ERROR			0x8A		/* EFramingError	if start bit = 1 or stop bit = 0 */ 
#define ST95HF_ERROR_CODE_EGT_ERROR				0x8B		/* EEgtError		EGT time out */ 
#define ST95HF_ERROR_CODE_INVALID_LEN			0x8C		/* EInvalidLen		Valid for ISO/IEC 18092, if Length <3 */ 
#define ST95HF_ERROR_CODE_CRC_ERROR				0x8D		/* ECrcError		CRC error, Valid only for ISO/IEC 18092 */ 
#define ST95HF_ERROR_CODE_RECV_LOST				0x8E		/* ERecvLost		hen reception is lost without EOF received (or subcarrier was lost) */ 
#define ST95HF_ERROR_CODE_NO_FIELD				0x8F		/* ENoField			When Listen command detects the absence of external field */ 
#define ST95HF_ERROR_CODE_UNINT_BYTE			0x90		/* EUnintByte		Residual bits in last byte. Useful for ACK/NAK reception of ISO/IEC 14443 Type A. */ 


/**
 * WakeUp Source
*/
#define ST95HF_WAKEUP_SOURCE_TIME_OUT				0x01
#define ST95HF_WAKEUP_SOURCE_TAG_DETECTION			0x02
#define ST95HF_WAKEUP_SOURCE_FIELD_DETECTOR			0x04
#define ST95HF_WAKEUP_SOURCE_NOT_DEFINED			0x06
#define ST95HF_WAKEUP_SOURCE_LOW_PULSE_ON_IRQ_IN	0x08
#define ST95HF_WAKEUP_SOURCE_LOW_PULSE_ON_SPI_SS	0x10


/**
 * Enter Control
*/
#define ST95HF_ENTER_CTRL_HIBERNATE									0x0400
#define ST95HF_ENTER_CTRL_SLEEP										0x0100
#define ST95HF_ENTER_CTRL_SLEEP_IF_TIMEOUT_SOURCE_ENABLED			0x2100
#define ST95HF_ENTER_CTRL_SLEEP_IF_FIELD_DETECTOR_SOURCE_ENABLED	0x0142
#define ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION					0xA100	//In the datasheet says it should be 0xA200, but in the example and their code use 0xA100
#define ST95HF_ENTER_CTRL_TAG_DETECTION								0x2100

/**
 * WakeUp Control
*/
#define ST95HF_WAKEUP_CTRL_HIBERNATE								0x0400
#define ST95HF_WAKEUP_CTRL_SLEEP_FIELD_DETECTOR						0x3800
#define ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION			0xF801
#define ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTION						0x7901

/**
 * Leave Control
*/
#define ST95HF_LEAVE_CTRL_HIBERNATE									0x1800
#define ST95HF_LEAVE_CTRL_SLEEP_FIELD_DETECTOR						0x1800
#define ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION			0x1800
#define ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTION						0x1800


/**
 * AC State
*/
#define ST95HF_AC_STATE_IDLE		0x00
#define ST95HF_AC_STATE_READY_A		0x01
#define ST95HF_AC_STATE_READY_B		0x02
#define ST95HF_AC_STATE_READY_C		0x03
#define ST95HF_AC_STATE_ACTIVE		0x04
#define ST95HF_AC_STATE_HALT		0x80
#define ST95HF_AC_STATE_READY_AX	0x81
#define ST95HF_AC_STATE_READY_BX	0x82
#define ST95HF_AC_STATE_READY_CX	0x83
#define ST95HF_AC_STATE_ACTIVE_X	0x84


#pragma pack(push,1)


typedef struct {
	uint8_t _unused_1_0:2;		/* Bits 1..0: 	Not significant*/
	uint8_t write_ready:1;		/* Bit 2: 		Data can be sent to the ST95HF when set. */
	uint8_t read_ready:1;		/* Bit 3: 		Data can be read from the ST95HF when set. */
	uint8_t _unused_7_4:4;		/* Bits 7..4: 	Not significant*/
} st95hf_poll_flag_t;


typedef struct {
	uint8_t cmd;
	uint8_t len;
	const void* data;
} st95hf_req_t;

typedef struct {
	uint8_t result_code;
	uint16_t len;
} st95hf_rsp_t;

/**
 * IDN command (0x01)
 */
typedef struct {
	char device_id[13];
	uint16_t crc;
} st95hf_idn_data_t;


/**
 * Protocol Select command (0x02)
*/

/*
Field OFF
*/
typedef struct {

	uint8_t rfu0;				/* 	Byte 0: 		RFU (reserved for future use)	*/

} st95hf_protocol_reader_field_off_t;


/*
ISO/IEC 15693
*/
typedef struct {

	uint8_t append_crc:1; 		/*	Byte 0 [0]: 	Append CRC if set to ‘1’ */

	uint8_t subcarrier:1; 		/*	Byte 0 [1]:		0: Single subcarrier (S)
													1: Dual subcarrier (D) 	*/

	uint8_t modulation:1; 		/*	Byte 0 [2]:		0: Respect 312-µs delay
													1: Wait for SOF 		*/

	uint8_t delay:1; 			/*	Byte 0 [3]:		0: 100% modulation (100)
													1: 10% modulation (10)	*/

	uint8_t bitrate:2; 			/*	Byte 0 [5..4]:	00: 26 Kbps (H)
													01: 52 Kbps
													10: 6 Kbps (L)
													11: RFU (reserved for future use)	*/
	
	uint8_t rfu0_7_6:2;			/*	Byte 0 [7..6]:	RFU (reserved for future use) */

} st95hf_protocol_reader_iec15693_t;

/*
ISO/IEC 14443 Type A
NFC Forum Tag Type 1 (Topaz)
NFC Forum Tag Type 2
NFC Forum Tag Type 4A
*/
typedef struct {

	uint8_t rfu0_2_0:3;			/*	Byte 0 [2..0]:	RFU (reserved for future use) */

	uint8_t rfu0_3:1;			/*	Byte 0 [3]:		RFU (reserved for future use) */
	
	uint8_t reception_dr:2; 	/* Reception Data Rate
									Byte 0 [5..4]:	00: 106 Kbps
													01: 212 Kbps
													10: 424 Kbps
													11: RFU (reserved for future use)	*/

	uint8_t transmission_dr:2; 	/* Transmission Data Rate
									Byte 0 [7..6]:	00: 106 Kbps
													01: 212 Kbps
													10: 424 Kbps
													11: RFU (reserved for future use)	*/


	/*  
	 	These 5 bytes are optional. The default PP:MM:DD value is 0	(corresponds to FDT 86/90µs).
		For other values, FDT = (2^PP)*(MM+1)*(DD+128)*32/13.56 µs
	*/
	uint8_t PP;					/*	Byte 1 			PP*/

	uint8_t MM;					/*	Byte 2 			MM*/

	uint8_t DD;					/*	Byte 3 			(optional to PP:MM) */

	uint8_t st_reserved[2];		/*	Bytes 5..4 		(optional) */	
	
} st95hf_protocol_reader_iec14443a_t;



/*
ISO/IEC 14443 Type B
NFC Forum Tag Type 4B
 */
typedef struct {

	uint8_t append_crc:1; 		/*	Byte 0 [0]: 	Append CRC if set to ‘1’ */

	uint8_t rfu0_3_1:3;			/*	Byte 0 [3..1]:	RFU (reserved for future use) */
	
	uint8_t reception_dr:2; 	/* Reception Data Rate
									Byte 0 [5..4]:	00: 106 Kbps
													01: 212 Kbps
													10: 424 Kbps
													11: 848 Kbps	*/

	uint8_t transmission_dr:2; 	/* Transmission Data Rate
									Byte 0 [7..6]:	00: 106 Kbps
													01: 212 Kbps
													10: 424 Kbps
													11: 848 Kbps	*/
	/*  
	 	These 9 bytes are optional. Default value of PP:MM:DD is 0 and corresponds to FWT ~302µs.
		FWT = (2^PP)*(MM+1)*(DD+128)* 32/13.56 µs
	*/
	uint8_t PP;					/* 	Byte 1 			PP */

	uint8_t MM;					/* 	Byte 2 			MM */

	uint8_t DD;					/* 	Byte 3 			(optional to PP:MM) */

	uint8_t TTTT[2];			/* 	Bytes 5..4 		(optional) TR0 = TTTT/FC (LSB first), default 1023 = 0x3FF */

	uint8_t YY;					/* 	Byte 6			(optional) PCD Min TR1 (Min_TR1 = 8 * XX / fS), default = 0 */

	uint8_t ZZ;					/* 	Byte 7			(optional) PCD Max TR1 (Max_TR1 = 8 * ZZ / fS), default = 26 = 0x1A */

	uint8_t st_reserved[2];		/* 	Bytes 9..8		(optional) */		

} st95hf_protocol_reader_iec14443b_t;


/*
ISO/IEC 18092
NFC Forum Tag Type 3
 */
typedef struct {

	uint8_t append_crc:1; 		/*	Byte 0 [0]: 	Append CRC if set to ‘1’ */
	
	uint8_t rfu0_3_1:3;			/*	Byte 0 [3..1]:	RFU (reserved for future use) */
	
	uint8_t reception_dr:2; 	/* Reception Data Rate
									Byte 0 [5..4]:	00: RFU (reserved for future use)
													01: 212 Kbps
													10: 424 Kbps
													11: RFU (reserved for future use)	*/

	uint8_t transmission_dr:2; 	/* Transmission Data Rate
									Byte 0 [7..6]:	00: RFU (reserved for future use)
													01: 212 Kbps
													10: 424 Kbps
													11: RFU (reserved for future use) */


	uint8_t slot_counter:4; 	/*	Byte 1 [3..0]: 	Slot counter
													0: 1 slot
													1: 2 slots
													…
													F: 16 slots	*/
	
	uint8_t fwt:1;				/*	Byte 1 [4]:		0: FWT = 2.4 ms
													1: FWT is specified by PP:MM bits */
	
	uint8_t rfu1_7_5:2; 		/* 	Byte 1 [7..5]:	RFU (reserved for future use) */


	/*  
	 	These 3 bytes are optional. Default value PP:MM:DD: is 0 and corresponds to RWT ~302µs.
		RWT = (2^PP)*(MM+1)*(DD+128)*32/13.56µs
	*/
	uint8_t PP;					/*  Byte 2 			PP */

	uint8_t MM;					/*  Byte 3 			MM */

	uint8_t DD;					/*  Byte 4 			(optional to PP:MM) */

} st95hf_protocol_reader_iec18092_t;



/*
ISO/IEC 14443 Type A 
Topaz is not supported in Card Emulation mode.
*/
typedef struct {

	uint8_t rfu0_0:1;			/*	Byte 0 [0]:		RFU (reserved for future use) */

	uint8_t hfo_clkrec:1;		/*	Byte 0 [1]:		0: HFO
													1: ClkRec */
	
	uint8_t rfu0_2:1;			/*	Byte 0 [2]:		RFU (reserved for future use) */


	uint8_t wait_rf:1;			/*	Byte 0 [3]:		0: Return an error, if no RF field
													1: Wait for RF field */
	

/*
Card Emulation for ISO/IEC 14443 Type A, Data rate is 106 Kbps for both up- and down-links.
212 Kbps and 424 Kbps Not qualified for this version.
*/
	uint8_t reception_dr:2; 	/* Reception Data Rate
									Byte 0 [5..4]:	00: 106 Kbps
													01: 212 Kbps
													10: 424 Kbps
													11: RFU (reserved for future use)	*/

	uint8_t transmission_dr:2; 	/* Transmission Data Rate
									Byte 0 [7..6]:	00: 106 Kbps
													01: 212 Kbps
													10: 424 Kbps
													11: RFU (reserved for future use)	*/

} st95hf_protocol_emulation_iec14443a_t;


typedef struct {
	uint8_t protocol;			/* Protocol Code */
	union {
		st95hf_protocol_reader_field_off_t		field_off;
		st95hf_protocol_reader_iec15693_t		reader_iec15693;
		st95hf_protocol_reader_iec14443a_t		reader_iec14443a;
		st95hf_protocol_reader_iec14443b_t		reader_iec14443b;
		st95hf_protocol_reader_iec18092_t		reader_iec18092;
		st95hf_protocol_emulation_iec14443a_t	card_emulation_iec14443a;
	} parameters;
} st95hf_protocol_selection_req_t;


/**
 * Pollfield command (0x03)
*/


typedef struct {
	uint8_t flags;				/* RF field presence (Optional)
									0x01: Wait for RF field appearance 
									0x00: Wait for RF field disappearance  */
	uint8_t presc;				/* Timer prescaler (Optional) */
	uint8_t timer;				/* Timer time-out (Optional) */

	/*
	Flags, Presc and Timer parameters are optional. They must be specidfied if the application 
	has to wait for RF field appearance or disappearance. 
	The time to wait is (presc+1)*(timer+1)/13.56 [µs].
	*/

} st95hf_pollfield_req_t;

typedef struct {
	uint8_t field_det:1;		/* Field Detect
									Byte 0 [0]: 	1: FieldDet is set (Field detected)
													0: Otherwise  */
	uint8_t rfu0_7_1:7;			/*	Byte 0 [7..1]:	RFU (reserved for future use) */

} st95hf_pollfield_data_t;


/**
 * Send Receive (SendRecv) command (0x04)
*/

typedef struct{
	uint8_t rfu0_6_0:7; 			/* 	Byte 0 [6..0] 	RFU (reserved for future use) */
	uint8_t parity:1; 				/* 	Byte 0 [7] 		Parity */
} st95hf_parity_byte_t;



typedef struct {
	uint8_t ack_nak;			/* ISO 14443-A ACK or NAK detection
									Byte 0: 		0x0A: ACK
													0x00: NAK */
	uint8_t xx;					/* 	Byte 1: Error type and number of significant bits in first data byte */
	uint8_t yy;					/* 	Byte 2: First byte collision */
	uint8_t zz;					/* 	Byte 3: First bit collision */
} st95hf_sendrecv_ack_nak_t;


typedef struct{
	uint8_t significant_bits:4;		/*	Byte 0 [3..0]	Number of significant bits in last byte */
	uint8_t parity_framing_mode:1; 	/* 	Byte 0 [4] 		Parity Framing mode */
	uint8_t append_crc:1; 			/* 	Byte 0 [5] 		Append CRC */
	uint8_t split_frame:1; 			/* 	Byte 0 [6] 		SplitFrame */
	uint8_t topaz_send_format:1;	/* 	Byte 0 [7] 		Topaz send format. Use EOF instead of parity bit 
														and use SOF at beginning of each byte. Pause between
														bytes and assume 1st byte is 7 bits. */
} st95hf_sendrecv_iec14443a_footer_req_t;


typedef struct{
	uint8_t collision_detect:1;		/*	Byte 0 [0]		0: Colision is not detected 
														1: Colision is detected*/
														
	uint8_t crc_error:1; 			/* 	Byte 0 [1] 		0: No CRC error
														1: CRC error */
	uint8_t rfu0_7_2:6; 			/* 	Byte 0 [7..2] 	RFU (reserved for future use) */
} st95hf_sendrecv_iec15693_footer_rsp_t;



typedef struct{
	uint8_t significant_bits:4;		/*	Byte 0 [3..0]	Number of significant bits in last byte */
	uint8_t parity_error:1; 		/* 	Byte 0 [4] 		0: No Parity error
														1: Parity error */
	uint8_t crc_error:1; 			/* 	Byte 0 [5] 		0: No CRC error
														1: CRC error */
	uint8_t rfu0_6:1; 				/* 	Byte 0 [6] 		RFU (reserved for future use) */

	uint8_t collision_detect:1;		/*	Byte 0 [7]		0: Colision is not detected 
														1: Colision is detected*/
	
	uint8_t	first_collision_byte;	/* 	Byte 1:			Index of the first byte where collision is detected */														


	uint8_t first_collision_bit:4;	/*	Byte 2 [3..0]	Index of the first bit where collision is detected */
	uint8_t rfu2_7_4:4; 			/* 	Byte 2 [7..4] 	RFU (reserved for future use) */

} st95hf_sendrecv_iec14443a_footer_rsp_t;




typedef struct{
	uint8_t rfu0_0:1; 				/* 	Byte 0 [0] 	RFU (reserved for future use) */
	uint8_t crc_error:1; 			/* 	Byte 0 [1] 		0: No CRC error
														1: CRC error */
	uint8_t rfu0_7_2:6; 			/* 	Byte 0 [7..2] 	RFU (reserved for future use) */
} st95hf_sendrecv_iec14443b_footer_rsp_t;


typedef st95hf_sendrecv_iec14443b_footer_rsp_t st95hf_sendrecv_iec18092_footer_rsp_t;




/**
 * Listen command (0x05)
*/

typedef struct{
	uint8_t bcc;					/* 	Byte 0: 		Received value of BCC (if any) */
	uint8_t crc[2];					/* 	Bytes 2..1: 	Received value of CRC (if any) */

	uint8_t significant_bits:4;		/*	Byte 3 [3..0]	Number of significant bits in last byte */
	uint8_t parity_error:1; 		/* 	Byte 3 [4] 		0: No Parity error
														1: Parity error */
	uint8_t crc_error:1; 			/* 	Byte 3 [5] 		0: No CRC error
														1: CRC error */
	uint8_t rfu0_6:1; 				/* 	Byte 3 [6] 		RFU (reserved for future use) */
	uint8_t rfu0_7:1; 				/* 	Byte 3 [7] 		RFU (reserved for future use) */

} st95hf_listen_iec14443a_footer_rsp_t;


/**
 * Send command (0x06)
*/

typedef struct{
	uint8_t significant_bits:4;		/*	Byte 0 [3..0]	Number of significant bits in last byte */
	uint8_t rfu0_4:1; 				/* 	Byte 0 [4] 		RFU (reserved for future use) (Do not append parity */
	uint8_t append_crc:1; 			/* 	Byte 0 [5] 		Append CRC */
	uint8_t rfu0_7_6:2; 			/* 	Byte 0 [7..6] 	RFU (reserved for future use) */
} st95hf_send_iec14443a_footer_req_t;

/**
 * Idle command (0x07)
*/


typedef struct {
	uint8_t wakeup_source;			/* Specifies authorized wake-up sources and the LFO frequency
										This byte defines the authorized wake-up sources in the Wake-up source register */
	
	/* Settings to enter WFE (Wait for event) mode 
		These two bytes (EnterCtrlL and EnterCtrlH) define the resources when entering WFE mode */
	uint8_t enter_ctrl_l;			
	uint8_t enter_ctrl_h;
	
	/* Settings to wake-up from WFE mode 
		These two bytes (WuCtrlL and WuCtrlH) define the wake-up resources 	*/
	uint8_t wakeup_ctrl_l;			
	uint8_t wakeup_ctrl_h;			
	
	/* Settings to leave WFE mode.
		These two bytes (LeaveCtrlL and LeaveCtrlH) define the resources when returning to Ready state. (Default value = 0x1800) */
	uint8_t leave_ctrl_l;			
	uint8_t leave_ctrl_h;
	
	uint8_t wakeup_period;			/* This byte is the coefficient used to adjust the time allowed between two tag
										detections. Also used to specify the duration before Timeout. (Typical value: 0x20)
										Duration before Timeout = 256 * tL * (WU period + 2) * (MaxSleep + 1)
										*/
	
	uint8_t osc_start;				/* Defines the Wait time for HFO to stabilize: <OscStart> * tL (Default value = 0x60) */
	
	uint8_t dac_start;				/* Defines the Wait time for DAC to stabilize: <DacStart> * tL (Default value = 0x60) */
	
	/* 	These two bytes (DacDataL and DacDataH) define the lower and higher comparator values, respectively. 
		These values are determined by a calibration process.*/
	uint8_t dac_data_l;				/* Lower compare value for tag detection.
										This value must be set to 0x00 during tag detection calibration. */
	uint8_t dac_data_h;				/* Higher compare value for tag detection. This is a variable used during tag detection calibration */

	uint8_t swings_count;			/* Number of swings HF during tag detection (Default value = 0x3F) (Recommended value: 0x3F) */ 

	uint8_t max_sleep;				/* Max. number of tag detection trials before Timeout. 
										This value must be set to 0x01 during tag detection calibration.
										Also used to specify duration before Timeout. 
										MaxSleep must be: 0x00 < MaxSleep < 0x1F 
										Duration before Timeout = 256 * tL * (WU period + 2) * (MaxSleep + 1) (Typical value: 0x28)
										*/

} st95hf_idle_req_t;

/* This response is sent only when ST95HF exits WFE mode. */
typedef struct {
	uint8_t timeout:1;				/* Byte 0 [0]:			Timeout */
	uint8_t tag_detect:1;			/* Byte 0 [1]:			Tag detected */
	uint8_t rfu0_2:1;				/* Byte 0 [2]:			RFU (reserved for future use) */
	uint8_t low_pulse_irq_in:1;		/* Byte 0 [3]:			Low pulse on IRQ_IN */
	uint8_t low_pulse_spi_ss:1;		/* Byte 0 [4]:			Low pulse on SPI_SS */
	uint8_t rfu0_7_5:3;				/* Byte 0 [7..5]:		RFU (reserved for future use) */
} st95hf_idle_data_t;


/**
 * Read Register (RdReg) command (0x08)
*/

typedef struct {
	uint8_t reg_addr;			/* 	Byte 0:				Register Address */
	uint8_t reg_size;			/* 	Byte 1:				Register Size. Always = 0x01 */
	uint8_t st_reserved;		/* 	Byte 2:				Reserved. Always = 0x00 */
} st95hf_read_req_t;

/**
 * Write Register (WrReg) command (0x09)
*/

typedef struct {
	uint8_t index;					/* Index pointier:		0x01: Points to the Modulation Index and Receiver Gain values in the ARC_B register */

	uint8_t receiver_gain:4;		/* 	Receiver Gain		0x0: 34dB
															0x1: 32dB
															0x3: 27dB
															0x7: 20dB
															0xF: 8dB */

	uint8_t modulation_index:4;		/* 	Modulation Index	0x1: 10%
															0x2: 17% 
															0x3: 25%
															0x4: 30%
															0x5: 33%
															0x6: 36%
															0xD: 95% */
} st95hf_write_arc_b_t;

typedef struct {
	uint8_t index;					/* Index pointier:		0x04: Points to Demodulator Sensitivity and Load Modulation values in ACC_A register*/

	uint8_t load_modulation:4;		/* Load Modulation		0x1: 10%
															0x2: 100% */

	uint8_t demod_sensitivity:4;	/* Demodulator Sensitivity (valid from 0x1 to 0xF) 	0x1: Min	0x7: Default	0xF: Max */
} st95hf_write_acc_a_t;

typedef struct {
	uint8_t value;					/* Set TimerW value (recommended value is 0x56 or 0x58)*/
	uint8_t confirmation;			/* TimerW value confirmation = 0x04 */
} st95hf_write_timer_w_t;

typedef struct {
	uint8_t enable;					/* AutoDetect filter enable = 0x02*/
	uint8_t confirmation;			/* AutoDetect filter confirmation = 0xA1 */
} st95hf_write_auto_detect_t;



typedef struct {
	uint8_t reg_addr;			/* 	Byte 0:	Register Address 	0x68: for AAC_A or ARC_B
																0x3A: Timer Window (TimerW) (ISO/IEC 14443 Type A)
																0x0A: AutoDetect filter control value (ISO/IEC 18092)*/
	uint8_t increment;			/* 	Byte 1:				Flag Increment address or not after Write
															- 0x00 to not increment
													 		- 0x01 to increment after write */
	union {
		st95hf_write_arc_b_t arc_b;
		st95hf_write_acc_a_t aac_a;
		st95hf_write_timer_w_t timer_w;
		st95hf_write_auto_detect_t auto_detect;
	} params;
} st95hf_write_req_t;

/**
 * Subcarrier frequency response (0x0B)
*/

typedef struct {
	uint8_t freq_div;	/* N, frequency divider.  Subcarrier frequency is fs = fc / (2*(N+1)) */
} st95hf_sub_freq_data_t;


/**
 * AcFilter command (0x0D)
*/

typedef struct {
	uint8_t atqa[2];		/* Unused and proprietary bits of SAK (protocol bits will be handled by firmware) */
	uint8_t sak;			/* Unused and proprietary bits of SAK (protocol bits will be handled by firmware) */
	uint32_t uid_1;			/* UID for cascade level 1 (Mandatory) */
	uint32_t uid_2;			/* UID for cascade level 2 (Optional) */
	uint32_t uid_3;			/* UID for cascade level 3 (Optional) */
} st95hf_ac_filter_req_t;

typedef struct {
	uint8_t ac_state;		/* AC state */
} st95hf_ac_filter_data_t;


#pragma pack(pop)


typedef enum {
	ST95HF_STATE_UNKNOWN = 0,
	ST95HF_STATE_HIBERNATE ,
	ST95HF_STATE_SLEEP,
	ST95HF_STATE_POWERUP,
	ST95HF_STATE_TAGDETECTOR,
	ST95HF_STATE_READY,
	ST95HF_STATE_READER,
	ST95HF_STATE_TAGHUNTING,
}st95hf_state_t;

typedef enum {
	ST95HF_MODE_UNKNOWN = 0,
	ST95HF_MODE_READER ,
	ST95HF_MODE_CARDEMULATOR ,
	ST95HF_MODE_PASSIVEP2P ,
	ST95HF_MODE_ACTIVEP2P ,
} st95hf_mode_t;



typedef struct {
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_irq_in;
#ifdef CONFIG_ST95HF_TRIGGER
	const struct gpio_dt_spec gpio_irq_out;
#endif /* CONFIG_ST95HF_TRIGGER */
} st95hf_config_t;

/*
#define ST95HF_PROTOCOL_CODE_READER_FIELD_OFF			0x00
#define ST95HF_PROTOCOL_CODE_READER_IEC15693			0x01
#define ST95HF_PROTOCOL_CODE_READER_IEC14443A			0x02
#define ST95HF_PROTOCOL_CODE_READER_IEC14443B			0x03
#define ST95HF_PROTOCOL_CODE_READER_IEC18092			0x04
#define ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC14443A	0x12
#define ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC14443B	0x13
#define ST95HF_PROTOCOL_CODE_CARD_EMULATION_IEC18092	0x14
*/
typedef enum {
	ST95HF_PROTOCOL_UNKNOWN = 0,
	ST95HF_PROTOCOL_READER_IEC14443A,
	ST95HF_PROTOCOL_READER_IEC14443B,
	ST95HF_PROTOCOL_READER_IEC15693,
	ST95HF_PROTOCOL_READER_IEC18092,
	ST95HF_PROTOCOL_CARD_EMULATION_IEC14443A,
	ST95HF_PROTOCOL_CARD_EMULATION_IEC14443B,
	ST95HF_PROTOCOL_CARD_EMULATION_IEC15693,
	ST95HF_PROTOCOL_CARD_EMULATION_IEC18092,
} st95hf_protocol_t;	


typedef struct {

#ifdef CONFIG_PM_DEVICE
	#erro todo
#endif

#ifdef CONFIG_ST95HF_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_irq_out_cb;

	struct k_sem rx_sem;

#endif /* CONFIG_ST95HF_TRIGGER */
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
int st95hf_subcarrier_frequency_cmd(const struct device* dev, st95hf_rsp_t* rsp, st95hf_sub_freq_data_t* data,k_timeout_t timeout);

int st95hf_ac_filter_deactivate_cmd(const struct device* dev, st95hf_rsp_t * rsp, k_timeout_t timeout);
int st95hf_ac_filter_set_state_cmd(const struct device* dev, uint8_t state, st95hf_rsp_t * rsp, k_timeout_t timeout);
int st95hf_ac_filter_get_state_cmd(const struct device* dev, st95hf_rsp_t * rsp, st95hf_ac_filter_data_t* data,k_timeout_t timeout);
int st95hf_ac_filter_activate_anti_colision_cmd(const struct device* dev, uint8_t uid_count, const st95hf_ac_filter_req_t* req, st95hf_rsp_t * rsp, k_timeout_t timeout);

int st95hf_echo_cmd(const struct device* dev,k_timeout_t timeout);



#endif /* __SENSOR_ST95HF__ */
