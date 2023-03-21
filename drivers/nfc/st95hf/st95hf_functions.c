#include "st95hf.h"
#include <zephyr/logging/log.h>
#include "iso14443a.h"
#include "iso14443b.h"
#include "iso18092.h"
#include "nfc_type4.h"

LOG_MODULE_DECLARE(st95hf, LOG_LEVEL_DBG);


int st95hf_tag_calibration(const struct device* dev, uint8_t wu_period, uint8_t* dac_data_ref){

	if (dac_data_ref==NULL){
		return -EINVAL;
	}
	
	uint8_t steps[6]  = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04};

	 st95hf_idle_req_t idle_req = {
        .wakeup_source=ST95HF_WAKEUP_SOURCE_TIME_OUT | ST95HF_WAKEUP_SOURCE_TAG_DETECTION,
        // .enter_ctrl=ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION,
        // .wakeup_ctrl=ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION,

        // .leave_ctrl=ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION,

        .enter_ctrl_h=ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION>>8,
        .enter_ctrl_l=ST95HF_ENTER_CTRL_TAG_DETECTOR_CALIBRATION&0xFF,
        .wakeup_ctrl_h=ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION>>8,
        .wakeup_ctrl_l=ST95HF_WAKEUP_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION&0xFF,

        .leave_ctrl_h=ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION>>8,
        .leave_ctrl_l=ST95HF_LEAVE_CTRL_SLEEP_TAG_DETECTOR_CALIBRATION&0xFF,
        .wakeup_period=wu_period,
        .osc_start=0x60, // Recomended value
        .dac_start=0x60, // Recomended value
        .dac_data_h=0x00,
        .dac_data_l=0x00,
        .swings_count=0x3F, // Recomended value
        .max_sleep=0x01, // From ST example

    };
    st95hf_rsp_t rsp;
    st95hf_idle_data_t data;
    //>>>0x07 0E 03 A1 00 F8 01 18 00 20 60 60 00 xx 3F 01
    int err = st95hf_idle_cmd(dev,&idle_req,&rsp,&data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error %d. Calibrating",err);
        return err;
    }
    if (rsp.result_code==0x00 && rsp.len==1 && data.byte==ST95HF_WAKEUP_SOURCE_TAG_DETECTION){
		LOG_INF("Tag Detected with %02x", idle_req.dac_data_h);
		idle_req.dac_data_h=0xFC;
		data.byte=0;
		err = st95hf_idle_cmd(dev,&idle_req,&rsp,&data,K_SECONDS(3));
		if (err!=0){
			LOG_ERR("Error %d. Calibrating",err);
			return err;
		}
		if (rsp.result_code==0x00 && rsp.len==1 && data.byte==ST95HF_WAKEUP_SOURCE_TIME_OUT){
			LOG_INF("Tag Timeout with %02x", idle_req.dac_data_h);
			LOG_INF("Running Algorithm");

			for(uint8_t i=0; i<6; i++) {					
				switch(data.byte) {
					case ST95HF_WAKEUP_SOURCE_TIME_OUT:
						LOG_INF("Tag Timeout with %02x step %d", idle_req.dac_data_h,i);

						idle_req.dac_data_h-= steps[i];
						break;
				
					case ST95HF_WAKEUP_SOURCE_TAG_DETECTION:
						LOG_INF("Tag Detected with %02x step %d", idle_req.dac_data_h,i);

						idle_req.dac_data_h+= steps[i];
						break;
				
					default:
						return -EINVAL;
				}
				data.byte=0;
				err = st95hf_idle_cmd(dev,&idle_req,&rsp,&data,K_SECONDS(3));
				if (err!=0){
					LOG_ERR("Error %d. Calibrating",err);
					return err;
				}
			}
		} else {
			LOG_INF("Tag Detected with %02x", idle_req.dac_data_h);
			return -EIO;
		}
		
		LOG_INF("Finish Algorithm %02x, %02x",data.byte,idle_req.dac_data_h);
		if (rsp.result_code==0x00 && rsp.len==1 && data.byte==ST95HF_WAKEUP_SOURCE_TIME_OUT){
			LOG_INF("Tag Timeout with %02x last step", idle_req.dac_data_h);

			*dac_data_ref = (idle_req.dac_data_h -0x04) ;
		}else{
			LOG_INF("Tag Detected with %02x last step", idle_req.dac_data_h);

			*dac_data_ref = idle_req.dac_data_h ;
		}
		return 0;
    } else {
        LOG_ERR("Calibration failed %02x",rsp.result_code);
    }
	return -EIO;
}

static int st95hf_field_off(const struct device* dev){
    st95hf_protocol_selection_req_t req = {
        .protocol=ST95HF_PROTOCOL_CODE_READER_FIELD_OFF,
        .parameters.field_off.rfu0=0,
        .options=ST95HF_PROTSEL_OPT_ALL,
    };
    st95hf_rsp_t rsp;
    int err =  st95hf_protocol_select_cmd(dev,&req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error response code %02x", rsp.result_code);
        return -EIO;
    }
    return 0;
}



int st95hf_tag_hunting(const struct device* dev, uint8_t tags_to_find, nfc_uid_t* uid){
    if (dev==NULL || uid==NULL){
        return -EINVAL;
    }
    st95hf_data_t* st95hf = dev->data;
    uid->type=NFC_CARD_PROTOCOL_NONE;
    st95hf->device_mode = ST95HF_DEVICE_MODE_UNDEFINED;
    st95hf->tag_type = ST95HF_TAG_TYPE_UNDEFINED;
    int err=0;
    if (tags_to_find & ST95HF_TRACK_NFCTYPE1){
        LOG_INF("Checking for Types 1");
        iso14443a_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso14443a_init(dev,&card);
        if (err!=0){
            LOG_ERR("Error initializing iso14443a %d",err);
            return err;
        }
        
        err = st95hf_iso14443a_is_present(dev,&card);
        if (err == 0){
            LOG_INF("ISO14443A is present");
            
            LOG_INF("Checking for ISO14443A Type 1");
            err = st95hf_iso14443a_check_type1(dev,&card);
            if (err==0){
                LOG_INF("ISO14443A Type 1 found");
                uid->type=NFC_CARD_PROTOCOL_ISO14443A_TYPE1;
                memcpy(&uid->uid.iso14443a,&card.uid,sizeof(iso14443a_uid_t));                
                return 0;
            } else {
                LOG_INF("ISO14443A Type 1 not found. Err %d",err);
            }
        
        }        
    }


  if ((tags_to_find & ST95HF_TRACK_NFCTYPE2)||(tags_to_find & ST95HF_TRACK_NFCTYPE4A)){
        LOG_INF("Checking for Types 2 and 4A");
        iso14443a_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso14443a_init(dev,&card);
        if (err!=0){
            LOG_ERR("Error initializing iso14443a %d",err);
            return err;
        }
        
        err = st95hf_iso14443a_is_present(dev,&card);
        if (err == 0){
            LOG_INF("ISO14443A is present");         
            LOG_INF("Checking for ISO14443A Type 2 or Type 4");
            err = st95hf_iso14443a_anticollision(dev,&card);
            if (err==0){
                LOG_INF("ISO14443A anticollision found");

                if ((card.sak& 0x60)==0x00){
                    LOG_INF("ISO14443A Type 2 found");
                    LOG_HEXDUMP_INF(card.uid.bytes,card.uid.size,"UID:");                
                    uid->type=NFC_CARD_PROTOCOL_ISO14443A_TYPE2;
                    memcpy(&uid->uid.iso14443a,&card.uid,sizeof(iso14443a_uid_t));     
                    return 0;
                } else if ((card.sak& 0x20)!=0x00){
                    LOG_INF("ISO14443A Type 4A found");
                    LOG_HEXDUMP_INF(card.uid.bytes,card.uid.size,"UID:");
                    uid->type=NFC_CARD_PROTOCOL_ISO14443A_TYPE4A;
                    memcpy(&uid->uid.iso14443a,&card.uid,sizeof(iso14443a_uid_t));                         
                    return 0;
                }
            } else {
                LOG_INF("ISO14443A anticollision not found. Err %d",err);
            }

        }        
    }
    //Test FeliCa (iso18092)

    if (tags_to_find & ST95HF_TRACK_NFCTYPE3){
        LOG_INF("Test FeliCa (iso18092)");
        iso18092_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso18092_init(dev,&card);
        if (err!=0){
            LOG_ERR("Error initiating iso18092 %d",err);
            return err;
        }
        
        err = st95hf_iso18092_is_present(dev,&card);
        if (err == 0){
            LOG_INF("iso18092 is present. Type 3 found");            
            uid->type=NFC_CARD_PROTOCOL_ISO18092_TYPE3;
            memcpy(&uid->uid.iso18092,&card.uid,sizeof(iso18092_uid_t));     
        } else {
            LOG_ERR("iso18092 is NOT present. Error: %d",err);
        }
    }


    //Test iso14443b)


    if (tags_to_find & ST95HF_TRACK_NFCTYPE4B){
        LOG_INF("Test iso14443b");
        iso14443b_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso14443b_init(dev,&card);
        if (err!=0){
            LOG_ERR("Error initiating iso14443b %d",err);
            return err;
        }
        
        err = st95hf_iso14443b_is_present(dev,&card);
        if (err == 0){
            LOG_INF("iso14443b is present");
            err = st95hf_iso14443b_anticollision(dev,&card);
            if (err==0){
                LOG_INF("Type 4b found");
                LOG_HEXDUMP_INF(card.atqb.fields.pupi.bytes,sizeof(card.atqb.fields.pupi),"PUPI:");
                uid->type=NFC_CARD_PROTOCOL_ISO14443B_TYPE4B;
                memcpy(&uid->uid.iso14443b,&card.atqb.fields.pupi,sizeof(iso14443b_pupi_t));     
                return 0;
            } else {
                LOG_INF("Type 4b not found. Err: %d",err);
            }
        } else {
            LOG_INF("iso14443b is NOT present. Err: %d",err);
        }
    }

    // Turn off the field if no tag has been detected
    err= st95hf_field_off(dev);
    if (err!=0){
        LOG_ERR("Error turning off the field %d",err);
        return err;
    }

    return 0;
}   



int st95hf_read_ndef_from_tag(const struct device* dev, uint16_t *ndef_size, uint8_t *ndef){
    if (dev==NULL || ndef_size==NULL || ndef==NULL){
        return -EINVAL;
    }
    st95hf_data_t* st95hf = dev->data;
    switch(st95hf->tag_type){
        default:
            return -ENOLINK;
        case ST95HF_TAG_TYPE_TT4A:
        case ST95HF_TAG_TYPE_TT4B:{

            int err = st95hf_type4_read_ndef(dev,ndef_size,ndef);
            if (err !=0){
                LOG_ERR("Error reading NDEF %d",err);
                return err;
            }
            return 0;
        }
    }
    return -EINVAL;
}