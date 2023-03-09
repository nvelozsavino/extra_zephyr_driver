#include "st95hf.h"
#include <zephyr/logging/log.h>
#include "iso14443a.h"
#include "iso14443b.h"
#include "iso18092.h"
LOG_MODULE_REGISTER(st95hf_func, CONFIG_NFC_LOG_LEVEL);


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



int st95hf_tag_hunting(const struct device* dev, uint8_t* tags_type){
    if (dev==NULL || tags_type==NULL){
        return -EINVAL;
    }
    st95hf_data_t* st95hf = dev->data;

    st95hf->device_mode = ST95HF_DEVICE_MODE_UNDEFINED;
    st95hf->tag_type = ST95HF_TAG_TYPE_UNDEFINED;
    uint8_t tags_to_find = *tags_type;
    *tags_type=0x00;
    int err=0;
    if ((tags_to_find & ST95HF_TRACK_NFCTYPE1) || (tags_to_find & ST95HF_TRACK_NFCTYPE2)||(tags_to_find & ST95HF_TRACK_NFCTYPE4A)){
        iso14443a_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso14443a_init(dev,&card);
        if (err!=0){
            return err;
        }
        
        err = st95hf_iso14443a_is_present(dev,&card);
        if (err == 0){
            LOG_INF("ISO14443A is present");
            if (tags_to_find & ST95HF_TRACK_NFCTYPE1){
                LOG_INF("Checking for ISO14443A Type 1");
                err = st95hf_iso14443a_check_type1(dev);
                if (err==0){
                    LOG_INF("ISO14443A Type 1 found");
                    *tags_type=ST95HF_TRACK_NFCTYPE1;
                    return 0;
                } else {
                    LOG_INF("ISO14443A Type 1 not found");
                }
            }
            if ((tags_to_find & ST95HF_TRACK_NFCTYPE2)||(tags_to_find & ST95HF_TRACK_NFCTYPE4A)){
                LOG_INF("Checking for ISO14443A Type 2 or Type 4");
                err = st95hf_iso14443a_anticollision(dev,&card);
                if (err==0){
                    LOG_INF("ISO14443A anticollision found");
                    if ((card.sak& 0x60)==0x00){
                        LOG_INF("ISO14443A Type 2 found");
                        *tags_type=ST95HF_TRACK_NFCTYPE2;
                        return 0;
                    } else if ((card.sak& 0x20)!=0x00){
                        LOG_INF("ISO14443A Type 4A found");
                        *tags_type=ST95HF_TRACK_NFCTYPE4A;
                        return 0;
                    }
                } else {
                    LOG_INF("ISO14443A anticollision not found");
                }
            }
        }        
    }

    //Test FeliCa (iso18092)

    if (tags_to_find & ST95HF_TRACK_NFCTYPE3){
        iso18092_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso18092_init(dev,&card);
        if (err!=0){
            return err;
        }
        
        err = st95hf_iso18092_is_present(dev,&card);
        if (err == 0){
            LOG_INF("iso18092 is present. Type 3 found");
            *tags_type=ST95HF_TRACK_NFCTYPE3;
        }
    }


    //Test iso14443b)

    if (tags_to_find & ST95HF_TRACK_NFCTYPE4B){
        iso14443b_card_t card;
        err= st95hf_field_off(dev);
        if (err!=0){
            LOG_ERR("Error turning off the field %d",err);
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iso14443b_init(dev,&card);
        if (err!=0){
            return err;
        }
        
        err = st95hf_iso14443b_is_present(dev,&card);
        if (err == 0){
            LOG_INF("iso14443b is present");
            err = st95hf_iso14443b_anticollision(dev,&card);
            if (err==0){
                LOG_INF("Type 4b found");
                *tags_type=ST95HF_TRACK_NFCTYPE4B;
            }
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
