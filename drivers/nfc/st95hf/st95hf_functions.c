#include "st95hf.h"
#include <zephyr/logging/log.h>
#include "iso14443a.h"
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
        .parameters.field_off.rfu0=0
    };
    st95hf_rsp_t rsp;
    int err =  st95hf_protocol_select_cmd(dev,&req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_SUCCESS){
        LOG_ERR("Error response code %02x", rsp.result_code);
        return -EIO;
    }
    return 0;
}

static int st95hf_iec14443a_init_anticolision(const struct device* dev){
    st95hf_data_t *st95hf = dev->data;
    st95hf_protocol_selection_req_t protocol_selection_req = { 0} ;
    protocol_selection_req.protocol=ST95HF_PROTOCOL_CODE_READER_IEC14443A;
    // req.parameters.reader_iec14443a.transmission_dr=00; //106 Kbps
    // req.parameters.reader_iec14443a.reception_dr=00; //106 Kbps
    // req.parameters.reader_iec14443a.PP=0x00;
    // req.parameters.reader_iec14443a.MM=0x00;
    // req.parameters.reader_iec14443a.DD=0x00; //The default PP:MM:DD value is 0 (corresponds to FDT 86/90µs)

    st95hf_rsp_t rsp={0};
    int err =  st95hf_protocol_select_cmd(dev,&protocol_selection_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_SUCCESS){
        LOG_ERR("Error %02x selecting protocol",rsp.result_code);
        return -EIO;
    }
    st95hf_write_req_t write_req={0};
    write_req.reg_addr = ST95HF_WREG_ADDR_TIMER_WINDOW;
    write_req.flags=ST95HF_WREG_FLAG_NOT_INCREMENT;
    write_req.params.timer_w.value=PCD_TYPEA_TIMERW;
    write_req.params.timer_w.confirmation=ST95HF_WREG_TIMERW_CONFIRMATION;
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_SUCCESS){
        LOG_ERR("Error %02x writing reg TimerW",rsp.result_code);
        return -EIO;
    }

    write_req.reg_addr = ST95HF_WREG_ADDR_AAC_A_ARC_B;
    write_req.flags=ST95HF_WREG_FLAG_INCREMENT_AFTER_WRITE;
    write_req.params.acc_arc.index=ST95HF_WREG_ACC_ARC_INDEX_ARC_B;
    write_req.params.acc_arc.value.arc_b.byte=PCD_TYPEA_ARConfigB;
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }
    st95hf->current_protocol = ST95HF_PROTOCOL_READER_IEC14443A;
    k_sleep(K_USEC(5100));
    return 0;
}


static void iec14443A_complete_structure ( const iec14443a_atqa_t *atqa, iec14443A_card_t* card )
{

	/* according to FSP ISO 11443-3 the b7&b8 bits of ATQA tag answer is UID size bit frame */
	/* Recovering the UID size */
	switch ((atqa->data[0] & ISO14443A_UID_MASK)>>6)
	{
			case ATQ_FLAG_UID_SINGLE_SIZE:
				card->uid_size 			= ISO14443A_UID_SINGLE_SIZE;
				card->cascade_level 	= CASCADE_LVL_1;
			break;
			case ATQ_FLAG_UID_DOUBLE_SIZE:
				card->uid_size 			= ISO14443A_UID_DOUBLE_SIZE;
				card->cascade_level 	= CASCADE_LVL_2;
			break;
			case ATQ_FLAG_UID_TRIPLE_SIZE:
				card->uid_size 			= ISO14443A_UID_TRIPLE_SIZE;
				card->cascade_level 	= CASCADE_LVL_3;
			break;
	}
	
}


static int st95hf_iec14443a_reqa(const struct device* dev, iec14443a_atqa_t* data){
    if (data!=NULL){
        return -EINVAL;
    }
    uint8_t req[2];

    req[0]=0x26;
    st95hf_sendrecv_iec14443a_footer_req_t *footer = (st95hf_sendrecv_iec14443a_footer_req_t *)&req[1];
    footer->byte=0x07; // = footer->fields.significant_bits=7;
    st95hf_rsp_t rsp={0};
    rsp.len = sizeof(iec14443a_atqa_t);
    int err = st95hf_send_receive_cmd(dev,sizeof(req),req,&rsp,data,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }

    return 0;
}


int st95hf_tag_hunting(const struct device* dev, uint8_t tags_to_find, uint8_t* tags_find){
    int err=0;
    if (tags_to_find & ST95HF_TRACK_NFCTYPE1){
        err= st95hf_field_off(dev);
        if (err!=0){
            return err;
        }
        k_sleep(K_MSEC(5));

        err = st95hf_iec14443a_init_anticolision(dev);
        if (err!=0){
            return err;
        }
        iec14443a_atqa_t atqa;
        err =st95hf_iec14443a_reqa(dev,&atqa); 
        if (err!=0){
            return err;
        }
        iec14443A_complete_structure(&atqa,NULL);

    }
    return 0;
}   
