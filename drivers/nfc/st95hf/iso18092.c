#include "iso18092.h"
#include "st95hf.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(st95hf_iso18092, CONFIG_NFC_LOG_LEVEL);

int st95hf_iso18092_init(const struct device* dev){
    st95hf_data_t *st95hf = dev->data;
    st95hf_protocol_selection_req_t protocol_selection_req = { 
        .protocol=ST95HF_PROTOCOL_CODE_READER_ISO18092,
        .parameters.reader_iso18092 = {
            .append_crc=1,
            .reception_dr=1, // 212 Kbps
            .transmission_dr=1, // 212 Kbps
            .slot_counter=3,    // 4 slots
            .fwt=1,     //FWT is specified by PP:MM bits
            .PP=0x01,
            .MM=0x0D,
        }
    } ;

    st95hf_rsp_t rsp={0};
    int err =  st95hf_protocol_select_cmd(dev,&protocol_selection_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x selecting protocol",rsp.result_code);
        return -EIO;
    }
    st95hf_write_req_t write_req={0};

    write_req.reg_addr = ST95HF_WREG_ADDR_AAC_A_ARC_B;
    write_req.flags=ST95HF_WREG_FLAG_INCREMENT_AFTER_WRITE;
    write_req.params.acc_arc.index=ST95HF_WREG_ACC_ARC_INDEX_ARC_B;
    write_req.params.acc_arc.value.arc_b.byte=PCD_TYPEF_ARConfigB;
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }



    write_req.reg_addr = ST95HF_WREG_ADDR_AUTO_DETECT;
    write_req.flags=ST95HF_WREG_FLAG_INCREMENT_AFTER_WRITE;
    write_req.params.auto_detect.enable=ST95HF_WREG_AUTO_DETECT_ENABLE;
    write_req.params.auto_detect.confirmation=ST95HF_WREG_AUTO_DETECT_CONFIRMATION;    
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x writing reg TimerW",rsp.result_code);
        return -EIO;
    }

    st95hf->current_protocol = ST95HF_PROTOCOL_READER_ISO18092;

    	/* GT min time to respect before sending REQ_C */
    k_sleep(K_USEC(20400));

    return 0;
}


static int st95hf_iso18092_reqc(const struct device* dev, iso18092_atqc_t* atqc){
    if (atqc!=NULL){
        return -EINVAL;
    }

    uint8_t req_data[] = { 0x12, 0xFC, 0x01, 0x03};
    
    st95hf_rsp_t rsp={0};
    rsp.len = sizeof(iso18092_atqc_t);
    int err = st95hf_send_receive_cmd(dev,sizeof(req_data),req_data,&rsp,atqc,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }
    if (atqc->data[0]!=0x01){
        LOG_ERR("Error Unexpected answer to REQC %02x!=0x01",atqc->data[0]);
        return -ENOMSG;
    }

    return 0;
}

int st95hf_iso18092_is_present(const struct device* dev,iso18092_card_t* card){
    iso18092_atqc_t atqc;
    int err = st95hf_iso18092_reqc(dev,&atqc);
    if (err!=0){
        return err;
    }
    if (card!=NULL){
        card->is_detected=true;
        memcpy(&card->atqc,&atqc,sizeof(iso18092_atqc_t));
        memcpy(card->uid,&card->atqc.data[1],sizeof(card->uid));
    }
    return 0;
}

