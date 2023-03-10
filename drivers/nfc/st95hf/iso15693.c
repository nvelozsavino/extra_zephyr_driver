#include "iso15693.h"
#include "st95hf.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(st95hf, LOG_LEVEL_DBG);

static void init_card(iso15693_card_t* card){
    // memset(&card->atqb , 0x00, sizeof(card->atqb));
	card->is_detected  = false;
	// memset(card->log_msg, 0x00, sizeof(card->log_msg));
}


int st95hf_iso15693_init(const struct device* dev, iso15693_card_t* card){
    if (card==NULL){
        return -EINVAL;
    }

  st95hf_data_t *st95hf = dev->data;
    st95hf_protocol_selection_req_t protocol_selection_req = { 
        .protocol=ST95HF_PROTOCOL_CODE_READER_ISO15693,
        .parameters.reader_iso15693 = {
            .append_crc = 1,
            .subcarrier=0,      //Single subcarrier (S)
            .modulation = 0,    //100% modulation (100)
            .delay = 1,         //Wait for SO
            .bitrate = 00,      //26 Kbps (H)
        },
        .options = ST95HF_PROTSEL_READER_ISO15693_OPT_ALL,
    };
    st95hf_rsp_t rsp={0};
    int err =  st95hf_protocol_select_cmd(dev,&protocol_selection_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x selecting protocol",rsp.result_code);
        return -EIO;
    }
    st95hf->current_protocol = ST95HF_PROTOCOL_CARD_EMULATION_ISO15693;
    st95hf_write_req_t write_req={0};

    write_req.reg_addr = ST95HF_WREG_ADDR_AAC_A_ARC_B;
    write_req.flags=ST95HF_WREG_FLAG_INCREMENT_AFTER_WRITE;
    write_req.params.acc_arc.index=ST95HF_WREG_ACC_ARC_INDEX_ARC_B;
    write_req.params.acc_arc.value.arc_b.byte=PCD_TYPEV_ARConfigB;
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -ENOMSG;
    }
 
    /* Min time to respect before sending Inventory */
    k_sleep(K_MSEC(5));

    return 0;
}



int st95hf_iso15693_is_present(const struct device* dev,iso15693_card_t* card){
    if (card==NULL){
        return -EINVAL;
    }

    int err = st95hf_iso15693_inventory(dev,&card->atqb);
    if (err!=0){
        return err;
    }
    
    card->is_detected=true;

    return 0;
}