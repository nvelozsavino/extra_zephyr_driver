#include "iso14443b.h"
#include "st95hf.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(st95hf_iso14443b, CONFIG_NFC_LOG_LEVEL);

static void init_card(iso14443b_card_t* card){
    if (card==NULL){
        return;
    }
    memset(&card->atqb , 0x00, sizeof(card->atqb));
	card->is_detected  = false;
	memset(card->log_msg, 0x00, sizeof(card->log_msg));
}

int st95hf_iso14443b_init(const struct device* dev, iso14443b_card_t* card){

    init_card(card);
    st95hf_data_t *st95hf = dev->data;
    st95hf_protocol_selection_req_t protocol_selection_req = { 
        .protocol=ST95HF_PROTOCOL_CODE_READER_ISO14443B,
        .parameters.reader_iso14443b = {
            .transmission_dr = 00, //106 Kbps
            .reception_dr = 00, //106 Kbps
            .append_crc=1,
            .PP = st95hf->ic_version<ST95HF_IC_VERSION_QJE?0x02:0x00,   // if  IcVers < QJE =  Set FDT otherwise set PP MM
            .MM = st95hf->ic_version<ST95HF_IC_VERSION_QJE?0x00:0x1A
        },
        .options = ST95HF_PROTSEL_READER_ISO14443B_OPT_NO_DD,
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
    st95hf->current_protocol = ST95HF_PROTOCOL_CARD_EMULATION_ISO14443B;
    st95hf_write_req_t write_req={0};

    write_req.reg_addr = ST95HF_WREG_ADDR_AAC_A_ARC_B;
    write_req.flags=ST95HF_WREG_FLAG_INCREMENT_AFTER_WRITE;
    write_req.params.acc_arc.index=ST95HF_WREG_ACC_ARC_INDEX_ARC_B;
    write_req.params.acc_arc.value.arc_b.byte=PCD_TYPEB_ARConfigB;
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -ENOMSG;
    }
    st95hf_select_reg_index_req_t sel_req = {
        .reg_addr=ST95HF_WREG_ADDR_AAC_A_ARC_B,
        .flags=ST95HF_WREG_FLAG_NOT_INCREMENT,
        .index=ST95HF_WREG_ACC_ARC_INDEX_ARC_B
    };
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_select_reg_index_cmd(dev,&sel_req,&rsp,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error selecting reg %02x[%02x]. %d",sel_req.reg_addr,sel_req.index,err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x selecting reg %02x",rsp.result_code, sel_req.index);
        return -ENOMSG;
    }

#ifdef CONFIG_ST95HF_READ_AS_DATASHEET
    st95hf_read_req_t read_req = {
        .reg_addr=ST95HF_WREG_ADDR_AAC_A_ARC_B,
        .reg_size=0x02,     
        .st_reserved= 0x01,        
    };

#else
    st95hf_read_req_t read_req = {
        .reg_addr=ST95HF_WREG_ADDR_AAC_A_ARC_B,
        .reg_count=0x02,
        .flags = ST95HF_RREG_FLAG_INCREMENT_AFTER_READ
    };
#endif

    memset(&rsp,0xFF,sizeof(rsp));
    uint8_t read_data[2];
    rsp.len=sizeof(read_data);
    err = st95hf_read_reg_cmd(dev,&read_req,&rsp,read_data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error reading reg %02x. %d",read_req.reg_addr,err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x reading reg %02x",rsp.result_code, read_req.reg_addr);
        return -ENOMSG;
    }
    if (read_data[0]!=PCD_TYPEB_ARConfigA || read_data[1] != PCD_TYPEB_ARConfigB){
        LOG_ERR("Error register don't match ARCA %02x!=%02x || ARCB %02x!=%02x",
            read_data[0], PCD_TYPEB_ARConfigA,
            read_data[1], PCD_TYPEB_ARConfigB
        );
        return -EBADE;
    }
    /* GT min time to respect before sending REQ_B */
    k_sleep(K_USEC(5100));
    return 0;
}

static int st95hf_iso14443b_reqb(const struct device* dev, iso14443b_atqb_t* atqb){
    if (atqb!=NULL){
        return -EINVAL;
    }			
    uint8_t req_data[] = {  ISO14443B_ANTICOLLISION_PREFIX_BYTE,   // APf
                            ISO14443B_AFI_ALL_FAMILIES, // AFI                        
                            ISO14443B_EXTENDED_ATQB_NOT_SUPPORTED | ISO14443B_REQB_ATTEMPT | ISO14443B_SLOT_MARKER_1  /* Parameters */
    };
    
    st95hf_rsp_t rsp={0};
    rsp.len = sizeof(iso14443b_atqb_t);
    int err = st95hf_send_receive_cmd(dev,sizeof(req_data),req_data,&rsp,atqb,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }
    return 0;
}


int st95hf_iso14443b_is_present(const struct device* dev,iso14443b_card_t* card){
    if (card==NULL){
        return -EINVAL;
    }

    int err = st95hf_iso14443b_reqb(dev,&card->atqb);
    if (err!=0){
        return err;
    }
    
    card->is_detected=true;

    return 0;
}


static int iso14443b_attrib(const struct device* dev, iso14443b_card_t* card){
    uint8_t attrib[] = { 
                            /* Start byte */ 
                            ATTRIB_FIRST_BYTE           ,
                            /* PUPI */ 
                            0x00                        ,  
                            /* PUPI */  
                            0x00                        ,
                            /* PUPI */   
                            0x00                        ,
                            /* PUPI */   
                            0x00                        ,
                            /* Parameter 1 */   
                            TR0_64_FS | TR1_64_FS | EOF_REQUIRED | SOF_REQUIRED,
                            /* Parameter 2 */                          
                            MAX_FRAME_SIZE_256_BYTES | PCD_TO_PICC_106K | PICC_TO_PCD_106K,
                            /* Parameter 3 */              
                            TR2_32_FS | PICC_COMPLIANT_ISO14443_4,
                            /* Parameter 4 */                          
                            CID_0	                     
    };

    memcpy(&attrib[1], card->atqb.fields.pupi, sizeof(card->atqb.fields.pupi));
    uint8_t attrib_rsp[1+sizeof(st95hf_sendrecv_iso14443b_footer_rsp_t)+32];
    st95hf_rsp_t rsp={0};
    rsp.len = sizeof(attrib_rsp);
    int err = st95hf_send_receive_cmd(dev,sizeof(attrib),attrib,&rsp,attrib_rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error %02x ATTRIB result code",rsp.result_code);
        return -EIO;
    }
    st95hf_sendrecv_iso14443b_footer_rsp_t* footer = (st95hf_sendrecv_iso14443b_footer_rsp_t*) &attrib_rsp[rsp.len-sizeof(st95hf_sendrecv_iso14443b_footer_rsp_t)];
    if (footer->fields.crc_error){
        LOG_ERR("Error CRC in ATTRIB rsp");
        return -EBADE;
    }
    return 0;

}


int st95hf_iso14443b_anticollision(const struct device* dev, iso14443b_card_t* card){
    st95hf_data_t *st95hf = dev->data;
    int err = iso14443b_attrib(dev,card);
    if (err!=0){
        return err;
    }

   	/* Change the PP:MM parameter to accept longer APDU timeout (2^PP)*(MM+1)*(DD+128)*32/13.56 ~= 304ms*/
    st95hf_protocol_selection_req_t protocol_selection_req = { 
        .protocol=ST95HF_PROTOCOL_CODE_READER_ISO14443B,
        .parameters.reader_iso14443b = {
            .transmission_dr = 00, //106 Kbps
            .reception_dr = 00, //106 Kbps
            .append_crc=1,
            .PP = 0x04,  
            .MM = 0x3E
        },
        .options = ST95HF_PROTSEL_READER_ISO14443B_OPT_NO_DD,
    };
    st95hf_rsp_t rsp={0};
    err =  st95hf_protocol_select_cmd(dev,&protocol_selection_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x selecting protocol",rsp.result_code);
        return -EIO;
    }
    st95hf->device_mode=ST95HF_DEVICE_MODE_PCD;
    st95hf->tag_type=ST95HF_TAG_TYPE_TT4B;

    return 0;
}