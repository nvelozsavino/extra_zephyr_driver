#include "iso14443a.h"
#include "st95hf.h"
#include <errno.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(st95hf, LOG_LEVEL_DBG);

static void iso14443a_complete_structure (iso14443a_card_t* card );
static int st95hf_iso14443a_reqa(const struct device* dev, iso14443a_atqa_t* data);
static int st95hf_iso14443a_emit_rats(const struct device* dev, iso14443a_rats_t* rats);
static uint16_t fsci_to_fsc(uint8_t fsci);
static int st95hf_iso14443a_config_fdt(const struct device* dev, uint8_t pp, uint8_t mm, uint8_t dd);
static int st95hf_iso14443a_ac_level(const struct device* dev, uint8_t level, iso14443a_card_t* card);
static int st95hf_iso14443a_ac(const struct device* dev, uint8_t cascade_level, iso14443a_anticollision_t* data);


static void init_card(iso14443a_card_t* card){

    memset(&card->atqa,0x00,sizeof(card->atqa));
    memset(&card->uid,0x00,sizeof(card->uid));
    card->cascade_level=0;
    card->uid_size=0;
    card->ats_supported=false;
    card->is_detected=false;
}

int st95hf_iso14443a_init(const struct device* dev,iso14443a_card_t* card){
    if (card==NULL){
        return -EINVAL;
    }
    
    init_card(card);        
    st95hf_data_t *st95hf = dev->data;
    st95hf_protocol_selection_req_t protocol_selection_req = { 
        .protocol=ST95HF_PROTOCOL_CODE_READER_ISO14443A,
        .parameters.reader_iso14443a = {
            .reception_dr=0,    //106 Kbps
            .transmission_dr=0, //106 Kbps
            .DD=0,  //The default PP:MM:DD value is 0 (corresponds to FDT 86/90µs)
            .PP=0,
            .MM=0,
            .st_reserved={0x02,0x02}, /* last 2 bytes since QJE version */
        },
        .options = ST95HF_PROTSEL_READER_ISO14443A_OPT_ALL,
        
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
    write_req.reg_addr = ST95HF_WREG_ADDR_TIMER_WINDOW;
    write_req.flags=ST95HF_WREG_FLAG_NOT_INCREMENT;
    write_req.params.timer_w.value=PCD_TYPEA_TIMERW;
    write_req.params.timer_w.confirmation=ST95HF_WREG_TIMERW_CONFIRMATION;
    memset(&rsp,0xFF,sizeof(rsp));
    err = st95hf_write_reg_cmd(dev,&write_req,&rsp,K_SECONDS(3));
    if (err!=0){
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
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
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }
    st95hf->current_protocol = ST95HF_PROTOCOL_READER_ISO14443A;
    k_sleep(K_USEC(5100));
    return 0;
}


static void iso14443a_complete_structure (iso14443a_card_t* card ){

	/* according to FSP ISO 11443-3 the b7&b8 bits of ATQA tag answer is UID size bit frame */
	/* Recovering the UID size */
	switch ((card->atqa.data[0] & ISO14443A_UID_MASK)>>6)
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

int st95hf_iso14443a_is_present(const struct device* dev,iso14443a_card_t* card){
    if (card==NULL){
        return -EINVAL;
    }
    
    int err =st95hf_iso14443a_reqa(dev,&card->atqa); 
    if (err!=0){
        LOG_ERR("Error REQA %d",err);
        return err;
    }
    card->is_detected=true;
    iso14443a_complete_structure(card);
    return 0;
}



int st95hf_iso14443a_check_type1(const struct device* dev){
    st95hf_data_t* st95hf = dev->data;
    const uint8_t data[] = {0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA8};
    uint8_t buffer[20]={0};
    st95hf_rsp_t rsp;
    rsp.len=sizeof(buffer);
    
    int err = st95hf_send_receive_cmd(dev,sizeof(data),data,&rsp,buffer,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error send/recv %d",err);
        return err;
    }

    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error rsp code %02x", rsp.result_code);
        return -ENOMSG;
    }

    if (rsp.len<sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)){
        LOG_ERR("Error rsp len less than footer");
        return -ENODATA;
    }
    st95hf_sendrecv_iso14443a_footer_rsp_t* footer = (st95hf_sendrecv_iso14443a_footer_rsp_t*)&buffer[rsp.len-sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)];
    if (footer->fields.crc_error){
        LOG_ERR("CRC Error");
        return -EBADF;
    }

    if(rsp.len==4){
        LOG_ERR("Len %d!=4", rsp.len);
        return -ENOMSG;
    } else {
        st95hf->device_mode = ST95HF_DEVICE_MODE_PCD;
        st95hf->tag_type=ST95HF_TAG_TYPE_TT1;
        return 0;
    }
        

}





static int st95hf_iso14443a_reqa(const struct device* dev, iso14443a_atqa_t* data){
    if (data==NULL){
        return -EINVAL;
    }
    uint8_t req[2];

    req[0]=0x26;
    st95hf_sendrecv_iso14443a_footer_req_t *footer = (st95hf_sendrecv_iso14443a_footer_req_t *)&req[1];
    footer->byte=0x07; // = footer->fields.significant_bits=7;
    st95hf_rsp_t rsp={0};
    uint8_t rsp_data[sizeof(iso14443a_atqa_t)+sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)];
    rsp.len = sizeof(rsp_data);
    int err = st95hf_send_receive_cmd(dev,sizeof(req),req,&rsp,rsp_data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error REQA %d",err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }
    LOG_HEXDUMP_DBG(rsp_data,rsp.len,"REQA:");
    memcpy(data,rsp_data,sizeof(iso14443a_atqa_t));
    return 0;
}

static int st95hf_iso14443a_ac(const struct device* dev, uint8_t cascade_level, iso14443a_anticollision_t* data){

    uint8_t anticol_parameter [7] = {cascade_level, ISO14443A_NVM_20, 0x08,0x00,0x00,0x00,0x00};
    st95hf_sendrecv_iso14443a_footer_req_t* footer_req = (st95hf_sendrecv_iso14443a_footer_req_t*)&anticol_parameter[2];
    footer_req->fields.significant_bits=8;
    uint8_t uid[4] = {0x00,0x00,0x00,0x00};
    uint8_t rsp_data[sizeof(iso14443a_anticollision_t)+sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)]; // 4 UID + 1 BCC + 3 footer
    int err;
    uint8_t byte_collision_index=0;
    uint8_t bit_collision_index=0;
    uint8_t new_byte_collision_index=0;
    uint8_t new_bit_collision_index=0;
    st95hf_rsp_t rsp;
    rsp.len=sizeof(rsp_data);
    err = st95hf_send_receive_cmd(dev,3,anticol_parameter,&rsp,rsp_data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error send/recv anticollision %d",err);
        return err;
    }

    if (rsp.len<sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)){
        LOG_ERR("Error rsp len %d <footer %d",rsp.len,sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t));
        return -ENODATA;
    }
    st95hf_sendrecv_iso14443a_footer_rsp_t* footer_rsp = (st95hf_sendrecv_iso14443a_footer_rsp_t*)&rsp_data[rsp.len-sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)];

    bool collision = footer_rsp->fields.collision_detect!=0;
    LOG_INF("Collision: %d",collision);
    byte_collision_index = footer_rsp->fields.byte_collision_index;
    bit_collision_index = footer_rsp->fields.bit_collision_index;
	/* case that should not happend, as occurs because we have miss another collision */
	if( byte_collision_index == 8){
        LOG_ERR("Error byte_collision_index=8");
		return -EBADE;
	}

    if (data!=NULL){
        memcpy(data,rsp_data,sizeof(iso14443a_anticollision_t));
    }

    while (collision){
        collision=false;
        anticol_parameter[1] = ISO14443A_NVM_20 + ((byte_collision_index) <<4) + (bit_collision_index+1);

        /**
         * According to the datasheet example the corrected uid should be 
         * anticol_parameter[2+byte_collision_index] = rsp_data[byte_collision_index] & (uint8_t)(~(0xFF<<bit_collision_index)))
        */
        if( byte_collision_index == 0){
            
            anticol_parameter[2] = rsp_data[0] & ((uint8_t)(~(0xFF<<(bit_collision_index+1)))); /* ISO said it's better to put collision bit to value 1 */
			anticol_parameter[3] = (bit_collision_index+1) | 0x40; /* add split frame bit */
			uid [0] = anticol_parameter[2];
        } else if( byte_collision_index == 1) {
			anticol_parameter[2] = rsp_data[0];
			anticol_parameter[3] = rsp_data[1] & ((uint8_t)(~(0xFF<<(bit_collision_index+1)))); /* ISO said it's better to put collision bit to value 1 */			
			anticol_parameter[4] = (bit_collision_index+1) | 0x40; /* add split frame bit */
			uid [0] = anticol_parameter[2];
			uid [1] = anticol_parameter[3];
		} else if( byte_collision_index == 2) {
			anticol_parameter[2] = rsp_data[0];
			anticol_parameter[3] = rsp_data[1];
			anticol_parameter[4] = rsp_data[2] & ((uint8_t)(~(0xFF<<(bit_collision_index+1)))); /* ISO said it's better to put collision bit to value 1 */			
			anticol_parameter[5] = (bit_collision_index+1) | 0x40; /* add split frame bit */;
			uid [0] = anticol_parameter[2];
			uid [1] = anticol_parameter[3];
			uid [2] = anticol_parameter[4];
		} else if( byte_collision_index == 3) {
			anticol_parameter[2] = rsp_data[0];
			anticol_parameter[3] = rsp_data[1];
			anticol_parameter[4] = rsp_data[2];
			anticol_parameter[5] = rsp_data[3] & ((uint8_t)(~(0xFF<<(bit_collision_index+1)))); /* ISO said it's better to put collision bit to value 1 */			
			anticol_parameter[6] = (bit_collision_index+1) | 0x40; /* add split frame bit */;
			uid [0] = anticol_parameter[2];
			uid [1] = anticol_parameter[3];
			uid [2] = anticol_parameter[4];
			uid [3] = anticol_parameter[5];
		} else {
            LOG_ERR("Error Byte collision index out of bonds %d>3",byte_collision_index);
			return -ERANGE;
        }
        err = st95hf_send_receive_cmd(dev,3+byte_collision_index,anticol_parameter,&rsp,rsp_data,K_SECONDS(3));
        if (err!=0){
            LOG_ERR("Error send/recv anticollision %d",err);
            return err;
        }
        if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
            LOG_ERR("Error result_code unexpected %02x", rsp.result_code);
            return -ENOMSG;
        }
        footer_rsp = (st95hf_sendrecv_iso14443a_footer_rsp_t*)&rsp_data[rsp.len-sizeof(st95hf_sendrecv_iso14443a_footer_rsp_t)];

        /* check if there is another collision to take into account*/
        collision = footer_rsp->fields.collision_detect!=0;
        if (collision){
            new_byte_collision_index = footer_rsp->fields.byte_collision_index;
            new_bit_collision_index = footer_rsp->fields.bit_collision_index;            
        }

        /* we can check that non-alignement is the one expected */
        uint8_t remaining_bit = 8 - footer_rsp->fields.significant_bits;

        if (remaining_bit == bit_collision_index+1){
            /* recreate the good UID */

            /**
         * According to the datasheet example the corrected uid should be 
         * uid[byte_collision_index] = ((~(0xFF << bit_collision_index)) & anticol_parameter[byte_collison_index+2]) | rsp_data[0] ;
        */
			if( byte_collision_index == 0) {
				uid [0] = ((~(0xFF << (bit_collision_index+1))) & anticol_parameter[2]) | rsp_data[0] ;
				uid [1] = rsp_data[1];
				uid [2] = rsp_data[2];
				uid [3] = rsp_data[3];
			} else if( byte_collision_index == 1) {
				uid [1] = ((~(0xFF << (bit_collision_index+1))) & anticol_parameter[3]) | rsp_data[0] ;
				uid [2] = rsp_data[1];
				uid [3] = rsp_data[2];
			} else if( byte_collision_index == 2) {
				uid [2] = ((~(0xFF << (bit_collision_index+1))) & anticol_parameter[4]) | rsp_data[0] ;
				uid [3] = rsp_data[1];
			} else if( byte_collision_index == 3) {
				uid [3] = ((~(0xFF << (bit_collision_index+1))) & anticol_parameter[5]) | rsp_data[0] ;
			} else {
				LOG_ERR("Error Byte collision index out of bonds %d>3",byte_collision_index);
			    return -ERANGE;
            }
        } else {
            LOG_ERR("Error Remaining bits doesn't match with colision index %d!=%d+1",remaining_bit,bit_collision_index);
            return -ERANGE;
        }

        if (data!=NULL){
            memcpy(data->uid,uid,sizeof(data->uid));
            data->bcc = uid[0]^uid[1]^uid[2]^uid[3];            
        }

        /* if collision was detected restart anticol */
        if (collision){
            if (byte_collision_index!= new_byte_collision_index){                
                bit_collision_index=new_bit_collision_index;
            } else {
                bit_collision_index += (new_bit_collision_index+1);				
            }
            byte_collision_index+=new_byte_collision_index;
        }
    }


    return 0;

}

static int st95hf_iso14443a_ac_level(const struct device* dev, uint8_t level, iso14443a_card_t* card){
    if (dev==NULL || card==NULL){
        return -EINVAL;
    }
    
    uint8_t cascade_level;
    uint8_t uid_size;
    uint8_t uid_start;
    uint8_t part_start;
    uint8_t part_size;
    bool only_part;
    switch (level){
        case 1:
            cascade_level = SEL_CASCADE_LVL_1;
            uid_size = ISO14443A_UID_SINGLE_SIZE;
            uid_start=0;
            only_part=false;
            part_start=1;
            part_size = ISO14443A_UID_PART;
            break;
        case 2:
            cascade_level = SEL_CASCADE_LVL_2;
            uid_size = ISO14443A_UID_DOUBLE_SIZE;
            uid_start=ISO14443A_UID_PART;
            only_part=false;
            part_start=1;
            part_size = ISO14443A_UID_PART;
            break;
        case 3:
            cascade_level = SEL_CASCADE_LVL_3;
            uid_size = ISO14443A_UID_TRIPLE_SIZE;
            uid_start=ISO14443A_UID_PART;
            only_part=true;
            part_start=0;
            part_size = ISO14443A_UID_SINGLE_SIZE;
            break;
        default:
        return -EINVAL;
    }

    int err;
    iso14443a_anticollision_t ac_data;
    uint8_t send_data[10];
    err = st95hf_iso14443a_ac(dev,cascade_level,&ac_data);
    if (err!=0){
        LOG_ERR("Error AC cascade_level %02x. Err: %d",cascade_level, err);
        return err;
    }
    if (!only_part && card->uid_size== uid_size){
        memcpy(&card->uid[uid_start],ac_data.uid,ISO14443A_UID_SINGLE_SIZE);
    } else {
        memcpy(&card->uid[uid_start],&ac_data.uid[part_start],part_size);
    }
    uint8_t bcc = ac_data.bcc;
    LOG_DBG("BCC = %02x", bcc);
    uint8_t len =0;
    send_data[len++]=cascade_level;
    send_data[len++]=ISO14443A_NVM_70;
    LOG_INF("UID Size: %d== Expected %d, only_part %d", card->uid_size, uid_size, only_part);
    if (card->uid_size== uid_size){
        memcpy(&send_data[len],&card->uid[uid_start],ISO14443A_UID_SINGLE_SIZE);
        len+=ISO14443A_UID_SINGLE_SIZE;
    } else if (!only_part) {
        send_data[len++] = 0x88;
        memcpy(&send_data[len],&card->uid[uid_start],ISO14443A_UID_PART);
        len+=ISO14443A_UID_PART;
    }
    send_data[len++]=bcc;
    send_data[len]=0;
    st95hf_sendrecv_iso14443a_footer_req_t* footer = (st95hf_sendrecv_iso14443a_footer_req_t*) &send_data[len];
    footer->fields.append_crc=1;
    footer->fields.significant_bits=8;
    len++;

    st95hf_rsp_t rsp;
    uint8_t rcv_data[20];
    rsp.len= sizeof(rcv_data);
    err = st95hf_send_receive_cmd(dev,len,send_data,&rsp,rcv_data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error send/rcv %d",err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error result code when sending ac level %d %02x",level,rsp.result_code);
        return -ENOMSG;
    }
    card->sak= rcv_data[0];
    LOG_INF("SAK: %02x",card->sak);
    return 0;
}

static int st95hf_iso14443a_config_fdt(const struct device* dev, uint8_t pp, uint8_t mm, uint8_t dd){


    st95hf_protocol_selection_req_t req = {
        .protocol = ST95HF_PROTOCOL_CODE_READER_ISO14443A,
        .parameters.reader_iso14443a = {
            .reception_dr=0,
            .transmission_dr=0,
            .PP = pp,
            .MM = mm,
            .DD= dd,
            .st_reserved={0x03,0x03}
        }
    };
    st95hf_rsp_t rsp;
    int err = st95hf_protocol_select_cmd(dev,&req,&rsp,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error selecting protocol FDT for RATS, %d",err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error result code protocol select %02x", rsp.result_code);
        return -ENOMSG;
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
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
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
    if (rsp.result_code!=ST95HF_STATUS_CODE_SUCCESS){
        LOG_ERR("Error %02x writing reg ARC B",rsp.result_code);
        return -EIO;
    }
    return 0;
}

static uint16_t fsci_to_fsc(uint8_t fsci){
	if (fsci == 0) return 16;
	else if (fsci == 1) return 24;
	else if (fsci == 2) return 32;
	else if (fsci == 3) return 40;
	else if (fsci == 4) return 48;
	else if (fsci == 5) return 64;
	else if (fsci == 6) return 96;
	else if (fsci == 7) return 128;
	else return 256;
}

static int st95hf_iso14443a_emit_rats(const struct device* dev, iso14443a_rats_t* rats){
    if (dev==NULL || rats==NULL){
        return -EINVAL;
    }
    uint8_t send_data[2+sizeof(st95hf_sendrecv_iso14443a_footer_req_t)] = {0xE0,0x80,0x28};
    // st95hf_sendrecv_iso14443a_footer_req_t* footer = (st95hf_sendrecv_iso14443a_footer_req_t*)&send_data[2];
    // footer->fields.append_crc=1;
    // footer->fields.significant_bits=8;
    uint8_t rsp_data[255]={0};
    st95hf_rsp_t rsp={
        .len = sizeof(rsp_data),
    };
    
    int err = st95hf_send_receive_cmd(dev,sizeof(send_data),send_data,&rsp,rsp_data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error emiting RATS %d",err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error emiting RATS, result code %02x",rsp.result_code);
        return -ENOMSG;
    }

    
    
    uint8_t fsci = rsp_data[1] & 0x0F;
    rats->fsc = fsci_to_fsc(fsci);
    /* Check if FWI is present */
    if ((rsp_data[1] & 0x20) == 0x20){
        if ((rsp_data[1]&0x10)==0x10) {
            rats->fwi=(rsp_data[3]&0xF0)>>4;            
        } else {
            rats->fwi=(rsp_data[2]&0xF0)>>4;
        }
    }
    

    return 0;
}




int st95hf_iso14443a_anticollision(const struct device* dev, iso14443a_card_t* card){

    st95hf_data_t *st95hf = dev->data;
    if (dev==NULL || card==NULL){
        return -EINVAL;
    }
    int err;
    uint8_t level=0;
    bool not_complete=true;
    do {
        LOG_INF("Trying AC Level %d",level+1);
        err = st95hf_iso14443a_ac_level(dev,++level,card);   
       	/* UID Complete ? */     
        not_complete = (card->sak & SAK_FLAG_UID_NOT_COMPLETE) == SAK_FLAG_UID_NOT_COMPLETE;
        LOG_DBG("AC Level %d, Err: %d, SAK: %02x",level, err, card->sak);
    } while (err==0 && level<3 && not_complete);

    if (err!=0){
        LOG_ERR("Error executing anticollision level %d. %d",level,err);
        return err;
    }
    LOG_INF("Anticollision done Level %d, not_complete %d",level,not_complete);


    card->rats.fsc =32;
    card->rats.fwi=4;
	/* Checks if the RATS command is supported by the card */
    if (card->sak & SAK_FLAG_ATS_SUPPORTED){
    LOG_INF("ATS Supported");


    /*  Change the PP:MM parameter to respect RATS timing TS-DP-1.1 13.8.1.1
	*   min to respect 
	*   FDT PCD = FWTt4at,activation = 71680 (1/fc) 
	*   (2^PP)*(MM+1)*(DD+128)*32 = 71680 ==> PP = 4 MM=0 DD=12
	*   max to respect not mandatory and as Tag has a FWT activation of 5.2us  
	*   adding 16.4ms does not make sense ... 
	*   FDT PCD = FWTt4at,activation + dela(t4at,poll) = 5286us + 16.4ms ~= 21.7ms 
	*   (2^PP)*(MM+1)*(DD+128)*32 = 21,7 ==> PP = 4 MM=0 DD=12
    */
        LOG_INF("Configuring FDT");
        err = st95hf_iso14443a_config_fdt(dev,4,0,12);
        if (err!=0){
            LOG_ERR("Error configuring FDT for RATS. %d",err);
            return err;
        }

        card->ats_supported=true;
        LOG_INF("Emmiting RATS");
        err = st95hf_iso14443a_emit_rats(dev,&card->rats);
        if (err!=0){
            LOG_ERR("Error emiting RATS. %d",err);
            return err;
        }
        LOG_INF("RATS done, fwi: %d",card->rats.fwi);
    }
    if (card->rats.fwi<4){
        card->rats.fwi=4;
    }
    uint8_t wtxm =1;
    	
	/*  FWI was updated thanks to ATS, if not the case use default value FWI = 4 TS-DP-1.1 13.6.2.11 
	    FDT PCD = FWT PICC + deltaFWT(t4at) + "deltaT(t4at,poll)" TS-DP-1.1 13.8
	    If we perform some identification:
			FDT = (2^PP)*(MM+1)*(DD+128)*32/13.56 
			FDT = (2^(PP))*(1)*(2*DD+256)*16/13.56 + (2^(PP))*(MM)*(2*DD+256)*16/13.56
			FDT = (256*16/fc)*2^FWI + ((2^FWI) *256*16*1/fc)*MM with PP=FWI and DD=0
			FDT = (2^FWI)*4096*1/fc + FWT*MM (EQUATION 1)

			I_ With the choice to NOT add deltaT(t4at,poll) = 16,4ms
			1)	In the standard case (No extension time cmd received) we want
				FDT = FWT + delta FWT(T4AT) 
				FDT = FWT + 49152 (1/fc)

				If we take the rules that we will never set FWI to a value less than 4.
				(EQUATION 1 comes)
				FDT = FWT*MM + 65536*1/fc => delta FWT(T4AT) is respected

				As a conclusion with
				PP=FWI (with FWI>=4)
				MM=1
				DD=0
			we are following the specification. 
			
			2) In the case of extension time request, M will take the WTXM value.			
    */
    LOG_INF("Reconfiguring FDT");

    err = st95hf_iso14443a_config_fdt(dev,card->rats.fwi,wtxm,0);
    if (err!=0){
        LOG_ERR("Error configuring FDT for RATS. %d",err);
        return err;
    }
    
    st95hf->device_mode=ST95HF_DEVICE_MODE_PCD;
    if ((card->sak& 0x60)==0x00){
        
        st95hf->tag_type=ST95HF_TAG_TYPE_TT2;
        return 0;
    } else if ((card->sak& 0x20)!=0x00){        
        st95hf->tag_type=ST95HF_TAG_TYPE_TT4A;
        return 0;
    }

    return 0;
}


int st95hf_iso14443a_read_ndef(const struct device* dev, iso14443a_card_t* card, uint8_t* buffer, size_t buffer_size){
    
    return 0;
}