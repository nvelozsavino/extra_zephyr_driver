#include "iso7816.h"
#include "st95hf.h"
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(st95hf, LOG_LEVEL_DBG);


#pragma pack(push,1)
typedef struct {
    uint8_t cla;    // Command class
    uint8_t ins;    // Instruction (Operation code)
    uint8_t p1;     // Selection mode
    uint8_t p2;     // Selection option
} apdu_header_t;

typedef struct {
    uint8_t lc;                 // Data field length
    uint8_t data[256];          // Data field
    uint8_t le;                 // Expected length
} apdu_body_t;


typedef struct{
    apdu_header_t header;
    apdu_body_t body;
} apdu_cmd_t;

typedef struct{
    uint8_t sw1;    // Command Processing status
    uint8_t sw2;    // Command Processing qualification
} apdu_rsp_t;

#pragma pack(pop)

static int st95hf_iso7816_send_receive_adpu(const struct device* dev, const apdu_cmd_t* cmd, uint16_t* rsp_length, uint8_t* data){
    st95hf_data_t *st95hf = dev->data;
    if (st95hf->tag_type!=ST95HF_TAG_TYPE_TT4B && st95hf->tag_type!=ST95HF_TAG_TYPE_TT4A){
        LOG_ERR("Invalid tag type %d",st95hf->tag_type);
        return -EPROTO;
    }
    uint8_t send_buffer[1+sizeof(apdu_cmd_t)+ sizeof(st95hf_sendrecv_iso14443a_footer_req_t)];
    uint16_t send_len = 0;
    send_buffer[send_len++] = st95hf->block_number;
    
    st95hf->block_number ^=0x01;
    memcpy(&send_buffer[send_len],&cmd->header,sizeof(apdu_header_t));
    send_len+=sizeof(apdu_header_t);

    if (cmd->body.lc>0){
        send_buffer[send_len++] = cmd->body.lc;
        memcpy(&send_buffer[send_len],cmd->body.data,cmd->body.lc);
        send_len+=cmd->body.lc;
    }
    if (cmd->body.le>0 || (cmd->header.p1 == 0x04 && cmd->header.p2==0x00)){
        send_buffer[send_len++] = cmd->body.le;
    }

     if (st95hf->tag_type==ST95HF_TAG_TYPE_TT4A){
        send_buffer[send_len]=0;
        st95hf_sendrecv_iso14443a_footer_req_t* footer =  (st95hf_sendrecv_iso14443a_footer_req_t*)&send_buffer[send_len];
        footer->fields.append_crc=1;
        footer->fields.significant_bits=8;
        send_len++;
     }
    uint8_t rsp_data[256];
    st95hf_rsp_t rsp={0};
    rsp.len = sizeof(rsp_data);
    int err = st95hf_send_receive_cmd(dev,send_len,send_buffer,&rsp,rsp_data,K_SECONDS(3));
    if (err!=0){
        LOG_ERR("Error ADPU cmd %d",err);
        return err;
    }
    if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
        LOG_ERR("Error %02x sending adpu cmd",rsp.result_code);
        return -EIO;
    }
    /* Test if a time extension is required */
	if (rsp_data[0] == 0xF2)
	{				
		/* Modify temporarly FDT */
        if (st95hf->fwi<4){
            st95hf->fwi=4;
        }
        err = st95hf_iso14443a_config_fdt(dev, st95hf->fwi, rsp_data[1], 0);
        if (err!=0){
            LOG_ERR("Configuring FDT failed %d",err);
            //return err;
        }
        uint8_t buffer_fwi[3];
        uint8_t fwi_len=2;
        memcpy(buffer_fwi,rsp_data,2);
        if (st95hf->tag_type==ST95HF_TAG_TYPE_TT4A){
            buffer_fwi[2]=0;
            st95hf_sendrecv_iso14443a_footer_req_t* footer =  (st95hf_sendrecv_iso14443a_footer_req_t*)&buffer_fwi[2];
            footer->fields.append_crc=1;
            footer->fields.significant_bits=8;
            fwi_len=3;
        }
        err = st95hf_send_receive_cmd(dev,send_len,send_buffer,&rsp,rsp_data,K_SECONDS(3));
        if (err!=0){
            LOG_ERR("Error FWI ADPU cmd %d",err);
            // return err;
        }
        if (rsp.result_code!=ST95HF_STATUS_CODE_FRAME_RECV_OK){
            LOG_ERR("Error %02x sending FWI adpu cmd",rsp.result_code);
            // return -EIO;
        }

		/* Setback FDT default value */

        err = st95hf_iso14443a_config_fdt(dev, st95hf->fwi, 1, 0);
        if (err!=0){
            LOG_ERR("Configuring FDT failed %d",err);
            //return err;
        }

	}

    
    apdu_rsp_t* adpu_rsp;
    if (st95hf->tag_type==ST95HF_TAG_TYPE_TT4A){
        /**
         * length>=7
         * data
         *          meaning
         * -----------------
         *          STATUS
         *          LENGTH =7+X
         *  [0,X-1] ??
         *  X       SW1
         *  X+1     SW2
         *  X+2     ?
         *  X+3     ?
         *  X+4     footer[0]
         *  X+5     footer[1]
         *  X+6     footer[2]
         *
         *  footer = data[length-3]
         *  data[length-7] = SW1
         *  data[length-6] = SW2
         */

        if (rsp.len<7){
            LOG_ERR("Error response size %d<7",rsp.len);
            return -EBADMSG;
        }
        adpu_rsp =(apdu_rsp_t* ) &rsp_data[rsp.len-7];
    } else {
        /**
         * length>=5
         * data
         *          meaning
         * -----------------
         *          STATUS
         *          LENGTH =5+X
         *  [0,X-1] ??
         *  X       SW1
         *  X+1     SW2
         *  X+2     ?
         *  X+3     ?
         *  X+4     footer[0]
         *
         *  footer = data[length-1]
         *  data[length-5] = SW1
         *  data[length-4] = SW2
         */
        if (rsp.len<7){
            LOG_ERR("Error response size %d<5",rsp.len);
            return -EBADMSG;
        }

        adpu_rsp =(apdu_rsp_t* ) &rsp_data[rsp.len-5];
    }
    if (adpu_rsp->sw1 == 0x90 && adpu_rsp->sw2 == 0x00){
        if (rsp_length!=NULL && data!=NULL){            
            if (rsp.len>*rsp_length){
                LOG_ERR("Response buffer not enough %d<%d",*rsp_length,rsp.len);
                return -ENOMEM;
            }
            memcpy(data,rsp_data,rsp.len);
            *rsp_length = rsp.len;
        }

		return 0;	
    } else {
        LOG_ERR("Error SW1 0x%02x!=0x90 or SW2 0x%02x!=0x00",adpu_rsp->sw1,adpu_rsp->sw2);
		return -ENOMSG;	
    }
}

int st95hf_iso7816_select_file (const struct device* dev, uint8_t mode, uint8_t option, uint8_t data_length, const uint8_t *data ){
    if (dev==NULL){
        return -EINVAL;
    }
    st95hf_data_t *st95hf = dev->data;
    apdu_cmd_t cmd = {
        .header.cla = ISO7816_CLASS_0X00,   // add the class byte
        .header.ins = ISO7816_SELECT_FILE,  // add the command code
        // add the P1 and P2 fields
        .header.p1 = mode,
        .header.p2 = option,
        .body.lc = data_length, // add the LC field
        .body.le=0  // the LE field is empty
    };

    //add the FileId field
    memcpy (cmd.body.data,data,cmd.body.lc);

    // Special case for the selectApplication
    // The block number HAS TO be 0x02
    if (cmd.header.p1 == 0x04 && cmd.header.p2 == 0x00){
        st95hf->block_number = 0x02;
    }
		
    return st95hf_iso7816_send_receive_adpu(dev,&cmd,NULL,NULL);
}
int st95hf_iso7816_read_binary (const struct device* dev, uint16_t offset, uint8_t expected_length, uint8_t *data ){
    
    apdu_cmd_t cmd = {
        .header.cla = ISO7816_CLASS_0X00,   // add the class byte
        .header.ins = ISO7816_READ_BINARY,  // add the command code
        // add the P1 and P2 fields
        .header.p1 = (offset>>8) & 0xFFu,
        .header.p2 = offset & 0xFFu,
        .body.lc = 0, // add the LC field    
        .body.le=expected_length  // the LE field
    };
    uint16_t le = expected_length;
    return st95hf_iso7816_send_receive_adpu(dev,&cmd,&le,data);
}

int st95hf_iso7816_update_binary (const struct device* dev, uint8_t mode, uint8_t option, uint8_t data_length, const uint8_t *data){
    
    apdu_cmd_t cmd = {
        .header.cla = ISO7816_CLASS_0X00,   // add the class byte
        .header.ins = ISO7816_UPDATE_BINARY,  // add the command code
        // add the P1 and P2 fields
        .header.p1 = mode,
        .header.p2 = option,
        .body.lc = data_length, // add the LC field    
        .body.le=0  // the LE field
    };
        //add the FileId field
    memcpy (cmd.body.data,data,cmd.body.lc);
    return st95hf_iso7816_send_receive_adpu(dev,&cmd,NULL,NULL);
}