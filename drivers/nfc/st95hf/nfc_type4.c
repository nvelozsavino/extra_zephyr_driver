#include "nfc_type4.h"
#include "iso7816.h"
#include <errno.h>
#include "st95hf.h"
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(st95hf, LOG_LEVEL_DBG);

int type4_read_binary(const struct device* dev, uint16_t offset, uint8_t bytes, uint8_t* data){
    int err = st95hf_iso7816_read_binary(dev,offset,bytes,data);
    if (err!=0){
        LOG_ERR("Error reading binary %d",err);
        return err;
    }
    return 0;
}

int st95hf_type4_read_ndef(const struct device* dev, uint16_t* ndef_length, uint8_t* ndef){

    st95hf_data_t *st95hf = dev->data;

    // SelectAppli
    const uint8_t select_appl[] =PCDNFCT4_SELECT_APPLI;
    int err = st95hf_iso7816_select_file( dev, 0x04 , 0x00 , 7 ,select_appl );
    if (err!=0){
        LOG_ERR("Error selecting Application %d",err);
        return err;
    }

	// SelectCC
    const uint8_t select_cc[] =PCDNFCT4_CC_ID;
    err = st95hf_iso7816_select_file( dev, 0x00 , 0x0C , sizeof(select_cc) ,select_cc );
    if (err!=0){
        LOG_ERR("Error selecting CC %d",err);
        return err;
    }
    uint8_t buffer[16];
    err = st95hf_iso7816_read_binary(dev,0,15,buffer);
    if (err!=0){
        LOG_ERR("Error reading CC %d",err);
        return err;
    }
    uint8_t ndef_id[] = {buffer[10], buffer[11]};
    
    uint16_t mle = MIN((buffer[4]<<8) | buffer[5],0xF6);

    // Check if read access is allowed

    if (buffer[14]!=PCDNFCT4_ACCESS_ALLOWED){
        LOG_ERR("Not allowed %02x!=%02x",buffer[14],PCDNFCT4_ACCESS_ALLOWED);
        return -EACCES;
    }
	// SelectNDEF
    err = st95hf_iso7816_select_file( dev, 0x00 , 0x0C , sizeof(ndef_id) ,ndef_id );
    if (err!=0){
        LOG_ERR("Error selecting NDEF %d",err);
        return err;
    }
    err = st95hf_iso7816_read_binary(dev,0,2,buffer);
    if (err!=0){
        LOG_ERR("Error reading NDEF length %d",err);
        return err;
    }

    uint16_t size = ((buffer[1]<<8) | buffer[2])+2;

    if (size>*ndef_length){
        LOG_WRN("Not enough memory for NDEF %d<%d",*ndef_length,size);
        size = *ndef_length;    
    }

    uint8_t limit =  MIN(mle, (st95hf->fsc - 5));

    uint16_t i=0;
    while (size>0){
        uint8_t size_to_copy = size;
        if (size_to_copy>limit){
            size_to_copy=limit;
        }

        err = st95hf_iso7816_read_binary(dev,i,size_to_copy,&ndef[i]);
        if (err!=0){
            LOG_ERR("Error reading NDEF length %d",err);
            return err;
        }
        size-=size_to_copy;
        i+=size_to_copy;
    }
    
    return 0;

}