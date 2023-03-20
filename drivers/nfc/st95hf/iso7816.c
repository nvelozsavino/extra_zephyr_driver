#include "iso7816.h"



int st95hf_iso7816_select_file (uint8_t mode, uint8_t option, uint8_t data_length, const uint8_t *data );
int st95hf_iso7816_read_binary (uint8_t mode, uint8_t option, uint8_t expected_length, uint8_t *data );
int st95hf_iso7816_update_binary (uint8_t mode, uint8_t option, uint8_t data_length, const uint8_t *data);