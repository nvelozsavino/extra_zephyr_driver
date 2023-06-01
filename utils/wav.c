#include "wav.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(wav, CONFIG_LOG_DEFAULT_LEVEL);


#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUMBER_OF_CHANNELS  2
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   256
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK*NUMBER_OF_CHANNELS)
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

int wav_check(const wav_t* wav){
    if (wav==NULL){
        LOG_ERR("WAV is null");
        return -EINVAL;
    }
    if (memcmp(wav->riff.id,"RIFF",4)!=0){
        LOG_ERR("WAV format RIFF error: %s",wav->riff.id);
        return -ENOMSG;
    }

    if (memcmp(wav->riff.format,"WAVE",4)!=0){
        LOG_ERR("WAV format RIFF WAVE error: %s",wav->riff.format);
        return -ENOMSG;
    }

    if (memcmp(wav->format.id,"fmt ",4)!=0){
        LOG_ERR("WAV format FMT error: %s", wav->format.id );
        return -ENOMSG;
    }

    if (memcmp(wav->data.id,"data",4)!=0){
        LOG_ERR("WAV format DATA error: %s",wav->data.id);
        return -ENOMSG;
    }

    if (wav->format.bits_per_sample>16){
        LOG_ERR("WAV file more than 16 bits per sample: %d",wav->format.bits_per_sample);
        return -E2BIG;
    }
    return 0;

}

static int prepare_transfer(const struct device *i2s_dev_tx, sound_t* sound)
{
	LOG_INF("PRE buffering");
	int ret;
	// sound->data_index=0;
	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to allocate TX block %d: %d\n", i, ret);
			return ret;
		}

		//fill_buf(sound,mem_block);
		memset(mem_block, 0, BLOCK_SIZE);

		ret = i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("Failed to write block %d: %d\n", i, ret);
			return ret;
		}
	}
	LOG_INF("PRE buffering done");
	return 0;
}

static ssize_t fill_buf(sound_t* sound, void *buffer)
{
	const uint8_t bytes = (sound->wav.format.bits_per_sample>>3);
	const uint8_t channels = sound->wav.format.channels;
	uint32_t num_samples = BLOCK_SIZE/(bytes*channels);
	if (num_samples>SAMPLES_PER_BLOCK){
		num_samples=SAMPLES_PER_BLOCK;
	}
	uint8_t data[BLOCK_SIZE];
	
	uint32_t to_read = num_samples*bytes*channels;

	ssize_t read=sound->read_func(sound->context,data,to_read);

	if (read<0){
		LOG_ERR("Error reading sound %d",read);
		return -1;
	}

	if (read<to_read){
		LOG_WRN("Done");
	}
	uint32_t samples = read/(bytes*channels);
	LOG_INF("Read %d, samples: %d",read, samples);
	// const uint8_t* p_data = &((const uint8_t*)sound->wav)[sizeof(wav_t)];


	int16_t offset = 0;
	int16_t gain=1;
	if (bytes==1){
		offset = -128;
		gain=256;
	}
	uint32_t index=0;
	for (uint32_t n=0;n<samples;n++){
		union {
			uint8_t block[BYTES_PER_SAMPLE*NUMBER_OF_CHANNELS];
			int16_t ch[NUMBER_OF_CHANNELS];
		} sample;


		const uint8_t *p_chunk = &data[index];
		
		//start of chunk in sound->data_index
		
		//sample[ch=0] = *p_chunk
		//sample[ch=1] = *(p_chunk+bytes)
		//sample[ch=c] = *(p_chunk+(c*bytes))

		//sample[ch=c] = *(p_chunk + c*bytes) =
		
		for (uint8_t c = 0;c<NUMBER_OF_CHANNELS;c++){			
			if (index+(channels*bytes)>read){
				sample.ch[c]=0;
			} else {

			
				if (c>=channels){			
					sample.ch[c]=((int16_t)p_chunk[0] + offset)*gain;
				} else {				
					sample.ch[c] = ((int16_t)p_chunk[c*bytes] +offset)*gain;
				}
				// buf_pos+=sizeof(samples);
			}
		}
		uint8_t* ptr = (uint8_t*) buffer; 
		memcpy(&ptr[n*BYTES_PER_SAMPLE*NUMBER_OF_CHANNELS],&sample,sizeof(sample));
		index+=channels*bytes;
		
	}
	return samples*BYTES_PER_SAMPLE*NUMBER_OF_CHANNELS;

	// sound->data_index+=index;
	// if (sound->data_index>=sound->wav.data.size || read<BLOCK_SIZE){
	// 	return 1;
	// }
	// return 0;
		
}


static int play_sound(const struct device * i2s_dev_tx, sound_t* sound){
	int ret;
	void* mem_block;
	LOG_INF("Allocating %d",sound->data_index);
	ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_FOREVER);
	if (ret<0){
		LOG_ERR("Failed to allocate TX block: %d\n", ret);
		return ret;
	}
	LOG_INF("Filling");
	ssize_t filled = fill_buf(sound,mem_block);
	LOG_INF("Writing %d", filled);
	
	if (filled<=0){
		LOG_INF("Done %d",filled);
		k_mem_slab_free(&mem_slab,&mem_block);
		return 1;
	}
	if (filled<BLOCK_SIZE){
		memset(&mem_block[filled],0,BLOCK_SIZE-filled);
	}
	ret = i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
	if (ret < 0) {
	LOG_ERR("Failed to write block: %d\n", ret);
		return ret;
	}
	return 0;	
}


static int wav_play_internal(const struct device * i2s_dev, sound_t* sound, struct k_sem* sem){

	int ret;
	ssize_t read = sound->read_func(sound->context,&sound->wav,sizeof(wav_t));
	if (read<0){
		LOG_ERR("Error reading sound %d",read);
		return read;
	}
	if (read!=sizeof(wav_t)){
		LOG_ERR("Error reading wav header %d!=%d", read, sizeof(wav_t));
		return -EIO;
	}
	ret = wav_check(&sound->wav);
	if (ret!=0){
		LOG_ERR("wav is invalid %d",ret);
		return ret;
	}	

	struct i2s_config config;
    config.word_size = SAMPLE_BIT_WIDTH;
    config.channels = NUMBER_OF_CHANNELS;
    config.format = I2S_FMT_DATA_FORMAT_I2S;
    config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    config.frame_clk_freq = sound->wav.format.sample_rate;
    config.mem_slab = &mem_slab;
    config.block_size = BLOCK_SIZE;
    config.timeout = TIMEOUT;

    ret = i2s_configure(i2s_dev, I2S_DIR_TX, &config);
	if (ret != 0) {
		LOG_INF("Failed to configure TX stream: %d\n", ret);
		return ret;
	}

    ret = prepare_transfer(i2s_dev,sound);
    if (ret!=0) {
        return ret;
    }

    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_INF("Failed to trigger command START on TX: %d\n",  ret);
		return ret;
	}

    LOG_INF("Streams started\n");

    while (sem==NULL || k_sem_take(sem, K_NO_WAIT) != 0) {

        int ret = play_sound(i2s_dev,sound);
        if (ret < 0) {
            LOG_INF("Failed to allocate play sound block: %d\n", ret);
            return ret;
        }
        
        if (ret==1){
             break;
        }
        
    }
    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_INF("Failed to trigger command DRAIN on TX: %d\n",  ret);
		return ret;
	}

	return 0;
}

int wav_play(const struct device * i2s_dev, sound_t* sound, struct k_sem* sem){

	int ret =0;
	if (sound==NULL){
		return -EINVAL;
	}
	if (sound->read_func==NULL){
		LOG_ERR("read_func is required");
		return -EINVAL;
	}
	
	if (sound->open_func){
		ret = sound->open_func(sound->context);
		if (ret!=0){
			LOG_ERR("Error opening sound %d",ret);
			return ret;
		}
	}

	ret = wav_play_internal(i2s_dev,sound,sem);
	if (sound->close_func){
		int err = sound->close_func(sound->context);
		if (err!=0){
			LOG_ERR("Error closing sound %d",err);			
		}
	}

    return ret;

}


int wav_init_local(wav_local_t* local, const void* ptr, size_t size){
	if (local==NULL || ptr==NULL){
		return -EINVAL;
	}
	local->ptr=ptr;
	local->pos=0;
	local->size=size;
	return 0;
}

ssize_t wav_read_local (void* context, void* ptr, size_t size){
	wav_local_t* local = (wav_local_t*)context;
	size_t to_copy = size;
	if ((local->pos+to_copy)>local->size){
		to_copy=local->size-local->pos;
	}
	memcpy(ptr,&local->ptr[local->pos],to_copy);
	local->pos+=to_copy;
	return to_copy;
}
