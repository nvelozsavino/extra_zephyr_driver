#include "wav.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>

#define MINIMP3_ONLY_MP3
// #define MINIMP3_ONLY_SIMD
// #define MINIMP3_NO_SIMD
// #define MINIMP3_NONSTANDARD_BUT_LOGICAL
// #define MINIMP3_FLOAT_OUTPUT
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#include <zephyr/fs/fs.h>

LOG_MODULE_REGISTER(mp3, LOG_LEVEL_INF);


#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUMBER_OF_CHANNELS  2
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   1024
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK*NUMBER_OF_CHANNELS)
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);



static int mp3_play_internal(const struct device * i2s_dev, struct fs_file_t* file, struct k_sem* sem){

	mp3dec_t mp3d;
    mp3dec_frame_info_t info={0};
    unsigned char buf[4096];
	int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];

	int samples= 0;
	mp3dec_init(&mp3d);

	int ret = fs_seek(file,0,FS_SEEK_END);
	 if (ret<0){
        return ret;
    }
	off_t file_size = fs_tell(file);
	if (file_size<0){
		return file_size;
	}
	ret = fs_seek(file,0,FS_SEEK_SET);
	 if (ret<0){
        return ret;
    }
	bool is_done=false;
	size_t buffer_ptr=0;
	size_t file_ptr=0;	//keeps track of how many bytes from the file has been read
	size_t buffer_used=0;
	int channels = 0;
	size_t i2s_writes = 0;
	
	void *mem_block=NULL;
	int sample_ptr=0;
	int max_samples=0;
	int bytes_per_sample = 0;	
	
	do {
		//read from file
		ssize_t read = fs_read(file,&buf[buffer_ptr],sizeof(buf)-buffer_ptr);
		if (read<0){
			LOG_ERR("Read file error %d",read);
			return read;
		}else if (read==0){
			//Nothing else to read
			LOG_DBG("Nothing more to read");
			is_done=true;
		} else {
			//Read success 'read' bytes in &buf[buffer_ptr]
			file_ptr+=read;		//Increment file_ptr 
			
			if (file_ptr>=file_size){
				is_done=true;
			}
			buffer_used=0;
			buffer_ptr+=read;
			LOG_DBG("Read %d bytes, %d/%d. filled: %d",read,file_ptr,(int)file_size,buffer_ptr);
			do {
				LOG_DBG("Dec [%d], %d", buffer_used,buffer_ptr-buffer_used);
				samples = mp3dec_decode_frame(&mp3d,buf+buffer_used,buffer_ptr-buffer_used,pcm,&info);
				
				buffer_used+=info.frame_bytes;
				LOG_DBG("s=%d, b=%d, used=%d", samples,info.frame_bytes,buffer_used);

				if (samples == 0 && info.frame_bytes>0){
					//id3tag read
					LOG_DBG("id3Tag");
					continue;
				} else if (samples > 0 && info.frame_bytes >0){



					// buffer_used+=info.frame_bytes;
					

					// LOG_INF("%d samp %d bytes", samples,info.frame_bytes);


					if (channels==0){
						LOG_DBG("Configuring i2s");
						//first time
						channels=info.channels;
						max_samples = BLOCK_SIZE/(BYTES_PER_SAMPLE*channels);
						struct i2s_config config;
						config.word_size = SAMPLE_BIT_WIDTH;
						config.channels = channels;
						config.format = I2S_FMT_DATA_FORMAT_I2S;
						config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
						config.frame_clk_freq = info.hz;
						config.mem_slab = &mem_slab;
						config.block_size = BLOCK_SIZE;
						config.timeout = TIMEOUT;

						ret = i2s_configure(i2s_dev, I2S_DIR_TX, &config);
						if (ret != 0) {
							LOG_ERR("Failed to configure TX stream: %d\n", ret);
							return ret;
						}
						i2s_writes=0;
						mem_block=NULL;			
						bytes_per_sample = sizeof(int16_t)*channels;
			
					}
					int remaining = samples;
					int index = 0;
					do {
						if (mem_block==NULL){
							int ret = k_mem_slab_alloc(&mem_slab, &mem_block, i2s_writes<INITIAL_BLOCKS?K_NO_WAIT:K_FOREVER);
							if (ret<0){
								LOG_ERR("Failed to allocate TX block: %d\n", ret);
								return ret;
							}
							sample_ptr=0;						
						}
						uint8_t* dst = (uint8_t*)mem_block;
						uint8_t* src = (uint8_t*)pcm;
						
						int samples_fit = max_samples-sample_ptr;
						int samples_to_copy = remaining;
						bool flush=false;
						if (samples_to_copy >= samples_fit){
							samples_to_copy=samples_fit;					
							flush=true;
						}
						size_t dst_ptr = sample_ptr*bytes_per_sample;
						size_t src_ptr = index*bytes_per_sample;
						size_t copy_bytes = samples_to_copy*bytes_per_sample;
						LOG_DBG("[%d] %d ->[%d]. %d",src_ptr,copy_bytes,dst_ptr,flush);
						memcpy(&dst[dst_ptr],&src[src_ptr],copy_bytes);
						sample_ptr += samples_to_copy;
						index+=samples_to_copy;
						remaining-=samples_to_copy;
						if (flush){
							//i2s_write
							// LOG_INF("I2S Write %d",i2s_writes);
							// LOG_INF("I2SW");
							ret = i2s_write(i2s_dev, mem_block, BLOCK_SIZE);
							if (ret < 0) {
								LOG_ERR("Failed to write block: %d\n", ret);
								return ret;
							}
							
							sample_ptr=0;
							i2s_writes++;
							if (i2s_writes==INITIAL_BLOCKS){
								LOG_DBG("I2S Trigger");
								ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
								if (ret < 0) {
									LOG_ERR("Failed to trigger command START on TX: %d\n",  ret);
									return ret;
								}
							}
							mem_block=NULL;
						}

					} while (remaining>0);
					


				} else if (samples==0 && info.frame_bytes==0 && !is_done){
					
					//need more bytes
					size_t N = buffer_ptr - buffer_used;
					if (buffer_used==0){
						LOG_ERR("Buffer not enough for decoding");
						return -1;						
					}
					LOG_DBG("More samples buf[%d] %d bytes to buf[0]",buffer_used,N);
					if (buffer_used>=N){
						memcpy(buf,&buf[buffer_used],N);
					} else {
						memcpy(buf,&buf[buffer_used], buffer_used);
						memcpy(&buf[buffer_used],&buf[2*buffer_used],N-buffer_used);
					}
					buffer_used=0;
					buffer_ptr=N;
					break;
				}
				if (is_done){
					break;
				}
			} while(1);

		}

	} while(!is_done);

	if (mem_block!=NULL){
		uint8_t* dst = (uint8_t*)mem_block;
		int samples_to_clear = max_samples-sample_ptr;
		memset(&dst[sample_ptr*bytes_per_sample],0,samples_to_clear*bytes_per_sample);
		LOG_DBG("last I2S Write %d",i2s_writes);
		ret = i2s_write(i2s_dev, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("Failed to write block: %d\n", ret);
			return ret;
		}
	}
	
    ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_INF("Failed to trigger command DRAIN on TX: %d\n",  ret);
		return ret;
	}
	LOG_INF("Sound stopped");
	return 0;
}

int mp3_play(const struct device * i2s_dev, const char* filename,struct k_sem* sem){
    struct fs_file_t file;

    fs_file_t_init(&file);
	LOG_INF("Opening File %s",filename);
    int ret = fs_open(&file,filename,FS_O_READ);
    if (ret<0){
		LOG_ERR("Failed to open File %s. Erro %d",filename,ret);
        return ret;
    }

	ret = mp3_play_internal(i2s_dev,&file,sem);
	ret = fs_close(&file);
	if (ret!=0){
		LOG_ERR("Error closing %s. Error %d",filename, ret);			
	}



    return ret;

}

