#pragma once
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <unistd.h>

#pragma pack(push,1)

typedef struct {
    struct {
        char id[4];
        uint32_t size;
        char format[4];
    } riff;
    struct {
        char id[4];
        uint32_t size;
        uint16_t format;
        uint16_t channels;
        uint32_t sample_rate;
        uint32_t byte_rate;
        uint16_t block_align;
        uint16_t bits_per_sample;
    } format;
    struct {
        char id[4];
        uint32_t size;
    }data;
} wav_t;
#pragma pack(pop)



typedef ssize_t (*wav_read_func_t) (void* context, void* ptr, size_t size);
typedef int (*wav_open_func_t) (void* context);
// typedef int (*wav_seek_func_t) (void* context, off_t offset, int whence);
typedef int (*wav_close_func_t) (void* context);

typedef struct {
	size_t data_index;
	wav_t wav;
	wav_open_func_t open_func;
	wav_close_func_t close_func;
	wav_read_func_t read_func;
	// wav_seek_func_t seek_func;
	void* context;
	// float volume;
    // size_t samples;
    // uint8_t channels;
    // uint8_t bytes_per_sample;    
} sound_t;

int wav_play(const struct device * i2s_dev, sound_t* sound,struct k_sem* sem);
int wav_check(const wav_t* wav);

typedef struct {
	const uint8_t * ptr;
	size_t pos;
	size_t size;
} wav_local_t;

ssize_t wav_read_local (void* context, void* ptr, size_t size);
int wav_init_local(wav_local_t* local, const void* ptr, size_t size);

