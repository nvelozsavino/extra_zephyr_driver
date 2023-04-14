#pragma once
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <zephyr/kernel.h>


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


int wav_play(const struct device * i2s_dev, const wav_t* wav,struct k_sem* sem);
int wav_check(const wav_t* wav);



