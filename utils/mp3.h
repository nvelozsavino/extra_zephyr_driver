#pragma once
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <unistd.h>

int mp3_play(const struct device * i2s_dev, const char* file,struct k_sem* sem);
