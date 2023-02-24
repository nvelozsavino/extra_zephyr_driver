#pragma once

#include <zephyr/drivers/counter.h>
#include <stdint.h>
#include <zephyr/kernel.h>

typedef struct
{
    uint32_t interval_us;
    struct k_sem sem;
    uint8_t channel;
    struct counter_alarm_cfg cfg;
    const struct device *dev;
    bool enabled;
} app_alarm_t;

int app_alarm_init(app_alarm_t *alarm, const struct device *dev, uint8_t channel);
int app_alarm_configure(app_alarm_t *alarm);
int app_alarm_stop(app_alarm_t *alarm);
