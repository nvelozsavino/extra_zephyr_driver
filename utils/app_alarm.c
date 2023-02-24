
#include "app_alarm.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_alarm, LOG_LEVEL_DBG);

static void app_alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks,
                               void *user_data) {
    // static uint32_t count=0;
    // if (count%100==0){
    //     LOG_INF("alarm %d callback", chan_id);
    // }
    // count++;
    app_alarm_t *alarm = (app_alarm_t *)user_data;

    k_sem_give(&alarm->sem);
    if (alarm->enabled) {
        int err;
        alarm->cfg.flags = COUNTER_ALARM_CFG_ABSOLUTE;
        alarm->cfg.ticks = ticks + counter_us_to_ticks(dev, alarm->interval_us);
        alarm->cfg.user_data = alarm;

        err = counter_set_channel_alarm(dev, alarm->channel, &alarm->cfg);
        if (err != 0) {
            LOG_ERR("Error setting alarm %d. %d", chan_id, err);
        }
    }
}

int app_alarm_configure(app_alarm_t *alarm) {

    LOG_INF("Configuring alarm %d to %dus", alarm->channel, alarm->interval_us);
    app_alarm_stop(alarm);
    alarm->cfg.flags = 0; // COUNTER_ALARM_CFG_ABSOLUTE;
    alarm->cfg.ticks = counter_us_to_ticks(alarm->dev, alarm->interval_us);
    alarm->cfg.callback = app_alarm_callback;
    alarm->cfg.user_data = alarm;
    alarm->enabled = true;

    LOG_DBG("Alarm %d, ticks %d top_value %d", alarm->channel, alarm->cfg.ticks,
            counter_get_top_value(alarm->dev));
    int err = counter_set_channel_alarm(alarm->dev, alarm->channel, &alarm->cfg);
    if (err != 0) {
        LOG_ERR("Error configuring alarm %d. %d", alarm->channel, err);
        return err;
    }
    return err; // counter_start(alarm->dev);
}
int app_alarm_stop(app_alarm_t *alarm) {
    LOG_INF("Stopping alarm %d ", alarm->channel);
    alarm->enabled = false;
    int err = counter_cancel_channel_alarm(alarm->dev, alarm->channel);
    if (err != 0) {
        LOG_ERR("Error stoping alarm %d. %d", alarm->channel, err);
    }
    // err = counter_stop(alarm->dev);
    // if (err!=0){
    //     LOG_ERR("Error stopping timer %s. %d", alarm->dev->name, err);
    // }
    return err;
}

int app_alarm_init(app_alarm_t *alarm, const struct device *dev, uint8_t channel) {
    alarm->dev = dev;
    alarm->channel = channel;
    alarm->enabled = false;
    alarm->interval_us = 0;
    // counter_start(alarm->dev);
    k_sem_init(&alarm->sem, 0, 1);
    int err = counter_cancel_channel_alarm(alarm->dev, alarm->channel);
    if (err != 0) {
        LOG_ERR("Error stoping alarm %d. %d", alarm->channel, err);
    }
    return err;
}
