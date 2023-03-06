#pragma once

#include <zephyr/drivers/gpio.h>

#define APP_PIN_INIT_BY_ALIAS(alias) \
    { \
        .name = #alias, .dt_spec = GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios), \
    }
#define APP_PIN_INIT_BY_USER(_pin) \
    { \
        .name = #_pin, .dt_spec = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), _pin##_gpios), \
    }
#define APP_PIN_INIT_BY_NODE(_node,_pin) \
    { \
        .name = #_pin, .dt_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(_node), _pin##_gpios), \
    }
/*
// .name = (_pin), \
// .label = DT_GPIO_LABEL(DT_N_S_ZEPHYR_USER_NODE,_pin##_gpios), \
// .pin = DT_GPIO_PIN(ZEPHYR_USER_NODE,_pin##_gpios), \
// .flags = DT_GPIO_FLAGS(ZEPHYR_USER_NODE,_pin##_gpio), \

// }
*/
typedef struct
{
    const char *name;
    // const char* label;
    // gpio_pin_t pin;
    // gpio_flags_t  flags;
    struct gpio_dt_spec dt_spec;

    // const struct device* dev;
} app_pin_t;

int app_pin_init(app_pin_t *pin, bool default_value, bool also_as_input);
int app_pin_set(app_pin_t *pin, bool on);
int app_pin_get(app_pin_t *pin);
int app_pin_toggle(app_pin_t *pin);
