#include "app_pin.h"

int app_pin_init(app_pin_t *pin, bool default_value, bool also_as_input) {
    if (pin == NULL) {
        return -EINVAL;
    }
    // pin->dev = device_get_binding(pin->dt_spec);
    // if (pin->dev==NULL){
    // 	return -1;
    // }
    gpio_flags_t flag = also_as_input?GPIO_INPUT:0;

    int err = gpio_pin_configure_dt(&pin->dt_spec, default_value ? flag|GPIO_OUTPUT_ACTIVE : flag|GPIO_OUTPUT_INACTIVE);

    // int err = gpio_pin_configure_dt(&pin->dt_spec,GPIO_OUTPUT_ACTIVE);
    if (err != 0) {
        return err;
    }

    return 0; // app_pin_set(pin,set);
}

int app_pin_set(app_pin_t *pin, bool set) {
    if (pin == NULL) {
        return -EINVAL;
    }
    return gpio_pin_set_dt(&pin->dt_spec, (int)set);
}

int app_pin_get(app_pin_t *pin) {
    if (pin == NULL) {
        return -EINVAL;
    }
    return gpio_pin_get(pin->dt_spec.port,pin->dt_spec.pin);
}

int app_pin_toggle(app_pin_t *pin) {
    if (pin == NULL) {
        return -EINVAL;
    }
    return gpio_pin_toggle_dt(&pin->dt_spec);
}
