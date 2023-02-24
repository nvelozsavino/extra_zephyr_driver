#include "app_pin.h"

int app_pin_init(app_pin_t *pin, bool set) {
    if (pin == NULL) {
        return -1;
    }
    // pin->dev = device_get_binding(pin->dt_spec);
    // if (pin->dev==NULL){
    // 	return -1;
    // }

    int err = gpio_pin_configure_dt(&pin->dt_spec, set ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);

    // int err = gpio_pin_configure_dt(&pin->dt_spec,GPIO_OUTPUT_ACTIVE);
    if (err != 0) {
        return err;
    }

    return 0; // app_pin_set(pin,set);
}

int app_pin_set(app_pin_t *pin, bool set) {
    if (pin == NULL /*|| pin->dev==NULL*/) {
        return -1;
    }
    return gpio_pin_set_dt(&pin->dt_spec, (int)set);
}

int app_pin_toggle(app_pin_t *pin) {
    if (pin == NULL) {
        return -1;
    }
    return gpio_pin_toggle_dt(&pin->dt_spec);
}
