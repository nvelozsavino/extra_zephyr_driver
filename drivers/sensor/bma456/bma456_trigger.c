/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma456

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bosch/bma4.h"

#ifdef CONFIG_BMA456_VARIANT_MM
#include "bosch/bma456mm.h"
#endif
#ifdef CONFIG_BMA456_VARIANT_H
#include "bosch/bma456h.h"
#endif
#ifdef CONFIG_BMA456_VARIANT_W
#include "bosch/bma456w.h"
#endif
#ifdef CONFIG_BMA456_VARIANT_AN
#include "bosch/bma456an.h"
#endif


// #define TRIGGED_INT1			4
// #define TRIGGED_INT2			5

#define START_TRIG_INT1			(0)
#define START_TRIG_INT2			(1)

#define TRIGGED_INT1			(2)
#define TRIGGED_INT2			(3)

LOG_MODULE_DECLARE(bma456, CONFIG_SENSOR_LOG_LEVEL);
#include "bma456.h"


static const uint8_t int1_triggers_array[] =  { 
	0,  /* Both int1 and int2 disabled */ 
	(BMA456_TRIGGER_DATA_READY|BMA456_TRIGGER_ANY_MOTION|BMA456_TRIGGER_NO_MOTION|BMA456_TRIGGER_TAP|BMA456_TRIGGER_DOUBLE_TAP), /* only int1 enabled */ 
	0,  /* only int2 enabled */ 
	BMA456_TRIGGER_DATA_READY  /* Both enabled */ 
};

static const uint8_t int2_triggers_array[] =  { 
	0, 	/* Both int1 and int2 disabled*/ 
	0, /* only int1 enabled */ 
	(BMA456_TRIGGER_DATA_READY|BMA456_TRIGGER_ANY_MOTION|BMA456_TRIGGER_NO_MOTION|BMA456_TRIGGER_TAP|BMA456_TRIGGER_DOUBLE_TAP), /* only int2 enabled */ 
	(BMA456_TRIGGER_ANY_MOTION|BMA456_TRIGGER_NO_MOTION|BMA456_TRIGGER_TAP|BMA456_TRIGGER_DOUBLE_TAP) /* Both enabled */ 
};

static inline void setup_int1(const struct device *dev,
			      bool enable)
{
	const struct bma456_config *cfg = dev->config;

	// LOG_WRN("Setup1 %d",enable);
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int1,
					enable
					? GPIO_INT_EDGE_TO_ACTIVE
					: GPIO_INT_DISABLE);
}

static inline void setup_int2(const struct device *dev,
			      bool enable)
{
	const struct bma456_config *cfg = dev->config;
	LOG_WRN("Setup2 %d",enable);
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int2,
					enable
					? GPIO_INT_EDGE_TO_ACTIVE
					: GPIO_INT_DISABLE);
}




static int bma456_trigger_map_helper(const struct device *dev, 
					bma456_trigger_t* trigger_handler,
					uint8_t trigger,
					uint16_t int_map,
				   const struct sensor_trigger *trig,
				   sensor_trigger_handler_t handler)
{
	const struct bma456_config *cfg = dev->config;
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	int8_t rslt;
	uint8_t map;

	if (bma456->int1_triggers & trigger){
		if (cfg->gpio_int1.port == NULL) {
			LOG_ERR("trigger_set %02x int not supported",trigger);
			return -ENOTSUP;
		}
		map = BMA4_INTR1_MAP;
		LOG_INF("Enabling trigger %02x in int1", trigger);
		// setup_int1(dev,false);
	} else if (bma456->int2_triggers & trigger){
		if (cfg->gpio_int2.port == NULL) {
			LOG_ERR("trigger_set %02x int not supported", trigger);
			return -ENOTSUP;
		}
		map = BMA4_INTR2_MAP;
		LOG_INF("Enabling trigger %02x in int2", trigger);
		// setup_int2(dev,false);
	} else {
		LOG_WRN("Int1:%08x, Int2: %08x, trigger %08x",bma456->int1_triggers,bma456->int2_triggers,trigger);
		return -ENOTSUP;
	}

	LOG_INF("Disabling int_map %04x in pin %d",int_map,map);

   	rslt = bma4_map_interrupt(map, int_map, BMA4_DISABLE, bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Error removing %04x map from int1 (%d)", int_map, rslt);
		return -EIO;
	}



	trigger_handler->handler = handler;
	if (handler == NULL) {
		LOG_INF("Trigger disabled");
		return 0;
	}

	trigger_handler->trigger = *trig;
	LOG_INF("Enabling int_map %04x in pin %d",int_map,map);
  	rslt = bma4_map_interrupt(map, int_map, BMA4_ENABLE, bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Error setting %04x map from int (%d)", int_map, rslt);
		return -EIO;
	}
	// if (map==BMA4_INTR1_MAP){
	// 	setup_int1(dev,true);
	// } else {
	// 	setup_int2(dev,true);
	// }
	LOG_WRN("Int %d pin %d",map, gpio_pin_get(cfg->gpio_int1.port,cfg->gpio_int1.pin));
	return 0;
}

static int bma456_trigger_set_helper(const struct device *dev, 
					bma456_trigger_t* trigger_handler,
					uint8_t trigger,
					uint16_t int_map,
				   const struct sensor_trigger *trig,
				   sensor_trigger_handler_t handler,
				   bool toggle_feature, uint8_t feature)
{
	const struct bma456_config *cfg = dev->config;
	struct bma456_data *bma456 = dev->data;
	struct bma4_dev *bma = &bma456->bma;
	int8_t rslt;
	uint8_t map;

	if (bma456->int1_triggers & trigger){
		if (cfg->gpio_int1.port == NULL) {
			LOG_ERR("trigger_set %02x int not supported",trigger);
			return -ENOTSUP;
		}
		map = BMA4_INTR1_MAP;

	} else if (bma456->int2_triggers & trigger){
		if (cfg->gpio_int2.port == NULL) {
			LOG_ERR("trigger_set %02x int not supported", trigger);
			return -ENOTSUP;
		}
		map = BMA4_INTR2_MAP;

	} else {
		return -ENOTSUP;
	}

	if (toggle_feature){
		rslt = bma456mm_feature_enable(feature,BMA4_DISABLE,bma);
		if (rslt!=BMA4_OK){
			LOG_ERR("Error enabling feature %02x. %d",feature,rslt);
			return -EIO;
		}	
	}

   	rslt = bma4_map_interrupt(map, int_map, BMA4_DISABLE, bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Error removing %04x map from int1 (%d)", int_map, rslt);
		return -EIO;
	}



	trigger_handler->handler = handler;
	if (handler == NULL) {
		return 0;
	}

	trigger_handler->trigger = *trig;

  	rslt = bma4_map_interrupt(map, int_map, BMA4_ENABLE, bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Error setting %04x map from int (%d)", int_map, rslt);
		return -EIO;
	}
	if (toggle_feature){
		rslt = bma456mm_feature_enable(feature,BMA4_ENABLE,bma);
		if (rslt!=BMA4_OK){
			LOG_ERR("Error enabling feature %02x. %d",feature,rslt);
			return -EIO;
		}	
	}
	LOG_WRN("Int1 pin %d",gpio_pin_get(cfg->gpio_int1.port,cfg->gpio_int1.pin));

	#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
		k_sem_give(&bma456->gpio_sem);
	#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
		k_work_submit(&bma456->work);
	#endif
	return 0;
}


// static int bma456_trigger_drdy_set(const struct device *dev,
// 				   const struct sensor_trigger *trig,
// 				   sensor_trigger_handler_t handler)
// {
// 	const struct bma456_config *cfg = dev->config;
// 	struct bma456_data *bma456 = dev->data;
// 	struct bma4_dev *bma = &bma456->bma;
// 	int8_t rslt;
// 	uint8_t map;

// 	if (bma456->int1_triggers & BMA456_TRIGGER_DATA_READY){
// 		if (cfg->gpio_int1.port == NULL) {
// 			LOG_ERR("trigger_set DRDY int not supported");
// 			return -ENOTSUP;
// 		}
// 		map = BMA4_INTR1_MAP;

// 	} else if (bma456->int2_triggers & BMA456_TRIGGER_DATA_READY){
// 		if (cfg->gpio_int2.port == NULL) {
// 			LOG_ERR("trigger_set DRDY int not supported");
// 			return -ENOTSUP;
// 		}
// 		map = BMA4_INTR2_MAP;

// 	} else {
// 		return -ENOTSUP;
// 	}

//    	rslt = bma4_map_interrupt(map, BMA4_DATA_RDY_INT, BMA4_DISABLE, &bma);
// 	if (rslt != BMA4_OK) {
// 		LOG_ERR("Error removing data rdy map from int1 (%d)", rslt);
// 		return -EIO;
// 	}


// 	bma456->handlers.data_ready.handler = handler;
// 	if (handler == NULL) {
// 		return 0;
// 	}


// 	bma456->handlers.data_ready.trigger = *trig;

//    	rslt = bma4_map_interrupt(map, BMA4_DATA_RDY_INT, BMA4_ENABLE, &bma);
// 	if (rslt != BMA4_OK) {
// 		LOG_ERR("Error removing data rdy map from int1 (%d)", rslt);
// 		return -EIO;
// 	}

// 	return 0;
// }

// static int bma456_trigger_anym_set(const struct device *dev,
//  					const struct sensor_trigger *trig,
// 				   	sensor_trigger_handler_t handler)
// {
// 	const struct bma456_config *cfg = dev->config;
// 	struct bma456_data *bma456 = dev->data;
// 	struct bma4_dev *bma = &bma456->bma;

// 	int8_t rslt;
// 	uint8_t map;

// 	if (bma456->int1_triggers & BMA456_TRIGGER_ANY_MOTION){
// 		if (cfg->gpio_int1.port == NULL) {
// 			LOG_ERR("trigger_set DRDY int not supported");
// 			return -ENOTSUP;
// 		}
// 		map = BMA4_INTR1_MAP;
// 	} else if (bma456->int2_triggers & BMA456_TRIGGER_ANY_MOTION){
// 		if (cfg->gpio_int2.port == NULL) {
// 			LOG_ERR("trigger_set DRDY int not supported");
// 			return -ENOTSUP;
// 		}
// 		map = BMA4_INTR2_MAP;

// 	} else {
// 		return -ENOTSUP;
// 	}


//    	rslt = bma4_map_interrupt(map, BMA456MM_ANY_MOT_INT, BMA4_DISABLE, &bma);
// 	if (rslt != BMA4_OK) {
// 		LOG_ERR("Error removing data rdy map from int1 (%d)", rslt);
// 		return -EIO;
// 	}

// 	bma456->handlers.any_motion.handler = handler;
// 	if (handler == NULL) {
// 		return 0;
// 	}
// 	bma456->handlers.any_motion.trigger = *trig;

//    	rslt = bma4_map_interrupt(map, BMA456MM_ANY_MOT_INT, BMA4_ENABLE, &bma);
// 	if (rslt != BMA4_OK) {
// 		LOG_ERR("Error removing data rdy map from int1 (%d)", rslt);
// 		return -EIO;
// 	}
// 	return 0;

// }

static int config_any_no_motion(const struct device* dev,const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler, bool is_any){
	struct bma456_data *bma456 = dev->data;	
	struct bma4_dev *bma = &bma456->bma;
	uint8_t axes;
	bool on=true;
	if (handler==NULL){
		on=false;
	}
	switch (trig->chan){
		case SENSOR_CHAN_ACCEL_XYZ:
			axes = BMA456MM_EN_ALL_AXIS;
			break;
		case SENSOR_CHAN_ACCEL_X:
			axes = BMA456MM_X_AXIS_EN;
			break;
		case SENSOR_CHAN_ACCEL_Y:
			axes = BMA456MM_Y_AXIS_EN;
			break;
		case SENSOR_CHAN_ACCEL_Z:
			axes = BMA456MM_Z_AXIS_EN;
			break;
		default:
			return -EINVAL;

	}

	bma456_trigger_t* trigger_handler;
	uint8_t bma_trigger;
	uint16_t int_map;
	if (is_any){
		trigger_handler = &bma456->handlers.any_motion;
		bma_trigger=BMA456_TRIGGER_ANY_MOTION;
		int_map = BMA456MM_ANY_MOT_INT;
	} else {
		trigger_handler = &bma456->handlers.no_motion;
		bma_trigger=BMA456_TRIGGER_NO_MOTION;
		int_map = BMA456MM_NO_MOT_INT;
	}
	int err = bma456_trigger_map_helper(dev,trigger_handler,bma_trigger,int_map,trig,handler);
	if (err!=0){
		return err;
	}

	int8_t rslt;
	struct bma456mm_any_no_mot_config any_no_mot = { 0 };
    /* Getting any-motion configuration to get default configuration */
	if (is_any){
    	rslt = bma456mm_get_any_mot_config(&any_no_mot, bma);
	} else {
		rslt = bma456mm_get_no_mot_config(&any_no_mot, bma);
	}
	if (rslt!=BMA4_OK){
		LOG_ERR("Error getting %s motion config %d",is_any?"any":"no", rslt);		
		return -EIO;
	}
	
	if (on){
		any_no_mot.axes_en |= axes;
		/*
         * Set the slope threshold:
         *  Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
         */
        any_no_mot.threshold = 10;
		/*
         * Set the duration for any-motion interrupt:
         *  Duration defines the number of consecutive data points for which threshold condition must be true(1
         * bit =
         * 20ms)
         */
        any_no_mot.duration = 4;

	} else {
		any_no_mot.axes_en &= ~axes;
	}
        

	/* Like threshold and duration, we can also change the config of int_bhvr and slope */

	/* Set the threshold and duration configuration */
	if (is_any){
		rslt = bma456mm_set_any_mot_config(&any_no_mot, bma);
	} else {
		rslt = bma456mm_set_no_mot_config(&any_no_mot, bma);
	}

	if (rslt!=BMA4_OK){
		LOG_ERR("Error setting %s motion config %d",is_any?"any":"no", rslt);		
		return -EIO;
	}

	#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
		k_sem_give(&bma456->gpio_sem);
	#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
		k_work_submit(&bma456->work);
	#endif
	return 0;
}


int bma456_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{	LOG_INF("Set trigger: %d",trig->type);
	// const struct bma456_config *cfg = dev->config;
	struct bma456_data *bma456 = dev->data;	
	switch (trig->type){
		case SENSOR_TRIG_DATA_READY:
			return bma456_trigger_set_helper(dev,
				&bma456->handlers.data_ready,
				BMA456_TRIGGER_DATA_READY,BMA4_DATA_RDY_INT,
			 	trig, handler, false,0);
		case SENSOR_TRIG_MOTION:
			return config_any_no_motion(dev,trig,handler,true);
			// return bma456_trigger_set_helper(dev,
			// 	&bma456->handlers.any_motion,
			// 	BMA456_TRIGGER_ANY_MOTION,BMA456MM_ANY_MOT_INT,
			//  	trig, handler, false,0);
		case SENSOR_TRIG_STATIONARY:
			return config_any_no_motion(dev,trig,handler,false);
			// return bma456_trigger_set_helper(dev,
			// 	&bma456->handlers.no_motion,
			// 	BMA456_TRIGGER_NO_MOTION,BMA456MM_NO_MOT_INT,
			//  	trig, handler, false,0);				
		case SENSOR_TRIG_TAP:
			return bma456_trigger_set_helper(dev,
				&bma456->handlers.tap,
				BMA456_TRIGGER_TAP,BMA456MM_TAP_OUT_INT,
			 	trig, handler,true,BMA456MM_SINGLE_TAP);
		case SENSOR_TRIG_DOUBLE_TAP:
			return bma456_trigger_set_helper(dev,
				&bma456->handlers.double_tap,
				BMA456_TRIGGER_DOUBLE_TAP,BMA456MM_TAP_OUT_INT,
			 	trig, handler, true,BMA456MM_DOUBLE_TAP);
		default:
		return -ENOTSUP;
	}
}

// int bma456_acc_slope_config(const struct device *dev,
// 			    enum sensor_attribute attr,
// 			    const struct sensor_value *val)
// {
// 	struct bma456_data *bma456 = dev->data;
// 	const struct bma456_config *cfg = dev->config;
// 	int status;

// 	if (attr == SENSOR_ATTR_SLOPE_TH) {
// 		uint8_t range_g, reg_val;
// 		uint32_t slope_th_ums2;

// 		status = bma456->hw_tf->read_reg(dev, BMA456_REG_CTRL4,
// 						 &reg_val);
// 		if (status < 0) {
// 			return status;
// 		}

// 		/* fs reg value is in the range 0 (2g) - 3 (16g) */
// 		range_g = 2 * (1 << ((BMA456_FS_MASK & reg_val)
// 				      >> BMA456_FS_SHIFT));

// 		slope_th_ums2 = val->val1 * 1000000 + val->val2;

// 		/* make sure the provided threshold does not exceed range */
// 		if ((slope_th_ums2 - 1) > (range_g * SENSOR_G)) {
// 			return -EINVAL;
// 		}

// 		/* 7 bit full range value */
// 		reg_val = 128 / range_g * (slope_th_ums2 - 1) / SENSOR_G;

// 		LOG_INF("int2_ths=0x%x range_g=%d ums2=%u", reg_val,
// 			    range_g, slope_th_ums2 - 1);

// 		status = bma456->hw_tf->write_reg(dev,
// 						  cfg->hw.anym_on_int1 ?
// 								BMA456_REG_INT1_THS :
// 								BMA456_REG_INT2_THS,
// 						  reg_val);
// 	} else { /* SENSOR_ATTR_SLOPE_DUR */
// 		/*
// 		 * slope duration is measured in number of samples:
// 		 * N/ODR where N is the register value
// 		 */
// 		if (val->val1 < 0 || val->val1 > 127) {
// 			return -ENOTSUP;
// 		}

// 		LOG_INF("int2_dur=0x%x", val->val1);

// 		status = bma456->hw_tf->write_reg(dev,
// 						  cfg->hw.anym_on_int1 ?
// 								BMA456_REG_INT1_DUR :
// 								BMA456_REG_INT2_DUR,
// 						  val->val1);
// 	}

// 	return status;
// }

static void bma456_gpio_int1_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct bma456_data *bma456 =
		CONTAINER_OF(cb, struct bma456_data, gpio_int1_cb);
	
	ARG_UNUSED(pins);
	// LOG_WRN("Callback1");

	atomic_set_bit(&bma456->trig_flags, TRIGGED_INT1);

	/* int is level triggered so disable until processed */
	setup_int1(bma456->dev, false);

#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_give(&bma456->gpio_sem);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bma456->work);
#endif
}

static void bma456_gpio_int2_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct bma456_data *bma456 =
		CONTAINER_OF(cb, struct bma456_data, gpio_int2_cb);

	ARG_UNUSED(pins);
	LOG_WRN("Callback2");
	atomic_set_bit(&bma456->trig_flags, TRIGGED_INT2);

	/* int is level triggered so disable until processed */
	setup_int2(bma456->dev, false);

#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_give(&bma456->gpio_sem);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&bma456->work);
#endif
}

static void bma456_thread_cb(const struct device *dev)
{
	struct bma456_data *bma456 = dev->data;
	const struct bma456_config *cfg = dev->config;
	struct bma4_dev *bma = &bma456->bma;

	bool int1=false;
	bool int2=false;
	int8_t rslt;
	// int status;

	// LOG_WRN("Thread");


	if (cfg->gpio_int1.port &&
			atomic_test_and_clear_bit(&bma456->trig_flags,
			TRIGGED_INT1)) {
				int1=true;
	}
	if (cfg->gpio_int2.port &&
			atomic_test_and_clear_bit(&bma456->trig_flags,
			TRIGGED_INT2)) {
				int2=true;
	}
	do {


		uint16_t int_status=0;
		if (int1 || int2){
			rslt = bma4_read_int_status(&int_status,bma);
			if (rslt!=BMA4_OK){
				LOG_WRN("Error reading int status %d",rslt);
				break;
			}
		}
		if (int_status & BMA4_ACCEL_DATA_RDY_INT){
			if (bma456->handlers.data_ready.handler!=NULL){
				struct sensor_trigger *drdy_trigger = &bma456->handlers.data_ready.trigger;
				bma456->handlers.data_ready.handler(dev,drdy_trigger);
			}
			//handle_data_ready
		}
		if (int_status & BMA456MM_TAP_OUT_INT){
 			struct bma456mm_out_state tap_out = { 0 };
			int rslt = bma456mm_output_state(&tap_out, bma);

			if (BMA4_OK == rslt){
				/* Enters only if the obtained interrupt is single-tap */
				if (tap_out.s_tap){
					//handle tap
					if (bma456->handlers.tap.handler!=NULL){
						struct sensor_trigger *tap_trigger = &bma456->handlers.tap.trigger;
						bma456->handlers.tap.handler(dev,tap_trigger);
					}
				}
				/* Enters only if the obtained interrupt is double-tap */
				else if (tap_out.d_tap){
					//handle double_tap
					if (bma456->handlers.double_tap.handler!=NULL){
						struct sensor_trigger *double_tap_trigger = &bma456->handlers.double_tap.trigger;;
						bma456->handlers.double_tap.handler(dev,double_tap_trigger);
					}
				}			
			} else {
				LOG_WRN("Eror trying to get output state %d",rslt);
			}

		}

		if (int_status & BMA456MM_ANY_MOT_INT){
			//Handle Any motion
			if (bma456->handlers.any_motion.handler!=NULL){
				struct sensor_trigger *any_motion_trigger = &bma456->handlers.any_motion.trigger;
				bma456->handlers.any_motion.handler(dev,any_motion_trigger);
			}
		}

                /* Enters only if the obtained interrupt is no-motion */
		if (int_status & BMA456MM_NO_MOT_INT){
			//Handle No motion
			if (bma456->handlers.no_motion.handler!=NULL){
				struct sensor_trigger *no_motion_trigger =  &bma456->handlers.no_motion.trigger;
				bma456->handlers.no_motion.handler(dev,no_motion_trigger);
			}
		}	

	} while (0);

	if (int1){
		LOG_INF("Enabling INT1 interrupt");
		setup_int1(dev,true);
		
	}
	if (int2){
		LOG_INF("Enabling INT2 interrupt");
		setup_int2(dev,true);
	}

}

#ifdef CONFIG_BMA456_TRIGGER_OWN_THREAD
static void bma456_thread(struct bma456_data *bma456)
{
	LOG_WRN("Thread start");
	while (1) {
		k_sem_take(&bma456->gpio_sem, K_FOREVER);
		bma456_thread_cb(bma456->dev);
	}
}
#endif

#ifdef CONFIG_BMA456_TRIGGER_GLOBAL_THREAD
static void bma456_work_cb(struct k_work *work)
{
	struct bma456_data *bma456 =
		CONTAINER_OF(work, struct bma456_data, work);

	bma456_thread_cb(bma456->dev);
}
#endif
static int configure_bma4_pin(const struct gpio_dt_spec* dt_specs, uint8_t int_line,struct bma4_dev *bma){
	uint8_t lvl;
	if (dt_specs->dt_flags & (GPIO_ACTIVE_LOW)) { 
		lvl=0;	//Using pull up or Active Low => pin should be configured as active low
	} else {
		lvl=1; //Any other combination should be configured as active high
	}

	uint8_t od;
	if (dt_specs->dt_flags & (GPIO_PULL_UP|GPIO_PULL_DOWN)) { 
		od=1;	//Using Open Drain
	} else {
		od=0; //Using push-pull
	}
	struct bma4_int_pin_config int_cfg = {
		.edge_ctrl=0, //Don't care, this is for input, we are using it for output
		.lvl = lvl,
		.od = od,
		.input_en=0,
		.output_en=1,
	};

	LOG_INF("int %d lvl: %d, od:%d",int_line,lvl,od);
	int8_t rslt = bma4_set_int_pin_config(&int_cfg,int_line,bma);	
	if (rslt != BMA4_OK) {
		LOG_ERR("Interrupt disable reg write failed (%d)", rslt);
		return -EIO;
	}	
	return 0;
}

int bma456_init_interrupt(const struct device *dev)
{
	struct bma456_data *bma456 = dev->data;
	bma456->dev=dev;
	const struct bma456_config *cfg = dev->config;
	struct bma4_dev *bma = &bma456->bma;
	int status;


	LOG_WRN("BOSH bma4_map_interrupt");
	/* disable interrupt in case of warm (re)boot */
	int8_t rslt = bma4_map_interrupt(BMA4_INTR1_MAP, 
									BMA4_FIFO_FULL_INT | 
									BMA4_FIFO_WM_INT | 
									BMA4_DATA_RDY_INT | 
									BMA4_MAG_DATA_RDY_INT | 
									BMA4_ACCEL_DATA_RDY_INT, 
									BMA4_DISABLE, bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Interrupt disable reg write failed (%d)", rslt);
		return -EIO;
	}
	
	
	/* set latched or non-latched interrupts */

	// rslt = bma4_set_interrupt_mode(cfg->hw.int_non_latched?BMA4_NON_LATCH_MODE:BMA4_LATCH_MODE,bma);
	// rslt = bma4_set_interrupt_mode(BMA4_NON_LATCH_MODE,bma);
	rslt = bma4_set_interrupt_mode(BMA4_LATCH_MODE,bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Setting Latch mode %d failed (%d)", cfg->hw.int_non_latched, rslt);
		return -EIO;
	}

	uint16_t int_status=0;
	rslt = bma4_read_int_status(&int_status,bma);
	if (rslt != BMA4_OK) {
		LOG_ERR("Reading int status failed (%d)", rslt);
		return -EIO;
	}
	LOG_WRN("Int Status:%04x", int_status);
	/*
	 * Setup INT1 (for DRDY) if defined in DT
	 */
	uint8_t int_pins=0;
	/* setup data ready gpio interrupt */
	if (!device_is_ready(cfg->gpio_int1.port)) {
		/* API may return false even when ptr is NULL */
		if (cfg->gpio_int1.port != NULL) {
			LOG_ERR("device %s is not ready", cfg->gpio_int1.port->name);
			return -ENODEV;
		}
		LOG_DBG("gpio_int1 not defined in DT");
	} else {
		
		/* data ready int1 gpio configuration */
		status = gpio_pin_configure_dt(&cfg->gpio_int1, GPIO_INPUT);
		if (status < 0) {
			LOG_ERR("Could not configure %s.%02u",
				cfg->gpio_int1.port->name, cfg->gpio_int1.pin);
			return status;
		}		
		LOG_WRN("Int1 pin %d",gpio_pin_get(cfg->gpio_int1.port,cfg->gpio_int1.pin));
		// gpio_pin_interrupt_configure_dt(&cfg->gpio_int1,GPIO_INT_EDGE_TO_ACTIVE);

		gpio_init_callback(&bma456->gpio_int1_cb,
				bma456_gpio_int1_callback,
				BIT(cfg->gpio_int1.pin));

		status = gpio_add_callback(cfg->gpio_int1.port, &bma456->gpio_int1_cb);
		if (status < 0) {
			LOG_ERR("Could not add gpio int1 callback");
			return status;
		}

		LOG_INF("%s: int1 on %s.%02u", dev->name,
						cfg->gpio_int1.port->name,
						cfg->gpio_int1.pin);
		int_pins|=1;
		LOG_WRN("Int1 pin %d",gpio_pin_get(cfg->gpio_int1.port,cfg->gpio_int1.pin));
		status = configure_bma4_pin(&cfg->gpio_int1,BMA4_INTR1_MAP,bma);
		if (status < 0) {
			LOG_ERR("Could not add gpio int1 callback");
			return status;
		}
		LOG_WRN("Int1 pin %d",gpio_pin_get(cfg->gpio_int1.port,cfg->gpio_int1.pin));
		setup_int1(dev,true);
		// bma456_gpio_int1_callback(dev,&bma456->gpio_int1_cb,BIT(cfg->gpio_int1.pin));
	}

	if (!device_is_ready(cfg->gpio_int2.port)) {
		/* API may return false even when ptr is NULL */
		if (cfg->gpio_int2.port != NULL) {
			LOG_ERR("device %s is not ready", cfg->gpio_int2.port->name);
			return -ENODEV;
		}
		LOG_DBG("gpio_int2 not defined in DT");
	} else {
		status = gpio_pin_configure_dt(&cfg->gpio_int2, GPIO_INPUT);
		if (status < 0) {
			LOG_ERR("Could not configure %s.%02u",
				cfg->gpio_int2.port->name, cfg->gpio_int2.pin);
			return status;
		}
		// gpio_pin_interrupt_configure_dt(&cfg->gpio_int2,GPIO_INT_EDGE_TO_ACTIVE);

		gpio_init_callback(&bma456->gpio_int2_cb,
				bma456_gpio_int2_callback,
				BIT(cfg->gpio_int2.pin));

		status = gpio_add_callback(cfg->gpio_int2.port, &bma456->gpio_int2_cb);
		if (status < 0) {
			LOG_ERR("Could not add gpio int2 callback");
			return status;
		}

		LOG_INF("%s: int2 on %s.%02u", dev->name,
						cfg->gpio_int2.port->name,
						cfg->gpio_int2.pin);
		int_pins|=2;	
		status = configure_bma4_pin(&cfg->gpio_int2,BMA4_INTR2_MAP,bma);
		if (status < 0) {
			LOG_ERR("Could not add gpio int1 callback");
			return status;
		}	
		setup_int2(dev,true);
	}


	if (int_pins==0){
		LOG_ERR("No int pin enabled");
		return -ENODEV;
	}

	bma456->int1_triggers = int1_triggers_array[int_pins];
	bma456->int2_triggers = int2_triggers_array[int_pins];



#if defined(CONFIG_BMA456_TRIGGER_OWN_THREAD)
	k_sem_init(&bma456->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&bma456->thread, bma456->thread_stack, CONFIG_BMA456_THREAD_STACK_SIZE,
			(k_thread_entry_t)bma456_thread, bma456, NULL, NULL,
			K_PRIO_COOP(CONFIG_BMA456_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_BMA456_TRIGGER_GLOBAL_THREAD)
	bma456->work.handler = bma456_work_cb;
#endif

	// if (int_pins&1){
	// 	setup_int1(dev,true);
	// } 
	// if (int_pins&2){
	// 	setup_int2(dev,true);
	// }
	return status;
}
