/*
 * Copyright (c) 2020 George Gkinis
 * Copyright (c) 2022 Jan Gnip
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ads1231
#include <zephyr/pm/device.h> 
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ads1231.h"

#define SAMPLE_FETCH_TIMEOUT_MS 1200
//600

LOG_MODULE_REGISTER(ADS1231, CONFIG_SENSOR_LOG_LEVEL);

static struct ads1231_data ads1231_data = {
	.reading = 0,
	.offset = CONFIG_ADS1231_OFFSET,
	.slope = {
		.val1 = CONFIG_ADS1231_SLOPE_INTEGER,
		.val2 = CONFIG_ADS1231_SLOPE_DECIMAL,
	},
	.gain = CONFIG_ADS1231_GAIN,
	.rate = CONFIG_ADS1231_SAMPLING_RATE,
	.power = ADS1231_POWER_ON,
};

static const struct ads1231_config ads1231_config = {
	.dout_pin = DT_INST_GPIO_PIN(0, dout_gpios),
	.dout_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(0), dout_gpios)),
	.dout_flags = DT_INST_GPIO_FLAGS(0, dout_gpios),

	.sck_pin = DT_INST_GPIO_PIN(0, sck_gpios),
	.sck_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(0), sck_gpios)),
	.sck_flags = DT_INST_GPIO_FLAGS(0, sck_gpios),

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	.rate_pin = DT_INST_GPIO_PIN(0, rate_gpios),
	.rate_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(0), rate_gpios)),
	.rate_flags = DT_INST_GPIO_FLAGS(0, rate_gpios),
#endif

};

static void ads1231_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct ads1231_data *data = CONTAINER_OF(cb, struct ads1231_data, dout_gpio_cb);
	const struct ads1231_config *cfg = data->dev->config;

	gpio_pin_interrupt_configure(data->dout_gpio, cfg->dout_pin, GPIO_INT_DISABLE);

	/* Signal thread that data is now ready */
	k_sem_give(&data->dout_sem);
}

/**
 * @brief Send a pulse on the SCK pin.
 *
 * @param data Pointer to the ads1231 driver data structure
 *
 * @retval The value of the DOUT pin (HIGH or LOW)
 *
 */
static int ads1231_cycle(struct ads1231_data *data)
{
	/* SCK set HIGH */
	gpio_pin_set(data->sck_gpio, ads1231_config.sck_pin, true);
	k_busy_wait(1);

	/* SCK set LOW */
	gpio_pin_set(data->sck_gpio, ads1231_config.sck_pin, false);
	k_busy_wait(1);

	/* Return DOUT pin state */
	return gpio_pin_get(data->dout_gpio, ads1231_config.dout_pin);
}

/**
 * @brief Read ADS1231 data. Also sets GAIN for the next cycle.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param chan Channel to fetch data for.
 *             Only ADS1231_SENSOR_CHAN_WEIGHT is available.
 *
 * @retval 0 on success,
 * @retval -EACCES error if module is not powered up.
 * @retval -EIO error if SAMPLE_FETCH_TIMEOUT_MS elapsed with no data available.
 *
 */
static int ads1231_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = 0;
	uint32_t count = 0;
	int i;

	struct ads1231_data *data = dev->data;
	const struct ads1231_config *cfg = dev->config;

	if (data->power != ADS1231_POWER_ON) {
		return -EACCES;
	}

	if (k_sem_take(&data->dout_sem, K_MSEC(SAMPLE_FETCH_TIMEOUT_MS))) {
		LOG_ERR("Weight data not ready within %d ms", SAMPLE_FETCH_TIMEOUT_MS);
		ret = -EIO;
		goto exit;
	}

	/* Clock data out. Optionally disable interrupts */
#ifdef CONFIG_ADS1231_DISABLE_INTERRUPTS_WHILE_POLLING
	uint32_t key = irq_lock();
#endif
	for (i = 0; i < 24; i++) {
		count = count << 1;
		if (ads1231_cycle(data)) {
			count++;
		}
	}

//	/* set GAIN for next read */
////	for (i = 0; i < data->gain; i++) {
		ads1231_cycle(data);
////	}

#ifdef CONFIG_ADS1231_DISABLE_INTERRUPTS_WHILE_POLLING
	irq_unlock(key);
#endif

	count ^= 0x800000;

	data->reading = count;

exit:
	ret = gpio_pin_interrupt_configure(data->dout_gpio, cfg->dout_pin, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to set dout GPIO interrupt");
	}

	return ret;
}

/**
 * @brief Set ADS1231 gain.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param val sensor_value struct. Only val1 is used.
 *            valid values are :
 *                ADS1231_GAIN_128X (default),
 *
 * @retval 0 on success,
 * @retval -ENOTSUP error if an invalid GAIN is provided or
 *         -EACCES if 600000 usec's are elapsed with no data available.
 *
 */
static int ads1231_attr_set_gain(const struct device *dev, const struct sensor_value *val)
{
	struct ads1231_data *data = dev->data;

	switch (val->val1) {
	case ADS1231_GAIN_128X:
		data->gain = ADS1231_GAIN_128X;
		break;
	default:
		return -ENOTSUP;
	}
	return ads1231_sample_fetch(dev, ADS1231_SENSOR_CHAN_WEIGHT);
}

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
/**
 * @brief Set ADS1231 rate.
 *
 * @param data Pointer to the ads1231 driver data structure
 * @param val sensor_value struct. Only val1 is used.
 *            valid values are :
 *               ADS1231_RATE_10HZ (default),
 *               ADS1231_RATE_80HZ
 *
 * @retval 0 on success,
 * @retval -EINVAL error if it fails to get RATE device.
 * @retval -ENOTSUP error if an invalid rate value is passed.
 *
 */
static int ads1231_attr_set_rate(struct ads1231_data *data, const struct sensor_value *val)
{
	int ret;

	switch (val->val1) {
	case ADS1231_RATE_10HZ:
	case ADS1231_RATE_80HZ:
		if (data->rate_gpio == NULL) {
			LOG_ERR("Failed to get pointer to RATE device");
			return -EINVAL;
		}
		data->rate = val->val1;
		ret = gpio_pin_set(data->rate_gpio, ads1231_config.rate_pin, data->rate);
		return ret;
	default:
		return -ENOTSUP;
	}
}
#endif

/**
 * @brief Set ADS1231 offset.
 *
 * @param data Pointer to the ads1231 driver data structure
 * @param offset sensor_value struct. Offset used to calculate the real weight.
 *        Formula is :
 *          weight = slope * (reading - offset)
 *          Only val1 is used.
 *
 * @return void
 *
 */
static void ads1231_attr_set_offset(struct ads1231_data *data, const struct sensor_value *offset)
{
	data->offset = offset->val1;
}

/**
 * @brief Set ADS1231 slope.
 *
 * @param data Pointer to the ads1231 driver data structure
 * @param slope sensor_value struct. Slope used to calculate the real weight.
 *        Formula is :
 *          weight = slope * (reading - offset)
 *
 * @return void
 *
 */
static void ads1231_attr_set_slope(struct ads1231_data *data, const struct sensor_value *slope)
{
	data->slope.val1 = slope->val1;
	data->slope.val2 = slope->val2;
}

/**
 * @brief Set ADS1231 attributes.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param chan Channel to set.
 *             Supported channels :
 *               ADS1231_SENSOR_CHAN_WEIGHT
 * @param attr Attribute to change.
 *             Supported attributes :
 *               SENSOR_ATTR_SAMPLING_FREQUENCY
 *               SENSOR_ATTR_OFFSET
 *               ADS1231_SENSOR_ATTR_SLOPE
 *               ADS1231_SENSOR_ATTR_GAIN
 * @param val   Value to set.
 * @retval 0 on success
 * @retval -ENOTSUP if an invalid attribute is given
 *
 */
static int ads1231_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;
	int ads1231_attr = (int)attr;
	struct ads1231_data *data = dev->data;

	switch (ads1231_attr) {
#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = ads1231_attr_set_rate(data, val);
		if (ret == 0) {
			LOG_ERR("Attribute RATE set to %d\n", data->rate);  //DBG
		}
		return ret;
#endif
	case SENSOR_ATTR_OFFSET:
		ads1231_attr_set_offset(data, val);
		LOG_DBG("Attribute OFFSET set to %d\n", data->offset);
		return ret;
	case ADS1231_SENSOR_ATTR_SLOPE:
		ads1231_attr_set_slope(data, val);
		LOG_DBG("Attribute SLOPE set to %d.%d\n", data->slope.val1, data->slope.val2);
		return ret;
	case ADS1231_SENSOR_ATTR_GAIN:
		ret = ads1231_attr_set_gain(dev, val);
		if (ret == 0) {
			LOG_DBG("Attribute GAIN set to %d\n", data->gain);
		}
		return ret;
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Get ADS1231 reading.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param chan Channel to set.
 *             Supported channels :
 *               ADS1231_SENSOR_CHAN_WEIGHT
 *
 * @param val  Value to write weight value to.
 *        Formula is :
 *          weight = slope * (reading - offset)
 *
 * @retval 0 on success
 * @retval  -ENOTSUP if an invalid channel is given
 *
 */
static int ads1231_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	enum ads1231_channel ads1231_chan = (enum ads1231_channel)chan;
	struct ads1231_data *data = dev->data;

	switch (ads1231_chan) {
	case ADS1231_SENSOR_CHAN_WEIGHT: {
		val->val1 = sensor_value_to_double(&data->slope) * (data->reading - data->offset);
		return 0;
	}
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Initialise ADS1231.
 *
 * @param dev Pointer to the ads1231 device structure
 *
 * @retval 0 on success
 * @retval -EINVAL if an invalid argument is given
 *
 */
static int ads1231_init(const struct device *dev)
{
	LOG_DBG("Initialising ADS1231\n"); 
	//DBG

	int ret = 0;
	struct ads1231_data *data = dev->data;
	const struct ads1231_config *cfg = dev->config;

	LOG_DBG("SCK GPIO port : %s\n", cfg->sck_ctrl->name);
	LOG_DBG("SCK Pin : %d\n", cfg->sck_pin);
	LOG_DBG("DOUT GPIO port : %s\n", cfg->dout_ctrl->name);
	LOG_DBG("DOUT Pin : %d\n", cfg->dout_pin);

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	LOG_ERR("RATE GPIO port : %s\n", cfg->rate_ctrl->name);
	LOG_ERR("RATE Pin : %d\n", cfg->rate_pin);
#endif

	LOG_DBG("Gain : %d\n", data->gain);
	LOG_DBG("Offset : %d\n", data->offset);
	LOG_DBG("Slope : %d.%d\n", data->slope.val1, data->slope.val2);

	/* Configure SCK as output, LOW */
	data->sck_gpio = cfg->sck_ctrl;
	LOG_ERR("SCK pin controller is %p, name is %s\n", data->sck_gpio, data->sck_gpio->name); //DBG

	ret = gpio_pin_configure(data->sck_gpio, cfg->sck_pin,
				 GPIO_OUTPUT_INACTIVE | cfg->sck_flags);
	if (ret != 0) {
		return ret;
	}

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	/* Configure RATE as output, LOW */
	data->rate_gpio = device_get_binding(cfg->rate_ctrl->name);
	if (data->rate_gpio == NULL) {
		LOG_ERR("Failed to get GPIO device %s.", cfg->rate_ctrl->name);
		return -EINVAL;
	}
	LOG_ERR("RATE pin controller is %p, name is %s\n", data->rate_gpio, data->rate_gpio->name); //DBG
	ret = gpio_pin_configure(data->rate_gpio, cfg->rate_pin,
				 GPIO_OUTPUT_INACTIVE | cfg->rate_flags);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_set(data->rate_gpio, ads1231_config.rate_pin, CONFIG_ADS1231_SAMPLING_RATE);
	if (ret != 0) {
		return ret;
	}
#endif

	k_sem_init(&data->dout_sem, 1, 1);

	/* Configure DOUT as input */
	data->dout_gpio = cfg->sck_ctrl;
	LOG_ERR("DOUT pin controller is %p, name is %s\n", data->dout_gpio, data->dout_gpio->name);  //DBG
	ret = gpio_pin_configure(data->dout_gpio, cfg->dout_pin, GPIO_INPUT | cfg->dout_flags);
	if (ret != 0) {
		return ret;
	}
	LOG_ERR("Set DOUT pin : %d\n", cfg->dout_pin);  //DBG

	gpio_init_callback(&data->dout_gpio_cb, ads1231_gpio_callback, BIT(cfg->dout_pin));

	if (gpio_add_callback(data->dout_gpio, &data->dout_gpio_cb) < 0) {
		LOG_DBG("Failed to set GPIO callback");
		return -EIO;
	}

	ret = gpio_pin_interrupt_configure(data->dout_gpio, cfg->dout_pin,
					   GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to set dout GPIO interrupt");
		return ret;
	}

	data->dev = dev;

	return ret;
}

/**
 * @brief Zero the ADS1231.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param readings Number of readings to get average offset.
 *        5~10 readings should be enough, although more are allowed.
 * @retval The offset value
 *
 */
int ti_ads1231_tare(const struct device *dev, uint8_t readings)
{
	int32_t avg = 0;
	struct ads1231_data *data = dev->data;
	int readings_avg = readings - 5;
	int throw;

	if (readings == 0) {
		readings = 1;
	}

	for (int i = 0; i < readings; i++) {
		ads1231_sample_fetch(dev, ADS1231_SENSOR_CHAN_WEIGHT);
		if(i < 5){   //added
		throw += data->reading;
		}else{
		avg += data->reading;
		LOG_INF("#A##Average AS PRINTED : %d", avg);
		}
	}
	LOG_DBG("Average before division : %d", avg);
	avg = avg / readings_avg;
	LOG_DBG("Average after division : %d", avg);
	data->offset = avg;

	return data->offset;
}

/**
 * @brief Callibrate the ADS1231.
 *
 * Given a target value of a known weight the slope gets calculated.
 * This is actually unit agnostic.
 * If the target weight is given in grams, lb, Kg or any other weight unit,
 * the slope will be calculated accordingly.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param target Target weight in grams.
 *        If target is represented in another unit (lb, oz, Kg) then the
 *        value returned by sensor_channel_get() will represent that unit.
 * @param readings Number of readings to take for calibration.
 *        5~10 readings should be enough, although more are allowed.
 * @retval The slope value
 *
 */
struct sensor_value ti_ads1231_calibrate(const struct device *dev, uint32_t target,
					 uint8_t readings)
{
	int ret_inc = 0;
	int32_t avg = 0;
	struct ads1231_data *data = dev->data;
	int throw;
	int readings_avg = readings - 5;

	if (readings == 0) {
		readings = 1;
	}

	for (int i = 0; i < readings; i++) {
		//ret_inc = 
		ads1231_sample_fetch(dev, ADS1231_SENSOR_CHAN_WEIGHT);
		//if(ret_inc != 0){
		//	ret_inc++;
		//}
		if(i < 5){   //added
		throw += data->reading;
		}else{
		avg += data->reading;
		LOG_INF("#B##Average AS PRINTED : %d", avg);
		}
	}
	LOG_DBG("Average before division : %d", avg);
	avg = avg / readings_avg;

	LOG_DBG("Average after division : %d", avg);
	double slope = (double)target / (double)(avg - data->offset);

	data->slope.val1 = (int)slope;
	data->slope.val2 = (slope - data->slope.val1) * 1e6;

	LOG_ERR("Slope set to : %d.%06d", data->slope.val1, data->slope.val2);  //DBG

	return data->slope;
}

/**
 * @brief Set the ADS1231 power.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param power one of ADS1231_POWER_OFF, ADS1231_POWER_ON
 * @retval The current power state or ENOTSUP if an invalid value pow is given
 *
 */
int ti_ads1231_power(const struct device *dev, enum ads1231_power pow)
{
	int ret;
	struct ads1231_data *data = dev->data;

	data->power = pow;
	switch (pow) {
	case ADS1231_POWER_ON:
	
	//data->sck_gpio = cfg->sck_ctrl; //added set low
	////ret = gpio_pin_set(data->sck_gpio, ads1231_config.sck_pin, false);
		ret = gpio_pin_set(data->sck_gpio, ads1231_config.sck_pin, data->power);
		/* Fetch a sample to set GAIN again.
		 * GAIN is set to 128 channel A after RESET
		 */
		ads1231_sample_fetch(dev, ADS1231_SENSOR_CHAN_WEIGHT);
		return ret;
	case ADS1231_POWER_OFF:
		ret = gpio_pin_set(data->sck_gpio, ads1231_config.sck_pin, data->power);
	////	ret = ret = gpio_pin_set(data->sck_gpio, ads1231_config.sck_pin, true);
		return ret;
	default:
		return -ENOTSUP;
	}
}

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Set the Device Power Management State.
 *
 * @param dev - The device structure.
 * @param action - power management state
 * @retval 0 on success
 * @retval -ENOTSUP if an unsupported action is given
 *
 */
int ads1231_pm_ctrl(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = ti_ads1231_power(dev, ADS1231_POWER_ON);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		ret = ti_ads1231_power(dev, ADS1231_POWER_OFF);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sensor_driver_api ads1231_api = {
	.sample_fetch = ads1231_sample_fetch,
	.channel_get = ads1231_channel_get,
	.attr_set = ads1231_attr_set,
};

DEVICE_DT_INST_DEFINE(0, ads1231_init, ads1231_pm_ctrl, &ads1231_data, &ads1231_config, POST_KERNEL,
		      CONFIG_SENSOR_INIT_PRIORITY, &ads1231_api);
