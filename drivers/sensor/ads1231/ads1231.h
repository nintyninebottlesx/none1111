/*
 * Copyright (c) 2020 George Gkinis
 * Copyright (c) 2022 Jan Gnip
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADS1231_H_
#define ZEPHYR_DRIVERS_SENSOR_ADS1231_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

/* Additional custom attributes */
enum ads1231_attribute {
	ADS1231_SENSOR_ATTR_SLOPE = SENSOR_ATTR_PRIV_START,
	ADS1231_SENSOR_ATTR_GAIN  = SENSOR_ATTR_PRIV_START + 1,
};

enum ads1231_channel {
	ADS1231_SENSOR_CHAN_WEIGHT = SENSOR_CHAN_PRIV_START,
};

enum ads1231_gain {
	ADS1231_GAIN_128X = 1,
};

enum ads1231_rate {
	ADS1231_RATE_10HZ,
	ADS1231_RATE_80HZ,
};

enum ads1231_power {
	ADS1231_POWER_ON,
	ADS1231_POWER_OFF,
};

struct ads1231_data {
	const struct device *dev;
	const struct device *dout_gpio;
	const struct device *sck_gpio;
	const struct device *rate_gpio;
	struct gpio_callback dout_gpio_cb;
	struct k_sem dout_sem;

	int32_t reading;

	int offset;
	struct sensor_value slope;
	char gain;
	enum ads1231_rate rate;
	enum ads1231_power power;
};

struct ads1231_config {
	gpio_pin_t dout_pin;
	const struct device *dout_ctrl;
	gpio_dt_flags_t dout_flags;

	gpio_pin_t sck_pin;
	const struct device *sck_ctrl;
	gpio_dt_flags_t sck_flags;

	gpio_pin_t rate_pin;
	const struct device *rate_ctrl;
	gpio_dt_flags_t rate_flags;
};

/**
 * @brief Zero the ADS1231.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param readings Number of readings to get average offset.
 *        5~10 readings should be enough, although more are allowed.
 * @retval The offset value
 *
 */
int ti_ads1231_tare(const struct device *dev, uint8_t readings);

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
struct sensor_value ti_ads1231_calibrate(const struct device *dev,
					 uint32_t target,
					 uint8_t readings);

/**
 * @brief Set the ADS1231 power.
 *
 * @param dev Pointer to the ads1231 device structure
 * @param power one of ADS1231_POWER_OFF, ADS1231_POWER_ON
 * @retval The current power state or ENOTSUP if an invalid value pow is given
 *
 */
int ti_ads1231_power(const struct device *dev, enum ads1231_power power);

#ifdef __cplusplus
}
#endif

#endif
