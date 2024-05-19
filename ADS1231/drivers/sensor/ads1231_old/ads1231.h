#ifndef ZEPHYR_DRIVERS_SENSOR_ADS1231_H_
#define ZEPHYR_DRIVERS_SENSOR_ADS1231_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>

/* 
 * IMPORTANT!!!
 * init() must be called prior to any other routine related with `ADS1231`
 * This routine is blocking for ~100 ms
 * After executing `ADS1231` module is pushed in STANDBY mode
 */
void ADS1231_init(void);

/*
 * This routine is blocking for more than ~500 ms
 * This time includes:
 *  - pushing `ADS1231` out of STANDBY mode
 *  - internal `ADS1231` data conversion time
 *  - data fetch from `ADS1231`
 *  - pushing `ADS1231` in STANDBY mode
 * NOTE:
 *  If RTOS is used timings in delay loop may be inacurate
 *  thus higher delay may occur.
 * 
 * Data is returned in signed integer.
 */
int32_t ADS1231_getData(void);  // value in range -8388608 <= x <= 8388607


#ifdef __cplusplus
}
#endif

#endif
