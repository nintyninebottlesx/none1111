#ifndef ZEPHYR_DRIVERS_SENSOR_ADS1231_MISC_H_
#define ZEPHYR_DRIVERS_SENSOR_ADS1231_MISC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>
#include <stdbool.h>

void ADS1231_initBus(void);
void ADS1231_enterStandByMode(void);
void ADS1231_leaveStandByMode(void);
uint32_t ADS1231_getBits(void);
int32_t convertTwosToOnesComplement(uint32_t data);
void ADS1231_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif

