
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <stdint.h>
#include "ads1231.h"
#include "ads1231_misc.h"

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>

#define DT_DRV_COMPAT ti_ads1231

LOG_MODULE_REGISTER(ADS1231, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define MCP320X_RESOLUTION 24U

struct ads1231_config {
	struct spi_dt_spec bus;
	uint8_t channels;
};

struct ads1231_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	uint8_t differential;
	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack,
			CONFIG_ADC_ADS1231_ACQUISITION_THREAD_STACK_SIZE);
};


//----------------------------

void ADS1231_init(void)
{

    ADS1231_initBus();
    ADS1231_enterStandByMode();
}


int32_t ADS1231_getData(void) {
    ADS1231_leaveStandByMode();
    uint32_t data;
    data = ADS1231_getBits();
    ADS1231_enterStandByMode();
    return convertTwosToOnesComplement(data);
}

//------------------------------



static int ads1231_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct ads1231_config *config = dev->config;
	struct ads1231_data *data = dev->data;

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_EXTERNAL0) {
		LOG_ERR("unsupported channel reference '%d'",
			channel_cfg->reference);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'",
			channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	WRITE_BIT(data->differential, channel_cfg->channel_id,
		  channel_cfg->differential);

	return 0;
}

static int ads1231_validate_buffer_size(const struct device *dev,
					const struct adc_sequence *sequence)
{
	const struct ads1231_config *config = dev->config;
	uint8_t channels = 0;
	size_t needed;
	uint32_t mask;

	for (mask = BIT(config->channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(uint16_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ads1231_start_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct ads1231_config *config = dev->config;
	struct ads1231_data *data = dev->data;
	int err;

	if (sequence->resolution != ADS1231_RESOLUTION) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x",
			sequence->channels);
		return -ENOTSUP;
	}

	err = mcp320x_validate_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int ads1231_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct ads1231_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = ads1231_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int ads1231_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	return ads1231_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads1231_data *data = CONTAINER_OF(ctx, struct ads1231_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct ads1231_data *data = CONTAINER_OF(ctx, struct mcp320x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

/*
static int mcp320x_read_channel(const struct device *dev, uint8_t channel,
				uint16_t *result)
{
	const struct mcp320x_config *config = dev->config;
	struct mcp320x_data *data = dev->data;
	uint8_t tx_bytes[2];
	uint8_t rx_bytes[2];
	int err;
	const struct spi_buf tx_buf[2] = {
		{
			.buf = tx_bytes,
			.len = sizeof(tx_bytes)
		},
		{
			.buf = NULL,
			.len = 1
		}
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1
		},
		{
			.buf = rx_bytes,
			.len = sizeof(rx_bytes)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf)
	};
	
		//
	 // Configuration bits consists of: 5 dummy bits + start bit +
	 // SGL/#DIFF bit + D2 + D1 + D0 + 6 dummy bits
	 //
	tx_bytes[0] = BIT(2) | channel >> 2;
	tx_bytes[1] = channel << 6;

	if ((data->differential & BIT(channel)) == 0) {
		tx_bytes[0] |= BIT(1);
	}

	err = spi_transceive_dt(&config->bus, &tx, &rx);
	if (err) {
		return err;
	}

	*result = sys_get_be16(rx_bytes);
	*result &= BIT_MASK(MCP320X_RESOLUTION);

	return 0;
}

*/

static void ads1231_acquisition_thread(struct ads1231_data *data)
{
	uint16_t result = 0;
	uint8_t channel;
	int err;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		while (data->channels) {
			channel = find_lsb_set(data->channels) - 1;

			LOG_DBG("reading channel %d", channel);

		//	err = ads1231_read_channel(data->dev, channel, &result);
		//	if (err) {
		//		LOG_ERR("failed to read channel %d (err %d)",
		//			channel, err);
		//		adc_context_complete(&data->ctx, err);
		//		break;
		//	}

			LOG_DBG("read channel %d, result = %d", channel,
				result);

			*data->buffer++ = result;
			WRITE_BIT(data->channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}


static int ads1231_init(const struct device *dev)
{
	const struct ads1231_config *config = dev->config;
	struct ads1231_data *data = dev->data;

	data->dev = dev;

	k_sem_init(&data->sem, 0, 1);

	if (!spi_is_ready(&config->bus)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	k_thread_create(&data->thread, data->stack,
			CONFIG_ADC_ADS1231_ACQUISITION_THREAD_STACK_SIZE,
			(k_thread_entry_t)ads1231_acquisition_thread,
			data, NULL, NULL,
			CONFIG_ADC_ADS1231_ACQUISITION_THREAD_PRIO,
			0, K_NO_WAIT);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api ads1231_adc_api = {
	.channel_setup = ads1231_channel_setup,
	.read = ads1231_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads1231_read_async,
#endif
};

#define INST_DT_ADS1231(inst, t) DT_INST(inst, microchip_mcp##t)

#define ADS1231_DEVICE(t, n, ch) \
	static struct ads1231_data mcp##t##_data_##n = { \
		ADC_CONTEXT_INIT_TIMER(mcp##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_LOCK(mcp##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_SYNC(mcp##t##_data_##n, ctx), \
	}; \
	static const struct ads1231_config mcp##t##_config_##n = { \
		.bus = SPI_DT_SPEC_GET(INST_DT_ADS1231(n, t), \
					 SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
					 SPI_WORD_SET(8), 0), \
		.channels = ch, \
	}; \
	DEVICE_DT_DEFINE(INST_DT_ADS1231(n, t), \
			 &ads1231_init, NULL, \
			 &mcp##t##_data_##n, \
			 &mcp##t##_config_##n, POST_KERNEL, \
			 CONFIG_ADC_INIT_PRIORITY, \
			 &ads1231_adc_api)

/*
 * ADS1231: 1 channels
 */
#define ADS1231_DEVICE(n) ADS1231_DEVICE(1231, n, 1)

#define CALL_WITH_ARG(arg, expr) expr(arg)

#define INST_DT_ADS1231_FOREACH(t, inst_expr)			\
	LISTIFY(DT_NUM_INST_STATUS_OKAY(microchip_mcp##t),	\
		CALL_WITH_ARG, (;), inst_expr)

INST_DT_ADS1231_FOREACH(1231, ADS1231_DEVICE);






/**
 * @brief Set HX711 attributes.
 *
 * @param dev Pointer to the hx711 device structure
 * @param chan Channel to set.
 *             Supported channels :
 *               HX711_SENSOR_CHAN_WEIGHT
 * @param attr Attribute to change.
 *             Supported attributes :
 *               SENSOR_ATTR_SAMPLING_FREQUENCY
 *               SENSOR_ATTR_OFFSET
 *               HX711_SENSOR_ATTR_SLOPE
 *               HX711_SENSOR_ATTR_GAIN
 * @param val   Value to set.
 * @retval 0 on success
 * @retval -ENOTSUP if an invalid attribute is given
 *
 */
 /*
static int hx711_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;
	int hx711_attr = (int)attr;
	struct hx711_data *data = dev->data;

	switch (hx711_attr) {

	case SENSOR_ATTR_OFFSET:
		hx711_attr_set_offset(data, val);
		LOG_DBG("Attribute OFFSET set to %d\n", data->offset);
		return ret;
	case HX711_SENSOR_ATTR_SLOPE:
		hx711_attr_set_slope(data, val);
		LOG_DBG("Attribute SLOPE set to %d.%d\n", data->slope.val1, data->slope.val2);
		return ret;
	case HX711_SENSOR_ATTR_GAIN:
		ret = hx711_attr_set_gain(dev, val);
		if (ret == 0) {
			LOG_DBG("Attribute GAIN set to %d\n", data->gain);
		}
		return ret;
	default:
		return -ENOTSUP;
	}
}


static const struct sensor_driver_api hx711_api = {
	.sample_fetch = hx711_sample_fetch,
	.channel_get = hx711_channel_get,
	.attr_set = hx711_attr_set,
};
*/
