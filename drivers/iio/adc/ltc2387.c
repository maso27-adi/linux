// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Linear Technology LTC2387 ADC driver
 *
 * Copyright 2021 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define LTC2387_VREF		4096
#define LTC2387_T_CNVH		8
#define LTC2387_T_FIRSTCLK	70

#define KHz 1000
#define MHz (1000 * KHz)

/* Get the period (in fetoseconds) starting from the frequency (kilohertz) */
#define LTC2387_FREQ_TO_PERIOD(f) DIV_ROUND_CLOSEST(1000*MHz, (f / (1*KHz)))

#define LTC2378_CHAN(resolution)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = 32,				\
			.realbits = resolution,				\
		},							\
	}

enum ltc2387_lane_modes {
	ONE_LANE = 0,
	TWO_LANES = 1
};

enum ltc2387_id {
	ID_LTC2387_16 = 1,
	ID_LTC2387_18
};

const unsigned int ltc2387_resolutions[] = {
	[ID_LTC2387_16] = 16,
	[ID_LTC2387_18] = 18
};

const unsigned int ltc2387_testpatterns[][2] = {
	[ID_LTC2387_16] = {
		[ONE_LANE] = 0b1010000001111111,
		[TWO_LANES] = 0b1100110000111111
	},
	[ID_LTC2387_18] = {
		[ONE_LANE] = 0b101000000111111100,
		[TWO_LANES] = 0b110011000011111100
	}
};

struct ltc2387_dev {
	struct mutex		lock;
	// void __iomem		*interface;

	int			sampling_freq;
	unsigned int		resolution;
	unsigned int		vref_mv;
	unsigned int		id;
	enum ltc2387_lane_modes	lane_mode;
	struct gpio_desc 	*gpio_testpat;
	struct clk		*ref_clk;
	struct pwm_device	*clk_en;
	struct regulator	*vref;
	struct pwm_device	*cnv;
};

static int ltc2387_sampling_rates[] = {
	15*MHz, 10*MHz, 5*MHz, 1*MHz, 500*KHz, 250*KHz, 100*KHz
};

static const struct iio_chan_spec ltc2387_channels[] = {
	[ID_LTC2387_16] = LTC2378_CHAN(16),
	[ID_LTC2387_18] = LTC2378_CHAN(18)
};

static const struct of_device_id ltc2387_of_match[] = {
	{
		.compatible = "ltc2387-18",
		.data = (void *) ID_LTC2387_18
	},
	{
		.compatible = "ltc2387-16",
		.data = (void *) ID_LTC2387_16
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2387_of_match);

static ssize_t ltc2387_sampling_freq_avail(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	ssize_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(ltc2387_sampling_rates); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 ltc2387_sampling_rates[i]);
	buf[len - 1] = '\n';

	return len;
}
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(ltc2387_sampling_freq_avail);

static int ltc2387_set_conversion(struct ltc2387_dev *ltc, bool enabled)
{
	struct pwm_state clk_en_state, cnv_state;
	int ret;

	pwm_get_state(ltc->clk_en, &clk_en_state);
	pwm_get_state(ltc->cnv, &cnv_state);

	clk_en_state.enabled = enabled;
	ret = pwm_apply_state(ltc->clk_en, &clk_en_state);
	if (ret < 0)
		return ret;

	cnv_state.enabled = enabled;
	ret = pwm_apply_state(ltc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	return 0;
}

static int ltc2387_set_sampling_freq(struct ltc2387_dev *ltc, int freq)
{
	struct pwm_state clk_en_state, cnv_state;
	unsigned long ref_clk_rate;
	unsigned int tclken;
	int ret, clk_en_time;

	ref_clk_rate = clk_get_rate(ltc->ref_clk);

	/* Gate the active period of the clock (see page 10-13 for both LTC's) */
	if (ltc->lane_mode == TWO_LANES)
		clk_en_time = DIV_ROUND_DOWN_ULL(ltc->resolution, 4) - 1;
	else
		clk_en_time = DIV_ROUND_DOWN_ULL(ltc->resolution, 2) - 1;
	tclken = (LTC2387_FREQ_TO_PERIOD(ref_clk_rate) * clk_en_time) +
		 (LTC2387_FREQ_TO_PERIOD(ref_clk_rate) / 2);
	clk_en_state.period = LTC2387_FREQ_TO_PERIOD(freq) / 1000;
	clk_en_state.offset = clk_en_state.period - (tclken / 1000);
	clk_en_state.duty_cycle = tclken / 1000;

	ret = pwm_apply_state(ltc->clk_en, &clk_en_state);
	if (ret < 0)
		return ret;

	cnv_state.period = LTC2387_FREQ_TO_PERIOD(freq) / 1000;
	cnv_state.duty_cycle = LTC2387_T_CNVH;

	ret = pwm_apply_state(ltc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	ltc->sampling_freq = freq;

	return 0;
}
/*
static int ltc2387_calibrate_interface(struct iio_dev *indio_dev)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);
	char __user local_buffer[32];
	struct iio_buffer *buf;
	int ret, i, j, k;
	
	buf = iio_buffer_get(indio_dev->buffer);
	ret = iio_dma_buffer_set_length(buf, 4);
	if (ret < 0)
		return ret;

	ltc2387_set_conversion(ltc, true);
	for (i = 0; i < 32; i++) {
		for (j = 0; j < 32; j++) {
			iowrite32(j, ltc->interface + 0);
			iowrite32(j, ltc->interface + 4);
			ret = iio_dma_buffer_enable(buf, indio_dev);
			if (ret < 0)
				return ret;
			// do {
			while(iio_dma_buffer_data_available(buf) == 0) {}
			pr_err("\n[ltc]avail%d",iio_dma_buffer_data_available(buf));
				ret = iio_dma_buffer_read(buf, 32, local_buffer);
			// } while (ret == -EBUSY);
			// if (ret < 0)
			// 	return ret;
			ret = iio_dma_buffer_disable(buf, indio_dev);
			// if (ret < 0)
			// 	return ret;
			// for (k = 1; k < 4; k++){
			for (k = 0; k < 32; k++){
				
				if (((k-1)% 4) == 0 && k > 0){
					unsigned int temp = local_buffer[k-3];
					temp |= (unsigned int)(local_buffer[k-2] << 8);
					temp |= (unsigned int)(local_buffer[k-1] << 16);
					temp |= (unsigned int)(local_buffer[k] << 24);
					
					pr_err("\n[ltc] sampl%d 0x%x",k%4,temp);
				}

				// if (local_buffer[k] == local_buffer[k - 1] ||
				//     local_buffer[k] == ltc2387_testpatterns[ltc->id][ltc->lane_mode])
				//     return 0;
				
			}
		}
	// }
	ltc2387_set_conversion(ltc, false);
	return -EINVAL;
}
*/
static int ltc2387_setup(struct iio_dev *indio_dev)
{
	struct fwnode_handle *fwnode = dev_fwnode(indio_dev->dev.parent);
	struct ltc2387_dev *ltc = iio_priv(indio_dev);
	int ret;

	if (fwnode_property_present(fwnode, "adi,use-two-lanes"))
		ltc->lane_mode = TWO_LANES;
	
	// if (ltc->gpio_testpat) {
	// ret = ltc2387_calibrate_interface(indio_dev);
	// if (!ret)
	// 	return ret;
	// }

	ret =  ltc2387_set_sampling_freq(ltc, 15*MHz);
	if (ret < 0)
		return ret;

	return ltc2387_set_conversion(ltc, false);
}

static int ltc2387_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long info)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ltc->sampling_freq;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ltc2387_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ltc2387_set_sampling_freq(ltc, val);
	case IIO_CHAN_INFO_SCALE:

	default:
		return -EINVAL;
	}
}

static int ltc2387_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	return ltc2387_set_conversion(ltc, true);
}

static int ltc2387_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	return ltc2387_set_conversion(ltc, false);
}

static int ltc2387_dma_submit(struct iio_dma_buffer_queue *queue,
			      struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static void ltc2387_clk_diasble(void *data)
{
	pwm_disable(data);
}

static void ltc2387_cnv_diasble(void *data)
{
	pwm_disable(data);
}

static void ltc2387_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ltc2387_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static struct attribute *ltc2387_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ltc2387_group = {
	.attrs = ltc2387_attributes,
};

static const struct iio_dma_buffer_ops ltc2387_dma_buffer_ops = {
	.submit = ltc2387_dma_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ltc2387_buffer_ops = {
	.preenable = &ltc2387_buffer_preenable,
	.postdisable = &ltc2387_buffer_postdisable,
};

static const struct iio_info ltc2387_info = {
	.attrs = &ltc2387_group,
	.read_raw = ltc2387_read_raw,
	.write_raw = ltc2387_write_raw,
};

static int ltc2387_probe(struct platform_device *pdev)
{
	const struct of_device_id	*device_id;
	struct iio_dev			*indio_dev;
	struct iio_buffer		*buffer;
	struct ltc2387_dev		*ltc;
	int				ret;
	int				id;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct ltc2387_dev));
	if (!indio_dev)
		return -ENOMEM;

	ltc = iio_priv(indio_dev);

	// ltc->interface = devm_platform_ioremap_resource(pdev, 0);
	// if (IS_ERR(ltc->interface))
	// 	return PTR_ERR(ltc->interface);

	ltc->vref = devm_regulator_get_optional(&pdev->dev, "vref");
	if (!IS_ERR(ltc->vref)) {
		ret = regulator_enable(ltc->vref);
		if (ret) {
			dev_err(&pdev->dev, "Failed to enable vref regulator\n");
			return ret;
		}
		ret = regulator_get_voltage(ltc->vref);
		if (ret < 0)
			return ret;
		ltc->vref_mv = ret / 1000;
		ret = devm_add_action_or_reset(&pdev->dev, ltc2387_regulator_disable, ltc->vref);
		if (ret)
			return ret;
	} else {
		/* Internal vref is used */
		ltc->vref_mv = 4096;
	}

	ltc->gpio_testpat = devm_gpiod_get_optional(&pdev->dev, "testpat", GPIOD_OUT_HIGH);
	if (IS_ERR(ltc->gpio_testpat))
		return PTR_ERR(ltc->gpio_testpat);

	ltc->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ltc->ref_clk))
		return PTR_ERR(ltc->ref_clk);

	ret = clk_prepare_enable(ltc->ref_clk);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_clk_disable, ltc->ref_clk);
	if (ret)
		return ret;

	ltc->clk_en = devm_pwm_get(&pdev->dev, "clk_en");
	if (IS_ERR(ltc->clk_en))
		return PTR_ERR(ltc->clk_en);
	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_clk_diasble, ltc->clk_en);

	ltc->cnv = devm_pwm_get(&pdev->dev, "cnv");
	if (IS_ERR(ltc->cnv))
		return PTR_ERR(ltc->cnv);
	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_cnv_diasble, ltc->cnv);

	device_id = of_match_device(of_match_ptr(ltc2387_of_match), &pdev->dev);
	id = (int) device_id->data;

	ltc->id = id;
	ltc->resolution = ltc2387_resolutions[id];

	indio_dev->channels = &ltc2387_channels[id];
	indio_dev->num_channels = 1;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "ltc2387";
	indio_dev->info = &ltc2387_info;
	indio_dev->setup_ops = &ltc2387_buffer_ops;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						"rx",
						&ltc2387_dma_buffer_ops,
						indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = ltc2387_setup(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "\nltc2387 setup failed\n");
		return ret;
	}

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver ltc2387_driver = {
	.probe          = ltc2387_probe,
	.driver         = {
		.name   = "ltc2387",
		.of_match_table = of_match_ptr(ltc2387_of_match),
	},
};
module_platform_driver(ltc2387_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Linear Technology LTC2387 ADC");
MODULE_LICENSE("Dual BSD/GPL");
