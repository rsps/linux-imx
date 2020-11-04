/*
 * LTC2606 Digital to analog convertors i2c driver
 *
 * Copyright 2018 Markku Nivala, Innokas Medical
 *
 * Licensed under the GPL.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <stdbool.h>

#define LTC2606_DAC_CHANNELS                    1
#define LTC2606_CMD_WRITE_INPUT                 0x0
#define LTC2606_CMD_UPDATE_DAC                  0x1
#define LTC2606_CMD_WRITE_INPUT_UPDATE          0x3
#define LTC2606_CMD_POWERDOWN_DAC               0x4
#define LTC2606_CMD_NOP                         0xf

#define LTC2606_MIDPOINT                        32768
#define LTC2606_ZERO                            0

/**
 * struct ltc2606_chip_info - chip specific information
 * @channels:		channel spec for the DAC
 */
struct ltc2606_chip_info {
	const struct iio_chan_spec *channels;
};

/**
 * struct ltc2606_state - driver instance specific data
 * @i2c_dev:			pointer to the i2c_client struct
 * @powerdown_cache_mask	used to show current channel powerdown state
 * @currValue       for read back operation
 */
struct ltc2606_state {
	struct i2c_client *i2c_dev;
	unsigned int powerdown_cache;
	int currValue;
};

enum ltc2606_supported_device_ids {
	ID_LTC2606,
};

static int ltc2606_i2c_write(struct i2c_client *i2c,
			     u8 cmd, u16 val, u8 shift)
{
//    __be32 data = (cmd << 20) | (val << shift);
    char msg[4];
    msg[0] = cmd << 4;
    msg[1] = val >> 8;
    msg[2] = val;

    return i2c_master_send(i2c, msg, 3);
}

static int ltc2606_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long m)
{
    const struct ltc2606_state *st = iio_priv(indio_dev);

    switch (m)
    {
        case IIO_CHAN_INFO_RAW:
            *val = st->currValue;
            return IIO_VAL_INT;
    }
    return -EINVAL;
}


static int ltc2606_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
    int ret = 0;
	struct ltc2606_state *st = iio_priv(indio_dev);

	switch (mask) {
	    case IIO_CHAN_INFO_RAW:

        ret = ltc2606_i2c_write(st->i2c_dev,
                     LTC2606_CMD_WRITE_INPUT_UPDATE,
                     val,
                     chan->scan_type.shift) ? 0 : -EIO;

		if(ret < 0) {
		    return ret;
		}

        st->currValue = val;
        msleep(1);
	    break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t ltc2606_write_dac_powerdown(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   const char *buf,
					   size_t len)
{
	bool pwr_down;
	int ret;
	struct ltc2606_state *st = iio_priv(indio_dev);

	ret = strtobool(buf, &pwr_down);
	if (ret)
		return ret;

	if (pwr_down)
		st->powerdown_cache = true;
	else
		st->powerdown_cache = false;

	ret = ltc2606_i2c_write(st->i2c_dev,
				LTC2606_CMD_POWERDOWN_DAC,
				0, 0);

	return ret ? ret : len;
}

static const struct iio_info ltc2606_info = {
    .driver_module = THIS_MODULE,
	.write_raw	= ltc2606_write_raw,
	.read_raw	= ltc2606_read_raw,
};

static const struct iio_chan_spec_ext_info ltc2606_ext_info[] = {
	{
		.name = "powerdown",
		.write = ltc2606_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	{ },
};

#define LTC2606_CHANNEL(_chan, _bits) { \
		.type = IIO_VOLTAGE, \
		.output = 1, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.scan_type = { \
			.realbits	= (_bits), \
			.shift		= 16 - (_bits), \
		}, \
/*		.ext_info = ltc2606_ext_info, */\
}

#define DECLARE_LTC2606_CHANNELS(_name, _bits) \
	const struct iio_chan_spec _name ## _channels[] = { \
		LTC2606_CHANNEL(0, _bits), \
	}

static DECLARE_LTC2606_CHANNELS(ltc2606, 16);

static const struct ltc2606_chip_info ltc2606_chip_info_tbl[] = {
	[ID_LTC2606] = {
		.channels	= ltc2606_channels,
	},
};

static int ltc2606_probe(struct i2c_client *i2c, const struct i2c_device_id *dev_id)
{
	struct ltc2606_state *st;
	struct iio_dev *indio_dev;
	int ret = -EAGAIN;

    pr_notice("ltc2606_probe\n");

	indio_dev = devm_iio_device_alloc(&i2c->dev, sizeof(*st));
	if (!indio_dev) {
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);

	i2c_set_clientdata(i2c, indio_dev);
	st->i2c_dev = i2c;

	indio_dev->dev.parent = &i2c->dev;
	indio_dev->name = dev_of_node(&i2c->dev) ? dev_of_node(&i2c->dev)->name
						 : i2c->name;
	indio_dev->info = &ltc2606_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2606_channels;
	indio_dev->num_channels = LTC2606_DAC_CHANNELS;

    if (!strcmp(st->i2c_dev->dev.kobj.name, "0-0010"))
    {
        ret = ltc2606_i2c_write(i2c,
                     LTC2606_CMD_WRITE_INPUT_UPDATE,
                     LTC2606_MIDPOINT,
                     0);

        st->currValue = LTC2606_MIDPOINT;
    }

    if (!strcmp(st->i2c_dev->dev.kobj.name, "3-0010"))
    {
        ret = ltc2606_i2c_write(i2c,
                     LTC2606_CMD_WRITE_INPUT_UPDATE,
                     LTC2606_MIDPOINT,
                     0);

        st->currValue = LTC2606_MIDPOINT;
    }

    if (!strcmp(st->i2c_dev->dev.kobj.name, "3-0031"))
    {
        ret = ltc2606_i2c_write(i2c,
                     LTC2606_CMD_WRITE_INPUT_UPDATE,
                     LTC2606_ZERO,
                     0);

        st->currValue = LTC2606_ZERO;
    }

    if (0 <= ret)
    {
        ret = devm_iio_device_register(&i2c->dev, indio_dev);
    }

	return ret;
}

static int ltc2606_remove(struct i2c_client *i2c)
{
    pr_notice("ltc2606_remove\n");
    return 0;
}

static int ltc2606_suspend(struct device *dev)
{
//    struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
//    struct ltc2606_state *state = iio_priv(indio_dev);
    int ret = 0;

    pr_notice("ltc2606_suspend\n");

    return ret;
}

static int ltc2606_resume(struct device *dev)
{
    int ret = 0;
    struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
    struct ltc2606_state *state = iio_priv(indio_dev);

    pr_notice("ltc2606_resume\n");

    ret = ltc2606_i2c_write(state->i2c_dev,
                 LTC2606_CMD_WRITE_INPUT_UPDATE,
                 state->currValue,
                 indio_dev->channels->scan_type.shift) ? 0 : -EIO;

    return ret;
}

static const struct dev_pm_ops ltc2606_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(ltc2606_suspend, ltc2606_resume)
};

static const struct i2c_device_id ltc2606_id[] = {
	{ "ltc2606", (kernel_ulong_t)&ltc2606_channels },
	{}
};
MODULE_DEVICE_TABLE(i2c, ltc2606_id);

static struct i2c_driver ltc2606_driver = {
	.driver		= {
		.name	= "ltc2606",
		.pm     = &ltc2606_pm_ops,
	},
	.probe		= ltc2606_probe,
    .remove = ltc2606_remove,
	.id_table	= ltc2606_id,
};
module_i2c_driver(ltc2606_driver);

static const struct of_device_id ltc2606_of_match[] = {
	{
		.compatible = "ltc,ltc2606",
		.data = &ltc2606_channels
	},
	{}
};
MODULE_DEVICE_TABLE(of, ltc2606_of_match);

MODULE_AUTHOR("Markku Nivala <markku.nivala@innokasmedical.fi>");
MODULE_DESCRIPTION("LTC2606 DAC I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
