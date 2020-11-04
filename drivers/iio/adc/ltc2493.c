/*
 * LTC2493 Analog to digital convertors i2c driver
 *
 * Copyright 2018 Markku Nivala, Innokas Medical
 *
 * Licensed under the ?.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <stdbool.h>

extern int p05_laser_board_power_state(void);

/* Power-on configuration: rejects both 50/60Hz, temperature compensation */
#define LTC2493_CONFIG_DEFAULT		0x80B0 //C0B0 internal temp measure //0x1618
#define LTC2493_CONFIG_PROBE		0x1000
#define LTC2493_WORK_DIV            6
const unsigned int conv_time = 150; /* conversion time ms */

static const struct iio_chan_spec ltc2493_channel[] = {
    {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .channel = 0,
        .scan_type.realbits = 24,
        .scan_type.storagebits = 32,
        .scan_type.shift = 6,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
    },
    {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .channel = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
        .scan_type.realbits = 24,
        .scan_type.storagebits = 32,
        .scan_type.shift = 6,
    },
    {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .channel = 2,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
        .scan_type.realbits = 24,
        .scan_type.storagebits = 32,
        .scan_type.shift = 6,
    },
    {
        .type = IIO_VOLTAGE,
        .indexed = 1,
        .channel = 3,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
        .scan_type.realbits = 24,
        .scan_type.storagebits = 32,
        .scan_type.shift = 6,
    },
};

int values[ARRAY_SIZE(ltc2493_channel)];

typedef const enum {
	LTC2493_CHAN0,
	LTC2493_CHAN1,
	LTC2493_CHAN2,
	LTC2493_CHAN3
}adc_channels;

static const uint channel_access[] = {
	[LTC2493_CHAN0] = 0,
	[LTC2493_CHAN1] = 8,
	[LTC2493_CHAN2] = 1,
	[LTC2493_CHAN3] = 9,
};

struct ltc2493_data {
	struct i2c_client   *client;
    struct mutex        lock[ARRAY_SIZE(ltc2493_channel)];
    struct delayed_work work;
    u16                 retries[ARRAY_SIZE(ltc2493_channel)];
    u16                 curr_chan;
};

static void ltc2493_wait_conv(const ktime_t time_prev, u16 wait_time)
{
	unsigned int time_elapsed;

	/* delay if conversion time not passed since last read or write */
	time_elapsed = ktime_ms_delta(ktime_get(), time_prev);

	if (time_elapsed < wait_time)
		msleep(wait_time - time_elapsed);
}

static int ltc2493_write(struct ltc2493_data *data, int channel)
{
	struct i2c_client *client = data->client;
	__be16 buf = LTC2493_CONFIG_DEFAULT | channel_access[channel];
	int ret;

	ret = i2c_master_send(client, (char *)&buf, 2);

	return ret;
}

static int ltc2493_read(struct ltc2493_data *data, int *val)
{
	struct i2c_client *client = data->client;
	__be32 buf;
	int ret;

    ret = i2c_master_recv(client, (char *)&buf, 4);
    if (ret < 0)  {
        return ret;
    }

	buf = be32_to_cpu(buf);
	if((0xc0000000 & buf) == 0xc0000000)
	{
	    *val = INT_MAX;
	}
	else if(!(buf & 0xc0000000))
	{
	    *val = INT_MIN;
	}
	else
	{
	    bool negative = buf & 0x40000000;
        buf &= ~0xc0000000;
	    buf >>= 6;
	    if(negative)
	    {
	        buf -= 0xFFFFFF;
	    }

	    *val = sign_extend32(buf, 25);
	}

	return ret;
}

static int ltc2493_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ltc2493_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (mask == IIO_CHAN_INFO_RAW) {
        mutex_lock(&data->lock[chan->channel]);

        *val = values[chan->channel];

        mutex_unlock(&data->lock[chan->channel]);

		return IIO_VAL_INT;

	} else {
		return -EINVAL;
	}
	return ret;
}

static const struct iio_info ltc2493_info = {
	.read_raw = ltc2493_read_raw,
	.driver_module = THIS_MODULE,
};

static void ltc2493_work(struct work_struct *work)
{
    struct ltc2493_data *data;
    int ret;
    int i;

    data = container_of(work, struct ltc2493_data, work.work);

    if (!p05_laser_board_power_state()) {
        // No power, we try again later
        pr_notice("ltc2493_work Power lane is off, trying later\n");

        schedule_delayed_work(&data->work, HZ / LTC2493_WORK_DIV);
        return;
    }

    ret = ltc2493_write(data, data->curr_chan);
    if (ret > 0)
    {
        msleep_interruptible(conv_time);
        for (i = 0; i < 3; i++)
        {
            mutex_lock(&data->lock[data->curr_chan]);
            ret = ltc2493_read(data, &values[data->curr_chan]);
            mutex_unlock(&data->lock[data->curr_chan]);
            if (ret > 0)
            {
                i = 3;
            }
            else
            {
                msleep_interruptible(20);
            }
        }

        if (ret < 0)
        {
            ++data->retries[data->curr_chan];
            dev_info(&data->client->dev, "Failed to get laser board readings from channel %d, Error:(%d)\n", ret, data->curr_chan);
        }
        else
        {
            ++data->curr_chan;
            if ( data->curr_chan >= ARRAY_SIZE(ltc2493_channel))
            {
                data->curr_chan = 0;
            }
        }
    }
    else
    {
        ++data->retries[data->curr_chan];
        if (data->retries[data->curr_chan] > 3)
        {
            values[data->curr_chan] = 0;
            ++data->curr_chan;
            if ( data->curr_chan >= ARRAY_SIZE(ltc2493_channel))
            {
                data->curr_chan = 0;
            }
        }
    }

    schedule_delayed_work(&data->work, HZ / LTC2493_WORK_DIV);
}

static int ltc2493_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ltc2493_data *data;
	int ret;
	int i;
    __be32 buf = LTC2493_CONFIG_DEFAULT | 1 << 8;

    pr_notice("ltc2493_probe\n");

    msleep(conv_time + 5);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_of_node(&client->dev) ? dev_of_node(&client->dev)->name
			 : client->name;
	indio_dev->info = &ltc2493_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2493_channel;
	indio_dev->num_channels = ARRAY_SIZE(ltc2493_channel);

	ret = i2c_master_send(data->client, (char *)&buf, 2);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ltc2493_channel); i++)
	{
	    mutex_init(&data->lock[i]);
	}

	INIT_DELAYED_WORK(&data->work, ltc2493_work);

	schedule_delayed_work(&data->work, HZ / LTC2493_WORK_DIV);

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int ltc2493_remove(struct i2c_client *i2c)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(i2c);
    struct ltc2493_data *data = iio_priv(indio_dev);

    pr_notice("ltc2493_remove\n");

    cancel_delayed_work_sync(&data->work);

    return 0;
}


static int ltc2493_suspend(struct device *dev)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
    struct ltc2493_data *data = iio_priv(indio_dev);

    pr_notice("ltc2493_suspend\n");

    cancel_delayed_work_sync(&data->work);

    return 0;
}

static int ltc2493_resume(struct device *dev)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
    struct ltc2493_data *data = iio_priv(indio_dev);

    pr_notice("ltc2493_resume\n");

    schedule_delayed_work(&data->work, HZ / LTC2493_WORK_DIV);

    return 0;
}

SIMPLE_DEV_PM_OPS(ltc2493_pm_ops, ltc2493_suspend, ltc2493_resume);

static const struct i2c_device_id ltc2493_id[] = {
	{ "ltc2493", (kernel_ulong_t)&ltc2493_channel },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ltc2493_id);

static struct i2c_driver ltc2493_driver = {
	.driver = {
		.name = "ltc2493",
		.pm = &ltc2493_pm_ops,
	},
	.probe = ltc2493_probe,
	.remove = ltc2493_remove,
	.id_table = ltc2493_id,
};

module_i2c_driver(ltc2493_driver);

MODULE_AUTHOR("Markku Nivala <markku.nivala@innokasmedical.fi>");
MODULE_DESCRIPTION("LTC2493 ADC I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");

