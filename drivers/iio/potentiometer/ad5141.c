/*
 *  ad5141.c - Linux kernel module for
 *  AD5141 digital potentiometer
 *
 *  Copyright (c) 2020 Markku Nivala, Innokas medical
 *
 *  Based on ad5272 driver
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/gpio.h>

extern int p05_laser_board_power_state(void);

#define  AD5141_RDAC_WR     0x10
#define  AD5141_CONTENT_RD  0x30
#define  AD5141_RDAC_EEPROM 0x70
#define  AD5141_RESET       0xb0
#define  AD5141_SHDN        0xc0

#define  AD5141_CONTENT_RD_RDAC    BIT(0) | BIT(1)

#define  AD5141_RDAC_EEPROM_CPY_TO_EEPROM  BIT(0)
#define  AD5141_RDAC_EEPROM_CPY_TO_RDAC    0

#define  AD5141_SHDN_NORMAL 0


struct ad5141_cfg {
    int max_pos;
    int kohms;
    int shift;
};

enum ad5141_type {
    AD5141_010,
    AD5141_100,
    AD5121_010,
    AD5121_100,
};

static const struct ad5141_cfg ad5141_cfg[] = {
    [AD5141_010] = { .max_pos = 256,  .kohms = 10 },
    [AD5141_100] = { .max_pos = 256,  .kohms = 100 },
    [AD5121_010] = { .max_pos = 128,  .kohms = 10, .shift = 1 },
    [AD5121_100] = { .max_pos = 128,  .kohms = 100, .shift = 1 },
};

struct ad5141_data {
    struct i2c_client       *client;
    struct mutex            lock;
    const struct ad5141_cfg *cfg;
    u8                      buf[2] ____cacheline_aligned;
};

static const struct iio_chan_spec ad5141_channels[] = {
        {
        .type = IIO_RESISTANCE,
        .output = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
        },
        {
        .type = IIO_RESISTANCE,
        .output = 0,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
        },
};

static int ad5141_write(struct ad5141_data *data, int reg, u8 val)
{
    int ret;

    data->buf[0] = reg;
    data->buf[1] = val;

    ret = i2c_master_send(data->client, data->buf, sizeof(data->buf));
    return ret < 0 ? ret : 0;
}

static int ad5141_read(struct ad5141_data *data, int reg, u8 *val)
{
    int ret;

    data->buf[0] = reg;
    data->buf[1] = *val;

    mutex_lock(&data->lock);

    ret = i2c_master_send(data->client, data->buf, sizeof(data->buf));
    if (ret < 0)
        goto error;

    ret = i2c_master_recv(data->client, data->buf, 1);
    if (ret < 0)
        goto error;

    *val = data->buf[0];
    ret = 0;
error:
    mutex_unlock(&data->lock);
    return ret;
}

static int ad5141_read_raw(struct iio_dev *indio_dev,
               struct iio_chan_spec const *chan,
               int *val, int *val2, long mask)
{
    struct ad5141_data *data = iio_priv(indio_dev);
    int ret;

    if (!p05_laser_board_power_state()) {
        return -EAGAIN;
    }

    switch (mask) {
    case IIO_CHAN_INFO_RAW: {
        if (!ad5141_write(data, AD5141_RDAC_EEPROM, AD5141_RDAC_EEPROM_CPY_TO_RDAC))
        {
            msleep(1);
            *val = AD5141_CONTENT_RD_RDAC;
            ret = ad5141_read(data, AD5141_CONTENT_RD, (u8 *)val);
            *val = *val >> data->cfg->shift;
        }
        else
            ret = -EIO;

        return ret ? ret : IIO_VAL_INT;
    }
    case IIO_CHAN_INFO_SCALE:
        *val = 1000 * data->cfg->kohms;
        *val2 = data->cfg->max_pos;
        return IIO_VAL_FRACTIONAL;
    }

    return -EINVAL;
}

static int ad5141_write_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int val, int val2, long mask)
{
    struct ad5141_data *data = iio_priv(indio_dev);
    int ret = -EINVAL;

    if (mask != IIO_CHAN_INFO_RAW)
        return -EINVAL;

    if (!p05_laser_board_power_state()) {
        return -EAGAIN;
    }

    if (val >= data->cfg->max_pos || val < 0 || val2)
        return -EINVAL;

    ret = ad5141_write(data, AD5141_RDAC_WR, val << data->cfg->shift);

    if(!ret)
    {
        ret = ad5141_write(data, AD5141_RDAC_EEPROM, AD5141_RDAC_EEPROM_CPY_TO_EEPROM);
        if (!ret)
            msleep(50);
    }

    return ret;
}

static const struct iio_info ad5141_info = {
    .read_raw = ad5141_read_raw,
    .write_raw = ad5141_write_raw,
};

static int ad5141_reset(struct ad5141_data *data)
{
    int ret;

    ret = ad5141_write(data, AD5141_RESET, 0);

    if (0 <= ret)
        usleep_range(1000, 2000);

    return ret;
}

static int ad5141_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct iio_dev *indio_dev;
    struct ad5141_data *data;
    int ret;

    pr_notice("ad5141_probe\n");

    ret = gpio_request(1, "aux_pwr");
    gpio_direction_output(1, 1);
    gpio_free(1);

    msleep(2);

    indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    i2c_set_clientdata(client, indio_dev);

    data = iio_priv(indio_dev);
    data->client = client;
    mutex_init(&data->lock);
    data->cfg = &ad5141_cfg[id->driver_data];

    ret = ad5141_reset(data);
    if (ret)
    {
        dev_err(dev, "Could not reset AD5141: %d\n", ret);
        return ret;
    }

    dev_set_drvdata(dev, data);

    ret = ad5141_write(data, AD5141_SHDN, AD5141_SHDN_NORMAL);
    if (ret < 0)
    {
        dev_err(dev, "Could not initialize AD5141: %d\n", ret);
        return -ENODEV;
    }

    indio_dev->dev.parent = dev;
    indio_dev->info = &ad5141_info;
    indio_dev->channels = ad5141_channels;
    indio_dev->num_channels = 1;
    indio_dev->name = client->name;

    ret = devm_iio_device_register(dev, indio_dev);

    dev_info(dev, "AD5141 register result: %d\n", ret);

    return ret;
}

static int ad5141_remove(struct i2c_client *client)
{
    struct ad5141_data *data;

    pr_notice("ad5141_remove\n");

    data = i2c_get_clientdata(client);

    return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id ad5141_dt_ids[] = {
    { .compatible = "adi,ad5141-010", .data = (void *)AD5141_010 },
    { .compatible = "adi,ad5141-100", .data = (void *)AD5141_100 },
    { .compatible = "adi,ad5121-010", .data = (void *)AD5121_010 },
    { .compatible = "adi,ad5121-100", .data = (void *)AD5121_100 },
    {}
};
MODULE_DEVICE_TABLE(of, ad5141_dt_ids);
#endif /* CONFIG_OF */

static const struct i2c_device_id ad5141_id[] = {
    { "ad5141-020", AD5141_010 },
    { "ad5141-100", AD5141_100 },
    { "ad5121-010", AD5121_010 },
    { "ad5121-100", AD5121_100 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ad5141_id);

static struct i2c_driver ad5141_driver = {
    .driver = {
        .name   = "ad5141",
        .of_match_table = of_match_ptr(ad5141_dt_ids),
    },
    .probe      = ad5141_probe,
    .remove     = ad5141_remove,
    .id_table   = ad5141_id,
};

module_i2c_driver(ad5141_driver);

MODULE_AUTHOR("Markku Nivala <markku.nivala innokasmedical fi>");
MODULE_DESCRIPTION("AD5141 digital potentiometer");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
