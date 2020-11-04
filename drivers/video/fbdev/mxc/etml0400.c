/*
 *  etml0400.c - Linux kernel module for
 *  RSP display driver init
 *
 *  Copyright (c) 2018 Markku Nivala, Innokas medical
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>


#define MAX_TRANSFER_SIZE 6
#define RCV_BUF 2
#define MAX_BUF_SIZE MAX_TRANSFER_SIZE * 2 + RCV_BUF

struct dma_buf
{
    u8* buf;
    dma_addr_t dma;
};

struct etml0400_message {
    union {
        struct message_part_s {
            u8 message : 8;
            u8 type : 1;
            u8 fill : 7;
        }value_parts[MAX_TRANSFER_SIZE];

        struct message_s {
            u16 message : 9;
            u16 fill : 7;
        }message_parts[MAX_TRANSFER_SIZE];

        __be16 complete_msg[MAX_TRANSFER_SIZE];
    }combined_parts;
    u8 count;
};

struct etml0400_data {
    struct spi_device   *spi;
    struct dma_buf dma[2];
    struct spi_transfer t[MAX_TRANSFER_SIZE];
    struct spi_message m;
    struct etml0400_message msg;
};

void set_command(struct etml0400_data *devData, int data)
{
    devData->msg.count = 1;

    devData->msg.combined_parts.value_parts[0].message = data;
    devData->msg.combined_parts.value_parts[0].type = 0;
}

void set_data(struct etml0400_data *devData, int data)
{
    devData->msg.combined_parts.value_parts[devData->msg.count].message = data;
    devData->msg.combined_parts.value_parts[devData->msg.count].type = 1;

    ++devData->msg.count;
}

void send_msg(struct etml0400_data *devData)
{
    int status;
    int i;

    devData->dma[1].buf = devData->dma[0].buf + devData->msg.count * sizeof(devData->msg.combined_parts.complete_msg);
    devData->dma[1].dma = devData->dma[0].dma + devData->msg.count * sizeof(devData->msg.combined_parts.complete_msg);

    memset(&devData->t, 0, sizeof(devData->t));

    for(i = 0; i < devData->msg.count; i++)
    {
        devData->t[i].tx_buf = devData->dma[0].buf + i * 2;
        devData->t[i].rx_buf = 0;
        devData->t[i].tx_dma = devData->dma[0].dma + i * 2;
        devData->t[i].rx_dma = 0;
        devData->t[i].bits_per_word = 9;
        devData->t[i].delay_usecs = 0;
        devData->t[i].len = 2;
    }

    memcpy(devData->dma[0].buf, devData->msg.combined_parts.complete_msg, devData->msg.count * sizeof(devData->msg.combined_parts.complete_msg[0]));

    spi_message_init(&devData->m);
    devData->m.is_dma_mapped = 1;

    spi_message_init_with_transfers(&devData->m, devData->t, devData->msg.count);
    status = spi_sync(devData->spi, &devData->m);
}

void etml_config(struct spi_device *spiDev)
{
    struct etml0400_data *devData = spi_get_drvdata(spiDev);

    gpio_direction_output(68, 1);
    msleep(5);
    gpio_direction_output(68, 0);
    msleep(10);
    gpio_direction_output(68, 1);
    msleep(200);

    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x01);
    send_msg(devData);

    set_command(devData, 0x08);
    set_data(devData, 0x18);
    send_msg(devData);

    set_command(devData, 0x20);
    set_data(devData, 0x01);  // RGB interface selection 01=SYNC mode£¬00=DE mode
    send_msg(devData);

    set_command(devData, 0x21);
    set_data(devData, 0x01);     //DE = 1 Active
    send_msg(devData);

    set_command(devData, 0x30);
    set_data(devData, 0x02);     //480 X 800
    send_msg(devData);

    set_command(devData, 0x31);
    set_data(devData, 0x02);     //Column Inversion
    send_msg(devData);

    set_command(devData, 0x40);
    set_data(devData, 0x16);     //2.5VCI/-2VCL  15
    send_msg(devData);

    set_command(devData, 0x41);
    set_data(devData, 0x55);     //6/-6 44  55
    send_msg(devData);

    set_command(devData, 0x42);
    set_data(devData, 0x02);     //DDVDH+VCI-VCL/DDVDL-DDVDH
    send_msg(devData);

    set_command(devData, 0x43);
    set_data(devData, 0x09);     //VGH_CLAMP 0FF ;
    send_msg(devData);

    set_command(devData, 0x44);
    set_data(devData, 0x09);     //VGL_CLAMP ON ;
    send_msg(devData);

    set_command(devData, 0x50);
    set_data(devData, 0x78);     //50
    send_msg(devData);

    set_command(devData, 0x51);
    set_data(devData, 0x78);     //50
    send_msg(devData);

    set_command(devData, 0x52);
    set_data(devData, 0x00);     //Flicker
    send_msg(devData);

    set_command(devData, 0x53);
    set_data(devData, 0x80);     //Flicker6D
    send_msg(devData);

    set_command(devData, 0x57);
    set_data(devData, 0x50);     //
    send_msg(devData);

    set_command(devData, 0x60);
    set_data(devData, 0x07);     //SDTI
    send_msg(devData);

    set_command(devData, 0x61);
    set_data(devData, 0x00);     //CRTI
    send_msg(devData);

    set_command(devData, 0x62);
    set_data(devData, 0x08);     //EQTI
    send_msg(devData);

    set_command(devData, 0x63);
    set_data(devData, 0x00);     //PCTI
    send_msg(devData);

    //++++++++++++++++++ Gamma Setting ++++++++++++++++++//

    set_command(devData, 0xA0);
    set_data(devData, 0x00);     //Gamma 0
    send_msg(devData);

    set_command(devData, 0xA1);
    set_data(devData, 0x0B);     //Gamma 4
    send_msg(devData);

    set_command(devData, 0xA2);
    set_data(devData, 0x1A);     //Gamma 8
    send_msg(devData);

    set_command(devData, 0xA3);
    set_data(devData, 0x11);     //Gamma 16
    send_msg(devData);

    set_command(devData, 0xA4);
    set_data(devData, 0x0A);     //Gamma 24
    send_msg(devData);

    set_command(devData, 0xA5);
    set_data(devData, 0x1B);     //Gamma 52
    send_msg(devData);

    set_command(devData, 0xA6);
    set_data(devData, 0x08);     //Gamma 80
    send_msg(devData);

    set_command(devData, 0xA7);
    set_data(devData, 0x06);     //Gamma 108
    send_msg(devData);

    set_command(devData, 0xA8);
    set_data(devData, 0x03);     //Gamma 147
    send_msg(devData);

    set_command(devData, 0xA9);
    set_data(devData, 0x0C);     //Gamma 175
    send_msg(devData);

    set_command(devData, 0xAA);
    set_data(devData, 0x03);     //Gamma 203
    send_msg(devData);

    set_command(devData, 0xAB);
    set_data(devData, 0x0B);     //Gamma 231
    send_msg(devData);

    set_command(devData, 0xAC);
    set_data(devData, 0x0F);     //Gamma 239
    send_msg(devData);

    set_command(devData, 0xAD);
    set_data(devData, 0x2F);     //Gamma 247
    send_msg(devData);

    set_command(devData, 0xAE);
    set_data(devData, 0x27);     //Gamma 251
    send_msg(devData);

    set_command(devData, 0xAF);
    set_data(devData, 0x00);     //Gamma 255
    send_msg(devData);
    ///==============Nagitive

    set_command(devData, 0xC0);
    set_data(devData, 0x00);     //Gamma 0
    send_msg(devData);

    set_command(devData, 0xC1);
    set_data(devData, 0x04);     //Gamma 4
    send_msg(devData);

    set_command(devData, 0xC2);
    set_data(devData, 0x0B);     //Gamma 8
    send_msg(devData);

    set_command(devData, 0xC3);
    set_data(devData, 0x0E);     //Gamma 16
    send_msg(devData);

    set_command(devData, 0xC4);
    set_data(devData, 0x08);     //Gamma 24
    send_msg(devData);

    set_command(devData, 0xC5);
    set_data(devData, 0x14);     //Gamma 52
    send_msg(devData);

    set_command(devData, 0xC6);
    set_data(devData, 0x09);     //Gamma 80
    send_msg(devData);

    set_command(devData, 0xC7);
    set_data(devData, 0x07);     //Gamma 108
    send_msg(devData);

    set_command(devData, 0xC8);
    set_data(devData, 0x06);     //Gamma 147
    send_msg(devData);

    set_command(devData, 0xC9);
    set_data(devData, 0x09);     //Gamma 175
    send_msg(devData);

    set_command(devData, 0xCA);
    set_data(devData, 0x08);     //Gamma 203
    send_msg(devData);

    set_command(devData, 0xCB);
    set_data(devData, 0x04);     //Gamma 231
    send_msg(devData);

    set_command(devData, 0xCC);
    set_data(devData, 0x0B);     //Gamma 239
    send_msg(devData);

    set_command(devData, 0xCD);
    set_data(devData, 0x29);     //Gamma 247
    send_msg(devData);

    set_command(devData, 0xCE);
    set_data(devData, 0x27);     //Gamma 251
    send_msg(devData);

    set_command(devData, 0xCF);
    set_data(devData, 0x00);     //Gamma 255
    send_msg(devData);





    //+++++++++++++++++++++++++++++++++++++++++++++++++++//

    //****************************************************************************//
    //****************************** Page 6 Command ******************************//
    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x06);        //Change to Page 6
    send_msg(devData);

    set_command(devData, 0x00);
    set_data(devData, 0x21);        //
    send_msg(devData);

    set_command(devData, 0x01);
    set_data(devData, 0x09);        //
    send_msg(devData);

    set_command(devData, 0x02);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x03);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x04);
    set_data(devData, 0x01);        //
    send_msg(devData);

    set_command(devData, 0x05);
    set_data(devData, 0x01);        //
    send_msg(devData);

    set_command(devData, 0x06);
    set_data(devData, 0x80);        //
    send_msg(devData);

    set_command(devData, 0x07);
    set_data(devData, 0x05);        //
    send_msg(devData);

    set_command(devData, 0x08);
    set_data(devData, 0x02);        //
    send_msg(devData);

    set_command(devData, 0x09);
    set_data(devData, 0x80);        //
    send_msg(devData);

    set_command(devData, 0x0A);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x0B);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x0C);
    set_data(devData, 0x0A);        //
    send_msg(devData);

    set_command(devData, 0x0D);
    set_data(devData, 0x0A);        //
    send_msg(devData);

    set_command(devData, 0x0E);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x0F);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x10);
    set_data(devData, 0xE0);        //
    send_msg(devData);

    set_command(devData, 0x11);
    set_data(devData, 0xE4);        //
    send_msg(devData);

    set_command(devData, 0x12);
    set_data(devData, 0x04);        //
    send_msg(devData);

    set_command(devData, 0x13);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x14);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x15);
    set_data(devData, 0xC0);        //
    send_msg(devData);

    set_command(devData, 0x16);
    set_data(devData, 0x08);        //
    send_msg(devData);

    set_command(devData, 0x17);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x18);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x19);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x1A);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x1B);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x1C);
    set_data(devData, 0x00);        //
    send_msg(devData);

    set_command(devData, 0x1D);
    set_data(devData, 0x00);        //
    send_msg(devData);


    set_command(devData, 0x20);
    set_data(devData, 0x01);        //
    send_msg(devData);

    set_command(devData, 0x21);
    set_data(devData, 0x23);        //
    send_msg(devData);

    set_command(devData, 0x22);
    set_data(devData, 0x45);        //
    send_msg(devData);

    set_command(devData, 0x23);
    set_data(devData, 0x67);        //
    send_msg(devData);

    set_command(devData, 0x24);
    set_data(devData, 0x01);        //
    send_msg(devData);

    set_command(devData, 0x25);
    set_data(devData, 0x23);        //
    send_msg(devData);

    set_command(devData, 0x26);
    set_data(devData, 0x45);        //
    send_msg(devData);

    set_command(devData, 0x27);
    set_data(devData, 0x67);        //
    send_msg(devData);

    set_command(devData, 0x30);
    set_data(devData, 0x01);        //
    send_msg(devData);

    set_command(devData, 0x31);
    set_data(devData, 0x11);        //GOUT1
    send_msg(devData);

    set_command(devData, 0x32);
    set_data(devData, 0x00);        //GOUT2
    send_msg(devData);

    set_command(devData, 0x33);
    set_data(devData, 0xEE);        //GOUT3
    send_msg(devData);

    set_command(devData, 0x34);
    set_data(devData, 0xFF);        //GOUT4
    send_msg(devData);

    set_command(devData, 0x35);
    set_data(devData, 0xCB);        //GOUT5   GCK4
    send_msg(devData);

    set_command(devData, 0x36);
    set_data(devData, 0xDA);        //GOUT6   GCK3
    send_msg(devData);

    set_command(devData, 0x37);
    set_data(devData, 0xAD);        //GOUT7   GCK2
    send_msg(devData);

    set_command(devData, 0x38);
    set_data(devData, 0xBC);        //GOUT8   GCK1
    send_msg(devData);

    set_command(devData, 0x39);
    set_data(devData, 0x76);        //GOUT9   STV1
    send_msg(devData);

    set_command(devData, 0x3A);
    set_data(devData, 0x67);        //GOUT10  STV2
    send_msg(devData);

    set_command(devData, 0x3B);
    set_data(devData, 0x22);        //GOUT11
    send_msg(devData);

    set_command(devData, 0x3C);
    set_data(devData, 0x22);        //GOUT12
    send_msg(devData);

    set_command(devData, 0x3D);
    set_data(devData, 0x22);        //GOUT13
    send_msg(devData);

    set_command(devData, 0x3E);
    set_data(devData, 0x22);        //GOUT14
    send_msg(devData);

    set_command(devData, 0x3F);
    set_data(devData, 0x22);        //
    send_msg(devData);

    set_command(devData, 0x40);
    set_data(devData, 0x22);        //
    send_msg(devData);

    set_command(devData, 0x52);
    set_data(devData, 0x10);        //
    send_msg(devData);

    set_command(devData, 0x53);
    set_data(devData, 0x10);        //
    send_msg(devData);

    //****************************************************************************//
    //****************************** Page 7 Command ******************************//
    //****************************************************************************//
    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x07);        //Change to Page 7
    send_msg(devData);

    set_command(devData, 0x17);
    set_data(devData, 0x22);     //VGL_RE
    send_msg(devData);

    set_command(devData, 0x02);
    set_data(devData, 0x77);     //
    send_msg(devData);

    set_command(devData, 0xE1);
    set_data(devData, 0x79);     //
    send_msg(devData);

    //****************************************************************************//

    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x00);      //Change to Page 0
    send_msg(devData);

    set_command(devData, 0x36);      //Display Control       Scan direction
    set_data(devData, 0x03);
    send_msg(devData);

    set_command(devData, 0x11);
    set_data(devData, 0x00);
    send_msg(devData);

    ndelay(120);

    set_command(devData, 0x29);      // Display On
    set_data(devData, 0x00);
    send_msg(devData);

    set_command(devData, 0x35);
    set_data(devData, 0x00);     //
    send_msg(devData);

    ndelay(120);

}

int etml_run_suspend(struct device *dev)
{
    struct etml0400_data *devData = dev_get_drvdata(dev);

    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x00);        //Change to Page 0
    send_msg(devData);

    mdelay(1);

    set_command(devData, 0x28);
    set_data(devData, 0x00);
    send_msg(devData);

    mdelay(10);


    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x01);
    send_msg(devData);

    set_command(devData, 0x58);
    set_data(devData, 0x91);
    send_msg(devData);

    mdelay(120);

    return 0;
}

static int etml_remove(struct spi_device *pdev)
{
    etml_run_suspend(&pdev->dev);

    gpio_free(68);

    return 0;
}

static int etml_probe(struct spi_device *spiDev)
{
    struct etml0400_data *devData;
    int ret;

    ret = gpio_request(68, "display");
    if (ret)
    {
        dev_err(&spiDev->dev, "Could not get reset handle");
    }

    devData = kzalloc(sizeof(struct etml0400_data), GFP_KERNEL);
    if (!devData)
        return -ENOMEM;

    devData->spi = spiDev;
    spi_set_drvdata(spiDev, devData);

    devData->spi->dev.coherent_dma_mask = ~0;
    devData->dma[0].buf = dmam_alloc_coherent(&devData->spi->dev, MAX_BUF_SIZE, &devData->dma[0].dma, GFP_KERNEL);

    if (devData->dma[0].buf == 0) {
        goto out_dmam;
    }
    devData->spi->mode = SPI_MODE_3;
    devData->spi->bits_per_word = 9;

    ret = spi_setup(devData->spi);
    if (ret < 0)
        return ret;

    etml_config(spiDev);

    dev_info(&spiDev->dev, "ETML0400 initialized\n");
    return 0;

    out_dmam:
        kfree(devData);
        return -1;
}

int etml_run_resume(struct device *dev)
{
    struct etml0400_data *devData = dev_get_drvdata(dev);

    set_command(devData, 0xFF);
    set_data(devData, 0xFF);
    set_data(devData, 0x98);
    set_data(devData, 0x06);
    set_data(devData, 0x04);
    set_data(devData, 0x00);        //Change to Page 0
    send_msg(devData);

    mdelay(1);

    set_command(devData, 0x11);
    set_data(devData, 0x00);
    send_msg(devData);

    mdelay(5);

    etml_config(to_spi_device(dev));

    set_command(devData, 0x29);      // Display On
    set_data(devData, 0x00);
    send_msg(devData);

    mdelay(120);

    return 0;
}

int etml_run_idle(struct device *dev)
{
    etml_run_suspend(dev);

    return 0;
}

static const struct dev_pm_ops etml0400_pm_ops = {
    SET_RUNTIME_PM_OPS(etml_run_suspend, etml_run_resume, etml_run_idle)
    SET_LATE_SYSTEM_SLEEP_PM_OPS(etml_run_suspend, etml_run_resume)
};

static const struct of_device_id etml_of_match[] = {
    { .compatible = "et,etml0400", },
    { .compatible = "etml0400", },
    { }
};

MODULE_DEVICE_TABLE(of, etml_of_match);

static struct spi_driver etml_driver = {
    .driver = {
        .name   = "etml0400",
        .of_match_table = etml_of_match,
        .pm     = &etml0400_pm_ops,
    },
    .probe = etml_probe,
    .remove = etml_remove,
};

module_spi_driver(etml_driver);


MODULE_AUTHOR("Markku Nivala <markku nivala at innokasmedical fi>");
MODULE_DESCRIPTION("RSP display driver init");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_ALIAS("spi:etml0400");
