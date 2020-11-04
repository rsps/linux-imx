/*
 *  wasatch.c - Linux kernel module for
 *  RSP custom FPGA to control Wasatch
 *
 *  Copyright (c) 2018 Markku Nivala, Innokas Medical
 *
 *  Release notes:
 *      2020.10.21 v0.6  Changed max integration time to 4000ms
 */

/*
ls -d  /home/fkb/kunder/rsp/work/wasatch/wasatch.c | entr sh -c  'make M=/home/fkb/kunder/rsp/work/wasatch/ modules'
~/tools/yocto/var/rsp/build_/workspace/sources/linux-variscite#
    make M=/home/fkb/kunder/rsp/work/wasatch/ modules'
*/
/* devtool modify -n linux-variscite /home/steffen/rsp/rsp-kernel/ */


/*  printf "\01" > /sys/class/camera/spectrometer/debugmode */
/*  printf "\07" > /sys/class/camera/spectrometer/debugmode */

#include <linux/types.h>
#include <stdbool.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/wasatch.h>

#define COMMAND_CANCEL    0x00
#define COMMAND_AQUIRE    0x01
#define COMMAND_TRANSFER  0x02
#define COMMAND_BINNING   0x08

#define WASATCH_ROW_ELEMENTS    1044
#define WASATCH_ROW_SIZE        4176
#define WASATCH_MAX_ROWS        70

#define  DEVICE_NAME "spectrometer"
#define  CLASS_NAME  "camera"

#define FPGA_WRITE      0x00
#define FPGA_READ       0x01
#define FPGA_READ_RAM   0x03

#define REG_COMMAND       0x00
#define REG_STATUS        0x01
#define REG_ROI_START     0x02
#define REG_ROI_END       0x03
#define REG_INTEGR_TIME   0x04
#define REG_CCD_TEMP      0x05
#define REG_CCD_ITEC      0x06
#define REG_FPGA_TEMP     0x07
#define REG_TEST_MODE     0x10
#define REG_FPGA_VERSION  0x16
#define REG_TEST_VALUE    0x20
#define REG_CDS_CONTROL   0x21

#define CHNL_COMMAND   0
#define CHNL_SPECTRUM  1

#define FPGA_CLK_PIN 131

/* CDS control register masks */
#define CDS_TEST_ENABLE 0x01
#define CDS_TEST_ACTUAL 0x02
#define CCD_PWR_ENABLE  0x04
#define CCD_PWR_EN_3V3  0x08


#define SPEC_STATE_NONE 0
#define SPEC_STATE_INIT 1
#define SPEC_STATE_DATA 2

static int    majorNumber;

static struct class*  cameraCharClass  = NULL;
static struct device* wasatchCharDevice = NULL;

struct spidev_data {
    struct spi_device   *spi;
    struct spi_transfer t[2]; /* 0 = Commands: (32-bit), 1 = Spectrum: (32-bit * WASATCH_ROW_ELEMENTS) */

    /* Cache for write only registers */
    u32 command_reg;

    /* Spectrum variables used in device read operations. */
    u32 spectrum_state;
    u32 spectrum_size;
    u8  spectrum_data[WASATCH_ROW_SIZE * WASATCH_MAX_ROWS];
    struct task_struct *spectrum_thread;

    struct mutex wasatch_mutex;
};

static DECLARE_WAIT_QUEUE_HEAD(spectrometer_wait);

static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static unsigned int dev_poll(struct file *file, poll_table *wait);

static struct spidev_data *wasatch_spiDev;

static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .release = dev_release,
   .poll = dev_poll
};

typedef struct {
    u8 LoVal;
    u8 HiVal;
    u8 Addr;
    u8 B_RW;
} Message;

static int debugMode = 0;


const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

/* **************************************************************************
 * Local Helpers
 * **************************************************************************
 */

/**
 * Helper function to execute SPI transfers
 *
 * @returns 0 on success
 */
static int spi_execute(struct spidev_data *spiDev, struct spi_transfer *t, const Message *aMsg, int len)
{
    int i;
    struct spi_message m;
    int err;

    memcpy((u32*)t->tx_buf, aMsg, 4);
    if (t->rx_buf)
        ((u32*)t->rx_buf)[0] = 0xefbeadde; /* Debugging ;-) */
    if (len > 0) {
        for (i = 1; i < len; ++i) {
            ((u32*)t->tx_buf)[i] = ((u32*)t->tx_buf)[0];
            if (t->rx_buf)
                ((u32*)t->rx_buf)[i] = 0xefbeadde;
        }
    }

    if (debugMode & 0x01) {
        pr_info("wasatch_spi_execute B_RW=%x Addr=%d HiVal=%x LoVal=%x (%u)\n", aMsg->B_RW, aMsg->Addr, aMsg->HiVal, aMsg->LoVal, ((int)(aMsg->HiVal) << 8) + aMsg->LoVal);
    }
    if (debugMode & 0x02) {
        print_hex_dump_bytes("wasatch_spi_execute tx  ", DUMP_PREFIX_NONE, t->tx_buf, (1 + len) * 4);
        pr_info("wasatch_spi_execute size=%u, bpw=%u\n", t->len, t->bits_per_word ? t->bits_per_word : spiDev->spi->bits_per_word);
    }

    spi_message_init_with_transfers(&m, t, 1);
    m.is_dma_mapped = 1;

    err = spi_sync(spiDev->spi, &m);

    if (err) {
        pr_err("wasatch_spi_execute error(%d), actual-length=%u, frame-length=%u, status=%d\n", err, m.actual_length, m.frame_length, m.status);
    }

    if ((debugMode & 0x02) && t->rx_buf) {
        print_hex_dump_bytes("wasatch_spi_execute rx  ", DUMP_PREFIX_NONE, t->rx_buf, (1 + len) * 4);
    }

    return err;
}

/**
 * Protocol function to write a single command
 *
 * @param struct spidev_data *spiDev Pointer to this driver instance.
 * @param const Message *aMsg Pointer to the message to send
 * @returns 0 on success
 */
static int spi_write_command(struct spidev_data *spiDev, const Message *aMsg)
{
    int err;

    err = spi_execute(spiDev, &spiDev->t[CHNL_COMMAND], aMsg, 0);

    return err;
}

/**
 * Protocol function to write a read command and read the result
 *
 * Read commands are two writes, the FPGA needs a toggle on the chip select to set the result ready.
 *
 * @param struct spidev_data *spiDev Pointer to this driver instance.
 * @param const Message *aMsg Pointer to the message to send
 * @param u32 *result, Pointer to the result variable.
 * @returns 0 on success
 */
static int spi_read_command(struct spidev_data *spiDev, const Message *aMsg, u32 *result)
{
    int err;

    err = spi_execute(spiDev, &spiDev->t[CHNL_COMMAND], aMsg, 0);
    if (err) return err;

    err = spi_execute(spiDev, &spiDev->t[CHNL_COMMAND], aMsg, 0);
    if (err) return err;

    if (debugMode & 0x02) {
        print_hex_dump_bytes("wasatch read result ", DUMP_PREFIX_NONE, spiDev->t[CHNL_COMMAND].rx_buf, 4);
    }

    *result = ((u32*)spiDev->t[CHNL_COMMAND].rx_buf)[0];

    return 0;
}

/**
 * Protocol function to read a row of spectrum data
 *
 * @param struct spidev_data *spiDev Pointer to this driver instance.
 * @returns 0 on success
 */
static int spi_read_spectrum(struct spidev_data *spiDev)
{
    Message msg = { .B_RW = FPGA_READ_RAM, .Addr = 0, .HiVal = 0, .LoVal = 0 };
    int err;
    int i;
    char output[(12 * 6 + 1) * 12];
    char *p;

    err = spi_execute(spiDev, &spiDev->t[CHNL_COMMAND], &msg, 0);
    if (err) return err;

    err = spi_execute(spiDev, &spiDev->t[CHNL_SPECTRUM], &msg, WASATCH_ROW_ELEMENTS); /* Repeat command message for all elements */
    if (err) return err;

    if (debugMode & 0x01) {
        p = &output[0];
        pr_info("spectrum result:\n");
        for (i = 0 ; i < WASATCH_ROW_ELEMENTS ; i++) {
            p += sprintf(p, " %5d", 0x0000ffff &  ((u32*)(spiDev->t[CHNL_SPECTRUM].rx_buf))[i]);

            if (!((i+1) % 12)) {
                p += sprintf(p, "\n");
                if (!((i+1) % 144)) {
                    pr_info("%s", output);
                    p = &output[0];
                }
            }
        }
        if (p != &output[0]) {
            pr_info("%s", output); /* Print remaining lines */
        }
    }

    return 0;
}

static int read_roi_start(struct spidev_data *spiDev, u32 *result)
{
    Message msg = { .Addr = REG_ROI_START, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;

    err = spi_read_command(spiDev, &msg, result);
    *result &= 0x000000FF;

    if (debugMode & 0x01)
        pr_info("wasatch: read_roi_start (%u) -> %d\n", *result, err);

    return err;
}

static int write_roi_start(struct spidev_data *spiDev, u32 value)
{
    int err;
    Message msg = { .Addr = REG_ROI_START, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };
    msg.LoVal = value;

    err = spi_write_command(spiDev, &msg);

    if (debugMode & 0x01)
        pr_info("wasatch: write_roi_start (%u) -> %d\n", value, err);

    return err;
}

static int read_roi_end(struct spidev_data *spiDev, u32 *result)
{
    Message msg = { .Addr = REG_ROI_END, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;

    err = spi_read_command(spiDev, &msg, result);
    *result &= 0x000000FF;

    if (debugMode & 0x01)
        pr_info("wasatch: read_roi_end (%u) -> %d\n", *result, err);

    return err;
}

static int write_roi_end(struct spidev_data *spiDev, u32 value)
{
    int err;
    Message msg = { .Addr = REG_ROI_END, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };
    msg.LoVal = value;

    err = spi_write_command(spiDev, &msg);

    if (debugMode & 0x01)
        pr_info("wasatch: write_roi_end (%d) -> %d\n", value, err);

    return err;
}

static int write_command(struct spidev_data *spiDev, u32 bit_mask)
{
    Message msg = { .Addr = REG_COMMAND, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };
    int err;
    u16 reg = (spiDev->command_reg & COMMAND_BINNING) | bit_mask;

    msg.LoVal = (u8)reg;
    msg.HiVal = (u8)(reg >> 8);

    err = spi_write_command(spiDev, &msg);
    if (err) return err;

    spiDev->command_reg = reg;

    return 0;
}

static int get_status(struct spidev_data *spiDev,
                 char *buf)
{
    Message msg = { .Addr = REG_STATUS, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;
    u32 result;

    err = spi_read_command(spiDev, &msg, &result);

    *buf = (char)result;

    if (debugMode & 0x02)
        pr_info("wasatch_get_status %d (%s)\n", *buf, bit_rep[*buf & 0x0F] );

    return err;
}

static int wait_for_status(struct spidev_data *spiDev, u8 bit_mask, int timeout_ms)
{
    int err;
    u8 status;

    timeout_ms = timeout_ms / 10; /* 10ms resolution */

    do {
        err = get_status(spiDev, &status);
        if (err) return err;

        if (status & bit_mask) {
            return 0;
        }

        mutex_unlock(&spiDev->wasatch_mutex);
        msleep(10);
        mutex_lock(&spiDev->wasatch_mutex);

        if (spiDev->spectrum_state != SPEC_STATE_INIT) {
            return -EINTR;
        }

    } while(timeout_ms--);

    return -ETIMEDOUT;
}

static ssize_t handle_error_and_return(struct spidev_data *spiDev, int err, int len)
{
    if (err) {
        dev_err(&spiDev->spi->dev, "device error: err(%d) len(%d)\n", err, len);
        return err; /* Other examples shows that low level operations return negative error codes, */
                    /* so I guess errno handling is done in user space read/write calls. */
/*        errno = err; */
/*        return -1; */
    }
    return len;
}

/*
static int set_ccd_power(struct spidev_data *spiDev, u8 enable)
{
    Message r_msg = { .Addr = REG_CDS_CONTROL, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    Message w_msg = { .Addr = REG_CDS_CONTROL, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };
    int err;
    u32 value;

    / * Read current value to keep CDS test mode * /
    err = spi_read_command(spiDev, &r_msg, &value);
    if (!err) {
        value &= (CDS_TEST_ENABLE | CDS_TEST_ACTUAL);
        value |= (enable ? (CCD_PWR_ENABLE | CCD_PWR_EN_3V3) : 0);

        w_msg.LoVal = value;

        err = spi_write_command(spiDev, &w_msg);

        if (debugMode & 0x01)
            pr_info("wasatch_set_ccd_power %d (%x)", w_msg.LoVal, w_msg.LoVal);
    }

    return err;
}
*/

/**
 * Thread function for reading in all requested lines of spectrum data.
 *
 * @param pointer spiDev Pointer to the device interface.
 * @return 0 on success
 */
static int read_spectrum_thrd(void *data)
{
    int err, row_count, current_row;
    int roi_start = 0;
    int roi_end = 0;
    u32 result;
    struct spidev_data *spiDev = (struct spidev_data *)data;

    if (debugMode & 0x01)
        pr_info("wasatch read_spectrum_thrd\n");

    mutex_lock(&spiDev->wasatch_mutex);

    if (spiDev->spectrum_state != SPEC_STATE_INIT) {
        err = -EBUSY;
        goto error_return;
    }

    /* Make sure we don't have old data pending in FPGA. */
    write_command(spiDev, COMMAND_CANCEL);

    if (!(spiDev->command_reg & COMMAND_BINNING)) {

        err = read_roi_start(spiDev, &result);
        if (err) goto error_return;
        roi_start = result & 0xFF;

        err = read_roi_end(spiDev, &result);
        if (err) goto error_return;
        roi_end = result & 0xFF;
    }

    spiDev->spectrum_size = (WASATCH_ROW_SIZE * (1 + roi_end - roi_start));

    if (spiDev->spectrum_size > (WASATCH_ROW_SIZE * WASATCH_MAX_ROWS)) {
        dev_err(&spiDev->spi->dev, "Spectrum size out of boundary (%ul)\n", spiDev->spectrum_size);
        spiDev->spectrum_size = 0;
        goto error_return;
    }

    err = write_command(spiDev, COMMAND_AQUIRE);
    if (err) goto error_return;

    err = wait_for_status(spiDev, STATUS_FRAME_AVAIL, 10000);
    if (err) goto error_return;

    err = write_command(spiDev, COMMAND_TRANSFER);
    if (err) goto error_return;

    err = wait_for_status(spiDev, STATUS_FRAME_RDY, 1000);
    if (err) goto error_return;

    row_count = spiDev->spectrum_size / WASATCH_ROW_SIZE;

    for (current_row = 0 ; current_row < row_count ; current_row++) {

        /* Check for abort!!! */
        if (spiDev->spectrum_state != SPEC_STATE_INIT) {
            err = -EINTR;
            goto error_return;
        }

        if (debugMode & 0x01) {
            pr_info("wasatch read_spectrum_thrd: Reading row(%d) of (%d)\n",  current_row, row_count);
        }

        err = spi_read_spectrum(spiDev);
        if (err) goto error_return;

        memcpy(&spiDev->spectrum_data[WASATCH_ROW_SIZE * current_row], spiDev->t[CHNL_SPECTRUM].rx_buf, WASATCH_ROW_SIZE);

        if (1 < row_count && current_row != row_count - 1) {
            /* Wait for next row */
            err = wait_for_status(spiDev, STATUS_FRAME_RDY, 2000);
            if (err) {
                dev_err(&spiDev->spi->dev, "Row status error (%d)\n", err);
                print_hex_dump_bytes("tx: ", DUMP_PREFIX_OFFSET, spiDev->t[CHNL_SPECTRUM].tx_buf, 32);
                print_hex_dump_bytes("rx: ", DUMP_PREFIX_OFFSET, spiDev->t[CHNL_SPECTRUM].rx_buf, 32);
                goto error_return;
            }
        }

    }

    if (debugMode & 0x01) {
        pr_info("wasatch read_spectrum_thrd: data ready\n");
    }
    spiDev->spectrum_state = SPEC_STATE_DATA;
    wake_up_interruptible(&spectrometer_wait);

error_return:

    /* Always try to clear capture, we don't care for the result */
    write_command(spiDev, COMMAND_CANCEL);

    mutex_unlock(&spiDev->wasatch_mutex);

    if (debugMode & 0x01) {
        pr_info("wasatch read_spectrum_thrd: exit(%d)\n", err);
    }

    do_exit(err);

    return err;
}

static int fpga_adc_to_temp(u16 aAdcCount)
{
    /* Use sorted list, to make binary jump search. */
    /* Max cost is 7 lookups. */
    /* Index -40 is temperature. */
    const u16 cTmpValues[] = { /* From -40 to 125 degrees C */
        3798, 3796, 3795, 3793, 3792, 3790, 3788, 3786, 3785, 3782,
        3781, 3780, 3779, 3777, 3775, 3773, 3771, 3770, 3768, 3766,
        3765, 3764, 3762, 3759, 3756, 3754, 3752, 3751, 3750, 3748,
        3746, 3744, 3742, 3740, 3738, 3736, 3733, 3732, 3731, 3730,
        3727, 3725, 3721, 3720, 3719, 3717, 3715, 3713, 3711, 3709,
        3707, 3704, 3703, 3702, 3700, 3699, 3698, 3697, 3696, 3695,
        3688, 3684, 3682, 3680, 3678, 3677, 3676, 3673, 3670, 3667,
        3666, 3664, 3662, 3660, 3658, 3656, 3654, 3651, 3648, 3645,
        3643, 3642, 3641, 3640, 3638, 3636, 3634, 3632, 3630, 3628,
        3625, 3622, 3619, 3616, 3613, 3610, 3607, 3604, 3601, 3598,
        3595, 3594, 3593, 3592, 3591, 3590, 3589, 3585, 3582, 3579,
        3576, 3573, 3570, 3567, 3564, 3561, 3558, 3555, 3552, 3551,
        3550, 3549, 3548, 3547, 3546, 3542, 3538, 3534, 3530, 3526,
        3525, 3524, 3522, 3519, 3516, 3513, 3510, 3507, 3504, 3501,
        3500, 3498, 3496, 3494, 3492, 3490, 3489, 3486, 3483, 3480,
        3477, 3474, 3471, 3468, 3465, 3461, 3460, 3459, 3456, 3451,
        3450, 3449, 3445, 3440, 3432, 3431
    };
    int jump = ((sizeof(cTmpValues) / sizeof(u16)) + 1) / 2;
    int i = 0;

/*    std::cout << "wasatch fpga_adc_to_temp(" << aAdcCount << ")\n"; */

/*    std::cout << "Size: " << sizeof(cTmpValues) << "Size: " << sizeof(int) << "\n"; */

    while (true) {

/*        std::cout << "jump: " << jump << " i: " << i << " value: " << (int)cTmpValues[i] << "\n"; */

        if (cTmpValues[i] == aAdcCount) {
            break;
        }
        else if (cTmpValues[i] < aAdcCount) {
            i -= jump;
        }
        else {
            i += jump;
        }

        if (i < 0) {
            return -40;
        }
        else if (i > (int)(sizeof(cTmpValues) / sizeof(u16))) {
            return 125;
        }

        if (jump == 0) {
            break;
        }
        else if (jump > 1) {
            jump = (jump + 1) / 2;
        }
        else {
            jump /= 2;
        }
    }

    return i - 40;
}

/* **************************************************************************
 * Exported methods
 * **************************************************************************
 */

/**
 * Show the CCD binning mode
 *
 * @param struct device *dev Pointer to device
 * @param struct device_attribute *attr Pointer to device attributes
 * @param const char *buf Pointer to output buffer
 */
static ssize_t wasatch_show_binning(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data *) dev->driver_data;

    mutex_lock(&spiDev->wasatch_mutex);

    *buf = ((spiDev->command_reg & COMMAND_BINNING) != 0) ? '1' : '0';
    buf[1] = '\n'; /* zero terminator */

    if (debugMode & 0x01)
        pr_info("wasatch_show_binning -> %c\n", *buf);

    mutex_unlock(&spiDev->wasatch_mutex);

    return 2;
}

/**
 * Set the CCD binning mode
 *
 * Changing binning always cancels an evt ongoing capture.
 *
 * @param struct device *dev Pointer to device
 * @param struct device_attribute *attr Pointer to device attributes
 * @param const char *buf Pointer to input buffer
 * @param size_t count Length of input buffer in bytes.
 */
static ssize_t wasatch_store_binning(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    int err;
    struct spidev_data *spiDev = (struct spidev_data *) dev->driver_data;

    if (count < 1) {
        return handle_error_and_return(spiDev, -ENOBUFS, count);
    }

    mutex_lock(&spiDev->wasatch_mutex);

    if (debugMode & 0x01) pr_info("wasatch_store_binning  binning = %c\n", *buf);

    spiDev->command_reg = ((*buf == '0') || (*buf == 0)) ? 0 : COMMAND_BINNING;

    err = write_command(spiDev, COMMAND_CANCEL);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, count);
}


static ssize_t wasatch_show_status(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    int err;
    u8 result;
    struct spidev_data *spiDev = (struct spidev_data *) dev->driver_data;

    mutex_lock(&spiDev->wasatch_mutex);

    err = get_status(spiDev, &result);

    sprintf(buf, "%u\n", (u32)result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_store_roi_start(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    u32 roi_end = 0;
    u32 roi_start = 0;
    int err;
    int c;

    c = sscanf(buf, "%u", &roi_start);
    if (c != 1) {
        dev_err(dev, "wasatch_store_roi_start invalid param (%s), %d, %u, %u\n", buf, c, count, roi_start);
        return -EINVAL;
    }

    if (roi_start > 69) {
        dev_err(dev, "wasatch_store_roi_start param out of range %d, (%s)\n", roi_start, buf);
        return -EINVAL;
    }

    mutex_lock(&spiDev->wasatch_mutex);

    err = read_roi_end(spiDev, &roi_end);
    if (err) goto error_return;

    if (roi_start > roi_end) {
        err = write_roi_end(spiDev, roi_start);
        if (err) goto error_return;
    }

    err = write_roi_start(spiDev, roi_start);

error_return:
    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, count);
}

static ssize_t wasatch_show_roi_start(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    int err = 0;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = read_roi_start(spiDev, &result);

    sprintf(buf, "%u\n", result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_store_roi_end(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    int err;
    u32 roi_start = 0;
    u32 roi_end = 0;
    int c;

    c = sscanf(buf, "%u", &roi_end);
    if (c != 1) {
        dev_err(dev, "wasatch_store_roi_end invalid param (%s), %d, %u, %u\n", buf, c, count, roi_end);
        return -EINVAL;
    }

    if (roi_end > 69) {
        dev_err(dev, "wasatch_store_roi_end param out of range %d, (%s)\n", roi_end, buf);
        return -EINVAL;
    }

    mutex_lock(&spiDev->wasatch_mutex);

    err = read_roi_start(spiDev, &roi_start);
    if (err) goto error_return;

    if (roi_end < roi_start) {
        err = write_roi_start(spiDev, roi_end);
        if (err) goto error_return;
    }

    err = write_roi_end(spiDev, roi_end);

error_return:
    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, count);
}

static ssize_t wasatch_show_roi_end(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    int err = 0;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = read_roi_end(spiDev, &result);

    sprintf(buf, "%u\n", result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_store_integration_time(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    u32 integrationTime;
    int err;
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_INTEGR_TIME, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };

    if (sscanf(buf, "%u", &integrationTime) != 1) {
        dev_err(dev, "wasatch_store_integration_time invalid param %s\n", buf);
        return -EINVAL;
    }
    if (integrationTime < 1 || integrationTime > 4000) {
        dev_err(dev, "wasatch_store_integration_time param out of range %d\n", integrationTime);
        return -EINVAL;
    }

    mutex_lock(&spiDev->wasatch_mutex);

    msg.LoVal = integrationTime;
    msg.HiVal = integrationTime >> 8;

    err = spi_write_command(spiDev, &msg);

    if (debugMode & 0x01)
        pr_info("wasatch_store_integration_time  %d\n", integrationTime);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, count);
}

static ssize_t wasatch_show_integration_time(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_INTEGR_TIME, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err = 0;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &result);
    result &= 0xFFFF;

    sprintf(buf, "%u\n", result);

    if (debugMode & 0x01)
        pr_info("wasatch_show_integration_time  %u\n", result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_show_fpga_temp(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_FPGA_TEMP, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;
    u32 adc_count;
    int temp;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &adc_count);

    adc_count &= 0x0fff;

    temp = fpga_adc_to_temp((u16)adc_count);

    sprintf(buf, "%d\n", temp);

    if (debugMode & 0x01)
        pr_info("wasatch_show_fpga_temp  %d\n", temp);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_show_sensor_temp(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_CCD_TEMP, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &result);

    result &= 0x0fff;

    sprintf(buf, "%u\n", result);

    if (debugMode & 0x01)
        pr_info("wasatch_show_sensor_temp  %u\n", result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

/**
 * Show the current amount in raw ADC counts.
 */
static ssize_t wasatch_show_sensor_itec_raw(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_CCD_ITEC, .B_RW = FPGA_READ };
    int err;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &result);

    result &= 0x0fff;
    sprintf(buf, "%u\n", result);

    if (debugMode & 0x01) {
        pr_info("wasatch_show_sensor_itec_raw %u\n", result);
    }

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

/**
 * Show the ADC count converted to actual current (Amperes)
 */
static ssize_t wasatch_show_sensor_itec(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_CCD_ITEC, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;
    u32 adc_count;
    s32 ampere;
    const char *sign = "";

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &adc_count);

    adc_count &= 0x0fff;

    /*
     * The Kernel has no math functionality, so convert original formula to integer calculation and encode result:
     *
     * ampere = ((double)adc_count * 0.001914463563) - 2.757352941;
     *
     * We can multiply constants with 100.000.000 to calculate with 8 decimals.
     */
    ampere = ((adc_count * 191446) - 275735294);
    if (ampere < 0) {
        sign = "-";
        ampere = (ampere - 500000) / 100000; /* Round down to 3 decimals */
        ampere *= -1;
    }
    else {
        ampere = (ampere + 500000) / 100000; /* Round up to 3 decimals */
    }
    sprintf(buf, "%s%d.%03d\n", sign, ampere / 1000, ampere % 1000);

    if (debugMode & 0x01) {
        pr_info("wasatch_show_sensor_itec %s", buf);
    }

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_store_debugmode(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_TEST_MODE, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };
    int err;
    u32 testmode;

    if (sscanf(buf, "%u", &testmode) != 1) {
        dev_err(dev, "wasatch_store_debugmode invalid param %s\n", buf);
        return -EINVAL;
    }

    mutex_lock(&spiDev->wasatch_mutex);

    if (testmode & 0x01) {
        pr_info("fpga testmode = 1");
        msg.LoVal = 1;
    } else {
        pr_info("fpga testmode = 0");
        msg.LoVal = 0;
    }

    debugMode = (testmode & 0xfe) >> 1;

    pr_info("wasatch_set_debugmode %x (%s) fpga(%x)", debugMode, bit_rep[debugMode & 0x0F], msg.LoVal);

    err = spi_write_command(spiDev, &msg);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, count);
}


static ssize_t wasatch_show_fpga_version(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    Message msg = { .Addr = REG_FPGA_VERSION, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err = 0;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &result);

    *buf = 0;
    sprintf(buf, "%u\n", result);

    if (debugMode & 0x01)
        pr_info("wasatch_show_fpga_version %u\n", result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_show_test_value(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    Message msg = { .Addr = REG_TEST_VALUE, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    u32 result = 0;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &result);

    *buf = 0;
    sprintf(buf, "%u\n", result);

    if (debugMode & 0x01)
        pr_info("wasatch_get_test_value 0x%x %s\n",
                result, (result == 0x12489669) ? "OK" : "FAIL");

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_store_cds_control(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    Message msg = { .Addr = REG_CDS_CONTROL, .B_RW = FPGA_WRITE, .HiVal = 0, .LoVal = 0 };
    int err;
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    u32 value;

    if (sscanf(buf, "%u", &value) != 1) {
        dev_err(dev, "wasatch_store_cds_control invalid param %s\n", buf);
        return -EINVAL;
    }

    mutex_lock(&spiDev->wasatch_mutex);

    msg.LoVal = (value & (CDS_TEST_ENABLE | CDS_TEST_ACTUAL)); /* | CCD_PWR_EN_3V3 | CCD_PWR_ENABLE; */

    if (debugMode & 0x01)
        pr_info("wasatch_set_cds_control %d (%x)", msg.LoVal, msg.LoVal);

    err = spi_write_command(spiDev, &msg);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, count);
}

static ssize_t wasatch_show_cds_control(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    Message msg = { .Addr = REG_CDS_CONTROL, .B_RW = FPGA_READ, .HiVal = 0, .LoVal = 0 };
    int err;
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    err = spi_read_command(spiDev, &msg, &result);

    *buf = 0;
    sprintf(buf, "%u\n", result & (CDS_TEST_ENABLE | CDS_TEST_ACTUAL));

    if (debugMode & 0x01)
        pr_info("wasatch_get_cds_control %d (%x)\n", result, result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return handle_error_and_return(spiDev, err, strlen(buf));
}

static ssize_t wasatch_store_standby(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    u32 value;

    if (sscanf(buf, "%u", &value) != 1) {
        dev_err(dev, "wasatch_store_standby invalid param %s\n", buf);
        return -EINVAL;
    }

    if (debugMode & 0x01)
        pr_info("wasatch_store_standby %d\n", value);

    mutex_lock(&spiDev->wasatch_mutex);

    if (!value) {
        /* resume */
        /* Set the 40MHZ_FPGA_EN pin on carrier board */
        gpio_set_value(FPGA_CLK_PIN, 1);
/*        set_ccd_power(spiDev, 1); */
    } else {
        /* suspend */
/*        set_ccd_power(spiDev, 0); */
        gpio_set_value(FPGA_CLK_PIN, 0);
    }


    mutex_unlock(&spiDev->wasatch_mutex);

    return count;
}

static ssize_t wasatch_show_standby(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int err;
    struct spidev_data *spiDev = (struct spidev_data*) dev->driver_data;
    u32 result;

    mutex_lock(&spiDev->wasatch_mutex);

    result = ! gpio_get_value(FPGA_CLK_PIN);

    *buf = 0;
    sprintf(buf, "%u\n", result);

    if (debugMode & 0x01)
        pr_info("wasatch_show_standby %d (%x)\n", result, result);

    mutex_unlock(&spiDev->wasatch_mutex);

    return strlen(buf);
}

static DEVICE_ATTR(binning, S_IWUSR | S_IRUSR, wasatch_show_binning, wasatch_store_binning);
static DEVICE_ATTR(status, S_IRUSR, wasatch_show_status, NULL);
static DEVICE_ATTR(roi_start, S_IWUSR | S_IRUSR, wasatch_show_roi_start, wasatch_store_roi_start);
static DEVICE_ATTR(roi_end, S_IWUSR | S_IRUSR, wasatch_show_roi_end, wasatch_store_roi_end);
static DEVICE_ATTR(integration_time, S_IWUSR | S_IRUSR, wasatch_show_integration_time, wasatch_store_integration_time);
static DEVICE_ATTR(sensor_temp, S_IRUSR, wasatch_show_sensor_temp, NULL);
static DEVICE_ATTR(sensor_itec_raw, S_IRUSR, wasatch_show_sensor_itec_raw, NULL);
static DEVICE_ATTR(sensor_itec, S_IRUSR, wasatch_show_sensor_itec, NULL);
static DEVICE_ATTR(fpga_temp, S_IRUSR, wasatch_show_fpga_temp, NULL);
static DEVICE_ATTR(debugmode, S_IWUSR, NULL, wasatch_store_debugmode);
static DEVICE_ATTR(fpga_version, S_IRUSR, wasatch_show_fpga_version, NULL);
static DEVICE_ATTR(test_value, S_IRUSR, wasatch_show_test_value, NULL);
static DEVICE_ATTR(cds_control, S_IWUSR | S_IRUSR, wasatch_show_cds_control, wasatch_store_cds_control);
static DEVICE_ATTR(standby, S_IWUSR | S_IRUSR, wasatch_show_standby, wasatch_store_standby);

static struct attribute *wasatch_attributes[] = {
    &dev_attr_binning.attr,
    &dev_attr_status.attr,
    &dev_attr_roi_start.attr,
    &dev_attr_roi_end.attr,
    &dev_attr_integration_time.attr,
    &dev_attr_sensor_temp.attr,
    &dev_attr_sensor_itec_raw.attr,
    &dev_attr_sensor_itec.attr,
    &dev_attr_fpga_temp.attr,
    &dev_attr_debugmode.attr,
    &dev_attr_fpga_version.attr,
    &dev_attr_test_value.attr,
    &dev_attr_cds_control.attr,
    &dev_attr_standby.attr,
    NULL
};

static const struct attribute_group wasatch_attr_group = {
    .attrs = wasatch_attributes,
};


static const struct attribute_group *attr_groups[] = {
    &wasatch_attr_group,
    NULL,
};

/**
 * Helper function to allocate the transfer blocks.
 *
 * @param pointer spiDev Pointer to SPI protocol device
 * @param int channel Channel number to initialize
 * @param int outBuf Size of output buffer
 * @param int inBuf Size of input buffer
 * @returns bool true on success
 */
static bool wasatch_init_channel(struct spidev_data *spiDev, int channel, int outBuf, int inBuf)
{
    struct device *dev = &spiDev->spi->dev;
    struct spi_transfer *t = &spiDev->t[channel];

    if (outBuf > 0) {
        t->tx_buf = dmam_alloc_coherent(dev, outBuf*4, &t->tx_dma, GFP_KERNEL);
        if (t->tx_buf == 0) {
            return false;
        }
    } else {
        t->tx_buf = NULL;
        t->tx_dma = 0;
    }

    if (inBuf > 0) {
        t->rx_buf = dmam_alloc_coherent(dev, inBuf*4, &t->rx_dma, GFP_KERNEL);
        if (t->rx_buf == 0) {
            return false;
        }
    } else {
        t->rx_buf = NULL;
        t->rx_dma = 0;
    }

    t->len = (outBuf ? outBuf : inBuf) * 4;
    t->bits_per_word = 32;
    t->speed_hz = 5000000;
    t->delay_usecs = 0;

    return true;
}

/**
 * Initialize module. Called on module load.
 * This function sets up the memory and spi driver.
 *
 * @param spi_device* spi Pointer to spi devcie struct.
 * @returns int errno
 */
static int wasatch_probe(struct spi_device *spi)
{
    int ret;
    struct spidev_data *spiDev;

    pr_notice("wasatch_probe\n");

    gpio_request(FPGA_CLK_PIN, "wasatch");
    gpio_direction_output(FPGA_CLK_PIN, 1);
    spiDev = kzalloc(sizeof(*spiDev), GFP_KERNEL);
    if (!spiDev) {
        return -ENOMEM;
    }

/*    gpio_request(115, "wasatch"); */

    spiDev->spi = spi;

    spiDev->spi->dev.coherent_dma_mask = ~0;

    memset(&spiDev->t, 0, sizeof(spiDev->t));

    /* command: single 32-bit, reads are send twice */
    if (!wasatch_init_channel(spiDev, CHNL_COMMAND, 1, 1)) {
        return -ENOMEM;
    }

    /* read spectrum: read entire row, copy write command to all outputs while reading */
    if (!wasatch_init_channel(spiDev, CHNL_SPECTRUM, WASATCH_ROW_ELEMENTS, WASATCH_ROW_ELEMENTS)) {
        return -ENOMEM;
    }

    /* Initialize cached variables */
    spiDev->command_reg = 0;
    spiDev->spectrum_state = SPEC_STATE_NONE;

    /* Set defaults for spi */
    spi->bits_per_word = 32;
    spi->mode = SPI_MODE_2;
    spi->max_speed_hz = 5000000;

    ret = spi_setup(spi);
    if (ret < 0)
        return ret;

    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber < 0) {
        dev_err(&spi->dev, "Wasatch failed to register a major number\n");
        return majorNumber;
    }

    cameraCharClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(cameraCharClass)) {
        unregister_chrdev(majorNumber, DEVICE_NAME);
        dev_err(&spi->dev, "Failed to register device class\n");
        return PTR_ERR(cameraCharClass);
    }
    cameraCharClass->dev_groups = attr_groups;

    wasatchCharDevice = device_create(cameraCharClass, &spi->dev, MKDEV(majorNumber, 0), spiDev, DEVICE_NAME);
    if (IS_ERR(wasatchCharDevice)) {
        class_destroy(cameraCharClass);
        unregister_chrdev(majorNumber, DEVICE_NAME);
        dev_err(&spi->dev, "Failed to create the device\n");
        return PTR_ERR(wasatchCharDevice);
    }

    wasatch_spiDev = spiDev;

    spi_set_drvdata(spi, spiDev);

    gpio_set_value(FPGA_CLK_PIN, 1);
/*    set_ccd_power(spiDev, 1); */

    mutex_init(&spiDev->wasatch_mutex);

    return 0;
}

static int wasatch_remove(struct spi_device *spi)
{
    struct spidev_data *spiDev = (struct spidev_data*)spi_get_drvdata(spi);

    pr_notice("wasatch_remove \n");

/*    set_ccd_power(spiDev, 0); */
    gpio_set_value(FPGA_CLK_PIN, 0);

    device_destroy(cameraCharClass, MKDEV(majorNumber, 0));
    class_unregister(cameraCharClass);
    class_destroy(cameraCharClass);
    unregister_chrdev(majorNumber, DEVICE_NAME);

    kfree(spiDev);

    gpio_free(FPGA_CLK_PIN);

    return 0;
}

/* **********************************************************************
 * Device handlers for /dev/spectrometer
 * **********************************************************************
 */

static int dev_open(struct inode *inodep, struct file *filep)
{
    char thread_name[16]="spectrum_thread";
    int err = 0;
    struct spidev_data *spiDev = wasatch_spiDev;

    if (debugMode & 0x01) printk("dev_open enter. state(%u)\n", spiDev->spectrum_state);

    mutex_lock(&spiDev->wasatch_mutex);

    /* This forces single user use of this file handle */
    if (spiDev->spectrum_state == SPEC_STATE_NONE) {

        spiDev->spectrum_state = SPEC_STATE_INIT;
        filep->private_data = spiDev;

        nonseekable_open(inodep, filep);

        spiDev->spectrum_thread = kthread_run(read_spectrum_thrd, spiDev, thread_name);

        if (spiDev->spectrum_thread == ERR_PTR(-ENOMEM)) {
            err = (int)spiDev->spectrum_thread;
        }

    } else {
        err = -EBUSY;
    }

    mutex_unlock(&wasatch_spiDev->wasatch_mutex);

    if (debugMode & 0x01) printk("dev_open exit. state(%u) return(%d)\n", spiDev->spectrum_state, err);

    return err;
}

static int dev_release(struct inode *inodep, struct file *filep)
{
    struct spidev_data *spiDev = filep->private_data;
    int ret = 0;

    if (debugMode & 0x01) pr_info("dev_release enter. state(%u)\n", spiDev->spectrum_state);

    mutex_lock(&spiDev->wasatch_mutex);

    spiDev->spectrum_state = SPEC_STATE_NONE;

    mutex_unlock(&spiDev->wasatch_mutex);

    if (debugMode & 0x01) pr_info("dev_release exit. state(%u) return(%d)\n", spiDev->spectrum_state, ret);

    return ret;
}

static unsigned int dev_poll(struct file *filep, poll_table *wait)
{
    unsigned int ret = 0;
    struct spidev_data *spiDev = filep->private_data;

    if (debugMode & 0x01) pr_info("dev_poll enter. state(%u)\n", spiDev->spectrum_state);

    poll_wait(filep, &spectrometer_wait, wait);

    if (spiDev->spectrum_state == SPEC_STATE_DATA) {
        ret = POLLIN | POLLRDNORM;
    }

    if (debugMode & 0x01) pr_info("dev_poll exit. state(%u) return(%u)\n", spiDev->spectrum_state, ret);

    return ret;
}


static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    int remaining;
    u32 result;
    struct spidev_data *spiDev = filep->private_data;

    if (debugMode & 0x01)
        pr_info("wasatch dev_read enter. len(%u) offset(%u) state(%u)\n", (u32)len, (u32)*offset, spiDev->spectrum_state );

    if(spiDev->spectrum_state != SPEC_STATE_DATA) {
        if (filep->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
    }

    while (spiDev->spectrum_state != SPEC_STATE_DATA) {
        if (spiDev->spectrum_state == SPEC_STATE_NONE) {
            return -ENODATA;
        }
        msleep(10);
    }

    if ((*offset > spiDev->spectrum_size) && (len > 0)) {
        return -ENOBUFS;
    }

    result = min((int)len, max(0, (int)(spiDev->spectrum_size - *offset)));

    if (debugMode & 0x01)
        pr_info("dev_read: Copying data offset(%u), result(%u)\n", (u32)*offset, result);

    /* copy_to_user returns number of bytes that could NOT be copied: 0 = success. */
    remaining = copy_to_user(buffer, &spiDev->spectrum_data[*offset], result);
    if(0 != remaining) {
        dev_err(&spiDev->spi->dev, "copy_to_user failed. offset=(%u), remaining=(%d)\n", (u32)*offset, remaining);
        return -ENOBUFS;
    }

    *offset += result;

    if (debugMode & 0x01)
        pr_info("wasatch dev_read exit. state(%u) return(%u)\n", spiDev->spectrum_state, result );

    return result;
}


#ifdef CONFIG_PM
static int wasatch_suspend(struct device *dev)
{
    const char *command = "1";

    pr_notice("wasatch_suspend\n");

    wasatch_store_standby(dev, NULL, command, 2);

    return 0;
}

static int wasatch_resume(struct device *dev)
{
    const char *command = "0";

    pr_notice("wasatch_resume\n");

    wasatch_store_standby(dev, NULL, command, 2);

    return 0;
}
#endif

SIMPLE_DEV_PM_OPS(wasatch_pm_ops, wasatch_suspend, wasatch_resume);
static const struct spi_device_id wasatch_id_table[] = {
    { "spectrometer" },
    { }
};

MODULE_DEVICE_TABLE(spi, wasatch_id_table);

static const struct of_device_id wasatch_of_match[] = {
    { .compatible = "wasatch,spectrometer", },
    { .compatible = "wasatch", },
    { }
};

MODULE_DEVICE_TABLE(of, wasatch_of_match);

static struct spi_driver wasatch_driver = {
    .driver = {
        .name   = "wasatch",
        .of_match_table = wasatch_of_match,
        .pm = &wasatch_pm_ops,
    },
    .probe  = wasatch_probe,
    .remove = wasatch_remove,
    .id_table = wasatch_id_table,
};

module_spi_driver(wasatch_driver);


MODULE_AUTHOR("Markku Nivala <markku.nivala@innokasmedical.fi>, Steffen Brummer <steffen@rspsystems.com>");
MODULE_DESCRIPTION("Driver for RSP Custom FPGA for Wasatch module");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.6");
MODULE_ALIAS("spi:wasatch");

