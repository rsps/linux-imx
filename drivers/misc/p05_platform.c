/*
 * p05_device.c - Linux kernel module for
 * RSP P0.5 Glucobeam device
 *
 *  Copyright (c) 2020 Steffen Brummer, RSP Systems
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Guides:
 *   https://www.kernel.org/doc/html/latest/driver-api/index.html
 *   https://www.kernel.org/doc/html/latest/driver-api/driver-model/platform.html
 *
 * History:
 *             ver. 0.1: Initial release
 *  2020-05-28 ver. 0.2: Add information to all error conditions in check_laser_on_requirements function
 *  2020-06-02 ver. 0.3: Add proximity reset if stuck high algorithm.
 *  2020-06-30 ver. 0.4: Changed power reads to use delayed variable contents.
 *  2020-07-03 ver. 0.5: Increased delays around power toggles.
 *  2020-09-04 ver. 0.6: Add check for laser on requirements on all safety sensor lookup, if laser is already on.
 *                       Used to turn of the laser on signal (blue LED) as fast as possible in case conditions are no longer met.
 *  2020-10-23 ver. 0.7: Increase time from aux on to goodix driver release
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <stdbool.h>

#define DRIVER_NAME "p05v2"

/* Platform specific GPIO definitions */
#define DISPLAY_3V3_EN   115
#define AUX_PWR_EN         1
#define ENABLE_5V        116
#define LASER_BRD_PWR_ON  25
#define LASER_TEC_EN      10
#define CCD_TEC_EN        19
#define LASER_ON           0
#define _HALL_BUF          9
#define PROX_OK_BUFF       5
#define IMP_OK            37


#define PROXIMITY_RESET_WORK_DIV 10

static int debug_mode = 0;
static int aux_power_state = 1;
static int laser_board_power_state = 1;
static int supply_5v_power_state = 1;


struct p05v2_state {
    struct platform_device *pdev;
    struct class*  platformClass;
    struct device* p05v2Device;

    struct gpio_desc *power5v;
    struct gpio_desc *power_aux;
    struct gpio_desc *power_display;

    struct gpio_desc *impedance_ok;
    struct gpio_desc *proximity_ok;
    struct gpio_desc *_hall_closed;

    struct gpio_desc *laser_board_pwr;
    struct gpio_desc *laser_on;

    struct gpio_desc *laser_tec_en;
    struct gpio_desc *laser_tec_ut;
    struct gpio_desc *laser_tec_ot;
    struct gpio_desc *laser_shutdown;

    struct gpio_desc *ccd_tec_en;
    struct gpio_desc *ccd_tec_ut;
    struct gpio_desc *ccd_tec_ot;

    struct gpio_desc *audio_output_en;

    struct delayed_work work;
    int proximity_check_state;
};

extern void m41t80_wdt_ping(void);
extern void m41t80_wdt_disable(void);


/* ************ Helpers ****************** */

static ssize_t handle_error_and_return(struct device *dev, int err, int len)
{
    if (err < 0) {
        pr_err("device error: err(%d) len(%d)\n", err, len);
        return err;
    }
    return len;
}

static struct p05v2_state * getState(struct device *dev)
{
/*
    pr_info("Device pointer: (%p)", dev);
    pr_info("Device driver data: %p", dev->driver_data);
    pr_info("Device kObject pointer: (%p)", &dev->kobj);
    pr_info("Device kObject name: %s", dev->kobj.name);
*/
    return dev_get_drvdata(dev);
}

static int getU32Param(struct device *dev, const char *buf, u32 *value)
{
    if (sscanf(buf, "%u", value) != 1) {
        dev_err(dev, "Invalid param %s\n", buf);
        return -EINVAL;
    }

    if (debug_mode & 0x01) {
        pr_info("Got parameter %u\n", *value);
    }

    return 0;
}

#define DBG_NOTICE(msg) \
    if (debug_mode & 0x01) { \
        pr_notice(msg); \
    }

#define DBG_NOTICE1(msg, a) \
    if (debug_mode & 0x01) { \
        pr_notice(msg, a); \
    }

#define DBG_NOTICE2(msg, a, b) \
    if (debug_mode & 0x01) { \
        pr_notice(msg, a, b); \
    }


static int readIOAndSetU32Result(struct device *dev, char *buf, struct gpio_desc *gpio)
{
    int ret;

/*    pr_info("readIOAndSetU32Result: %p %p %p", dev, buf, gpio);*/

    if (!dev) {
        pr_err("Device handle is NULL\n");
        return -EFAULT;
    }
    if (!buf) {
        pr_err("Output buffer is NULL\n");
        return -EFAULT;
    }

    if (!gpio) {
        pr_err("GPIO handle is NULL\n");
        return -EFAULT;
    }

    ret = gpiod_get_value(gpio);
    if (ret < 0) {
        pr_err("Error reading IO (%d)\n", ret);
        return ret;
    }

    *buf = 0;
    sprintf(buf, "%u\n", (u32)ret);

    DBG_NOTICE1("Returning value %u\n", (u32)ret);

    return 0;
}

static void p05_proximity_reset_work(struct work_struct *work)
{
    struct p05v2_state *state;
    int ret;
    static int impedance_ok_count = 0;

    state = container_of(work, struct p05v2_state, work.work);

    pr_notice("p05_proximity_reset_work. State = %d\n", state->proximity_check_state);

    if (state->proximity_check_state < 20) {
        ret = gpiod_get_value(state->proximity_ok);
        if (ret < 0) {
            pr_err("proximity_ok read error: err(%d)\n", ret);
            goto stop_work;
        }
        else if (!ret) {
            /* Proximity is low, so it is not stuck high. Just stop checking. */
            goto stop_work;
        }

        ret = gpiod_get_value(state->_hall_closed);
        if (ret < 0) {
            pr_err("hall read error: err(%d)\n", ret);
            goto stop_work;
        }
        else if (!ret) {
            /* Hall is open, we can detect hand on impedance */
            ret = gpiod_get_value(state->impedance_ok);
            if (ret < 0) {
                pr_err("impedance_ok read error: err(%d)\n", ret);
                goto stop_work;
            }
            else if (ret) {
                /* Impedance is high again, so it was just a hand adjustment. Stop checking proximity. */
                if (impedance_ok_count++ > 2) {
                    goto stop_work;
                }
            }
            else {
                impedance_ok_count = 0;
            }
        }

        /* Tell Goodix driver that power is lost. (~500ms notice) */
        if (state->proximity_check_state == 15) {
            aux_power_state = 0;
        }
    }
    else if (state->proximity_check_state == 20) {
        gpiod_set_value(state->power_aux, 0);
    }
    else if (state->proximity_check_state == 22) {
        gpiod_set_value(state->power_aux, 1);
    }
    else if (state->proximity_check_state == 28) {
        /* Read proximity again to verify that it is low. */
        ret = gpiod_get_value(state->proximity_ok);
        if (ret < 0) {
            pr_err("proximity_ok read error: err(%d)\n", ret);
            goto stop_work;
        }
        else if (!ret) {
            /* Proximity is low, so it is not stuck high. Just stop checking. */
            goto stop_work;
        }
        else {
            state->proximity_check_state = 19;
        }
    }

    state->proximity_check_state++;

    schedule_delayed_work(&state->work, HZ / PROXIMITY_RESET_WORK_DIV);
    return;

stop_work:
    aux_power_state = 1;
    state->proximity_check_state = 0;

    pr_notice("Stopping p05_proximity_reset_work. Err(%d)\n", ret);
}

/**
 * Helper function to check all IO's are in a correct state before turning on the laser.
 *
 * @param aState
 * @return Negative errno on failure, 0 on success
 */
static int check_laser_on_requirements(struct p05v2_state *aState)
{
    int ret;
    int hall;
    int imp_ok;
    int prox_ok;

    if (!aux_power_state) {
        pr_err("power_aux is low\n");
        return -EIO;
    }

    if (!supply_5v_power_state) {
        pr_err("power5v is low\n");
        return -EIO;
    }

    if (!laser_board_power_state) {
        pr_err("laser board power is low\n");
        return -EIO;
    }


    ret = gpiod_get_value(aState->_hall_closed);
    if (ret < 0) {
        pr_err("hall error: err(%d)\n", ret);
        return ret;
    }
    hall = ret;

    ret = gpiod_get_value(aState->impedance_ok);
    if (ret < 0) {
        pr_err("impedance_ok error: err(%d)\n", ret);
        return ret;
    }
    imp_ok = ret;

    ret = gpiod_get_value(aState->proximity_ok);
    if (ret < 0) {
        pr_err("proximity_ok error: err(%d)\n", ret);
        return ret;
    }
    prox_ok = ret;

    /* Only two valid combinations for laser on: lid closed or hand in place. */
    if ( !((hall && imp_ok && !prox_ok) || (!hall && imp_ok && prox_ok)) ) {
        pr_err("safety sensor combination error: hall(%d), imp(%d), prox(%d)\n", hall, imp_ok, prox_ok);
        return -EIO;
    }


    ret = gpiod_get_value(aState->ccd_tec_en);
    if (ret < 0) {
        pr_err("ccd_tec_en error: err(%d)\n", ret);
        return ret;
    }
    else if (!ret) {
        pr_err("ccd_tec_en is low\n");
        return -EIO;
    }

    ret = gpiod_get_value(aState->laser_tec_en);
    if (ret < 0) {
        pr_err("laser_tec_en error: err(%d)\n", ret);
        return ret;
    }
    else if (!ret) {
        pr_err("laser_tec_en is low\n");
        return -EIO;
    }

    return 0;
}

static int validate_laser_on_requirements(struct p05v2_state *aState)
{
    int ret;

    ret = gpiod_get_value(aState->laser_on);
    if (ret == 1) {
        if (check_laser_on_requirements(aState) != 0) {
            gpiod_set_value(aState->laser_on, 0);
            m41t80_wdt_disable(); /* Turn off watchdog after laser off */
            return 0;
        }
    }

    return ret;
}

/* ************ Sysfs handlers ****************** */

static ssize_t p05_show_aux_pwr(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    DBG_NOTICE("p05_show_aux_pwr\n");

    *buf = 0;
    sprintf(buf, "%u\n", (u32)aux_power_state);

    DBG_NOTICE1("Returning value %s\n", buf);

    return strlen(buf);
}

static ssize_t p05_store_aux_pwr(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct p05v2_state *state = getState(dev);
    u32 value;
    int ret = 0;

    DBG_NOTICE("p05_store_aux_pwr\n");

    if (!getU32Param(dev, buf, &value)) {
        if (!value) {
            aux_power_state = (int)value;
            msleep(1000); /* Wait for drivers to halt operation before turning off power. */
        }

        gpiod_set_value(state->power_aux, (int)value);
        gpiod_set_value(state->audio_output_en, (int)value); /* Change audio at the same time */

        if (value) {
            msleep(500); /* Let power settle before releasing touch driver (goodix). */
            aux_power_state = value;
        }
    }

    return handle_error_and_return(dev, ret, count);
}

int p05_aux_power_state(void)
{
    return aux_power_state;
}
EXPORT_SYMBOL(p05_aux_power_state);

static ssize_t p05_show_5v_pwr(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    DBG_NOTICE("p05_show_5v_pwr\n");

    *buf = 0;
    sprintf(buf, "%u\n", (u32)supply_5v_power_state);

    DBG_NOTICE1("Returning value %s\n", buf);

    return strlen(buf);
}

static ssize_t p05_store_5v_pwr(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct p05v2_state *state = getState(dev);
    u32 value;

    DBG_NOTICE("p05_store_5v_pwr\n");

    if (!getU32Param(dev, buf, &value)) {
        if (!value) {
            supply_5v_power_state = (int)value;
            msleep(1000); /* Wait for drivers to halt operation before turning off power. */
        }
        gpiod_set_value(state->power5v, (int)value);
        if (value) {
            msleep(500);
            supply_5v_power_state = (int)value;
        }
    }

    return count;
}

int p05_5v_power_state(void)
{
    return supply_5v_power_state;
}
EXPORT_SYMBOL(p05_5v_power_state);

static ssize_t p05_show_display_pwr(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_display_pwr\n");

    ret = readIOAndSetU32Result(dev, buf, state->power_display);

    return handle_error_and_return(dev, ret, strlen(buf));
}

static ssize_t p05_store_display_pwr(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct p05v2_state *state = getState(dev);
    u32 value;

    DBG_NOTICE("p05_store_display_pwr\n");

    if (!getU32Param(dev, buf, &value)) {
        gpiod_set_value(state->power_display, (int)value);
    }

    return count;
}


static ssize_t p05_show_laser_board_pwr(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    *buf = 0;
    sprintf(buf, "%u\n", (u32)laser_board_power_state);

    DBG_NOTICE1("Returning value %s\n", buf);

    return strlen(buf);
}

static ssize_t p05_store_laser_board_pwr(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct p05v2_state *state = getState(dev);
    u32 value;

    DBG_NOTICE("p05_store_laser_board_pwr\n");

    if (!getU32Param(dev, buf, &value)) {
        if (!value) {
            laser_board_power_state = (int)value;
            msleep(1000); /* Wait for drivers to halt operation before turning off power. */
        }
        gpiod_set_value(state->laser_board_pwr, (int)value);
        if (value) {
            msleep(500);
            laser_board_power_state = (int)value;
        }
    }

    return count;
}

int p05_laser_board_power_state(void)
{
    return laser_board_power_state;
}
EXPORT_SYMBOL(p05_laser_board_power_state);


static ssize_t p05_show_laser_on(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_laser_on\n");

    ret = readIOAndSetU32Result(dev, buf, state->laser_on);

    return handle_error_and_return(dev, ret, strlen(buf));
}

static ssize_t p05_store_laser_on(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    int err;
    struct p05v2_state *state = getState(dev);
    u32 value;

    DBG_NOTICE("p05_store_laser_on\n");


    if (!getU32Param(dev, buf, &value)) {
        if (value) {

            err = check_laser_on_requirements(state);
            if (err) {
                return err;
            }

            m41t80_wdt_ping(); /* Turn on watchdog before laser on */
        }

        gpiod_set_value(state->laser_on, (int)value);

        if (!value) {
            m41t80_wdt_disable(); /* Turn off watchdog after laser off */
        }
    }

    return count;
}


static ssize_t p05_show_laser_tec_en(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_laser_tec_en\n");

    ret = readIOAndSetU32Result(dev, buf, state->laser_tec_en);

    return handle_error_and_return(dev, ret, strlen(buf));
}

static ssize_t p05_store_laser_tec_en(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct p05v2_state *state = getState(dev);
    u32 value;

    DBG_NOTICE("p05_store_laser_tec_en\n");

    if (!getU32Param(dev, buf, &value)) {
        gpiod_set_value(state->laser_tec_en, (int)value);
/*        gpiod_set_value(state->laser_board_pwr, (int)value);*/
    }

    return count;
}


static ssize_t p05_show_ccd_tec_en(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_ccd_tec_en\n");

    ret = readIOAndSetU32Result(dev, buf, state->ccd_tec_en);

    return handle_error_and_return(dev, ret, strlen(buf));
}

static ssize_t p05_store_ccd_tec_en(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    struct p05v2_state *state = getState(dev);
    u32 value;

    DBG_NOTICE("p05_store_ccd_tec_en\n");

    if (!getU32Param(dev, buf, &value)) {
        /*
         *  We decided that our use of the TEC controllers is not harmful,
         *  so no requirement for watchdog here.
         */
        gpiod_set_value(state->ccd_tec_en, (int)value);
    }

    return count;
}


static ssize_t p05_show_impedance_ok(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;
    static char old_imp_ok = '0';

    DBG_NOTICE("p05_show_impedance_ok\n");

    validate_laser_on_requirements(state);

    ret = readIOAndSetU32Result(dev, buf, state->impedance_ok);

    if (ret >= 0) {
        /* Detect skin removed from impedance sensor */
        if ((*buf == '0') && (old_imp_ok == '1')) {
            cancel_delayed_work_sync(&state->work);
            /*
             * Start polling loop for 2 seconds, to detect if proximity is stuck high.
             * Re-power proximity in case it is stuck.
             */
            schedule_delayed_work(&state->work, HZ / PROXIMITY_RESET_WORK_DIV);
        }
        old_imp_ok = *buf;
    }


    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_proximity_ok(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_proximity_ok\n");

    validate_laser_on_requirements(state);

    ret = readIOAndSetU32Result(dev, buf, state->proximity_ok);

    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_hall_closed(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;
    static char old_hall_closed = '0';

    DBG_NOTICE("p05_show_hall_closed\n");

    validate_laser_on_requirements(state);

    ret = readIOAndSetU32Result(dev, buf, state->_hall_closed);

    if (ret >= 0) {
        /* Detect when lid closes */
        if ((*buf == '1') && (old_hall_closed == '0')) {
            cancel_delayed_work_sync(&state->work);
            /*
             * Start polling loop for 2 seconds, to detect if proximity is stuck high.
             * Re-power proximity in case it is stuck.
             */
            schedule_delayed_work(&state->work, HZ / PROXIMITY_RESET_WORK_DIV);
        }
        old_hall_closed = *buf;
    }

    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_laser_tec_ut(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_laser_tec_ut\n");

    ret = readIOAndSetU32Result(dev, buf, state->laser_tec_ut);

    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_laser_tec_ot(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_laser_tec_ot\n");

    ret = readIOAndSetU32Result(dev, buf, state->laser_tec_ot);

    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_laser_shutdown(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE2("p05_show_laser_shutdown (%p) (%p)\n", state, state->laser_shutdown);

    ret = readIOAndSetU32Result(dev, buf, state->laser_shutdown);

    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_ccd_tec_ut(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_ccd_tec_ut\n");

    ret = readIOAndSetU32Result(dev, buf, state->ccd_tec_ut);

    return handle_error_and_return(dev, ret, strlen(buf));
}


static ssize_t p05_show_ccd_tec_ot(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct p05v2_state *state = getState(dev);
    int ret;

    DBG_NOTICE("p05_show_ccd_tec_ot\n");

    ret = readIOAndSetU32Result(dev, buf, state->ccd_tec_ot);

    return handle_error_and_return(dev, ret, strlen(buf));
}

static ssize_t p05_store_debug_mode(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    u32 value;

    DBG_NOTICE("p05_store_debug_mode\n");

    if (!getU32Param(dev, buf, &value)) {
        debug_mode = value;
    }

    return count;
}


static ssize_t p05_show_debug_mode(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    DBG_NOTICE("p05_show_debug_mode\n");

    *buf = 0;
    sprintf(buf, "%u\n", (u32)debug_mode);

    return strlen(buf);
}


static DEVICE_ATTR(aux_pwr, S_IWUSR | S_IRUSR, p05_show_aux_pwr, p05_store_aux_pwr);
static DEVICE_ATTR(5v_pwr, S_IWUSR | S_IRUSR, p05_show_5v_pwr, p05_store_5v_pwr);
static DEVICE_ATTR(display_pwr, S_IWUSR | S_IRUSR, p05_show_display_pwr, p05_store_display_pwr);
static DEVICE_ATTR(laser_board_pwr, S_IWUSR | S_IRUSR, p05_show_laser_board_pwr, p05_store_laser_board_pwr);
static DEVICE_ATTR(laser_on, S_IWUSR | S_IRUSR, p05_show_laser_on, p05_store_laser_on);
static DEVICE_ATTR(laser_tec_en, S_IWUSR | S_IRUSR, p05_show_laser_tec_en, p05_store_laser_tec_en);
static DEVICE_ATTR(ccd_tec_en, S_IWUSR | S_IRUSR, p05_show_ccd_tec_en, p05_store_ccd_tec_en);
static DEVICE_ATTR(impedance_ok, S_IRUSR, p05_show_impedance_ok, NULL);
static DEVICE_ATTR(proximity_ok, S_IRUSR, p05_show_proximity_ok, NULL);
static DEVICE_ATTR(hall_closed, S_IRUSR, p05_show_hall_closed, NULL);
static DEVICE_ATTR(laser_tec_ut, S_IRUSR, p05_show_laser_tec_ut, NULL);
static DEVICE_ATTR(laser_tec_ot, S_IRUSR, p05_show_laser_tec_ot, NULL);
static DEVICE_ATTR(laser_shutdown, S_IRUSR, p05_show_laser_shutdown, NULL);
static DEVICE_ATTR(ccd_tec_ut, S_IRUSR, p05_show_ccd_tec_ut, NULL);
static DEVICE_ATTR(ccd_tec_ot, S_IRUSR, p05_show_ccd_tec_ot, NULL);
static DEVICE_ATTR(debug_mode, S_IWUSR | S_IRUSR, p05_show_debug_mode, p05_store_debug_mode);


static struct attribute *p05_attributes[] = {
    &dev_attr_aux_pwr.attr,
    &dev_attr_5v_pwr.attr,
    &dev_attr_display_pwr.attr,
    &dev_attr_laser_board_pwr.attr,
    &dev_attr_laser_on.attr,
    &dev_attr_laser_tec_en.attr,
    &dev_attr_ccd_tec_en.attr,
    &dev_attr_impedance_ok.attr,
    &dev_attr_proximity_ok.attr,
    &dev_attr_hall_closed.attr,
    &dev_attr_laser_tec_ut.attr,
    &dev_attr_laser_tec_ot.attr,
    &dev_attr_laser_shutdown.attr,
    &dev_attr_ccd_tec_ut.attr,
    &dev_attr_ccd_tec_ot.attr,
    &dev_attr_debug_mode.attr,
    NULL
};

static const struct attribute_group p05_attr_group = {
    .attrs = p05_attributes,
};


static const struct attribute_group *attr_groups[] = {
    &p05_attr_group,
    NULL,
};


/* ******** P0.5v2 platform driver ********** */

static struct gpio_desc* try_devm_gpiod_get(struct device *apDev, const char *apConId, enum gpiod_flags aFlags)
{
    struct gpio_desc *result = devm_gpiod_get(apDev, apConId, aFlags);
    if (IS_ERR(result)) {
        dev_err(apDev, "Failed to get GPIO handle\n");
        result = NULL;
    }

    return result;
}

static void try_gpiod_set_value(struct gpio_desc *apDesc, int aValue)
{
    if (apDesc) {
        gpiod_set_value(apDesc, aValue);
    }
}


static int p05_probe(struct platform_device *pdev)
{
    struct p05v2_state *st;

    pr_notice("p05_probe (coreinit)\n");

    st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
    if (!st) {
        return -ENOMEM;
    }

    st->pdev = pdev;
    dev_set_drvdata(&pdev->dev, st);

    st->power5v = try_devm_gpiod_get(&pdev->dev, "power5v", GPIOD_OUT_HIGH);
    st->power_aux = try_devm_gpiod_get(&pdev->dev, "aux_power", GPIOD_OUT_HIGH);
    st->power_display = try_devm_gpiod_get(&pdev->dev, "display_power", GPIOD_OUT_HIGH);

    st->impedance_ok = try_devm_gpiod_get(&pdev->dev, "impedance_ok", GPIOD_IN);
    st->proximity_ok = try_devm_gpiod_get(&pdev->dev, "proximity_ok", GPIOD_IN);
    st->_hall_closed = try_devm_gpiod_get(&pdev->dev, "_hall_closed", GPIOD_IN);

    st->laser_board_pwr = try_devm_gpiod_get(&pdev->dev, "laser_board_pwr", GPIOD_OUT_HIGH);
    st->laser_on = try_devm_gpiod_get(&pdev->dev, "laser_on", GPIOD_OUT_LOW);

    st->laser_tec_en = try_devm_gpiod_get(&pdev->dev, "laser_tec_en", GPIOD_OUT_HIGH);
    st->laser_tec_ut = try_devm_gpiod_get(&pdev->dev, "laser_tec_ut", GPIOD_IN);
    st->laser_tec_ot = try_devm_gpiod_get(&pdev->dev, "laser_tec_ot", GPIOD_IN);
    st->laser_shutdown = try_devm_gpiod_get(&pdev->dev, "laser_shutdown", GPIOD_IN);

    st->ccd_tec_en = try_devm_gpiod_get(&pdev->dev, "ccd_tec_en", GPIOD_OUT_HIGH);
    st->ccd_tec_ut = try_devm_gpiod_get(&pdev->dev, "ccd_tec_ut", GPIOD_IN);
    st->ccd_tec_ot = try_devm_gpiod_get(&pdev->dev, "ccd_tec_ot", GPIOD_IN);

    st->audio_output_en = try_devm_gpiod_get(&pdev->dev, "audio_output_en", GPIOD_OUT_HIGH);

    aux_power_state = 1;

    st->platformClass = class_create(THIS_MODULE, "platform");
    if (IS_ERR(st->platformClass)) {
        dev_err(&pdev->dev, "Failed to create device class\n");
        return PTR_ERR(st->platformClass);
    }
    st->platformClass->dev_groups = attr_groups;

    st->p05v2Device = device_create(st->platformClass, &pdev->dev, MKDEV(0, 0), st, "p05v2");
    if (IS_ERR(st->p05v2Device)) {
        class_destroy(st->platformClass);
        dev_err(&pdev->dev, "Failed to create the device\n");
        return PTR_ERR(st->p05v2Device);
    }

    pr_notice("p05_probe Initializing Worker\n");
    INIT_DELAYED_WORK(&st->work, p05_proximity_reset_work);
    schedule_delayed_work(&st->work, HZ / PROXIMITY_RESET_WORK_DIV);

    return 0;
}

static int p05_remove(struct platform_device *pdev)
{
    struct p05v2_state *state = dev_get_drvdata(&pdev->dev);

    pr_notice("p05_remove\n");

    try_gpiod_set_value(state->laser_on, 0);
    try_gpiod_set_value(state->laser_board_pwr, 0);

    try_gpiod_set_value(state->laser_tec_en, 0);
    try_gpiod_set_value(state->ccd_tec_en, 0);

    cancel_delayed_work_sync(&state->work);

    device_destroy(state->platformClass, MKDEV(0, 0));
    class_unregister(state->platformClass);
    class_destroy(state->platformClass);

    return 0;
}

#ifdef CONFIG_PM
static int p05_suspend_late(struct device *dev)
{
    struct p05v2_state *state = dev_get_drvdata(dev);
    int ret = 0;

    pr_notice("p05_suspend_late\n");

    cancel_delayed_work_sync(&state->work);

    aux_power_state = 0;
    laser_board_power_state = 0;
    supply_5v_power_state = 0;

    msleep(700);

    try_gpiod_set_value(state->laser_on, 0);
    try_gpiod_set_value(state->laser_board_pwr, 0);

    try_gpiod_set_value(state->laser_tec_en, 0);
    try_gpiod_set_value(state->ccd_tec_en, 0);

    try_gpiod_set_value(state->power5v, 0);
    try_gpiod_set_value(state->power_aux, 0);
/*    try_gpiod_set_value(state->power_display, 0); / * We should keep display on always */

    try_gpiod_set_value(state->audio_output_en, 0);

    pr_notice("p05_suspend_late Finished\n");

    return ret;
}

static int p05_resume_early(struct device *dev)
{
    struct p05v2_state *state = dev_get_drvdata(dev);
    int ret = 0;

    pr_notice("p05_resume_early\n");

    try_gpiod_set_value(state->power_aux, 1);
    try_gpiod_set_value(state->power5v, 1);
    try_gpiod_set_value(state->laser_board_pwr, 1); /* Needed to power Laser board temperature sensor. */
    try_gpiod_set_value(state->laser_tec_en, 1);
    try_gpiod_set_value(state->ccd_tec_en, 1);
    try_gpiod_set_value(state->audio_output_en, 1);

    msleep(800);

    aux_power_state = 1;
    laser_board_power_state = 1;
    supply_5v_power_state = 1;

    /* Goodix complains on resume without settling delay */
    msleep(200);

    pr_notice("p05_resume_early Finished\n");

    return ret;
}
#endif

const struct dev_pm_ops pm_ops = {
    SET_LATE_SYSTEM_SLEEP_PM_OPS(p05_suspend_late, p05_resume_early)
};


static const struct platform_device_id p05_id_table[] = {
    { "rsps,p05v2" },
    { "rsps,glucobeam" },
    { }
};

MODULE_DEVICE_TABLE(platform, p05_id_table);


/* of = Open Firmware */
static const struct of_device_id p05_of_match[] = {
    { .compatible = "rsps,p05v2" },
    { .compatible = "rsps,glucobeam" },
    { }
};


MODULE_DEVICE_TABLE(of, p05_of_match);


static struct platform_driver p05_platform_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .of_match_table = p05_of_match,
        .probe_type = PROBE_FORCE_SYNCHRONOUS,
        .pm = &pm_ops,
    },
    .probe  = p05_probe,
    .remove = p05_remove,
    .id_table = p05_id_table
};

static int __init p05_init(void)
{
    return platform_driver_register(&p05_platform_driver);
}

static void __exit p05_exit(void)
{
    platform_driver_unregister(&p05_platform_driver);
}


/* We must initialize early, because some subsystems register i2c drivers
 * in subsys_initcall() code, but are linked (and initialized) before i2c.
 */
core_initcall_sync(p05_init);
module_exit(p05_exit);


MODULE_AUTHOR("Steffen Brummer <steffen@rspsystems.com>");
MODULE_DESCRIPTION("Platform driver for RSP P0.5v2 devices");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.7.0");
MODULE_ALIAS("p05v2");

