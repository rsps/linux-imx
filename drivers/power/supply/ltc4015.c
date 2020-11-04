/*
 *  ltc4015.c - Linux kernel module for
 *  LTC4015 Battery Gauge
 *
 *  Copyright (c) 2018 Markku Nivala, Innokas Medical
 *
 *  Based on LTC294x driver
 *
 * History:
 *  2020-06-30 ver. 0.3: Add power lane check before i2c transfers
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/swab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <stdbool.h>

extern int p05_aux_power_state(void);

#define LTC4015_DELAY                   5
#define LTC4015_WORK_DELAY_S            LTC4015_DELAY * HZ
#define LTC4015_WORKER_DELAY_S(x) ((x < LTC4015_DELAY ? LTC4015_DELAY : x) / LTC4015_DELAY)  /* Interval precision by LTC4015_DELAY */

#define GHARGE_STATUS_CV    0x1
#define GHARGE_STATUS_CC    0x2

#define CHARGER_STATE_ABSORB        0x200
#define CHARGER_STATE_SUSPEND       0x100
#define CHARGER_STATE_TEMP          0x20
#define CHARGER_STATE_TIMER         0x10
#define CHARGER_STATE_CX_TERM       0x8
#define CHARGER_STATE_TIMER_FAULT   0x4
#define CHARGER_STATE_BATT_MISSING  0x2
#define CHARGER_STATE_BATT_SHORTED  0x1

#define SYSTEM_STATUS_EN    0x2000
#define SYSTEM_STATUS_MPPT  0x800
#define SYSTEM_STATUS_EQ    0x400

#define CONFIG_COUNT_EN      0x4
#define CONFIG_MEAS_EN       0x10

#define HW_CONFIG_CELLS     0x2
#define HW_CONFIG_LIFEPO    0x600

#define COUNTER_MID         32538
#define COUNTER_MIN         16384
#define COUNTER_MAX         COUNTER_MID + COUNTER_MIN
#define PRESCALE_STEP       1638
#define BATT_FULL_CURR_LIM  100000
#define PRESCALE_COMP       81


typedef enum {
    VBAT_LO_ALERT_LIMIT = 0x01,
    VBAT_HI_ALERT_LIMIT = 0x02,
    VIN_LO_ALERT_LIMIT = 0x03,
    VIN_HI_ALERT_LIMIT = 0x04,
    VSYS_LO_ALERT_LIMIT = 0x05,
    VSYS_HI_ALERT_LIMIT = 0x06,
    IIN_HI_ALERT_LIMIT = 0x07,
    IBAT_LO_ALERT_LIMIT = 0x08,
    DIE_TEMP_HI_ALERT_LIMIT = 0x09,
    BSR_HI_ALERT_LIMIT = 0x0A,
    NTC_RATIO_HI_ALERT_LIMIT = 0x0B,
    NTC_RATIO_LO_ALERT_LIMIT = 0x0C,
    EN_LIMIT_ALERTS = 0x0D,
    EN_CHARGER_STATE_ALERTS = 0x0E,
    EN_CHARGE_STATUS_ALERTS = 0x0F,
    QCOUNT_LO_ALERT_LIMIT = 0x10,
    QCOUNT_HI_ALERT_LIMIT = 0x11,
    QCOUNT_PRESCALE_FACTOR = 0x12,
    QCOUNT = 0x13,
    CONFIG_BITS = 0x14,
    IIN_LIMIT_SETTING = 0x15,
    VIN_UVCL_SETTING = 0x16,
    ARM_SHIP_MODE = 0x19,
    ICHARGE_TARGET = 0x1A,
    VCHARGE_SETTING = 0x1B,
    C_OVER_X_THRESHOLD = 0x1C,
    MAX_CV_TIME = 0x1D,
    MAX_CHARGE_TIME = 0x1E,
    JEITA_T1 = 0x1F,
    JEITA_T2 = 0x20,
    JEITA_T3 = 0x21,
    JEITA_T4 = 0x22,
    JEITA_T5 = 0x23,
    JEITA_T6 = 0x24,
    VCHARGE_JEITA_6_5 = 0x25,
    VCHARGE_JEITA_4_3_2 = 0x26,
    ICHARGE_JEITA_6_5 = 0x27,
    ICHARGE_JEITA_4_3_2 = 0x28,
    CHARGER_CONFIG_BITS = 0x29,
    VABSORB_DELTA = 0x2A,
    MAX_ABSORB_TIME = 0x2B,
    VEQUALIZE_DELTA = 0x2C,
    EQUALIZE_TIME = 0x2D,
    LIFEP04_RECHARGE_THRESHOLD = 0x2E,
    MAX_CHARGE_TIMER = 0x30,
    CV_TIMER = 0x31,
    ABSORB_TIMER = 0x32,
    EQUALIZE_TIMER = 0x33,
    CHARGER_STATE = 0x34,
    CHARGE_STATUS = 0x35,
    LIMIT_ALERTS = 0x36,
    CHARGER_STATE_ALERTS = 0x37,
    CHARGE_STATUS_ALERTS = 0x38,
    SYSTEM_STATUS = 0x39,
    VBAT = 0x3A,
    VIN = 0x3B,
    VSYS = 0x3C,
    IBAT = 0x3D,
    IIN = 0x3E,
    DIE_TEMP = 0x3F,
    NTC_RATIO = 0x40,
    BSR = 0x41,
    JEITA_REGION = 0x42,
    CHEM_CELLS = 0x43,
    ICHARGE_DAC = 0x44,
    VCHARGE_DAC = 0x45,
    IIN_LIMIT_DAC = 0x46,
    VBAT_FILT = 0x47,
    ICHARGE_BSR = 0x48,
    MEAS_SYS_VALID = 0x4A
}ltc4015_reg;


typedef enum {
    START_ESTIMATION,
    CC_1,
    CC_2,
    CV_1,
    CV_2,
    CV_3,
    CV_4,
    CV_5,
    CV_6,
    STOP_ESTIMATION
}estimation_state;


typedef enum {
    CALIBRATION_NEEDED,
    CALIBRATION_CC,
    CALIBRATION_CV,
    CALIBRATION_NEEDED_ACCURATE,
    CALIBRATION_OK
}calib_state; /* Maintain order */


typedef enum {
    BATT_STATUS_CHARGER_DISCONNECTED,
    BATT_STATUS_CHARGING,
    BATT_STATUS_FULL,
    BATT_STATUS_ERR
}charging_kernel_status;


struct ltc4015_info {
    struct i2c_client *client;
    struct power_supply *supply;
    struct power_supply_desc supply_desc;
    struct delayed_work work;
    int capacity;
    int r_sense;    /* mOhm */
    int r_bias; /* mOhm */
    int prescale;
    int cell_count;
    estimation_state curr_area;
    int area_count;
    calib_state calibration_state;
    bool config_error;
    int meas_period;
};


static int ltc4015_read_regs(const struct i2c_client* const client,
        const ltc4015_reg reg, u16* const buf, const int num_regs)
{
    s32 ret;

    if (!p05_aux_power_state()) {
        return -EBUSY;
    }

    if (2 == num_regs)
    {
        ret = i2c_smbus_read_word_data(client, reg);
    }
    else
    {
        ret = i2c_smbus_read_byte_data(client, reg);
    }

    if (ret < 0) {
        dev_err(&client->dev, "ltc4015 read_reg failed!\n");
        return ret;
    }

    *buf = ret;

    return 0;
}


static int ltc4015_write_regs(const struct i2c_client* const client,
        const ltc4015_reg reg, const u16* const buf, const int num_regs)
{
    int ret;

    if (!p05_aux_power_state()) {
        return -EBUSY;
    }

    ret = i2c_smbus_write_i2c_block_data(client, reg, num_regs, (u8 *)buf);
    if (ret < 0) {
        dev_err(&client->dev, "ltc4015 write_reg failed!\n");
        return ret;
    }

    return 0;
}


static int ltc4015_get_voltage(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 value;

    ret = ltc4015_read_regs(info->client, VBAT_FILT, &value, 2);
    *val = ((u32)value * 192264) / 1000;
    return ret;
}


static int ltc4015_get_voltage_in(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 value;

    ret = ltc4015_read_regs(info->client, VIN, &value, 2);
    *val = (u32)value * 1648;
    return ret;
}


static int ltc4015_get_charge_status(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;

    ret = ltc4015_read_regs(info->client, CHARGE_STATUS, &data, 2);
    if (ret < 0)
        return ret;

    *val = data;

    return 0;
}


static int ltc4015_get_charger_state(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;

    ret = ltc4015_read_regs(info->client, CHARGER_STATE, &data, 2);
    if (ret < 0)
        return ret;

    *val = data;

    return 0;
}


static int ltc4015_get_system_status(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;

    ret = ltc4015_read_regs(info->client, SYSTEM_STATUS, &data, 2);
    if (ret < 0)
        return ret;

    *val = data;

    return 0;
}


static int ltc4015_get_battery_config(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;

    ret = ltc4015_read_regs(info->client, CHEM_CELLS, &data, 2);
    if (ret < 0)
        return ret;

    *val = data;

    return 0;
}


static int ltc4015_get_charge_current(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;

    ret = ltc4015_read_regs(info->client, IBAT, &data, 2);
    if (ret < 0)
        return ret;

    *val = (((u32) data * 146487) / 1300);

    return 0;
}


static int ltc4015_get_charge_value(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;

    ret = ltc4015_read_regs(info->client, QCOUNT, &data, 2);
    if (ret < 0)
        return ret;

    *val = data;
    return 0;
}


static int ltc4015_get_configuration(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;
    u32 value;

    ret = ltc4015_read_regs(info->client, CONFIG_BITS, &data, 2);
    value = data;
    *val = value;

    return ret;
}


static int ltc4015_get_temperature_val(const struct ltc4015_info* const info, int* const val)
{
    int ret;
    u16 data;
    u32 value;

    ret = ltc4015_read_regs(info->client, NTC_RATIO, &data, 2);
    value = data;
    *val = value;

    return ret;
}


static int ltc4015_get_capacity(const struct ltc4015_info *info, int *val)
{
    int ret;
    int value = 0;
    ret = ltc4015_get_charge_value(info, &value);

    if (ret < 0)
        return ret;

    *val = (100 * (value - COUNTER_MIN)) / COUNTER_MID + 1;

    if ( 1 > *val)
    {
        *val = 1;
    }

    if ( 100 < *val)
    {
        *val = 100;
    }

    return 0;
}


static int ltc4015_battery_charged(const struct ltc4015_info* const info, bool* const charged)
{
    int chargeCurr = 0;
    int ret = ltc4015_get_charge_current(info, &chargeCurr);

    if (BATT_FULL_CURR_LIM > chargeCurr)
        *charged = true;
    else
        *charged = false;

    return ret;
}


static int ltc4015_get_charging_status(const struct ltc4015_info* const info, charging_kernel_status* const status)
{
    int ret;
    int vin;
    int state = 0;
    bool charged = false;

    *status = BATT_STATUS_ERR;
    ret = ltc4015_get_voltage_in(info, &vin);
    if (!ret)
        ret = ltc4015_battery_charged(info, &charged);

    if (11000000 < vin)
    {
        if (charged)
        {
            *status = BATT_STATUS_FULL;
        }
        else
        {
            ret = ltc4015_get_charger_state(info, &state);
            if (CHARGER_STATE_SUSPEND & state)
            {
                *status = BATT_STATUS_ERR;
            }
            else
            {
                *status = BATT_STATUS_CHARGING;
            }
        }

    }
    else
    {
        *status = BATT_STATUS_CHARGER_DISCONNECTED;
    }

    return ret;
}


static int ltc4015_set_charge_value(const struct ltc4015_info* const info, const int val)
{
    int ret;
    u16 data = val;

    ret = ltc4015_write_regs(info->client, QCOUNT, &data, 2);
    if (ret < 0)
        return ret;
    return data;
}


static int ltc4015_set_prescale_value(const struct ltc4015_info* const info, const int val)
{
    int ret;
    u16 data = val;

    ret = ltc4015_write_regs(info->client, QCOUNT_PRESCALE_FACTOR, &data, 2);
    if (ret < 0)
        return ret;
    return data;
}


static int ltc4015_set_configuration(const struct ltc4015_info* const info, const int val)
{
    int ret;
    u16 data = val;

    ret = ltc4015_write_regs(info->client, CONFIG_BITS, &data, 2);
    if (ret < 0)
        return ret;
    return data;
}


static int ltc4015_set_capacity(const struct ltc4015_info* const info, const int capacity)
{
    u16 counter;
    int ret;

    counter = (COUNTER_MID * capacity) / 100 + COUNTER_MIN;
    ret = ltc4015_write_regs(info->client, QCOUNT, &counter, 2);

    return ret;
}


static int ltc4015_enable_meas(const struct ltc4015_info *info)
{
    int control;
    int value;
    int ret;

    ret = ltc4015_get_configuration(info, &value);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not read registers from device\n");
        goto error_exit;
    }

    control = CONFIG_MEAS_EN | value;

    ret = ltc4015_set_configuration(info, control);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not write register\n");
        goto error_exit;
    }

error_exit:

    return ret;
}


static int ltc4015_disable_meas(const struct ltc4015_info *info)
{
    int control;
    int value;
    int ret;

    ret = ltc4015_get_configuration(info, &value);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not read registers from device\n");
        goto error_exit;
    }

    control = ~CONFIG_MEAS_EN & value;

    ret = ltc4015_set_configuration(info, control);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not write register\n");
        goto error_exit;
    }


error_exit:

    return ret;
}


static int ltc4015_get_property(struct power_supply *psy,
                enum power_supply_property prop,
                union power_supply_propval *val)
{
    struct ltc4015_info *info = power_supply_get_drvdata(psy);

    switch (prop) {
    case POWER_SUPPLY_PROP_CAPACITY:
    {
        val->intval =  info->capacity;

        return 0;
    }
    break;
    default:
        return -EINVAL;
    }
}


static int ltc4015_set_property(struct power_supply *psy,
    enum power_supply_property psp,
    const union power_supply_propval *val)
{
    switch (psp) {
    default:
        return -EPERM;
    }
}


static int ltc4015_property_is_writeable(
    struct power_supply *psy, enum power_supply_property psp)
{
    switch (psp) {
    default:
        return 0;
    }
}


static calib_state ltc4015_calib_capacity_state(const struct ltc4015_info* const info)
{
    return info->calibration_state;
}


static void ltc4015_calib_capacity_set_state(struct ltc4015_info* const info, const calib_state state)
{
    if (state != info->calibration_state)
    {
        info->calibration_state = state;
    }
}


/* Take account battery wear */
static void ltc4015_calib_capacity(struct ltc4015_info *info)
{
    int ret = 0;
    int status = 0;
    int counter = 0;
    bool charged;
    ret = ltc4015_get_charge_value(info, &counter);
    if (!ret)
        ret = ltc4015_battery_charged(info, &charged);

    if (!ret && CALIBRATION_OK > ltc4015_calib_capacity_state(info) && charged)
    {
        int difference = 0;
        difference = COUNTER_MAX - (counter - COUNTER_MIN);

        if (difference > PRESCALE_STEP)
        {
            info->prescale -= difference / PRESCALE_STEP;
        }
        else if (difference < PRESCALE_STEP)
        {
            info->prescale += difference / PRESCALE_STEP;
        }
        ret = ltc4015_set_prescale_value(info, info->prescale);
        if (!ret)
            ret = ltc4015_set_capacity(info, 100);

        if (!ret)
            ltc4015_calib_capacity_set_state(info, CALIBRATION_OK);
    }

    if (CALIBRATION_NEEDED_ACCURATE > ltc4015_calib_capacity_state(info))
    {
        ret = ltc4015_get_charge_status(info, &status);
        if (!ret && (status & GHARGE_STATUS_CC))
        {
            ltc4015_calib_capacity_set_state(info, CALIBRATION_CC);
        }

        if (!ret && (status & GHARGE_STATUS_CV) && CALIBRATION_CC == ltc4015_calib_capacity_state(info))
        {
            /* Need to take account too small measurement window when estimating battery wear during */
            /* partial charge/discharge */
            counter = (COUNTER_MID * 95) / 100 + COUNTER_MIN - PRESCALE_COMP;
            ret = ltc4015_set_charge_value(info, counter);
            if (!ret)
                ltc4015_calib_capacity_set_state(info, CALIBRATION_CV);
        }
    }
}


static int ltc4015_capacity_estimation_state_time(struct ltc4015_info *info)
{
    return info->area_count;
}


static estimation_state ltc4015_capacity_estimation_state(struct ltc4015_info *info)
{
    return info->curr_area;
}


static void ltc4015_capacity_estimation_increase_state_time(struct ltc4015_info *info, estimation_state area)
{
    if (info->curr_area != area)
    {
        info->area_count = 0;
        info->curr_area = area;
    }
    else
    {
        info->area_count++;
    }
}


static void ltc4015_capacity_estimation_change_state(struct ltc4015_info *info, estimation_state area)
{

    if (area == STOP_ESTIMATION || area == START_ESTIMATION)
    {
        info->curr_area = 0;
        info->area_count = 0;
    }

    info->curr_area = area;
}


/* Estimate battery charge roughly. Must be made only if battery has gone totally empty or during initial charge, */
/* full precision is achieved by fully charging device */
static void ltc4015_estimate_capacity(struct ltc4015_info *info)
{
    int charge_status = 0;
    int charge_curr;
    int charge_v;
    int ret = 0;
    bool charged;

    ret = ltc4015_get_charge_status(info, &charge_status);

    switch(charge_status)
    {
        case GHARGE_STATUS_CV:
            {
                ret = ltc4015_get_charge_current(info, &charge_curr);
                if (!ret)
                {
                    if (charge_curr > 2300000)
                    {
                        ltc4015_capacity_estimation_increase_state_time(info, CC_1);
                        ret = ltc4015_set_capacity(info, 95);
                    }
                    else
                    {
                        ltc4015_capacity_estimation_increase_state_time(info, CC_2);
                        ret = ltc4015_set_capacity(info, 98);
                    }
                }

                if (!ret)
                {
                    if (ltc4015_capacity_estimation_state_time(info) > LTC4015_WORKER_DELAY_S(50))
                    {
                        ltc4015_capacity_estimation_change_state(info, STOP_ESTIMATION);
                    }
                }
            }
            break;
        case GHARGE_STATUS_CC:
            {
                ret = ltc4015_get_voltage(info, &charge_v);
                if (!ret)
                {
                    if (charge_v < 3375000)
                    {
                        ltc4015_capacity_estimation_increase_state_time(info, CV_1);
                        ret = ltc4015_set_capacity(info, 1);
                        ltc4015_calib_capacity_set_state(info, CALIBRATION_NEEDED_ACCURATE);
                    }
                    else if (charge_v < 3515000)
                    {
                        ltc4015_capacity_estimation_increase_state_time(info, CV_4);
                        ret = ltc4015_set_capacity(info, 20);
                    }
                    else if (charge_v < 3540000)
                    {
                        ltc4015_capacity_estimation_increase_state_time(info, CV_5);
                        ret = ltc4015_set_capacity(info, 40);
                    }
                    else
                    {
                        ltc4015_capacity_estimation_increase_state_time(info, CV_6);
                        ret = ltc4015_set_capacity(info, 50);
                    }

                    if (!ret)
                    {
                        if (ltc4015_capacity_estimation_state_time(info) > LTC4015_WORKER_DELAY_S(60))
                        {
                            ltc4015_capacity_estimation_change_state(info, STOP_ESTIMATION);
                        }
                    }
                }
            }
        break;
        default:
        break;
    }

    if (!ret)
    {
        ret = ltc4015_battery_charged(info, &charged);
        if (!ret && charged)
        {
            ret = ltc4015_set_capacity(info, 100);
            if (!ret)
                ltc4015_capacity_estimation_change_state(info, STOP_ESTIMATION);
        }
    }

}


static int ltc4015_init(struct ltc4015_info *info)
{
    int ret;
    int value;
    int control = 0;
    int config = 0;
    int batt_config;
    int system_status;

    info->curr_area = 0;
    info->area_count = 0;
    ltc4015_calib_capacity_set_state(info, CALIBRATION_NEEDED);

    /* Find correct scaling for RSP battery and HW config */
    info->prescale = ((((u32)(((u32)info->capacity * 3600) / 65535) * (833333 * (u32)info->r_sense)) / 10000000) * 2 + 5) / 10;

    ret = ltc4015_get_battery_config(info, &config);
    info->cell_count = config & 0xf;
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not read registers from device\n");
        goto error_exit;
    }

    ret = ltc4015_get_configuration(info, &value);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not read registers from device\n");
        goto error_exit;
    }

    ret = ltc4015_set_prescale_value(info, info->prescale);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not write register\n");
        goto error_exit;
    }

    if (!(value & CONFIG_COUNT_EN))
    {
        ret = ltc4015_set_capacity(info, 25);
        if (ret < 0) {
            dev_err(&info->client->dev,
                "Could not write register\n");
            goto error_exit;
        }
        control = CONFIG_COUNT_EN | value;

        ret = ltc4015_set_configuration(info, control);
        if (ret < 0) {
            dev_err(&info->client->dev,
                "Could not write register\n");
            goto error_exit;
        }
        ltc4015_capacity_estimation_change_state(info, START_ESTIMATION);
    }
    else
    {
        ltc4015_capacity_estimation_change_state(info, STOP_ESTIMATION);
    }

    ret = ltc4015_get_battery_config(info, &batt_config);
    if (!ret)
        ret = ltc4015_get_system_status(info, &system_status);

    if (!ret)
    {
        info->config_error = !(batt_config & HW_CONFIG_LIFEPO) || !(batt_config & HW_CONFIG_CELLS) || (system_status & SYSTEM_STATUS_MPPT) || (system_status & SYSTEM_STATUS_EQ);
        if (info->config_error)
        {
            goto error_exit;
        }
    }
    else
    {
        goto error_exit;
    }

    return 0;

error_exit:
    control |= 0x100;
    ret = ltc4015_set_configuration(info, control);
    if (ret < 0) {
        dev_err(&info->client->dev,
            "Could not write register\n");
    }

    return ret;
}


static int check_meas_period(struct ltc4015_info* const info)
{
    return ++info->meas_period;
}


static void reset_meas_period(struct ltc4015_info* const info)
{
    info->meas_period = 0;
}


static void ltc4015_update(struct ltc4015_info *info)
{
	int ret;
	int capacity = 0;
	charging_kernel_status charging_status;

    ret = ltc4015_get_charging_status(info, &charging_status);
    if (!ret && BATT_STATUS_CHARGING == charging_status)
    {
        if (STOP_ESTIMATION > ltc4015_capacity_estimation_state(info))
        {
            ltc4015_estimate_capacity(info);
        }
        else
        {
            ltc4015_calib_capacity(info);
        }
    }

    ret = ltc4015_get_capacity(info, &capacity);

    if (!ret && capacity != info->capacity) {
        info->capacity = capacity;
        power_supply_changed(info->supply);
    }

    if (BATT_STATUS_CHARGER_DISCONNECTED == charging_status && 90 > capacity)
    {
        ltc4015_calib_capacity_set_state(info, CALIBRATION_NEEDED);
    }

    if (check_meas_period(info) > LTC4015_WORKER_DELAY_S(5))
    {
        reset_meas_period(info);
        /* Just ignore returns */
        ret = ltc4015_enable_meas(info);
        msleep(40);
        ret = ltc4015_disable_meas(info);
    }
}

static void ltc4015_work(struct work_struct *work)
{
    struct ltc4015_info *info;
    bool ret;

    info = container_of(work, struct ltc4015_info, work.work);

    if (p05_aux_power_state()) {
        ltc4015_update(info);
    }

    /* Just ignore return */
    ret = schedule_delayed_work(&info->work, LTC4015_WORK_DELAY_S);
}

static enum power_supply_property ltc4015_properties[] = {
    POWER_SUPPLY_PROP_CAPACITY,
};


static ssize_t ltc4015_set_prescale(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t count)
{
    struct power_supply *psy = dev_get_drvdata(dev);
    struct ltc4015_info *info = power_supply_get_drvdata(psy);

    int val;

    if (kstrtoint (buf, 10, &val) < 0)
        return -EINVAL;

    info->prescale = val;

    return count;
}


static ssize_t ltc4015_show_prescale(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
    struct ltc4015_info *info = power_supply_get_drvdata(psy);

    return sprintf(buf, "%d\n", info->prescale);
}


static ssize_t ltc4015_show_state(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
    struct ltc4015_info *info = power_supply_get_drvdata(psy);
    int ret;
    int chargstate = 0;

    typedef struct {
        bool tempHigh;
        bool chargeTimeExceeded;
        bool battMissing;
        bool battShorted;
        bool battConfigurationError;
        int tempVal;
    }charger_kernel_state;

    charger_kernel_state state;

    ret = ltc4015_get_charger_state(info, &chargstate);

    state.tempHigh = chargstate & CHARGER_STATE_TEMP;
    state.chargeTimeExceeded = chargstate & CHARGER_STATE_TIMER_FAULT;
    state.battMissing = chargstate & CHARGER_STATE_BATT_MISSING;
    state.battShorted = chargstate & CHARGER_STATE_BATT_SHORTED;
    state.battConfigurationError = info->config_error;

    if (!ret)
    {
        ret = ltc4015_get_temperature_val(info, &state.tempVal);
    }

    memcpy(buf, &state, sizeof(state));
    return 0 > ret ? ret : sizeof(state);
}


static ssize_t ltc4015_show_status(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
    struct ltc4015_info *info = power_supply_get_drvdata(psy);
    int ret;
    charging_kernel_status status;

    ret = ltc4015_get_charging_status(info, &status);

    return ret ? ret : sprintf(buf, "%d\n", status);
}


static ssize_t ltc4015_set_storage_mode(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t count)
{
    struct power_supply *psy = dev_get_drvdata(dev);
    struct ltc4015_info *info = power_supply_get_drvdata(psy);

    bool enable;
    u16 val = 0x534D;
    int ret = 0;

    if (kstrtobool(buf, &enable) < 0)
        return -EINVAL;

    if (enable)
    {
        ret = ltc4015_write_regs(info->client, ARM_SHIP_MODE, &val, 2);
    }

    return ret ? ret : count;
}


static DEVICE_ATTR(prescale, S_IWUSR | S_IRUSR,
        ltc4015_show_prescale, ltc4015_set_prescale);
static DEVICE_ATTR(status, S_IRUSR,
        ltc4015_show_status, NULL);
static DEVICE_ATTR(state, S_IRUSR,
        ltc4015_show_state, NULL);
static DEVICE_ATTR(enable_storage_mode, S_IWUSR,
        NULL, ltc4015_set_storage_mode);

static struct attribute *ltc4015_attributes[] = {
    &dev_attr_prescale.attr,
    &dev_attr_status.attr,
    &dev_attr_state.attr,
    &dev_attr_enable_storage_mode.attr,
    NULL,
};

static const struct attribute_group ltc4015_attr_group = {
    .attrs = ltc4015_attributes,
};


static int ltc4015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct power_supply_config psy_cfg = {};
    struct ltc4015_info *info;
    int ret;
    u32 capacity;
    s32 r_sense;
    s32 r_bias;
    struct device_node *np;

    pr_notice("ltc4015_probe\n");

    info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
    if (info == NULL)
        return -ENOMEM;

    i2c_set_clientdata(client, info);

    np = of_node_get(client->dev.of_node);

    info->supply_desc.name = np->name;

    ret = of_property_read_u32(np, "ltc,r_sense", &r_sense);
    if (ret < 0) {
        dev_err(&client->dev,
            "No ltc,r_sense provided\n");
        return ret;
    }
    info->r_sense = r_sense;

    ret = of_property_read_u32(np, "ltc,capacity", &capacity);
    if (ret < 0) {
        dev_warn(&client->dev,
            "No ltc,capacity provided\n");
        return ret;
    }
    info->capacity = capacity;

    ret = of_property_read_u32(np, "ltc,r_bias", &r_bias);
    if (ret < 0) {
        dev_warn(&client->dev,
            "No ltc,r_bias provided\n");
        return ret;
    }
    info->r_bias = r_bias;

    info->client = client;
    info->supply_desc.type = POWER_SUPPLY_TYPE_BATTERY;
    info->supply_desc.properties = ltc4015_properties;
    info->supply_desc.num_properties =
        ARRAY_SIZE(ltc4015_properties);
    info->supply_desc.get_property = ltc4015_get_property;
    info->supply_desc.set_property = ltc4015_set_property;
    info->supply_desc.property_is_writeable = ltc4015_property_is_writeable;
    info->supply_desc.external_power_changed = NULL;

    psy_cfg.drv_data = info;

    INIT_DELAYED_WORK(&info->work, ltc4015_work);

    ret = ltc4015_init(info);
    if (ret < 0) {
        dev_err(&client->dev, "Communication with chip failed\n");
        return ret;
    }

    info->supply = power_supply_register(&client->dev, &info->supply_desc,
                         &psy_cfg);
    if (IS_ERR(info->supply)) {
        dev_err(&client->dev, "failed to register ltc4015\n");
        return PTR_ERR(info->supply);
    } else {
        /* Just ignore return */
        ret = schedule_delayed_work(&info->work, LTC4015_WORK_DELAY_S);
    }

    ret = sysfs_create_group(&info->supply->dev.kobj, &ltc4015_attr_group);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to create handles\n");
        return ret;
    }

    dev_info(&client->dev, "Registered\n");

    return 0;
}


static int ltc4015_remove(struct i2c_client *client)
{
    int ret;
    struct ltc4015_info *info = i2c_get_clientdata(client);
    u16 control = 0x0;

    pr_notice("ltc4015_remove\n");

    /* Just ignore return */
    ret = cancel_delayed_work_sync(&info->work);

    sysfs_remove_group(&info->supply->dev.kobj, &ltc4015_attr_group);

    power_supply_unregister(info->supply);

    ret = ltc4015_set_configuration(info, control);

    return ret;
}


static int ltc4015_suspend(struct device *dev)
{
    int ret;
    struct i2c_client *client = to_i2c_client(dev);
    struct ltc4015_info *info = i2c_get_clientdata(client);

    pr_notice("ltc4015_suspend\n");

    /* Just ignore return */
    ret = cancel_delayed_work(&info->work);
    ret = ltc4015_disable_meas(info);

    return 0;
}


static int ltc4015_resume(struct device *dev)
{
    bool ret;
    struct i2c_client *client = to_i2c_client(dev);
    struct ltc4015_info *info = i2c_get_clientdata(client);

    pr_notice("ltc4015_resume\n");

    /* Just ignore return */
    ret = schedule_delayed_work(&info->work, LTC4015_WORK_DELAY_S);
    return 0;
}


static SIMPLE_DEV_PM_OPS(ltc4015_pm_ops, ltc4015_suspend, ltc4015_resume);
#define LTC4015_PM_OPS (&ltc4015_pm_ops)


static const struct i2c_device_id ltc4015_id[] = {
    {"ltc4015", },
    { },
};
MODULE_DEVICE_TABLE(i2c, ltc4015_id);

static struct i2c_driver ltc4015_driver = {
    .driver = {
        .name   = "ltc4015",
        .pm = LTC4015_PM_OPS,
    },
    .probe      = ltc4015_probe,
    .remove     = ltc4015_remove,
    .id_table   = ltc4015_id,
};
module_i2c_driver(ltc4015_driver);

MODULE_AUTHOR("Markku Nivala, Innokas Medical");
MODULE_DESCRIPTION("LTC4015 Battery Gauge IC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");
