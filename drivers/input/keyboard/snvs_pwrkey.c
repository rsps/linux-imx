// SPDX-License-Identifier: GPL-2.0+
//
// Driver for the IMX SNVS ON/OFF Power Key
// Copyright (C) 2015 Freescale Semiconductor, Inc. All Rights Reserved.

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define SNVS_LPSR_REG	0x4C	/* LP Status Register */
#define SNVS_LPCR_REG	0x38	/* LP Control Register */
#define SNVS_HPSR_REG	0x14
#define SNVS_HPSR_BTN	BIT(6)
#define SNVS_LPSR_SPO	BIT(18)
#define SNVS_LPCR_DEP_EN BIT(5)

#define DEBOUNCE_TIME 500
#define REPEAT_INTERVAL 60

#define ALTERNATE_KEYCODE KEY_PROG1

static int use_alternate_keycode = 0;

struct pwrkey_drv_data {
	struct regmap *snvs;
	int irq;
	int keycode;
	int keystate;  /* 1:pressed */
	int wakeup;
	bool suspended;
	struct clk *clk;
	struct timer_list check_timer;
	struct input_dev *input;
};

static void imx_imx_snvs_check_for_events(struct timer_list *t)
{
	struct pwrkey_drv_data *pdata = from_timer(pdata, t, check_timer);
	struct input_dev *input = pdata->input;
	u32 state;

	int keycode = (use_alternate_keycode != 0) ? ALTERNATE_KEYCODE : pdata->keycode;

	if (pdata->clk) {
		if (pdata->suspended)
			clk_prepare_enable(pdata->clk);
		else
			clk_enable(pdata->clk);
	}

	regmap_read(pdata->snvs, SNVS_HPSR_REG, &state);

	if (pdata->clk) {
		if (pdata->suspended)
			clk_disable_unprepare(pdata->clk);
		else
			clk_disable(pdata->clk);
	}

	state = state & SNVS_HPSR_BTN ? 1 : 0;

	/* only report new event if status changed */
	if (state ^ pdata->keystate) {
		pdata->keystate = state;
		input_event(input, EV_KEY, keycode, state);
		input_sync(input);
		pm_relax(pdata->input->dev.parent);
	}

	/* repeat check if pressed long */
	if (state) {
		mod_timer(&pdata->check_timer,
			  jiffies + msecs_to_jiffies(REPEAT_INTERVAL));
	}
}

static void imx_imx_snvs_check_for_release_events(struct timer_list *t)
{
	struct pwrkey_drv_data *pdata = from_timer(pdata, t, check_timer);
	struct input_dev *input = pdata->input;
	u32 state;

	int keycode = (use_alternate_keycode != 0) ? ALTERNATE_KEYCODE : pdata->keycode;

	/* interrupt only reports release of key so do not wait for state change */
	state=1;
	input_event(input, EV_KEY, keycode, state);
	input_sync(input);
	state=0;
	input_event(input, EV_KEY, keycode, state);
	input_sync(input);
}

static irqreturn_t imx_snvs_pwrkey_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);
	struct input_dev *input = pdata->input;
	u32 lp_status;

	pm_wakeup_event(pdata->input->dev.parent, 0);

	if (pdata->clk)
		clk_enable(pdata->clk);

	if (pdata->suspended) {
		pdata->keystate = 1;
		input_event(input, EV_KEY, pdata->keycode, 1);
		input_sync(input);
	}

	regmap_read(pdata->snvs, SNVS_LPSR_REG, &lp_status);
	if (lp_status & SNVS_LPSR_SPO)
		mod_timer(&pdata->check_timer, jiffies + msecs_to_jiffies(DEBOUNCE_TIME));

	/* clear SPO status */
	regmap_write(pdata->snvs, SNVS_LPSR_REG, SNVS_LPSR_SPO);

	if (pdata->clk)
		clk_disable(pdata->clk);

	return IRQ_HANDLED;
}

static void imx_snvs_pwrkey_act(void *pdata)
{
	struct pwrkey_drv_data *pd = pdata;

	del_timer_sync(&pd->check_timer);
}

static ssize_t use_alternate_keycode_store(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
    u32 value;

    if (sscanf(buf, "%u", &value) != 1) {
        dev_err(dev, "use_alternate_keycode_store invalid param %s\n", buf);
        return -EINVAL;
    }

    use_alternate_keycode = (int)(value != 0);

    pr_info("use_alternate_keycode_store %d\n", use_alternate_keycode);

    return count;
}

static ssize_t use_alternate_keycode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    pr_notice("use_alternate_keycode_show\n");

    return sprintf(buf, "%d\n", use_alternate_keycode);
}
static DEVICE_ATTR_RW(use_alternate_keycode);

static int imx_snvs_pwrkey_probe(struct platform_device *pdev)
{
	struct pwrkey_drv_data *pdata = NULL;
	struct input_dev *input = NULL;
	struct device_node *np;
	int error;

	pr_notice("imx_snvs_pwrkey_probe\n");

	/* Get SNVS register Page */
	np = pdev->dev.of_node;
	if (!np)
		return -ENODEV;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->snvs = syscon_regmap_lookup_by_phandle(np, "regmap");
	if (IS_ERR(pdata->snvs)) {
		dev_err(&pdev->dev, "Can't get snvs syscon\n");
		return PTR_ERR(pdata->snvs);
	}

	if (of_property_read_u32(np, "linux,keycode", &pdata->keycode)) {
		pdata->keycode = KEY_POWER;
		dev_warn(&pdev->dev, "KEY_POWER without setting in dts\n");
	}

	pdata->wakeup = of_property_read_bool(np, "wakeup-source");

	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq < 0)
		return -EINVAL;

	pdata->clk = devm_clk_get(&pdev->dev, "snvs");
	if (IS_ERR(pdata->clk)) {
		pdata->clk = NULL;
	} else {
		error = clk_prepare_enable(pdata->clk);
		if (error) {
			dev_err(&pdev->dev,
				"Could not enable the snvs clock\n");
			return error;
		}
	}

	regmap_update_bits(pdata->snvs, SNVS_LPCR_REG, SNVS_LPCR_DEP_EN, SNVS_LPCR_DEP_EN);

	/* clear the unexpected interrupt before driver ready */
	regmap_write(pdata->snvs, SNVS_LPSR_REG, SNVS_LPSR_SPO);

	if (of_property_read_bool(np, "key-release-only"))
		timer_setup(&pdata->check_timer, imx_imx_snvs_check_for_release_events, 0);
	else
		timer_setup(&pdata->check_timer, imx_imx_snvs_check_for_events, 0);

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		error = -ENOMEM;
		goto error_probe;
	}

	input->name = pdev->name;
	input->phys = "snvs-pwrkey/input0";
	input->id.bustype = BUS_HOST;

	input_set_capability(input, EV_KEY, pdata->keycode);
	input_set_capability(input, EV_KEY, ALTERNATE_KEYCODE);
	
	/* input customer action to cancel release timer */
	error = devm_add_action(&pdev->dev, imx_snvs_pwrkey_act, pdata);
	if (error) {
		dev_err(&pdev->dev, "failed to register remove action\n");
		goto error_probe;
	}

	sysfs_create_file(&input->dev.kobj, &dev_attr_use_alternate_keycode.attr);

	pdata->input = input;
	platform_set_drvdata(pdev, pdata);

	error = devm_request_irq(&pdev->dev, pdata->irq,
			       imx_snvs_pwrkey_interrupt,
			       0, pdev->name, pdev);

	if (error) {
		dev_err(&pdev->dev, "interrupt not available.\n");
		goto error_probe;
	}

	error = input_register_device(input);
	if (error < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto error_probe;
	}

	device_init_wakeup(&pdev->dev, pdata->wakeup);
	error = dev_pm_set_wake_irq(&pdev->dev, pdata->irq);
	if (error)
		dev_err(&pdev->dev, "irq wake enable failed.\n");

	return 0;

error_probe:
	if (pdata->clk)
		clk_disable_unprepare(pdata->clk);

	return error;
}

static int __maybe_unused imx_snvs_pwrkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	pr_notice("imx_snvs_pwrkey_suspend\n");

	if (pdata->clk)
		clk_disable_unprepare(pdata->clk);

	pdata->suspended = true;

	return 0;
}

static int __maybe_unused imx_snvs_pwrkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	pr_notice("imx_snvs_pwrkey_resume\n");

	if (pdata->clk)
		clk_prepare_enable(pdata->clk);

	pdata->suspended = false;

	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_snvs_pwrkey_pm_ops, imx_snvs_pwrkey_suspend,
				imx_snvs_pwrkey_resume);

static const struct of_device_id imx_snvs_pwrkey_ids[] = {
	{ .compatible = "fsl,sec-v4.0-pwrkey" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_snvs_pwrkey_ids);

static struct platform_driver imx_snvs_pwrkey_driver = {
	.driver = {
		.name = "snvs_pwrkey",
		.pm     = &imx_snvs_pwrkey_pm_ops,
		.of_match_table = imx_snvs_pwrkey_ids,
	},
	.probe = imx_snvs_pwrkey_probe,
};
module_platform_driver(imx_snvs_pwrkey_driver);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("i.MX snvs power key Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("RSP-0.2");
