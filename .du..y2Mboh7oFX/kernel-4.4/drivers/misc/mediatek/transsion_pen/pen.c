/*
* XPEN driver
*
* Copyright (C) 2017 TRANSSION HOLDINGS
*
* Author: achang.zhang@reallytek.com
*
* This program is free software; you can redistribute  it and/or modify it
* under  the terms of  the GNU General  Public License as published by the
* Free Software Foundation;  either version 2 of the  License, or (at your
* option) any later version.
*
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <mtk_boot_common.h>
#include <linux/pinctrl/consumer.h>

enum {
        XPEN_IN,
        XPEN_OUT
};

struct xpen_priv {
        struct switch_dev sdev;
        struct input_dev *idev;
};

static int xpen_state = XPEN_OUT;
static struct pinctrl *pinctrl1;
static struct pinctrl_state *pins_default;
//add XLLWHLSE-35 by hao.wu2 2018.02.26 start
static struct pinctrl_state *xpen_power_on , *xpen_power_off;
int xpen_get_gpio_info (struct platform_device *pdev)
{
	int ret;

	pr_info("[xpen %d] xpen_pinctrl+++++++++++++++++\n", pdev->id);
	pr_info("xpen Lomen 0.1\n");
	pinctrl1 = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		dev_err(&pdev->dev, "xpen Cannot find touch pinctrl1!\n");
		return ret;
	}
	pr_info("xpen Lomen 0.2\n");
	pins_default = pinctrl_lookup_state(pinctrl1, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		dev_err(&pdev->dev, "xpen Cannot find touch pinctrl default %d!\n", ret);
	}

	xpen_power_off = pinctrl_lookup_state(pinctrl1, "state_xpen_power_off");
	if (IS_ERR(xpen_power_off)) {
		ret = PTR_ERR(xpen_power_off);
		dev_err(&pdev->dev, "xpen Cannot find touch pinctrl state_xpen_power_off!\n");
		return ret;
	}
	xpen_power_on = pinctrl_lookup_state(pinctrl1, "state_xpen_power_on");
	if (IS_ERR(xpen_power_on)) {
		ret = PTR_ERR(xpen_power_on);
		dev_err(&pdev->dev, "xpen Cannot find touch pinctrl state_xpen_power_on!\n");
		return ret;
	}
	pr_info("[xpen%d] xpen_pinctrl----------\n", pdev->id);
	return 0;
}

void xpen_tpd_power(int onoff)
{
	printk("wuhao onoff=%d\n",onoff);
    //mutex_lock(&tpd_set_gpio_mutex);
	if (onoff)
			pinctrl_select_state(pinctrl1,xpen_power_on);
		else
			pinctrl_select_state(pinctrl1, xpen_power_off);
	//mutex_unlock(&tpd_set_gpio_mutex);
}
static irqreturn_t xpen_thread_factory_func(int irq_num, void *data)
{
        struct xpen_priv *priv = data;
        static u8 xpen_factory_status = 0;

        if (xpen_state == XPEN_IN) {
                xpen_factory_status++;
                xpen_state = XPEN_OUT;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_LOW);
        } else {
                xpen_factory_status++;
                xpen_state = XPEN_IN;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_HIGH);
        }
        if(2 == xpen_factory_status) {
                input_report_key(priv->idev, KEY_F22, 1);
                input_sync(priv->idev);
                input_report_key(priv->idev, KEY_F22, 0);
                input_sync(priv->idev);
                xpen_factory_status = 0;
        }
        pr_info("xpen_thread_factory_func xpen_state = %d[%s]\n",
                        xpen_state, xpen_state?"XPEN_OUT":"XPEN_IN");

        return IRQ_HANDLED;
}

static irqreturn_t xpen_thread_func(int irq_num, void *data)
{
        struct xpen_priv *priv = data;

        if (xpen_state == XPEN_OUT) {
                xpen_state = XPEN_IN;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_HIGH);
                switch_set_state(&priv->sdev, xpen_state);
                input_report_key(priv->idev, KEY_F22, 1);
                input_sync(priv->idev);
                input_report_key(priv->idev, KEY_F22, 0);
                input_sync(priv->idev);
		   xpen_tpd_power(1);
        } else {
                xpen_state = XPEN_OUT;
                irq_set_irq_type(irq_num, IRQF_TRIGGER_LOW);
                switch_set_state(&priv->sdev, xpen_state);
                input_report_key(priv->idev, KEY_F21, 1);
                input_sync(priv->idev);
                input_report_key(priv->idev, KEY_F21, 0);
                input_sync(priv->idev);
		   xpen_tpd_power(0);
        }

        pr_info("xpen_thread_func xpen_state = %d[%s]\n",
                        xpen_state, xpen_state?"XPEN_OUT":"XPEN_IN");

        return IRQ_HANDLED;
}

static int xpen_probe(struct platform_device *pdev)
{
        struct xpen_priv *priv;
        struct device_node *node = pdev->dev.of_node;
        int irq_num = 0, error;

        pr_info("xpen_probe enter\n");
	 xpen_get_gpio_info(pdev);

        priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
        if (!priv) {
                dev_err(&pdev->dev, "Failed to allocate memory\n");
                return -ENOMEM;
        }

        priv->idev = devm_input_allocate_device(&pdev->dev);
        if (!priv->idev) {
                dev_err(&pdev->dev, "Failed to allocate input device\n");
                return -ENOMEM;
        }

        priv->idev->name = pdev->name;
        priv->idev->dev.parent = &pdev->dev;

        __set_bit(EV_KEY, priv->idev->evbit);
        input_set_capability(priv->idev, EV_KEY, KEY_F21);
        input_set_capability(priv->idev, EV_KEY, KEY_F22);

        error = input_register_device(priv->idev);
        if (error) {
                dev_err(&pdev->dev, "Failed to register input device\n");
                return error;
        }

        priv->sdev.name = "xpen";
        error = switch_dev_register(&priv->sdev);
        if (error) {
                dev_err(&pdev->dev, "Failed to register switch device\n");
                return error;
        }
        switch_set_state(&priv->sdev, XPEN_OUT);

        if (!node) {
                dev_err(&pdev->dev, "Device_node is null\n");
                error = -ENOENT;
                goto err_unregister_sdev;
        }

        irq_num = irq_of_parse_and_map(node, 0);
        if (!irq_num) {
                dev_err(&pdev->dev, "Failed to parse and map irq\n");
                error = -EINVAL;
                goto err_unregister_sdev;
        }
        if (FACTORY_BOOT == get_boot_mode()) {
                error = devm_request_threaded_irq(&pdev->dev, irq_num, NULL,
                        xpen_thread_factory_func, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
                        dev_name(&pdev->dev), priv);
        } else {
                error = devm_request_threaded_irq(&pdev->dev, irq_num, NULL,
                        xpen_thread_func, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
                        dev_name(&pdev->dev), priv);
        }

        if (error) {
                dev_err(&pdev->dev, "Failed to devm_request_threaded_irq\n");
                goto err_unregister_sdev;
        }
        enable_irq_wake(irq_num);

        platform_set_drvdata(pdev, priv);

        pr_info("xpen_probe ok\n");

        return 0;

err_unregister_sdev:
        switch_dev_unregister(&priv->sdev);
        return error;
}

static int xpen_remove(struct platform_device *pdev)
{
        struct xpen_priv *priv = platform_get_drvdata(pdev);

        switch_dev_unregister(&priv->sdev);

        return 0;
}

static const struct of_device_id xpen_of_match[] = {
        { .compatible = "mediatek,xpen"},
        { },
};

static struct platform_driver xpen_driver = {
        .probe = xpen_probe,
        .remove = xpen_remove,
        .driver = {
                .name = "xpen",
                .of_match_table = xpen_of_match,
        },
};

module_platform_driver(xpen_driver);

/* Module information */
MODULE_AUTHOR("<hao.wu2@reallytek.com>");
MODULE_DESCRIPTION("XPEN driver");
MODULE_LICENSE("GPL v2");
