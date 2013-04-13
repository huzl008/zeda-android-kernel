/*
 * TWL4030/TPS65950 BCI (Battery Charger Interface) driver
 *
 * Copyright (C) 2010 Gražvydas Ignotas <notasas@gmail.com>
 *
 * based on bq24032a_battery.c by TI
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <mach/gpio.h>

#define TWL4030_MADC_CTRL1		0x00
#define TWL4030_MADC_SW1SELECT_LSB	0x06
#define TWL4030_MADC_SW1AVERAGE_LSB	0x08
#define TWL4030_MADC_CTRL_SW1		0x12
#define TWL4030_MADC_GPCH0_LSB		0x37
#define TWL4030_MADC_GPCH0_MSB		0x38

#define TWL4030_BCICTL1			0x23
#define TWL4030_REG_GPBR1               0x0c


#define TWL4030_MADCON			BIT(0)
#define TWL4030_SW1_IMR1		BIT(1)
#define TWL4030_SW1_ISR1		BIT(1)
#define TWL4030_SW1_CH0			BIT(0)
#define TWL4030_SW1_AVCH0		BIT(0)
#define TWL4030_MADC_SW_START		BIT(5)

#define TWL4030_GPBR1_MADC_HFCLK_EN     BIT(7)

#define CURRENT_STATE_AC		BIT(0)
#define CURRENT_STATE_BAT		BIT(1)

#define CHG_ACPG_GPIO	174
#define CHG_S1_GPIO	178
#define CHG_S2_GPIO	180

static int voltage = 8400;
static int backup_level = 100;

struct bq24032a {
	struct device		*dev;
	struct power_supply	ac;
	struct power_supply	bat;
	struct delayed_work	adc_work;
	int			irq_chg;
	int			irq_bci;
};

/*
 * clear and set bits on an given register on a given module
 */
static int twl4030_clear_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	u8 val = 0;
	int ret;

	ret = twl_i2c_read_u8(mod_no, &val, reg);
	if (ret)
		return ret;

	val &= ~clear;
	val |= set;

	return twl_i2c_write_u8(mod_no, val, reg);
}

/*
 * AC charger presence events
 */
static irqreturn_t bq24032a_charger_interrupt(int irq, void *arg)
{
	struct bq24032a *bci = arg;

	power_supply_changed(&bci->ac);

	return IRQ_HANDLED;
}

static void read_adc_val(struct bq24032a *bci)
{
	u8 msb, lsb;

	/* read LSB */
	if(twl_i2c_read_u8(TWL4030_MODULE_MADC, &lsb,TWL4030_MADC_GPCH0_LSB))
		dev_err(bci->dev, "unable to read register: %d\n",TWL4030_MADC_GPCH0_LSB);

	/* read MSB */
	if(twl_i2c_read_u8(TWL4030_MODULE_MADC, &msb,TWL4030_MADC_GPCH0_MSB))
		dev_err(bci->dev, "unable to read register: %d\n",TWL4030_MADC_GPCH0_MSB);

	voltage = (int)(((msb << 8) | lsb) >> 6);

	/* voltage = conv_result * step_size / R
	 * step_size = 1.5 / ( 2 ^ 10 -1 )
	 * R = 0.25
	 */
	voltage = voltage * 3 * 1000 * 4 / (2 * 1023);

	/* ry board */
	voltage = voltage * 252 / 100 ;
}

static int confirm_battery_level(int is_online)
{
	int level;

	if(is_online)
	{
		if(voltage > 8400)
			level = 100;
		else if(voltage > 8160)
			level = 90;
		else if(voltage > 8120)
			level = 80;
		else if(voltage > 8060)
			level = 70;
		else if(voltage > 7990)
			level = 60;
		else if(voltage > 7930)
			level = 50;
		else if(voltage > 7880)
			level = 40;
		else if(voltage > 7780)
			level = 30;
		else if(voltage > 7710)
			level = 20;
		else if(voltage > 7550)
			level = 10;
		else
			level = 5;
	}
	else
	{
		if(voltage > 8060)
			level = 100;
		else if(voltage > 7930)
			level = 90;
		else if(voltage > 7800)
			level = 80;
		else if(voltage > 7690)
			level = 70;
		else if(voltage > 7580)
			level = 60;
		else if(voltage > 7470)
			level = 50;
		else if(voltage > 7440)
			level = 40;
		else if(voltage > 7400)
			level = 30;
		else if(voltage > 7250)
			level = 20;
		else if(voltage > 7200)
			level = 10;
		else if(voltage > 7090)
			level = 5;
		else
			level = 0;
	}

	backup_level = level;
	return level;
}

/*
 * BCI monitoring events
 */
static irqreturn_t bq24032a_bci_interrupt(int irq, void *arg)
{
	struct bq24032a *bci = arg;
	u8 val;

	/* Clear interrupt state */
	if(twl_i2c_read_u8(TWL4030_MODULE_MADC, &val, TWL4030_MADC_ISR1))
		dev_err(bci->dev, "unable to read register: %d\n", TWL4030_MADC_ISR1);
		
	if(val & TWL4030_SW1_ISR1)
	{
		/* Read GP conversion result */
		read_adc_val(bci);

		power_supply_changed(&bci->bat);

		/* 10 second delay */
		schedule_delayed_work(&bci->adc_work, msecs_to_jiffies(10000));
	}

	return IRQ_HANDLED;
}

static int bq24032a_online(void)
{
	int ret;
	u8 state;

	ret = gpio_get_value(CHG_ACPG_GPIO);
	if (ret) 
		state = CURRENT_STATE_BAT;
	else
		state = CURRENT_STATE_AC;

	return state;
}

static int bq24032a_state(int is_online)
{
	if(is_online)
	{
		if(gpio_get_value(CHG_S2_GPIO) == 1)
			return POWER_SUPPLY_STATUS_CHARGING;
		else if (gpio_get_value(CHG_S1_GPIO) == 1)
			return POWER_SUPPLY_STATUS_FULL;
		else
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	else
		return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int bq24032a_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	static int backup_state = CURRENT_STATE_BAT;
	int is_online;
	int state;

	state = bq24032a_online();
	is_online = state & CURRENT_STATE_AC;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24032a_state(is_online);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = voltage;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = is_online;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if( (psy->type == POWER_SUPPLY_TYPE_BATTERY) && (backup_state & state) )
			val->intval = confirm_battery_level(is_online);
		else if( psy->type == POWER_SUPPLY_TYPE_BATTERY) {
			val->intval = backup_level;
			backup_state = state;
		} else
			val->intval = 100;	
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property bq24032a_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};


static int bci_conversion_setup(void)
{
	u8 val;
	int ret;

	do {
		/* Set up the MADC_IMR1 register to receive interrupt */
		val = TWL4030_SW1_IMR1;
		ret = twl4030_clear_set(TWL4030_MODULE_MADC, val, 0, TWL4030_MADC_IMR1);
		if(ret)
			break;

		/* Power on the MADC */
		val = TWL4030_MADCON;	
		ret = twl4030_clear_set(TWL4030_MODULE_MADC, 0, val, TWL4030_MADC_CTRL1);
		if(ret)
			break;

		/* Select ADCIN0 for conversion */
		val = TWL4030_SW1_CH0;
		ret = twl4030_clear_set(TWL4030_MODULE_MADC, 0, val, TWL4030_MADC_SW1SELECT_LSB);
		if(ret)
			break;

		/* Set up the register for averaging */
		val = TWL4030_SW1_AVCH0;
		ret = twl4030_clear_set(TWL4030_MODULE_MADC, 0, val, TWL4030_MADC_SW1AVERAGE_LSB);
		if(ret)
			break;

		/* Turn on MADC clock */
		val = TWL4030_GPBR1_MADC_HFCLK_EN;
		ret = twl4030_clear_set(TWL4030_MODULE_INTBR, 0, val, TWL4030_REG_GPBR1);
		if(ret)
			break;

		/* Start conversion */
		val = TWL4030_MADC_SW_START;
		ret = twl4030_clear_set(TWL4030_MODULE_MADC, 0, val, TWL4030_MADC_CTRL_SW1);
	} while(0);

	return ret;
}

static void bci_conversion_end(void)
{
	int val;

	val = TWL4030_GPBR1_MADC_HFCLK_EN;
	twl4030_clear_set(TWL4030_MODULE_INTBR, val, 0, TWL4030_REG_GPBR1);

	val = TWL4030_SW1_IMR1;	
	twl4030_clear_set(TWL4030_MODULE_MADC, 0, val, TWL4030_MADC_IMR1);

	val = TWL4030_MADCON;	
	twl4030_clear_set(TWL4030_MODULE_MADC, val, 0, TWL4030_MADC_CTRL1);

	val = TWL4030_SW1_CH0;
	twl4030_clear_set(TWL4030_MODULE_MADC, val, 0, TWL4030_MADC_SW1SELECT_LSB);

	val = TWL4030_SW1_AVCH0;
	twl4030_clear_set(TWL4030_MODULE_MADC, val, 0, TWL4030_MADC_SW1AVERAGE_LSB);
}

static void adc_conversion_start(struct work_struct *work)
{
	bci_conversion_setup();
}

static int __init bq24032a_probe(struct platform_device *pdev)
{
	struct bq24032a *bci;
	int ret;

	if (gpio_request(CHG_ACPG_GPIO, "ac power detect") < 0)
		printk(KERN_ERR "can't detect ac power\n");
	gpio_direction_input(CHG_ACPG_GPIO);
	enable_irq(gpio_to_irq(CHG_ACPG_GPIO));

	if (gpio_request(CHG_S2_GPIO, "charger detect") < 0)
		printk(KERN_ERR "can't detect ac charger\n");
	gpio_direction_input(CHG_S2_GPIO);

	if (gpio_request(CHG_S1_GPIO, "charger state") < 0)
		printk(KERN_ERR "can't detect charger state\n");
	gpio_direction_input(CHG_S1_GPIO);

	bci = kzalloc(sizeof(*bci), GFP_KERNEL);
	if (bci == NULL)
		return -ENOMEM;

	bci->dev = &pdev->dev;
	bci->irq_chg = OMAP_GPIO_IRQ(CHG_ACPG_GPIO);
	bci->irq_bci = platform_get_irq(pdev, 0);

	platform_set_drvdata(pdev, bci);

	bci->ac.name = "bq24032a_ac";
	bci->ac.type = POWER_SUPPLY_TYPE_MAINS;
	bci->ac.properties = bq24032a_charger_props;
	bci->ac.num_properties = ARRAY_SIZE(bq24032a_charger_props);
	bci->ac.get_property = bq24032a_get_property;

	ret = power_supply_register(&pdev->dev, &bci->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	bci->bat.name = "bq24032a_bat";
	bci->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	bci->bat.properties = bq24032a_charger_props;
	bci->bat.num_properties = ARRAY_SIZE(bq24032a_charger_props);
	bci->bat.get_property = bq24032a_get_property;

	ret = power_supply_register(&pdev->dev, &bci->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register bat: %d\n", ret);
		goto fail_register_bat;
	}

	ret = request_irq(bci->irq_chg, bq24032a_charger_interrupt, 
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 
			pdev->name, bci);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n",
			bci->irq_chg, ret);
		goto fail_chg_irq;
	}

	ret = request_threaded_irq(bci->irq_bci, NULL, bq24032a_bci_interrupt,
			IRQF_TRIGGER_RISING, pdev->name, bci);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n",
			bci->irq_bci, ret);
		goto fail_bci_irq;
	}
 
	if(bci_conversion_setup()) {
		dev_err(&pdev->dev, "failed to madc conversion\n");
		goto fail_conversion;
	}

	INIT_DELAYED_WORK(&bci->adc_work, adc_conversion_start);
		
	return 0;

fail_conversion:
	free_irq(bci->irq_bci, bci);

fail_bci_irq:
	free_irq(bci->irq_chg, bci);

fail_chg_irq:
	power_supply_unregister(&bci->bat);

fail_register_bat:
	power_supply_unregister(&bci->ac);

fail_register_ac:
	platform_set_drvdata(pdev, NULL);
	kfree(bci);

	return ret;
}

static int __exit bq24032a_remove(struct platform_device *pdev)
{
	struct bq24032a *bci = platform_get_drvdata(pdev);

	cancel_delayed_work(&bci->adc_work);
	bci_conversion_end();
	free_irq(bci->irq_chg, bci);
	free_irq(bci->irq_bci, bci);
	power_supply_unregister(&bci->ac);
	power_supply_unregister(&bci->bat);
	platform_set_drvdata(pdev, NULL);
	kfree(bci);
	gpio_free(CHG_ACPG_GPIO);
	gpio_free(CHG_S1_GPIO);
	gpio_free(CHG_S2_GPIO);

	return 0;
}

static struct platform_driver bq24032a_driver = {
	.driver	= {
		.name	= "twl4030_madc",
		.owner	= THIS_MODULE,
	},
	.remove	= __exit_p(bq24032a_remove),
};

static int __init bq24032a_init(void)
{
	return platform_driver_probe(&bq24032a_driver, bq24032a_probe);
}
module_init(bq24032a_init);

static void __exit bq24032a_exit(void)
{
	platform_driver_unregister(&bq24032a_driver);
}
module_exit(bq24032a_exit);

MODULE_DESCRIPTION("Bq24032a Battery Charger Interface driver");
MODULE_LICENSE("GPL");
