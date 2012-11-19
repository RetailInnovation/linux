/*
 * Copyright 2012 Unknown
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/timer.h>

static struct timer_list timer;

struct pin_info_t {
	char *name;
	char* of_name;
	int flags;
	int gpio;
};

static struct pin_info_t pins[] = {
	{"reset", "reset-gpios",
		GPIOF_DIR_OUT | GPIOF_INIT_HIGH | GPIOF_OPEN_DRAIN |
		GPIOF_EXPORT_DIR_CHANGEABLE},
	{"on-off", "on-off-gpios",
		GPIOF_DIR_OUT | GPIOF_INIT_HIGH | GPIOF_OPEN_DRAIN |
		GPIOF_EXPORT_DIR_CHANGEABLE},
	{"status", "status-gpios",
		GPIOF_DIR_IN | GPIOF_EXPORT},
	{"dcd", "dcd-gpios",
		GPIOF_DIR_IN | GPIOF_EXPORT},
	{"dsr", "dsr-gpios",
		GPIOF_DIR_IN | GPIOF_EXPORT},
	{"dtr", "dtr-gpios",
		GPIOF_DIR_OUT | GPIOF_INIT_LOW | GPIOF_EXPORT},
	{"ring", "ring-gpios",
		GPIOF_DIR_IN | GPIOF_EXPORT},
	{NULL, NULL, 0},
};

static int setup_pin(struct pin_info_t *p, struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	if (np == NULL) {
		return -EINVAL;
	}

	p->gpio = of_get_named_gpio(np, p->of_name, 0);
	if (p->gpio < 0) {
		dev_err(&pdev->dev, "Failed to get %s pin\n", p->name);
		return p->gpio;
	}

	ret = devm_gpio_request_one(&pdev->dev, p->gpio, p->flags, p->name);
	if (ret) {
		dev_err(&pdev->dev, "Request for %s pin failed\n", p->name);
		return ret;
	}

	ret = gpio_export_link(&pdev->dev, p->name, p->gpio);
	if (ret) {
		dev_err(&pdev->dev, "Export-link for %s pin failed\n", p->name);
		return ret;
	}

	return 0;
}

static void timer_callback(unsigned long data)
{
	static int timer_count = 0;
	struct platform_device *pdev = (struct platform_device*)data;
	int ret;

	/* Check status pin */
	if (gpio_get_value(pins[2].gpio) == 0) {

		if (timer_count == 10) {
			dev_err(&pdev->dev, "Failure: Module did not start\n");
			return;
		}

		dev_err(&pdev->dev, "Module has not started yet, wating...!\n");

		ret = mod_timer(&timer, jiffies + msecs_to_jiffies(5000));
		if (ret) {
			dev_err(&pdev->dev, "Error in mod_timer\n");
		}

		timer_count++;
		return;
	}

	del_timer(&timer);

	dev_info(&pdev->dev, "Telit HE910 GSM Module has "
			"started successfully!\n");

	/* Restore on_off pin */
	gpio_set_value(pins[1].gpio, 1);
}

static int __devinit he910_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pin_info_t *p;

	for (p = pins; p->name != NULL; p++) {
		setup_pin(p, pdev);
	}

	dev_info(&pdev->dev, "Starting Telit HE910 GSM Module\n");

	if (gpio_get_value(pins[2].gpio) == 1) {
		return ret;
	}

	setup_timer(&timer, timer_callback, (unsigned long)pdev);

	/* Pull on_off pin low */
	gpio_set_value(pins[1].gpio, 0);

	ret = mod_timer(&timer, jiffies + msecs_to_jiffies(5000));
	if (ret) {
		dev_err(&pdev->dev, "Error in mod_timer\n");
	}

	return ret;
}

static int __devexit he910_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id he910_dt_ids[] = {
	{ .compatible = "telit,he910", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, he910_dt_ids);

static struct platform_driver he910_driver = {
	.driver = {
		.name = "he910",
		.owner = THIS_MODULE,
		.of_match_table = he910_dt_ids,
	},
	.probe = he910_probe,
	.remove = __devexit_p(he910_remove),
};
module_platform_driver(he910_driver);

MODULE_AUTHOR("Rickard Engberg <engberg83@gmail.com>");
MODULE_DESCRIPTION("Telit HE910 bootstrap driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:he910");
