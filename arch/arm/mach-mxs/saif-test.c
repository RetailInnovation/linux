/*
 * Copyright 2012 Retail Innovation
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
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#define SAIF_CTRL	(base + 0x00)
#define SAIF_CTRL_SET	(base + 0x04)
#define SAIF_CTRL_CLR	(base + 0x08)
#define SAIF_CTRL_TOG	(base + 0x0C)
#define SAIF_STAT	(base + 0x10)
#define SAIF_STAT_SET	(base + 0x14)
#define SAIF_STAT_CLR	(base + 0x18)
#define SAIF_STAT_TOG	(base + 0x1C)
#define SAIF_DATA	(base + 0x20)

#define RATE		44100

static struct clk *clk = NULL;
static void __iomem *base = NULL;

static int mxs_saif_test_open(struct inode *inode, struct file *file)
{
	/* Reset and gate off SAIF clock */
	writel((0x3 << 30), SAIF_CTRL_SET);

	/* Program SAIF clock */
	clk_prepare_enable(clk);
	clk_set_rate(clk, 512 * RATE);
	clk_disable_unprepare(clk);

	/* Clear reset and un-gate SAIF CLK */
	writel((0x3 << 30), SAIF_CTRL_CLR);

	/* MCLK: 11.2896 Mhz, I2S Mode */
	writel((1 << 27) | (1 << 11), SAIF_CTRL);

	/* Enable SAIF clock */
	clk_prepare_enable(clk);

	/* RUN! */
	writel(0x1, SAIF_CTRL_SET);

	return 0;
}

static int mxs_saif_test_release(struct inode *inode, struct file *file)
{
	/* Stop! */
	writel(0x1, SAIF_CTRL_CLR);
	return 0;
}

static ssize_t mxs_saif_test_write(struct file *file, const char __user *buf,
				size_t count, loff_t *off)
{
	uint32_t sample;
	uint32_t *ptr = (uint32_t*)buf;

	if ((count % 4) != 0) {
		return -EINVAL;
	}

	local_irq_disable();

	while (ptr < (uint32_t*)(buf + count)) {

		/* Busy-wait until FIFO is ready */
		while(!(readl(SAIF_STAT) & (1 << 4)));

		get_user(sample, ptr++);

		/* Write sample */
		writel(sample, SAIF_DATA);

		*off += 4;
	}

	local_irq_enable();

	return count;
}

static struct file_operations mxs_saif_test_md_fops = {
	.owner   = THIS_MODULE,
	.open    = mxs_saif_test_open,
	.release = mxs_saif_test_release,
	.write   = mxs_saif_test_write,
};

struct miscdevice mxs_saif_test_md = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "mxs-saif-test",
	.fops  = &mxs_saif_test_md_fops,
};

static int mxs_saif_test_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl *pinctrl;
	struct resource *iores;
	int ret = 0;

	if (!np) {
		return -EINVAL;
	}

	/* We only support master-device */
	if (of_parse_phandle(np, "fsl,saif-master", 0)) {
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
        if (IS_ERR(pinctrl)) {
                ret = PTR_ERR(pinctrl);
                return ret;
        }

        clk = devm_clk_get(&pdev->dev, NULL);
        if (IS_ERR(clk)) {
                ret = PTR_ERR(clk);
                dev_err(&pdev->dev, "Cannot get the clock: %d\n",
                        ret);
                return ret;
        }

        iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);

        base = devm_request_and_ioremap(&pdev->dev, iores);
        if (!base) {
                dev_err(&pdev->dev, "ioremap failed\n");
                return -ENODEV;
        }

	ret = misc_register(&mxs_saif_test_md);
	if (ret) {
		return ret;
	}

	dev_info(&pdev->dev, "Registered MXS SAIF-TEST misc device\n");

	return ret;
}

static int mxs_saif_test_remove(struct platform_device *pdev)
{
	int ret = 0;

	ret = misc_deregister(&mxs_saif_test_md);
	if (ret) {
		return ret;
	}

	return 0;
}

static const struct of_device_id mxs_saif_test_dt_ids[] = {
	{ .compatible = "fsl,imx28-saif", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_saif_test_dt_ids);

static struct platform_driver mxs_saif_test_driver = {
	.driver = {
		.name = "mxs-saif-test",
		.owner = THIS_MODULE,
		.of_match_table = mxs_saif_test_dt_ids,
	},
	.probe = mxs_saif_test_probe,
	.remove = mxs_saif_test_remove,
};
module_platform_driver(mxs_saif_test_driver);

MODULE_AUTHOR("Rickard Engberg <engberg83@gmail.com>");
MODULE_DESCRIPTION("MXS SAIF test module");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mxs-saif-test");
