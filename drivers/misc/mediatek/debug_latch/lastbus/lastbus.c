#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/kallsyms.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <mach/mt_chip.h>
#include <mach/mt_debug_latch.h>
#include "lastbus.h"

static const struct of_device_id lastbus_of_ids[] = {
	{ .compatible = "mediatek,lastbus", },
	{}
};

static int lastbus_probe(struct platform_device *pdev);
static int lastbus_remove(struct platform_device *pdev);
static int lastbus_suspend(struct platform_device *pdev, pm_message_t state);
static int lastbus_resume(struct platform_device *pdev);

static char *lastbus_dump_buf;

static struct lastbus lastbus_drv = {
	.plt_drv = {
		.driver = {
			.name = "lastbus",
			.bus = &platform_bus_type,
			.owner = THIS_MODULE,
			.of_match_table = lastbus_of_ids,
		},
		.probe = lastbus_probe,
		.remove = lastbus_remove,
		.suspend = lastbus_suspend,
		.resume = lastbus_resume,
	},
};

static int lastbus_probe(struct platform_device *pdev)
{
	struct lastbus_plt *plt = NULL;

	pr_debug("%s:%d: enter\n", __func__, __LINE__);

	plt = lastbus_drv.cur_plt;

	if (plt && plt->ops && plt->ops->probe)
		return plt->ops->probe(plt, pdev);

	lastbus_drv.mcu_base = of_iomap(pdev->dev.of_node, 0);
	lastbus_drv.peri_base = of_iomap(pdev->dev.of_node, 1);
	if (!lastbus_drv.mcu_base || !lastbus_drv.peri_base)
		return -ENODEV;

	return 0;
}

static int lastbus_remove(struct platform_device *pdev)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	pr_debug("%s:%d: enter\n", __func__, __LINE__);

	if (plt && plt->ops && plt->ops->remove)
		return plt->ops->remove(plt, pdev);

	return 0;
}

static int lastbus_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	pr_debug("%s:%d: enter\n", __func__, __LINE__);

	if (plt && plt->ops && plt->ops->suspend)
		return plt->ops->suspend(plt, pdev, state);

	return 0;
}

static int lastbus_resume(struct platform_device *pdev)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	pr_debug("%s:%d: enter\n", __func__, __LINE__);

	if (plt && plt->ops && plt->ops->resume)
		return plt->ops->resume(plt, pdev);

	return 0;
}

int lastbus_register(struct lastbus_plt *plt)
{
	if (!plt) {
		pr_warn("%s%d: plt is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	plt->common = &lastbus_drv;
	lastbus_drv.cur_plt = plt;

	return 0;
}

int lastbus_dump(char *buf, int len)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	if (!buf) {
		pr_warn("%s:%d: buf is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!plt) {
		pr_warn("%s:%d: plt is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->common) {
		pr_warn("%s:%d: plt->common is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->common->mcu_base) {
		pr_warn("%s:%d: plt->common->mcu_base is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->common->peri_base) {
		pr_warn("%s:%d: plt->common->peri_base is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (plt->ops && plt->ops->dump)
		return plt->ops->dump(plt, buf, len);
	else
		pr_warn("no dump function implemented\n");

	return 0;
}

int lastbus_enable(void)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	if (!plt) {
		pr_warn("%s:%d: plt is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->common) {
		pr_warn("%s:%d: plt->common is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->common->mcu_base) {
		pr_warn("%s:%d: plt->common->mcu_base is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->common->peri_base) {
		pr_warn("%s:%d: plt->common->peri_base is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}


	if (plt->ops && plt->ops->enable)
		return plt->ops->enable(plt);
	else
		pr_warn("no enable function implemented\n");

	return 0;
}

int lastbus_dump_min_len(void)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	if (!plt) {
		pr_warn("%s:%d: plt is NULL\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (!plt->min_buf_len)
		pr_warn("%s:%d: min_buf_len is 0\n", __func__, __LINE__);

	return plt->min_buf_len;
}

int mt_lastbus_dump(char *buf)
{
	strncpy(buf, lastbus_dump_buf, strlen(lastbus_dump_buf)+1);
	return 0;
}

static ssize_t lastbus_dump_show(struct device_driver *driver, char *buf)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;
	int ret = 0;

	ret = plt->ops->dump(plt, buf, -1);
	if (ret)
		pr_err("%s:%d: dump failed\n", __func__, __LINE__);

	return strlen(buf);
}

static ssize_t lastbus_dump_store(struct device_driver *driver, const char *buf, size_t count)
{
	return count;
}

DRIVER_ATTR(lastbus_dump, 0664, lastbus_dump_show, lastbus_dump_store);

static ssize_t lastbus_test_show(struct device_driver *driver, char *buf)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	if (plt && plt->ops)
		return plt->ops->test_show(buf);

	return -ENODEV;
}

static ssize_t lastbus_test_store(struct device_driver *driver, const char *buf, size_t count)
{
	return count;
}

DRIVER_ATTR(lastbus_test, 0664, lastbus_test_show, lastbus_test_store);

static int lastbus_start(void)
{
	struct lastbus_plt *plt = lastbus_drv.cur_plt;

	if (!plt->ops) {
		pr_err("%s:%d: ops not installed\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (plt->ops->start)
		return plt->ops->start(plt);

	return 0;
}

static int __init lastbus_init(void)
{
	int ret = 0;

	ret = lastbus_start();
	if (ret) {
		pr_err("%s:%d: lastbus_start failed\n", __func__, __LINE__);
		return -ENODEV;
	}

	/* since kernel already populates dts, our probe would be callback after this registration */
	ret = platform_driver_register(&lastbus_drv.plt_drv);
	if (ret) {
		pr_err("%s:%d: platform_driver_register failed\n", __func__, __LINE__);
		return -ENODEV;
	}

	ret = driver_create_file(&lastbus_drv.plt_drv.driver, &driver_attr_lastbus_dump);
	if (ret)
		pr_err("%s:%d: driver_create_file failed.\n", __func__, __LINE__);

	ret = driver_create_file(&lastbus_drv.plt_drv.driver, &driver_attr_lastbus_test);
	if (ret)
		pr_err("%s:%d: driver_create_file failed.\n", __func__, __LINE__);

	lastbus_dump_buf = kzalloc(lastbus_drv.cur_plt->min_buf_len, GFP_KERNEL);
	if (!lastbus_dump_buf) {
		pr_err("%s:%d: kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}

	/* we dump here and then return lastbus_dump_buf
		to users to prevent lastbus values cleaned by low power mechanism
		(MCUSYS might be turned off before lastbus_dump()) */
	lastbus_dump(lastbus_dump_buf, lastbus_drv.cur_plt->min_buf_len);
	lastbus_enable();

	return 0;
}

static void __exit lastbus_exit(void)
{
}

module_init(lastbus_init);
module_exit(lastbus_exit);
