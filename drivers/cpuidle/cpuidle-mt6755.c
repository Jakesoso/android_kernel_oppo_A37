/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/printk.h>
#include "mt_idle.h"

#define IDLE_TAG     "[Power/swap]"
#define idle_dbg(fmt, args...)		pr_debug(IDLE_TAG fmt, ##args)

static int mt_dpidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	return dpidle_enter(smp_processor_id());
}

static int mt_soidle3_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	return soidle3_enter(smp_processor_id());
}

static int mt_soidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	return soidle_enter(smp_processor_id());
}

static int mt_mcidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	return mcidle_enter(smp_processor_id());
}

static int mt_slidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	return slidle_enter(smp_processor_id());
}

static int mt_rgidle_enter(struct cpuidle_device *dev,
			      struct cpuidle_driver *drv, int index)
{
	return rgidle_enter(smp_processor_id());
}

static struct cpuidle_driver mt6755_cpuidle_driver = {
	.name             = "mt6755_cpuidle",
	.owner            = THIS_MODULE,
	.states[0] = {
		.enter            = mt_dpidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "dpidle",
		.desc             = "deepidle",
	},
	.states[1] = {
		.enter            = mt_soidle3_enter,
		.exit_latency     = 5000,            /* 5 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "SODI3",
		.desc             = "SODI3",
	},
	.states[2] = {
		.enter            = mt_soidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "SODI",
		.desc             = "SODI",
	},
	.states[3] = {
		.enter            = mt_mcidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "MCDI",
		.desc             = "MCDI",
	},
	.states[4] = {
		.enter            = mt_slidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "slidle",
		.desc             = "slidle",
	},
	.states[5] = {
		.enter            = mt_rgidle_enter,
		.exit_latency     = 2000,            /* 2 ms */
		.target_residency = 1,
		.flags            = CPUIDLE_FLAG_TIME_VALID,
		.name             = "rgidle",
		.desc             = "WFI",
	},
	.state_count = 6,
	.safe_state_index = 0,
};

int __init mt6755_cpuidle_init(void)
{
	return cpuidle_register(&mt6755_cpuidle_driver, NULL);
}
device_initcall(mt6755_cpuidle_init);
