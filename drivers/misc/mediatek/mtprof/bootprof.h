/*
  boot logger: drivers/misc/mtprof/bootprof
  interface: /proc/bootprof
*/
#ifndef _BOOTPROF_H_
#define _BOOTPROF_H_
#ifdef CONFIG_SCHEDSTATS
extern void log_boot(char *str);
#else
#define log_boot(str)
#endif

#include <linux/sched.h>
#ifndef TIME_LOG_START
#define TIME_LOG_START() \
	({ts = sched_clock(); })
#endif

#ifndef TIME_LOG_END
#define TIME_LOG_END() \
	({ts = sched_clock() - ts; })
#endif

#include <linux/platform_device.h>
void bootprof_initcall(initcall_t fn, unsigned long long ts);
void bootprof_probe(unsigned long long ts, struct device *dev,
		    struct device_driver *drv, unsigned long probe);
void bootprof_pdev_register(unsigned long long ts,
			    struct platform_device *pdev);
#endif
