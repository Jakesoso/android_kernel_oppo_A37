#define DEBUG 1

#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>
#include <linux/printk.h>
#include <linux/platform_device.h>

#include "internal.h"

#define BOOT_STR_SIZE 256
#define BOOT_LOG_NUM 192
#define TRACK_TASK_COMM

struct boot_log_struct {
	/* task cmdline for first 16 bytes
	 * and boot event for the rest
	 * if TRACK_TASK_COMM is on */
	char *comm_event;
#ifdef TRACK_TASK_COMM
	pid_t pid;
#endif
	u64 timestamp;
} mt_bootprof[BOOT_LOG_NUM];

static int boot_log_count;
static DEFINE_MUTEX(mt_bootprof_lock);
static bool mt_bootprof_enabled;
static int bootprof_lk_t, bootprof_pl_t;
static u64 timestamp_on, timestamp_off;
int boot_finish = 0;

module_param_named(pl_t, bootprof_pl_t, int, S_IRUGO | S_IWUSR);
module_param_named(lk_t, bootprof_lk_t, int, S_IRUGO | S_IWUSR);

#define MSG_SIZE 128

void log_boot(char *str)
{
	unsigned long long ts;
	struct boot_log_struct *p = &mt_bootprof[boot_log_count];
	size_t n = strlen(str) + 1;

	if (!mt_bootprof_enabled)
		return;
	ts = sched_clock();
	pr_err("BOOTPROF:%10Ld.%06ld:%s\n", nsec_high(ts), nsec_low(ts), str);
	if (boot_log_count >= BOOT_LOG_NUM) {
		pr_err("[BOOTPROF] not enuough bootprof buffer\n");
		return;
	}
	mutex_lock(&mt_bootprof_lock);
	p->timestamp = ts;
#ifdef TRACK_TASK_COMM
	p->pid = current->pid;
	n += TASK_COMM_LEN;
#endif
	p->comm_event = kzalloc(n, GFP_ATOMIC | __GFP_NORETRY |
			  __GFP_NOWARN);
	if (!p->comm_event) {
		mt_bootprof_enabled = false;
		goto out;
	}
#ifdef TRACK_TASK_COMM
	memcpy(p->comm_event, current->comm, TASK_COMM_LEN);
	memcpy(p->comm_event + TASK_COMM_LEN, str, n - TASK_COMM_LEN);
#else
	memcpy(p->comm_event, str, n);
#endif
	boot_log_count++;
out:
	mutex_unlock(&mt_bootprof_lock);
}

void bootprof_initcall(initcall_t fn, unsigned long long ts)
{
#define INITCALL_THRESHOLD 15000000
	/* log more than 15ms initcalls */
	unsigned long msec_rem;
	char msgbuf[MSG_SIZE];

	if (ts > INITCALL_THRESHOLD) {
		msec_rem = do_div(ts, NSEC_PER_MSEC);
		snprintf(msgbuf, MSG_SIZE, "initcall: %pf %5llu.%06lums",
			 fn, ts, msec_rem);
		log_boot(msgbuf);
	}
}

void bootprof_probe(unsigned long long ts, struct device *dev,
			   struct device_driver *drv, unsigned long probe)
{
#define PROBE_THRESHOLD 15000000
	/* log more than 15ms probes*/
	unsigned long msec_rem;
	char msgbuf[MSG_SIZE];

	if (ts <= PROBE_THRESHOLD)
		return;
	msec_rem = do_div(ts, NSEC_PER_MSEC);
	snprintf(msgbuf, MSG_SIZE, "probe:%s%s(%p)%s%s(%p) probe=%pf %5llu.%06lums",
		 drv ? " drv=" : "",
		 (drv && drv->name) ? drv->name : "", (void *)drv,
		 dev ? " dev=" : "",
		 (dev && dev->init_name) ? dev->init_name : "", (void *)dev,
		 (void *)probe, ts, msec_rem);
	log_boot(msgbuf);
}

void bootprof_pdev_register(unsigned long long ts, struct platform_device *pdev)
{
#define PROBE_THRESHOLD 15000000
	/* log more than 15ms probes*/
	unsigned long msec_rem;
	char msgbuf[MSG_SIZE];

	if (ts <= PROBE_THRESHOLD || !pdev)
		return;
	msec_rem = do_div(ts, NSEC_PER_MSEC);
	snprintf(msgbuf, MSG_SIZE, "probe: pdev=%s(%p) %5llu.%06lums",
		 pdev->name, (void *)pdev, ts, msec_rem);
	log_boot(msgbuf);
}

static void bootup_finish(void)
{
#ifdef CONFIG_MT_PRINTK_UART_CONSOLE
	mt_disable_uart();
#endif
	set_logtoomuch_enable(1);
}

/* extern void (*set_intact_mode)(void); */
static void mt_bootprof_switch(int on)
{
	mutex_lock(&mt_bootprof_lock);
	if (mt_bootprof_enabled ^ on) {
		unsigned long long ts = sched_clock();

		pr_err("BOOTPROF:%10Ld.%06ld: %s\n",
		       nsec_high(ts), nsec_low(ts), on ? "ON" : "OFF");

		if (on) {
			mt_bootprof_enabled = 1;
			timestamp_on = ts;
		} else {
			/* boot up complete */
			/* mt_bootprof_enabled = 0; */
			timestamp_off = ts;
			boot_finish = 1;
			/* log_store_bootup(); */
			bootup_finish();
		}
	}
	mutex_unlock(&mt_bootprof_lock);
}

static ssize_t
mt_bootprof_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[BOOT_STR_SIZE];
	size_t copy_size = cnt;

	if (cnt >= sizeof(buf))
		copy_size = BOOT_STR_SIZE - 1;

	if (copy_from_user(&buf, ubuf, copy_size))
		return -EFAULT;

	if (cnt == 1 && buf[0] == '1') {
		mt_bootprof_switch(1);
		return 1;
	} else if (cnt == 1 && buf[0] == '0') {
		mt_bootprof_switch(0);
		return 1;
	}

	buf[copy_size] = 0;
	log_boot(buf);

	return cnt;

}

static int mt_bootprof_show(struct seq_file *m, void *v)
{
	int i;
	struct boot_log_struct *p;

	SEQ_printf(m, "----------------------------------------\n");
	SEQ_printf(m, "%d	    BOOT PROF (unit:msec)\n", mt_bootprof_enabled);
	SEQ_printf(m, "----------------------------------------\n");

	if (bootprof_pl_t > 0 && bootprof_lk_t > 0) {
		SEQ_printf(m, "%10d        : %s\n", bootprof_pl_t, "preloader");
		SEQ_printf(m, "%10d        : %s\n", bootprof_lk_t, "lk");
		SEQ_printf(m, "%10d        : %s\n",
			   gpt_boot_time() - bootprof_pl_t - bootprof_lk_t, "lk->Kernel");
		SEQ_printf(m, "----------------------------------------\n");
	}

	SEQ_printf(m, "%10Ld.%06ld : ON\n",
		   nsec_high(timestamp_on), nsec_low(timestamp_on));

	for (i = 0; i < boot_log_count; i++) {
		p = &mt_bootprof[i];
		if (!p->comm_event)
			continue;
#ifdef TRACK_TASK_COMM
#define FMT "%10Ld.%06ld :%5d-%-16s: %s\n"
#else
#define FMT "%10Ld.%06ld : %s\n"
#endif
		SEQ_printf(m, FMT, nsec_high(p->timestamp),
			   nsec_low(p->timestamp),
#ifdef TRACK_TASK_COMM
			   p->pid, p->comm_event, p->comm_event + TASK_COMM_LEN
#else
			   p->comm_event
#endif
			   );
	}

	SEQ_printf(m, "%10Ld.%06ld : OFF\n",
		   nsec_high(timestamp_off), nsec_low(timestamp_off));
	SEQ_printf(m, "----------------------------------------\n");
	return 0;
}

/*** Seq operation of mtprof ****/
static int mt_bootprof_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt_bootprof_show, inode->i_private);
}

static const struct file_operations mt_bootprof_fops = {
	.open = mt_bootprof_open,
	.write = mt_bootprof_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_boot_prof(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("bootprof", 0664, NULL, &mt_bootprof_fops);
	if (!pe)
		return -ENOMEM;
	/* set_intact_mode = NULL; */
	return 0;
}

static int __init init_bootprof_buf(void)
{
	mt_bootprof_switch(1);
	return 0;
}

early_initcall(init_bootprof_buf);
device_initcall(init_boot_prof);
