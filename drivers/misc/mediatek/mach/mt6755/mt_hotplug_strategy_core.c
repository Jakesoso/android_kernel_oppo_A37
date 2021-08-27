/**
* @file    mt_hotplug_strategy_core.c
* @brief   hotplug strategy(hps) - core
*/

/*============================================================================*/
/* Include files */
/*============================================================================*/
/* system includes */
#include <linux/kernel.h>
#include <linux/module.h>	/* MODULE_DESCRIPTION, MODULE_LICENSE */
#include <linux/init.h>		/* module_init, module_exit */
#include <linux/cpu.h>		/* cpu_up */
#include <linux/kthread.h>	/* kthread_create */
#include <linux/wakelock.h>	/* wake_lock_init */
#include <asm-generic/bug.h>	/* BUG_ON */

/* project includes */
#include <mach/hotplug.h>
#include <mach/mt_spm_cpu.h>
#include <mach/mt_spm_mtcmos.h>
#include <mach/mt_ppm_api.h>

/* local includes */
#include <mach/mt_hotplug_strategy_internal.h>
#include <mach/mt_hotplug_strategy.h>

/* forward references */

/*============================================================================*/
/* Macro definition */
/*============================================================================*/
/*
 * static
 */
#define STATIC
/* #define STATIC static */
#define MS_TO_NS(x)     (x * 1E6L)
/*
 * config
 */

/*============================================================================*/
/* Local type definition */
/*============================================================================*/

/*============================================================================*/
/* Local function declarition */
/*============================================================================*/

/*============================================================================*/
/* Local variable definition */
/*============================================================================*/

/*============================================================================*/
/* Global variable definition */
/*============================================================================*/
static unsigned long long hps_cancel_time;
static ktime_t ktime;
/*============================================================================*/
/* Local function definition */
/*============================================================================*/
/*
 * hps timer callback
 */
static int _hps_timer_callback(unsigned long data)
{
	int ret;
	/*hps_warn("_hps_timer_callback\n"); */
/* yan.chen@swdp.shanghai add hps enable bit check for wakeup */
#ifdef VENDOR_EDIT
	if (!hps_ctxt.enabled)
		return HRTIMER_NORESTART;
#endif

	if (hps_ctxt.tsk_struct_ptr) {
		ret = wake_up_process(hps_ctxt.tsk_struct_ptr);
		if (!ret)
			pr_err("hps task wake up fail %d\n", ret);
	}else {
		pr_err("hps ptr is NULL\n");
	}

	return HRTIMER_NORESTART;
}

static long int hps_get_current_time_ms(void)
{
	struct timeval t;

	do_gettimeofday(&t);
	return ((t.tv_sec & 0xFFF) * 1000000 + t.tv_usec) / 1000;
}

static void hps_get_sysinfo(void)
{
	unsigned int cpu;
	char str1[64];
	char str2[64];
	int i, j;
	char *str1_ptr = str1;
	char *str2_ptr = str2;
	/*
	 * calculate cpu loading
	 */
	hps_ctxt.cur_loads = 0;
	str1_ptr = str1;
	str2_ptr = str2;

	for_each_possible_cpu(cpu) {
		per_cpu(hps_percpu_ctxt, cpu).load = hps_cpu_get_percpu_load(cpu);
		hps_ctxt.cur_loads += per_cpu(hps_percpu_ctxt, cpu).load;

		if (hps_ctxt.cur_dump_enabled) {
			if (cpu_online(cpu))
				i = sprintf(str1_ptr, "%4u", 1);
			else
				i = sprintf(str1_ptr, "%4u", 0);
			str1_ptr += i;
			j = sprintf(str2_ptr, "%4u", per_cpu(hps_percpu_ctxt, cpu).load);
			str2_ptr += j;
		}
	}

	/*Get heavy task information */
	hps_ctxt.cur_nr_heavy_task = hps_cpu_get_nr_heavy_task();

	/*Get sys TLP information */
	hps_cpu_get_tlp(&hps_ctxt.cur_tlp, &hps_ctxt.cur_iowait);

#if 0
	ppm_lock(&ppm_main_info.lock);
	ppm_hps_algo_data.ppm_cur_loads = hps_ctxt.cur_loads;
	ppm_hps_algo_data.ppm_cur_nr_heavy_task = hps_ctxt.cur_nr_heavy_task;
	ppm_hps_algo_data.ppm_cur_tlp = hps_ctxt.cur_tlp;
	ppm_unlock(&ppm_main_info.lock);
#endif
	/* hps_warn("@%s:hps_ctxt  CPU loading : %d,     heavy task : %d, TLP : %d\n", __func__, hps_ctxt.cur_loads, hps_ctxt.cur_nr_heavy_task, hps_ctxt.cur_tlp); */
	/* hps_warn("@%s: ppm_hps_algo_data CPU loading : %d,    heavy task : %d, TLP : %d\n", __func__, ppm_hps_algo_data.ppm_cur_loads, ppm_hps_algo_data.ppm_cur_nr_heavy_task, ppm_hps_algo_data.ppm_cur_tlp); */


}

/*
 * hps task main loop
 */
static int _hps_task_main(void *data)
{
	int cnt = 0;
	void (*algo_func_ptr)(void);

	hps_ctxt_print_basic(1);

	if (hps_ctxt.is_hmp)
		algo_func_ptr = hps_algo_hmp;
	else if (hps_ctxt.is_amp)
		algo_func_ptr = hps_algo_amp;
	else
		algo_func_ptr = hps_algo_smp;

	while (1) {
		/* TODO: showld we do dvfs? */
		/* struct cpufreq_policy *policy; */
		/* policy = cpufreq_cpu_get(0); */
		/* dbs_freq_increase(policy, policy->max); */
		/* cpufreq_cpu_put(policy); */
#ifdef CONFIG_CPU_ISOLATION
		if (hps_ctxt.wake_up_by_fasthotplug) {

			mutex_lock(&hps_ctxt.lock);
			struct cpumask cpu_down_cpumask;

			cpumask_setall(&cpu_down_cpumask);
			cpumask_clear_cpu(hps_ctxt.root_cpu, &cpu_down_cpumask);
			cpu_down_by_mask(&cpu_down_cpumask);

			hps_ctxt.wake_up_by_fasthotplug = 0;
			mutex_unlock(&hps_ctxt.lock);
			goto HPS_WAIT_EVENT;
		}
#endif
		/*Get sys status */
		hps_get_sysinfo();

		mt_ppm_hica_update_algo_data(hps_ctxt.cur_loads, hps_ctxt.cur_nr_heavy_task,
					     hps_ctxt.cur_tlp);

		/*Execute PPM main function */
		mt_ppm_main();

		/*execute hotplug algorithm */
		(*algo_func_ptr) ();

#ifdef CONFIG_CPU_ISOLATION
HPS_WAIT_EVENT:
#endif
		if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_WAIT_QUEUE) {
			wait_event_timeout(hps_ctxt.wait_queue,
					   atomic_read(&hps_ctxt.is_ondemand) != 0,
					   msecs_to_jiffies(HPS_TIMER_INTERVAL_MS));
		} else if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_TIMER) {
			if (atomic_read(&hps_ctxt.is_ondemand) == 0) {
				mod_timer(&hps_ctxt.tmr_list,
					  (jiffies + msecs_to_jiffies(HPS_TIMER_INTERVAL_MS)));
				set_current_state(TASK_INTERRUPTIBLE);
				schedule();
			}
		} else if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_HR_TIMER) {
			hrtimer_cancel(&hps_ctxt.hr_timer);
			hrtimer_start(&hps_ctxt.hr_timer, ktime, HRTIMER_MODE_REL);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
		}

		if (kthread_should_stop())
			break;
	}			/* while(1) */

	hps_warn("leave _hps_task_main, cnt:%08d\n", cnt++);
	return 0;
}

/*============================================================================*/
/* Gobal function definition */
/*============================================================================*/
/*
 * hps task control interface
 */
int hps_task_start(void)
{
	struct sched_param param = {.sched_priority = HPS_TASK_PRIORITY };

	if (hps_ctxt.tsk_struct_ptr == NULL) {
		hps_ctxt.tsk_struct_ptr = kthread_create(_hps_task_main, NULL, "hps_main");
		if (IS_ERR(hps_ctxt.tsk_struct_ptr))
			return PTR_ERR(hps_ctxt.tsk_struct_ptr);

		sched_setscheduler_nocheck(hps_ctxt.tsk_struct_ptr, SCHED_FIFO, &param);
		get_task_struct(hps_ctxt.tsk_struct_ptr);
		wake_up_process(hps_ctxt.tsk_struct_ptr);
		hps_warn("hps_task_start success, ptr: %p, pid: %d\n", hps_ctxt.tsk_struct_ptr,
			 hps_ctxt.tsk_struct_ptr->pid);
	} else {
		hps_warn("hps task already exist, ptr: %p, pid: %d\n", hps_ctxt.tsk_struct_ptr,
			 hps_ctxt.tsk_struct_ptr->pid);
	}
	return 0;
}

void hps_task_stop(void)
{
	if (hps_ctxt.tsk_struct_ptr) {
		kthread_stop(hps_ctxt.tsk_struct_ptr);
		put_task_struct(hps_ctxt.tsk_struct_ptr);
		hps_ctxt.tsk_struct_ptr = NULL;
	}
}

void hps_task_wakeup_nolock(void)
{
/* yan.chen@swdp.shanghai add hps enable bit check for wakeup */
#ifdef VENDOR_EDIT
	if (!hps_ctxt.enabled)
		return;
#endif

	if (hps_ctxt.tsk_struct_ptr) {
		atomic_set(&hps_ctxt.is_ondemand, 1);
		if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_WAIT_QUEUE)
			wake_up(&hps_ctxt.wait_queue);
		else if ((hps_ctxt.periodical_by == HPS_PERIODICAL_BY_TIMER)
			 || (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_HR_TIMER))
			wake_up_process(hps_ctxt.tsk_struct_ptr);
	}
}

void hps_task_wakeup(void)
{
	mutex_lock(&hps_ctxt.lock);

	hps_task_wakeup_nolock();

	mutex_unlock(&hps_ctxt.lock);
}

int little_min = -1;
int little_max = -1;
int big_min = -1;
int big_max = -1;

static void ppm_limit_callback(struct ppm_client_req req)
{
	struct ppm_client_req *p = (struct ppm_client_req *)&req;
	void (*algo_func_ptr)(void);

	if (!p->cpu_limit[0].has_advise_core) {
		little_min = p->cpu_limit[0].min_cpu_core;
		little_max = p->cpu_limit[0].max_cpu_core;
	} else {
		little_min = little_max = p->cpu_limit[0].advise_cpu_core;
	}
	if (!p->cpu_limit[1].has_advise_core) {
		big_min = p->cpu_limit[1].min_cpu_core;
		big_max = p->cpu_limit[1].max_cpu_core;
	} else {
		big_min = big_max = p->cpu_limit[1].advise_cpu_core;
	}
	hps_set_PPM_request(little_min, little_max, big_min, big_max);

	if (hps_ctxt.is_hmp)
		algo_func_ptr = hps_algo_hmp;
	else if (hps_ctxt.is_amp)
		algo_func_ptr = hps_algo_amp;
	else
		algo_func_ptr = hps_algo_smp;

	/*execute hotplug algorithm */
	(*algo_func_ptr) ();
	/*hps_warn("[ppm_limit_callback]little_max %d, big_max %d, little_min %d, big_min %d\n",
	   little_max, big_max, little_min, big_min); */
	/* hps_set_cpu_num_limit(LIMIT_PPM_SERV,(little_max == -1) ? hps_ctxt.little_cpu_id_max : little_max, (big_max == -1) ? hps_ctxt.big_cpu_id_max : big_max); */
	/* hps_set_cpu_num_base(BASE_PPM_SERV,(little_min == -1) ? 0 : little_min, (big_min == -1) ? 0 : big_min); */
	return;
}

/*
 * init
 */
int hps_core_init(void)
{
	int r = 0;

	hps_warn("hps_core_init\n");
	if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_TIMER) {
		/*init timer */
		init_timer(&hps_ctxt.tmr_list);
		/*init_timer_deferrable(&hps_ctxt.tmr_list); */
		hps_ctxt.tmr_list.function = (void *)&_hps_timer_callback;
		hps_ctxt.tmr_list.data = (unsigned long)&hps_ctxt;
		hps_ctxt.tmr_list.expires = jiffies + msecs_to_jiffies(HPS_TIMER_INTERVAL_MS);
		add_timer(&hps_ctxt.tmr_list);
	} else if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_HR_TIMER) {
		ktime = ktime_set(0, MS_TO_NS(HPS_TIMER_INTERVAL_MS));
		/*init Hrtimer */
		hrtimer_init(&hps_ctxt.hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hps_ctxt.hr_timer.function = (void *)&_hps_timer_callback;
		hrtimer_start(&hps_ctxt.hr_timer, ktime, HRTIMER_MODE_REL);

	}
	/* init and start task */
	r = hps_task_start();
	if (r) {
		hps_error("hps_task_start fail(%d)\n", r);
		return r;
	}

	mt_ppm_register_client(PPM_CLIENT_HOTPLUG, &ppm_limit_callback);	/* register PPM callback */
	return r;
}

/*
 * deinit
 */
int hps_core_deinit(void)
{
	int r = 0;

	hps_warn("hps_core_deinit\n");
	if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_TIMER) {
		/*deinit timer */
		del_timer_sync(&hps_ctxt.tmr_list);
	} else if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_HR_TIMER) {
		/*deinit timer */
		r = hrtimer_cancel(&hps_ctxt.hr_timer);
		if (r)
			hps_error("hps hr timer delete error!\n");
	}

	hps_task_stop();
	return r;
}

int hps_del_timer(void)
{
/* yan.chen@swdp.shanghai: Provide the possibility to allow hps_task is fully disabled */
#ifdef VENDOR_EDIT
	if (hps_ctxt.init_state != INIT_STATE_DONE)
		return 0;
#endif

#if 1
	if (!hps_cancel_time)
		hps_cancel_time = hps_get_current_time_ms();
	if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_TIMER) {
		/*deinit timer */
		del_timer_sync(&hps_ctxt.tmr_list);
	} else if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_HR_TIMER) {
		hrtimer_cancel(&hps_ctxt.hr_timer);
	}
#endif
	return 0;
}

int hps_restart_timer(void)
{
/* yan.chen@swdp.shanghai: Provide the possibility to allow hps_task is fully disabled */
#ifdef VENDOR_EDIT
	if (hps_ctxt.init_state != INIT_STATE_DONE)
		return 0;
#endif

#if 1
	unsigned long long time_differ = 0;

	time_differ = hps_get_current_time_ms() - hps_cancel_time;
	if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_TIMER) {
		/*init timer */
		init_timer(&hps_ctxt.tmr_list);
		/*init_timer_deferrable(&hps_ctxt.tmr_list); */
		hps_ctxt.tmr_list.function = (void *)&_hps_timer_callback;
		hps_ctxt.tmr_list.data = (unsigned long)&hps_ctxt;

		if (time_differ >= HPS_TIMER_INTERVAL_MS) {
			hps_ctxt.tmr_list.expires =
			    jiffies + msecs_to_jiffies(HPS_TIMER_INTERVAL_MS);
			add_timer(&hps_ctxt.tmr_list);
			hps_task_wakeup_nolock();
			hps_cancel_time = 0;
		} else {
			hps_ctxt.tmr_list.expires =
			    jiffies + msecs_to_jiffies(HPS_TIMER_INTERVAL_MS - time_differ);
			add_timer(&hps_ctxt.tmr_list);
		}
	} else if (hps_ctxt.periodical_by == HPS_PERIODICAL_BY_HR_TIMER) {
#if 1
		hrtimer_start(&hps_ctxt.hr_timer, ktime, HRTIMER_MODE_REL);
		if (time_differ >= HPS_TIMER_INTERVAL_MS) {
			hps_task_wakeup_nolock();
			hps_cancel_time = 0;
		}
#else
		if (time_differ >= HPS_TIMER_INTERVAL_MS) {
			/*init Hrtimer */
			hrtimer_start(&hps_ctxt.hr_timer, ktime, HRTIMER_MODE_REL);
			hps_task_wakeup_nolock();
			hps_cancel_time = 0;
		}
#endif
	}
#endif
	return 0;
}

void hps_dump_task_info(void)
{
	if (hps_ctxt.tsk_struct_ptr) {
		pr_err("HPS task info (run on CPU %d)\n", hps_ctxt.tsk_struct_ptr->on_cpu);
		if (hps_ctxt.tsk_struct_ptr->on_cpu == 0) {
			show_stack(hps_ctxt.tsk_struct_ptr, NULL);
		}
		else {
		}
	}
	else {
		pr_err("%s: no hps_main task\n", __func__);
	}
}
