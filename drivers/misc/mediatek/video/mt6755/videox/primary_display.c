#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/rtpm_prio.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/ion_drv.h>
#include <linux/mtk_ion.h>
#include <mach/mt_idle.h>
#include <mach/mt_spm_reg.h>
#include <mach/pcm_def.h>
#include <mach/mt_spm_idle.h>
#include "mach/eint.h"
#if defined(CONFIG_MTK_LEGACY)
#include <cust_eint.h>
#endif
#include <mach/mt_smi.h>
#include <mach/m4u.h>
#include <mach/m4u_port.h>
#include <mach/mt_vcorefs_manager.h>

#include "disp_drv_platform.h"
#include "debug.h"
#include "disp_drv_log.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "mtkfb.h"
#include "ddp_hal.h"
#include "ddp_dump.h"
#include "ddp_path.h"
#include "ddp_drv.h"
#include "ddp_reg.h"
#include "disp_session.h"
#include "primary_display.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_manager.h"
#include "mtkfb_fence.h"
#include "display_recorder.h"
#include "fbconfig_kdebug.h"
#include "ddp_mmp.h"
#include "mtk_sync.h"
#include "ddp_irq.h"
#include "disp_session.h"
#include "disp_helper.h"
#include "ddp_reg.h"
#include "mtk_disp_mgr.h"
#include "ddp_dsi.h"
#include "m4u.h"
#include "mtkfb_console.h"
#if defined(CONFIG_MTK_LEGACY)
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#else
#include "disp_dts_gpio.h"
#endif
#include "ddp_clkmgr.h"
#include "mt_smi.h"
#include "mmdvfs_mgr.h"
#include "disp_lowpower.h"
#include "disp_recovery.h"
#include "smart_ovl.h"

#ifdef VENDOR_EDIT
/* liping-m@Mobile Phone Software Dept.Driver, 2016/01/12  Add for except log */
#include <linux/soc/oppo/mmkey_log.h>
#include <soc/oppo/oppo_project.h>
#endif /*VENDOR_EDIT*/

#define _DEBUG_DITHER_HANG_

#define FRM_UPDATE_SEQ_CACHE_NUM (DISP_INTERNAL_BUFFER_COUNT+1)

static disp_internal_buffer_info *decouple_buffer_info[DISP_INTERNAL_BUFFER_COUNT];
static disp_internal_buffer_info *gmo_decouple_buffer_info;
static RDMA_CONFIG_STRUCT decouple_rdma_config;
static WDMA_CONFIG_STRUCT decouple_wdma_config;
static disp_mem_output_config mem_config;
static unsigned int primary_session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
static disp_frm_seq_info frm_update_sequence[FRM_UPDATE_SEQ_CACHE_NUM];
static unsigned int frm_update_cnt;
static unsigned int gPresentFenceIndex;
static unsigned int g_keep;
static unsigned int g_skip;
/* dvfs */
static atomic_t dvfs_ovl_req_status = ATOMIC_INIT(OPPI_UNREQ);
static int dvfs_last_ovl_req = OPPI_UNREQ;

static struct hrtimer cmd_mode_update_timer;
/* static ktime_t cmd_mode_update_timer_period; */
static int is_fake_timer_inited;

static struct task_struct *primary_display_switch_dst_mode_task;
static struct task_struct *present_fence_release_worker_task;
static struct task_struct *primary_path_aal_task;
static struct task_struct *decouple_update_rdma_config_thread;
static struct task_struct *decouple_trigger_thread;
static struct sg_table table;
static DECLARE_WAIT_QUEUE_HEAD(display_state_wait_queue);

static int primary_display_recovery_thread(void *data);
static int decouple_mirror_update_rdma_config_thread(void *data);
static int decouple_trigger_worker_thread(void *data);
static unsigned int _need_lfr_check(void);
struct task_struct *primary_display_frame_update_task = NULL;
wait_queue_head_t primary_display_frame_update_wq;
atomic_t primary_display_frame_update_event = ATOMIC_INIT(0);
DISP_PRIMARY_PATH_MODE primary_display_mode = DIRECT_LINK_MODE;
int primary_display_def_dst_mode = 0;
int primary_display_cur_dst_mode = 0;
unsigned long long last_primary_trigger_time;
bool is_switched_dst_mode = false;
int primary_trigger_cnt = 0;
atomic_t decouple_update_rdma_event = ATOMIC_INIT(0);
DECLARE_WAIT_QUEUE_HEAD(decouple_update_rdma_wq);
atomic_t decouple_trigger_event = ATOMIC_INIT(0);
DECLARE_WAIT_QUEUE_HEAD(decouple_trigger_wq);
wait_queue_head_t primary_display_present_fence_wq;
atomic_t primary_display_present_fence_update_event = ATOMIC_INIT(0);

/*detect hang and recovery*/
static struct task_struct *primary_display_Recovery_task;
static wait_queue_head_t primary_recovery_task_wq;
static atomic_t primary_recovery_task_wakeup = ATOMIC_INIT(0);

typedef struct {
	DISP_POWER_STATE state;
	unsigned int lcm_fps;
	int max_layer;
	int need_trigger_overlay;
	int need_trigger_ovl1to2;
	int need_trigger_dcMirror_out;
	DISP_PRIMARY_PATH_MODE mode;
	unsigned int session_id;
	int session_mode;
	int ovl1to2_mode;
	unsigned int last_vsync_tick;
	unsigned long framebuffer_mva;
	unsigned long framebuffer_va;
/* yan.chen@swdp.shanghai: add frame cnt variable to summarize all frame updates on MTK platform
 * The structure location is different on MT6755, MT6795 and MT6752.
 * On MT6755, it is in drivers/misc/mediatek/video/mt6755/videox/primary_display.c
 * On MT6795, it is in drivers/misc/mediatek/videox/mt6795/primary_display.c
 * On MT6752, it is in drivers/videox/primary_display.c
 */
#ifdef VENDOR_EDIT
        unsigned int frame_cnt;
#endif
	struct mutex lock;
	struct mutex capture_lock;
	struct mutex switch_dst_lock;
	disp_lcm_handle *plcm;
	cmdqRecHandle cmdq_handle_config_esd;
	cmdqRecHandle cmdq_handle_config;
	disp_path_handle dpmgr_handle;
	disp_path_handle ovl2mem_path_handle;
	cmdqRecHandle cmdq_handle_ovl1to2_config;
	cmdqRecHandle cmdq_handle_trigger;
	char *mutex_locker;
	int vsync_drop;
	unsigned int dc_buf_id;
	unsigned int dc_buf[DISP_INTERNAL_BUFFER_COUNT];
	unsigned int force_fps_keep_count;
	unsigned int force_fps_skip_count;
	cmdqBackupSlotHandle cur_config_fence;
	cmdqBackupSlotHandle subtractor_when_free;
	cmdqBackupSlotHandle rdma_buff_info;
	cmdqBackupSlotHandle ovl_status_info;
	cmdqBackupSlotHandle ovl_config_time;
	cmdqBackupSlotHandle dither_status_info;
	#ifdef VENDOR_EDIT
	/* pdl@oppo.com,2016/04/28 add for debug system hang */
	cmdqBackupSlotHandle ovl_status_info_debug;
	#endif

	int is_primary_sec;
} display_primary_path_context;

#define pgc	_get_context()


static display_primary_path_context *_get_context(void)
{
	static int is_context_inited;
	static display_primary_path_context g_context;
	if (!is_context_inited) {
		memset((void *)&g_context, 0, sizeof(display_primary_path_context));
		is_context_inited = 1;
	}

	return &g_context;
}


#ifdef VENDOR_EDIT
/* pdl@oppo.com,2016/04/28 add for debug system hang */
void dump_ovl_status_debug(void)
{
    unsigned int status_ovl0;
    unsigned int status_ovl0_2;
    unsigned int status_ovl1_2;
    cmdqBackupReadSlot(pgc->ovl_status_info_debug, 0, &status_ovl0);
    cmdqBackupReadSlot(pgc->ovl_status_info_debug, 1, &status_ovl0_2);
    cmdqBackupReadSlot(pgc->ovl_status_info_debug, 2, &status_ovl1_2);
    DISPERR("------------ovl0_status_info_debug 0x%x-----------\n", status_ovl0);
    DISPERR("------------ovl0_2_status_info_debug 0x%x-----------\n", status_ovl0_2);
    DISPERR("------------ovl1_2_status_info_debug 0x%x-----------\n", status_ovl0_2);
}
#endif


/* yan.chen@swdp.shanghai: add frame cnt variable to summarize all frame updates on MTK platform
 */
#ifdef VENDOR_EDIT
unsigned int get_frame_cnt(void)
{
	return pgc->frame_cnt;
}
EXPORT_SYMBOL(get_frame_cnt);

unsigned int get_display_state(void)
{
	if (pgc->state == DISP_ALIVE)
		return 0;
	else
		return 1;
}
EXPORT_SYMBOL(get_display_state);
#endif

static void _primary_path_lock(const char *caller)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_MUTEX, 0, 0);
	disp_sw_mutex_lock(&(pgc->lock));
	pgc->mutex_locker = (char *)caller;
}

static void _primary_path_unlock(const char *caller)
{
	pgc->mutex_locker = NULL;
	disp_sw_mutex_unlock(&(pgc->lock));
	dprec_logger_done(DPREC_LOGGER_PRIMARY_MUTEX, 0, 0);
}

const char *p_session_mode_spy(unsigned int mode)
{
	switch (mode) {
	case DISP_SESSION_DIRECT_LINK_MODE:
		return "DIRECT_LINK";
	case DISP_SESSION_DIRECT_LINK_MIRROR_MODE:
		return "DIRECT_LINK_MIRROR";
	case DISP_SESSION_DECOUPLE_MODE:
		return "DECOUPLE";
	case DISP_SESSION_DECOUPLE_MIRROR_MODE:
		return "DECOUPLE_MIRROR";
	case DISP_SESSION_RDMA_MODE:
		return "RDMA_MODE";
	default:
		return "UNKNOWN";
	}
}

int primary_display_is_directlink_mode(void)
{
	DISP_MODE mode = pgc->session_mode;
	if (mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE || mode == DISP_SESSION_DIRECT_LINK_MODE)
		return 1;
	else
		return 0;
}

DISP_MODE primary_get_sess_mode(void)
{
	return pgc->session_mode;
}

unsigned int primary_get_sess_id(void)
{
	return pgc->session_id;
}

disp_lcm_handle *primary_get_lcm(void)
{
	return pgc->plcm;
}

void *primary_get_dpmgr_handle(void)
{
	return pgc->dpmgr_handle;
}

void *primary_get_config_handle(void)
{
	return pgc->cmdq_handle_config;
}

void *primary_get_ovl2mem_handle(void)
{
	return pgc->ovl2mem_path_handle;
}
int primary_display_is_decouple_mode(void)
{
	DISP_MODE mode = pgc->session_mode;
	if (mode == DISP_SESSION_DECOUPLE_MODE || mode == DISP_SESSION_DECOUPLE_MIRROR_MODE)
		return 1;
	else
		return 0;
}

int _is_mirror_mode(DISP_MODE mode)
{
	if (mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE || mode == DISP_SESSION_DECOUPLE_MIRROR_MODE)
		return 1;
	else
		return 0;
}
int primary_display_is_mirror_mode(void)
{
	return _is_mirror_mode(pgc->session_mode);
}

int primary_is_sec(void)
{
	return pgc->is_primary_sec;
}

void primary_set_sec(int has_sec_layer)
{
	pgc->is_primary_sec = has_sec_layer;
}

void _primary_path_switch_dst_lock(void)
{
	mutex_lock(&(pgc->switch_dst_lock));
}

void _primary_path_switch_dst_unlock(void)
{
	mutex_unlock(&(pgc->switch_dst_lock));
}

static int _disp_primary_path_switch_dst_mode_thread(void *data)
{
	while (1) {
		msleep(1000);

		if (((sched_clock() - last_primary_trigger_time) / 1000) > 500000) {	/* 500ms not trigger disp */
			primary_display_switch_dst_mode(0);	/* switch to cmd mode */
			is_switched_dst_mode = true;
		}
		if (kthread_should_stop())
			break;
	}
	return 0;
}

DISP_POWER_STATE primary_get_state(void)
{
	return pgc->state;
}
static DISP_POWER_STATE primary_set_state(DISP_POWER_STATE new_state)
{
	DISP_POWER_STATE old_state = pgc->state;
	pgc->state = new_state;
	DISPMSG("%s %d to %d\n", __func__, old_state, new_state);
	wake_up(&display_state_wait_queue);
	return old_state;
}

void primary_display_set_state(DISP_POWER_STATE new_state)
{
	primary_set_state(new_state);
}

/* use MAX_SCHEDULE_TIMEOUT to wait for ever
 * NOTES: primary_path_lock should NOT be held when call this func !!!!!!!!
 */
#define __primary_display_wait_state(condition, timeout)			\
({									\
	wait_event_timeout(display_state_wait_queue, condition, timeout);\
})

long primary_display_wait_state(DISP_POWER_STATE state, long timeout)
{
	long ret;
	ret = __primary_display_wait_state(primary_get_state() == state, timeout);
	return ret;
}
long primary_display_wait_not_state(DISP_POWER_STATE state, long timeout)
{
	long ret;
	ret = __primary_display_wait_state(primary_get_state() != state, timeout);
	return ret;
}


/*********************** idle manager *****************************************/
int primary_display_get_debug_state(char *stringbuf, int buf_len)
{
	int len = 0;
	LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);
	LCM_DRIVER *lcm_drv = pgc->plcm->drv;
	len += scnprintf(stringbuf + len, buf_len - len,
		      "|--------------------------------------------------------------------------------------|\n");
	len += scnprintf(stringbuf + len, buf_len - len,
		      "|********Primary Display Path General Information********\n");
	len += scnprintf(stringbuf + len, buf_len - len, "|Primary Display is %s\n",
		      dpmgr_path_is_idle(pgc->dpmgr_handle) ? "idle" : "busy");

	if (mutex_trylock(&(pgc->lock))) {
		mutex_unlock(&(pgc->lock));
		len += scnprintf(stringbuf + len, buf_len - len,
			      "|primary path global mutex is free\n");
	} else {
		len += scnprintf(stringbuf + len, buf_len - len,
			      "|primary path global mutex is hold by [%s]\n", pgc->mutex_locker);
	}

	if (lcm_param && lcm_drv)
		len += scnprintf(stringbuf + len, buf_len - len,
			      "|LCM Driver=[%s]\tResolution=%dx%d,Interface:%s\n", lcm_drv->name,
			      lcm_param->width, lcm_param->height,
			      (lcm_param->type == LCM_TYPE_DSI) ? "DSI" : "Other");
	len += scnprintf(stringbuf + len, buf_len - len,
		      "|State=%s\tlcm_fps=%d\tmax_layer=%d\tmode:%d\tvsync_drop=%d\n",
		      pgc->state == DISP_ALIVE ? "Alive" : "Sleep", pgc->lcm_fps, pgc->max_layer,
		      pgc->mode, pgc->vsync_drop);
	len += scnprintf(stringbuf + len, buf_len - len,
		      "|cmdq_handle_config=%p\tcmdq_handle_trigger=%p\tdpmgr_handle=%p\tovl2mem_path_handle=%p\n",
		      pgc->cmdq_handle_config, pgc->cmdq_handle_trigger, pgc->dpmgr_handle,
		      pgc->ovl2mem_path_handle);
	len += scnprintf(stringbuf + len, buf_len - len,
			"|PathMode:%s\n",(char *)p_session_mode_spy(pgc->session_mode));
	len += scnprintf(stringbuf + len, buf_len - len, "|Current display driver status=%s + %s\n",
		      primary_display_is_video_mode() ? "video mode" : "cmd mode",
		      primary_display_cmdq_enabled() ? "CMDQ Enabled" : "CMDQ Disabled");

	return len;
}

static DISP_MODULE_ENUM _get_dst_module_by_lcm(disp_lcm_handle *plcm)
{
	DISP_MODULE_ENUM ret = DISP_MODULE_UNKNOWN;

	if (plcm == NULL) {
		DISPERR("plcm is null\n");
		return ret;
	}

	if (plcm->params->type == LCM_TYPE_DSI) {
		if (plcm->lcm_if_id == LCM_INTERFACE_DSI0)
			ret = DISP_MODULE_DSI0;
		else if (plcm->lcm_if_id == LCM_INTERFACE_DSI1)
			ret = DISP_MODULE_DSI1;
		else if (plcm->lcm_if_id == LCM_INTERFACE_DSI_DUAL)
			ret = DISP_MODULE_DSIDUAL;
		else
			ret = DISP_MODULE_DSI0;
	} else if (plcm->params->type == LCM_TYPE_DPI) {
		ret = DISP_MODULE_DPI;
	} else {
		DISPERR("can't find primary path dst module\n");
		ret = DISP_MODULE_UNKNOWN;
	}
	return ret;
}


/***************************************************************
***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 1.wait idle:           N         N       Y        Y
*** 2.lcm update:          N         Y       N        Y
*** 3.path start:	idle->Y      Y    idle->Y     Y
*** 4.path trigger:     idle->Y      Y    idle->Y     Y
*** 5.mutex enable:        N         N    idle->Y     Y
*** 6.set cmdq dirty:      N         Y       N        N
*** 7.flush cmdq:          Y         Y       N        N
****************************************************************/

int _should_wait_path_idle(void)
{
	/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
	*** 1.wait idle:	          N         N        Y        Y					*/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 0;
		else
			return 0;

	} else {
		if (primary_display_is_video_mode())
			return dpmgr_path_is_busy(pgc->dpmgr_handle);
		else
			return dpmgr_path_is_busy(pgc->dpmgr_handle);

	}
}

int _should_update_lcm(void)
{
	int ret = 0;
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 2.lcm update:          N         Y       N        Y        **/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			ret = 0;
		else {
			/* lcm_update can't use cmdq now */
			ret = 0;
		}
	} else {
		if (primary_display_is_video_mode())
			ret = 0;
		else
			ret = 1;
	}
	return ret;
}

int _should_start_path(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 3.path start:	idle->Y      Y    idle->Y     Y        ***/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode()) {
			return 0;
			/* return dpmgr_path_is_idle(pgc->dpmgr_handle); */
		} else {
			return 0;
		}
	} else {
		if (primary_display_is_video_mode())
			return dpmgr_path_is_idle(pgc->dpmgr_handle);
		else
			return 1;

	}
}

int _should_trigger_path(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 4.path trigger:     idle->Y      Y    idle->Y     Y
*** 5.mutex enable:        N         N    idle->Y     Y        ***/

	/* this is not a perfect design, we can't decide path trigger(ovl/rdma/dsi..) separately with mutex enable */
	/* but it's lucky because path trigger and mutex enable is the same w/o cmdq, and it's correct w/ CMDQ(Y+N). */
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode()) {
			return 0;
			/* return dpmgr_path_is_idle(pgc->dpmgr_handle); */
		} else {
			return 0;
		}
	} else {
		if (primary_display_is_video_mode())
			return dpmgr_path_is_idle(pgc->dpmgr_handle);
		else
			return 1;

	}
}

int _should_set_cmdq_dirty(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 6.set cmdq dirty:	    N         Y       N        N     ***/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 0;
		else
			return 1;

	} else {
		if (primary_display_is_video_mode())
			return 0;
		else
			return 0;

	}
}

int _should_flush_cmdq_config_handle(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 7.flush cmdq:          Y         Y       N        N        ***/
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 1;
		else
			return 1;

	} else {
		if (primary_display_is_video_mode())
			return 0;
		else
			return 0;

	}
}

int _should_reset_cmdq_config_handle(void)
{
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 1;
		else
			return 1;

	} else {
		if (primary_display_is_video_mode())
			return 0;
		else
			return 0;

	}
}

int _should_insert_wait_frame_done_token(void)
{
/***trigger operation:  VDO+CMDQ  CMD+CMDQ VDO+CPU  CMD+CPU
*** 7.flush cmdq:          Y         Y       N        N      */
	if (primary_display_cmdq_enabled()) {
		if (primary_display_is_video_mode())
			return 1;
		else
			return 1;

	} else {
		if (primary_display_is_video_mode())
			return 0;
		else
			return 0;

	}
}

int _should_trigger_interface(void)
{
	if (pgc->mode == DECOUPLE_MODE)
		return 0;
	else
		return 1;

}

int _should_config_ovl_input(void)
{
	/* should extend this when display path dynamic switch is ready */
	if (pgc->mode == SINGLE_LAYER_MODE || pgc->mode == DEBUG_RDMA1_DSI0_MODE)
		return 0;
	else
		return 1;

}

static long int get_current_time_us(void)
{
	struct timeval t;
	do_gettimeofday(&t);
	return (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;
}


static enum hrtimer_restart _DISP_CmdModeTimer_handler(struct hrtimer *timer)
{
	DISPMSG("fake timer, wake up\n");
	dpmgr_signal_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
#if 0
	if ((get_current_time_us() - pgc->last_vsync_tick) > 16666) {
		dpmgr_signal_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
		pgc->last_vsync_tick = get_current_time_us();
	}
#endif
	hrtimer_forward_now(timer, ns_to_ktime(16666666));
	return HRTIMER_RESTART;
}

int _init_vsync_fake_monitor(int fps)
{
	if (is_fake_timer_inited)
		return 0;

	is_fake_timer_inited = 1;

	if (fps == 0)
		fps = 6000;


	hrtimer_init(&cmd_mode_update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cmd_mode_update_timer.function = _DISP_CmdModeTimer_handler;
	hrtimer_start(&cmd_mode_update_timer, ns_to_ktime(16666666), HRTIMER_MODE_REL);

	return 0;
}

static int _build_path_decouple(void)
{
	return 0;
}

static int _build_path_single_layer(void)
{
	return 0;
}

static int _build_path_debug_rdma1_dsi0(void)
{
	int ret = 0;

	DISP_MODULE_ENUM dst_module = 0;

	pgc->mode = DEBUG_RDMA1_DSI0_MODE;

	pgc->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_SUB_RDMA1_DISP, pgc->cmdq_handle_config);
	if (pgc->dpmgr_handle) {
		DISPDBG("dpmgr create path SUCCESS(%p)\n", pgc->dpmgr_handle);
	} else {
		DISPERR("dpmgr create path FAIL\n");
		return -1;
	}

	dst_module = _get_dst_module_by_lcm(pgc->plcm);
	dpmgr_path_set_dst_module(pgc->dpmgr_handle, dst_module);

	if (disp_helper_get_option(DISP_OPT_USE_M4U)) {
		M4U_PORT_STRUCT sPort;
		sPort.ePortID = M4U_PORT_DISP_RDMA1;
		sPort.Virtuality = disp_helper_get_option(DISP_OPT_USE_M4U);
		sPort.Security = 0;
		sPort.Distance = 1;
		sPort.Direction = 0;
		ret = m4u_config_port(&sPort);
		if (ret == 0) {
			DISPDBG("config M4U Port %s to %s SUCCESS\n",
				  ddp_get_module_name(DISP_MODULE_RDMA1),
				  disp_helper_get_option(DISP_OPT_USE_M4U) ? "virtual" :
				  "physical");
		} else {
			DISPERR("config M4U Port %s to %s FAIL(ret=%d)\n",
				  ddp_get_module_name(DISP_MODULE_RDMA1),
				  disp_helper_get_option(DISP_OPT_USE_M4U) ? "virtual" :
				  "physical", ret);
			return -1;
		}
	}

	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);

	return ret;
}

static void _cmdq_build_trigger_loop(void)
{

	int ret = 0;
	if (pgc->cmdq_handle_trigger == NULL) {
		cmdqRecCreate(CMDQ_SCENARIO_TRIGGER_LOOP, &(pgc->cmdq_handle_trigger));
		DISPMSG("primary path trigger thread cmd handle=%p\n", pgc->cmdq_handle_trigger);
	}
	cmdqRecReset(pgc->cmdq_handle_trigger);

	if (primary_display_is_video_mode()) {

		ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), pgc->cmdq_handle_trigger, 0);

		cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);
		cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_EVENT_MUTEX0_STREAM_EOF);
		cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);
		cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_MUTEX0_STREAM_EOF);

		/* wait and clear rdma0_sof for vfp change */
		cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_SOF);

		/* for some module(like COLOR) to read hw register to GPR after frame done */
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_AFTER_STREAM_EOF, 0);
	} else {
		/* DSI command mode doesn't have mutex_stream_eof, need use CMDQ token instead */
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);

		if (need_wait_esd_eof()) {
			/* Wait esd config thread done. */
			ret = cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_ESD_EOF);
		}
		/* ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_MDP_DSI0_TE_SOF); */
		/* for operations before frame transfer, such as waiting for DSI TE */
#ifndef CONFIG_MTK_FPGA		/* fpga has no TE signal */
		if (islcmconnected)
			dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_WAIT_LCM_TE, 0);
#endif
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CABC_EOF);
		/* cleat frame done token, now the config thread will not allowed to config registers. */
		/* remember that config thread's priority is higher than trigger thread,
		 * so all the config queued before will be applied then STREAM_EOF token be cleared */
		/* this is what CMDQ did as "Merge" */
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_STREAM_EOF);

		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);

		/* clear rdma EOF token before wait */
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);

		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_BEFORE_STREAM_SOF, 0);


		if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT))
			exit_pd_by_cmdq(pgc->cmdq_handle_trigger);

		/* enable mutex, only cmd mode need this */
		/* this is what CMDQ did as "Trigger" */
		dpmgr_path_trigger(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_ENABLE);

		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_AFTER_STREAM_SOF, 1);

		/* waiting for frame done, because we can't use mutex stream eof here,
		 * so need to let dpmanager help to decide which event to wait */
		/* most time we wait rdmax frame done event. */
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_WAIT_STREAM_EOF_EVENT, 0);

		/* dsi is not idle rightly after rdma frame done,
		 * so we need to polling about 1us for dsi returns to idle */
		/* do not polling dsi idle directly which will decrease CMDQ performance */
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_CHECK_IDLE_AFTER_STREAM_EOF, 0);

		/* for some module(like COLOR) to read hw register to GPR after frame done */
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger,
				      CMDQ_AFTER_STREAM_EOF, 0);

		if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT))
			enter_pd_by_cmdq(pgc->cmdq_handle_trigger);

		/* polling DSI idle */
		/* ret = cmdqRecPoll(pgc->cmdq_handle_trigger, 0x1401b00c, 0, 0x80000000); */
		/* polling wdma frame done */
		/* ret = cmdqRecPoll(pgc->cmdq_handle_trigger, 0x140060A0, 1, 0x1); */

		/* now frame done, config thread is allowed to config register now */
		ret = cmdqRecSetEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_STREAM_EOF);
		ret = cmdqRecSetEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CABC_EOF);
		/* RUN forever!!!! */
		BUG_ON(ret < 0);
	}

	/* dump trigger loop instructions to check whether dpmgr_path_build_cmdq works correctly */
	DISPCHECK("primary display BUILD cmdq trigger loop finished\n");

	return;

}

void disp_spm_enter_cg_mode(void)
{
	MMProfileLogEx(ddp_mmp_get_events()->cg_mode, MMProfileFlagPulse, 0, 0);
}

void disp_spm_enter_power_down_mode(void)
{
	MMProfileLogEx(ddp_mmp_get_events()->power_down_mode, MMProfileFlagPulse, 0, 0);
}

static void _cmdq_build_monitor_loop(void)
{
	int ret = 0;
	cmdqRecHandle g_cmdq_handle_monitor;
	cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &(g_cmdq_handle_monitor));
	DISPMSG("primary path monitor thread cmd handle=%p\n", g_cmdq_handle_monitor);

	cmdqRecReset(g_cmdq_handle_monitor);

	/* wait and clear stream_done, HW will assert mutex enable automatically in frame done reset. */
	/* todo: should let dpmanager to decide wait which mutex's eof. */
	ret = cmdqRecWait(g_cmdq_handle_monitor, CMDQ_EVENT_DISP_RDMA0_UNDERRUN);

	cmdqRecReadToDataRegister(g_cmdq_handle_monitor, 0x10006b0c,
				  CMDQ_DATA_REG_2D_SHARPNESS_1_DST);
	cmdqRecWriteFromDataRegister(g_cmdq_handle_monitor, CMDQ_DATA_REG_2D_SHARPNESS_1_DST,
				     0x1401b280);

	cmdqRecReadToDataRegister(g_cmdq_handle_monitor, 0x10006b08,
				  CMDQ_DATA_REG_2D_SHARPNESS_1_DST);
	cmdqRecWriteFromDataRegister(g_cmdq_handle_monitor, CMDQ_DATA_REG_2D_SHARPNESS_1_DST,
				     0x1401b284);

	cmdqRecReadToDataRegister(g_cmdq_handle_monitor, 0x10006b04,
				  CMDQ_DATA_REG_2D_SHARPNESS_1_DST);
	cmdqRecWriteFromDataRegister(g_cmdq_handle_monitor, CMDQ_DATA_REG_2D_SHARPNESS_1_DST,
				     0x1401b288);

	cmdqRecReadToDataRegister(g_cmdq_handle_monitor, 0x1401b16c,
				  CMDQ_DATA_REG_2D_SHARPNESS_1_DST);
	cmdqRecWriteFromDataRegister(g_cmdq_handle_monitor, CMDQ_DATA_REG_2D_SHARPNESS_1_DST,
				     0x1401b28C);

	ret = cmdqRecStartLoop(g_cmdq_handle_monitor);

	return;
}

void _cmdq_start_trigger_loop(void)
{
	int ret = 0;
	/*cmdqRecDumpCommand(pgc->cmdq_handle_trigger);*/
	/* this should be called only once because trigger loop will nevet stop */
	ret = cmdqRecStartLoop(pgc->cmdq_handle_trigger);
	if (!primary_display_is_video_mode()) {
		if (need_wait_esd_eof()) {
			/* Need set esd check eof synctoken to let trigger loop go. */
			cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_ESD_EOF);
		}
		/* need to set STREAM_EOF for the first time, otherwise we will stuck in dead loop */
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF);
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CABC_EOF);
		cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);
		dprec_event_op(DPREC_EVENT_CMDQ_SET_EVENT_ALLOW);
	}

	DISPCHECK("primary display START cmdq trigger loop finished\n");

}

void _cmdq_stop_trigger_loop(void)
{
	int ret = 0;
	/* this should be called only once because trigger loop will nevet stop */
	ret = cmdqRecStopLoop(pgc->cmdq_handle_trigger);
	DISPCHECK("primary display STOP cmdq trigger loop finished\n");
}

static void _cmdq_set_config_handle_dirty(void)
{
	if (!primary_display_is_video_mode()) {
		dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
		/* only command mode need to set dirty */
		cmdqRecSetEventToken(pgc->cmdq_handle_config, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		dprec_event_op(DPREC_EVENT_CMDQ_SET_DIRTY);
		dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
	}
}

static void _cmdq_handle_clear_dirty(cmdqRecHandle cmdq_handle)
{
	if (!primary_display_is_video_mode()) {
		dprec_logger_trigger(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 1);
		cmdqRecClearEventToken(cmdq_handle, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
	}
}

static void _cmdq_set_config_handle_dirty_mira(void *handle)
{
	if (!primary_display_is_video_mode()) {
		dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
		/* only command mode need to set dirty */
		cmdqRecSetEventToken(handle, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		dprec_event_op(DPREC_EVENT_CMDQ_SET_DIRTY);
		dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY, 0, 0);
	}
}

static void _cmdq_reset_config_handle(void)
{
	cmdqRecReset(pgc->cmdq_handle_config);
	dprec_event_op(DPREC_EVENT_CMDQ_RESET);
}

static void _cmdq_flush_config_handle(int blocking, CmdqAsyncFlushCB callback, unsigned int userdata)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, blocking,
			   (unsigned int)(unsigned long)callback);
	if (blocking) {
		cmdqRecFlush(pgc->cmdq_handle_config);
	} else {
		if (callback)
			cmdqRecFlushAsyncCallback(pgc->cmdq_handle_config,
						  callback, userdata);
		else
			cmdqRecFlushAsync(pgc->cmdq_handle_config);
	}

	dprec_event_op(DPREC_EVENT_CMDQ_FLUSH);
	dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, userdata, 0);

	if (dprec_option_enabled())
		cmdqRecDumpCommand(pgc->cmdq_handle_config);

}

static void _cmdq_flush_config_handle_mira(void *handle, int blocking)
{
	dprec_logger_start(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, 0, 0);
	if (blocking)
		cmdqRecFlush(handle);
	else
		cmdqRecFlushAsync(handle);

	dprec_event_op(DPREC_EVENT_CMDQ_FLUSH);
	dprec_logger_done(DPREC_LOGGER_PRIMARY_CMDQ_FLUSH, 0, 0);
}

void _cmdq_insert_wait_primary_path_frame_done(void *handle)
{
	if (primary_display_is_video_mode())
		cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	else
		cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
}

void _cmdq_insert_wait_frame_done_token_mira(void *handle)
{
	if (primary_display_is_video_mode()) {
		cmdqRecWaitNoClear(handle, CMDQ_EVENT_DISP_RDMA0_EOF);
		cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
		ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), handle, 0);

	} else {
		cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
	}

	dprec_event_op(DPREC_EVENT_CMDQ_WAIT_STREAM_EOF);
}

static void update_frm_seq_info(unsigned int addr, unsigned int addr_offset, unsigned int seq,
				DISP_FRM_SEQ_STATE state)
{
	int i = 0;

	if (FRM_CONFIG == state) {
		frm_update_sequence[frm_update_cnt].state = state;
		frm_update_sequence[frm_update_cnt].mva = addr;
		frm_update_sequence[frm_update_cnt].max_offset = addr_offset;
		if (seq > 0)
			frm_update_sequence[frm_update_cnt].seq = seq;
		MMProfileLogEx(ddp_mmp_get_events()->primary_seq_config, MMProfileFlagPulse, addr,
			       seq);

	} else if (FRM_TRIGGER == state) {
		frm_update_sequence[frm_update_cnt].state = FRM_TRIGGER;
		MMProfileLogEx(ddp_mmp_get_events()->primary_seq_trigger, MMProfileFlagPulse,
			       frm_update_cnt, frm_update_sequence[frm_update_cnt].seq);

		dprec_logger_frame_seq_begin(pgc->session_id,
					     frm_update_sequence[frm_update_cnt].seq);

		frm_update_cnt++;
		frm_update_cnt %= FRM_UPDATE_SEQ_CACHE_NUM;


	} else if (FRM_START == state) {
		for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
			if ((abs(addr - frm_update_sequence[i].mva) <= frm_update_sequence[i].max_offset)
			    && (frm_update_sequence[i].state == FRM_TRIGGER)) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_seq_rdma_irq,
					       MMProfileFlagPulse, frm_update_sequence[i].mva,
					       frm_update_sequence[i].seq);
				frm_update_sequence[i].state = FRM_START;
				dprec_logger_frame_seq_end(pgc->session_id,
							   frm_update_sequence[i].seq);
				dprec_logger_frame_seq_begin(0, frm_update_sequence[i].seq);
				/* /break; */
			}
		}
	} else if (FRM_END == state) {
		for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
			if (FRM_START == frm_update_sequence[i].state) {
				frm_update_sequence[i].state = FRM_END;
				dprec_logger_frame_seq_end(0, frm_update_sequence[i].seq);
				MMProfileLogEx(ddp_mmp_get_events()->primary_seq_release,
					       MMProfileFlagPulse, frm_update_sequence[i].mva,
					       frm_update_sequence[i].seq);

			}
		}
	}

}

static int _config_wdma_output(WDMA_CONFIG_STRUCT *wdma_config,
			       disp_path_handle disp_handle, cmdqRecHandle cmdq_handle)
{
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->wdma_config = *wdma_config;
	pconfig->wdma_dirty = 1;
	dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
	return 0;
}

static int _config_rdma_input_data(RDMA_CONFIG_STRUCT *rdma_config,
				   disp_path_handle disp_handle, cmdqRecHandle cmdq_handle)
{
	disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->rdma_config = *rdma_config;
	pconfig->rdma_dirty = 1;
	dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
	return 0;
}

static void directlink_path_add_memory(WDMA_CONFIG_STRUCT *p_wdma, DISP_MODULE_ENUM after_engine)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle = NULL;
	cmdqRecHandle cmdq_wait_handle = NULL;
	disp_ddp_path_config *pconfig = NULL;

	/*create config thread */
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
	if (ret != 0) {
		DISPERR("dl_to_dc capture:Fail to create cmdq handle\n");
		ret = -1;
		goto out;
	}
	cmdqRecReset(cmdq_handle);

	/*create wait thread */
	ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);
	if (ret != 0) {
		DISPERR("dl_to_dc capture:Fail to create cmdq wait handle\n");
		ret = -1;
		goto out;
	}
	cmdqRecReset(cmdq_wait_handle);

	/*configure  config thread */
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	dpmgr_path_add_memout(pgc->dpmgr_handle, after_engine, cmdq_handle);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->wdma_config = *p_wdma;

	if (disp_helper_get_option(DISP_OPT_DECOUPLE_MODE_USE_RGB565)) {
		pconfig->wdma_config.outputFormat = UFMT_RGB565;
		pconfig->wdma_config.dstPitch = pconfig->wdma_config.srcWidth * 2;
	}

	pconfig->wdma_dirty = 1;
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);

	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);
	DISPDBG("dl_to_dc capture:Flush add memout mva(0x%lx)\n", p_wdma->dstAddress);

	/*wait wdma0 sof */
	cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	cmdqRecFlush(cmdq_wait_handle);
	DISPMSG("dl_to_dc capture:Flush wait wdma sof\n");
out:
	cmdqRecDestroy(cmdq_handle);
	cmdqRecDestroy(cmdq_wait_handle);
}


void disp_enable_emi_force_on(unsigned int enable, void *cmdq_handle)
{
}

static int _DL_switch_to_DC_fast(void)
{
	int ret = 0;
	DDP_SCENARIO_ENUM old_scenario, new_scenario;
	RDMA_CONFIG_STRUCT rdma_config = decouple_rdma_config;
	WDMA_CONFIG_STRUCT wdma_config = decouple_wdma_config;

	disp_ddp_path_config *data_config_dl = NULL;
	disp_ddp_path_config *data_config_dc = NULL;
	unsigned int mva = pgc->dc_buf[pgc->dc_buf_id];	/* mva for 1. ovl->wdma and 2.Rdma->dsi , */
	struct ddp_io_golden_setting_arg gset_arg;

	wdma_config.dstAddress = mva;
	wdma_config.security = DISP_NORMAL_BUFFER;

	/* 1.save a temp frame to intermediate buffer */
	directlink_path_add_memory(&wdma_config, DISP_MODULE_OVL0);

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 1, 0);

	/* 2.reset primary handle */
	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	/* 3.modify interface path handle to new scenario(rdma->dsi) */
	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP;

	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);

	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, pgc->cmdq_handle_config,
			  primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);


	/* 4.config rdma from directlink mode to memory mode */
	rdma_config.address = mva;
	rdma_config.security = DISP_NORMAL_BUFFER;

	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	data_config_dl->rdma_config = rdma_config;
	data_config_dl->rdma_config.dst_x = 0;
	data_config_dl->rdma_config.dst_y = 0;
	data_config_dl->rdma_config.dst_h = data_config_dl->dst_h;
	data_config_dl->rdma_config.dst_w = data_config_dl->dst_w;
	data_config_dl->rdma_dirty = 1;

	/* no need ioctl because of rdma_dirty */
	set_is_dc(1);

	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config_dl, pgc->cmdq_handle_config);

	screen_logger_add_message("sess_mode", MESSAGE_REPLACE, (char *)p_session_mode_spy(DISP_SESSION_DECOUPLE_MODE));
	dynamic_debug_msg_print(mva, rdma_config.width, rdma_config.height, rdma_config.pitch,
			UFMT_GET_Bpp(rdma_config.inputFormat));

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 1;
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	/* screen_logger_add_message("sess_mode", MESSAGE_REPLACE, p_session_mode_spy(DISP_SESSION_DECOUPLE_MODE)); */
	/* dynamic_debug_msg_print(mva, rdma_config.width, rdma_config.height, rdma_config.pitch, */
	/* ovl_get_Bpp_from_dpColorFmt(rdma_config.inputFormat)); */

	/* 5. backup rdma address to slots */
	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 0, rdma_config.address);
	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 1, rdma_config.pitch);
	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 2, rdma_config.inputFormat);


	/* 6 .flush to cmdq */
	_cmdq_set_config_handle_dirty();
	_cmdq_flush_config_handle(1, NULL, 0);

	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 2, 0);

	/* ddp_mmp_rdma_layer(&rdma_config, 0,  20, 20); */

	/* 7.reset  cmdq */
	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	/* 9. create ovl2mem path handle */
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle =
	    dpmgr_create_path(DDP_SCENARIO_PRIMARY_OVL_MEMOUT, pgc->cmdq_handle_ovl1to2_config);

	if (pgc->ovl2mem_path_handle) {
		DISPDBG("dpmgr create ovl memout path SUCCESS(%p)\n", pgc->ovl2mem_path_handle);
	} else {
		DISPERR("dpmgr create path FAIL\n");
		return -1;
	}

	dpmgr_path_set_video_mode(pgc->ovl2mem_path_handle, 0);
	dpmgr_path_init(pgc->ovl2mem_path_handle, CMDQ_ENABLE);

	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
	data_config_dc->dst_w = rdma_config.width;
	data_config_dc->dst_h = rdma_config.height;
	data_config_dc->dst_dirty = 1;

	/* move ovl config info from dl to dc */
	memcpy(data_config_dc->ovl_config, data_config_dl->ovl_config,
	       sizeof(data_config_dl->ovl_config));

	ret = dpmgr_path_config(pgc->ovl2mem_path_handle, data_config_dc,
			      pgc->cmdq_handle_ovl1to2_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->ovl2mem_path_handle);
	gset_arg.is_decouple_mode = 1;
	dpmgr_path_ioctl(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	ret = dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_ENABLE);

	/* cmdqRecDumpCommand(pgc->cmdq_handle_ovl1to2_config); */
	/*cmdqRecClearEventToken(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_EOF);*/
	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 0);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	cmdqRecWait(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_EOF);
	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 3, 0);

	/* 11..enable event for new path */
/* dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_COMPLETE); */
/* dpmgr_map_event_to_irq(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START, DDP_IRQ_WDMA0_FRAME_COMPLETE); */
/* dpmgr_enable_event(pgc->ovl2mem_path_handle, DISP_PATH_EVENT_FRAME_START); */

	if (primary_display_is_video_mode()) {
		if (_need_lfr_check()) {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_DSI0_FRAME_DONE);
		} else {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_RDMA0_DONE);
		}
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	/* dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE); */
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 4, 0);

	return ret;
}

static int DL_switch_to_DC_fast(int sw_only)
{
	int ret = 0;

	if (!sw_only)
		ret = _DL_switch_to_DC_fast();
	else
		ret = -1;	/*not support yet */

	return ret;
}

static int modify_path_power_off_callback(unsigned long userdata)
{
	DDP_SCENARIO_ENUM old_scenario, new_scenario;
	int layer;
	old_scenario = userdata >> 16;
	new_scenario = userdata & ((1 << 16) - 1);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	/* release output buffer */
	layer = disp_sync_get_output_interface_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);
	return 0;
}

static int _DC_switch_to_DL_fast(void)
{
	int ret = 0;
	int layer = 0;
	disp_ddp_path_config *data_config_dl = NULL;
	disp_ddp_path_config *data_config_dc = NULL;
	DDP_SCENARIO_ENUM old_scenario, new_scenario;
	struct ddp_io_golden_setting_arg gset_arg;

	/* 3.destroy ovl->mem path. */
	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	/*copy ovl config from DC handle to DL handle */;
	memcpy(data_config_dl->ovl_config, data_config_dc->ovl_config, sizeof(data_config_dl->ovl_config));

	dpmgr_path_deinit(pgc->ovl2mem_path_handle,
			  (int)(unsigned long)pgc->cmdq_handle_ovl1to2_config);
	dpmgr_destroy_path(pgc->ovl2mem_path_handle,
			   pgc->cmdq_handle_ovl1to2_config);
	/*clear sof token for next dl to dc */
	cmdqRecClearEventToken(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_SOF);

	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 1);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle = NULL;

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 1, 1);

	/* release output buffer */
	layer = disp_sync_get_output_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	/* 4.modify interface path handle to new scenario(rdma->dsi) */
	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_DISP;

	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);

	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, pgc->cmdq_handle_config,
			  primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);

	/* 5.config rdma from memory mode to directlink mode */
	data_config_dl->rdma_config = decouple_rdma_config;
	data_config_dl->rdma_config.address = 0;
	data_config_dl->rdma_config.pitch = 0;
	data_config_dl->rdma_config.security = DISP_NORMAL_BUFFER;
	data_config_dl->rdma_dirty = 1;
	data_config_dl->dst_dirty = 1;

	/* no need ioctl because of rdma_dirty */
	set_is_dc(0);

	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config_dl, pgc->cmdq_handle_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 0;
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 0, 0);

	/* cmdqRecDumpCommand(pgc->cmdq_handle_config); */
	_cmdq_set_config_handle_dirty();
	/*if blocking flush won't cause UX issue, we should simplify this code: remove callback *
	 *else we should move disable_sodi to callback, and change to nonblocking flush */
	if (disp_helper_get_option(DISP_OPT_GMO_OPTIMIZE)) {
		_cmdq_flush_config_handle(1, modify_path_power_off_callback, (old_scenario << 16) | new_scenario);
		modify_path_power_off_callback((old_scenario << 16) | new_scenario);
		pd_release_dc_buffer();
	} else {
		_cmdq_flush_config_handle(0, modify_path_power_off_callback, (old_scenario << 16) | new_scenario);
		/* modify_path_power_off_callback((old_scenario << 16) | new_scenario); */
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 2, 1);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	/* 9.enable event for new path */
	if (primary_display_is_video_mode()) {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 3, 1);

	return ret;
}

static int _DC_switch_to_DL_sw_only(void)
{
	int ret = 0;
	int layer = 0;
	disp_ddp_path_config *data_config_dc = NULL;
	disp_ddp_path_config *data_config_dl = NULL;
	DDP_SCENARIO_ENUM old_scenario, new_scenario;

	/* 3.destroy ovl->mem path. */
	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	dpmgr_destroy_path_handle(pgc->ovl2mem_path_handle);

	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle = NULL;

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 1, 1);

	/* release output buffer */
	layer = disp_sync_get_output_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	/* 4.modify interface path handle to new scenario(rdma->dsi) */
	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_DISP;
	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 1);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, pgc->cmdq_handle_config,
			  primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 1);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 1);

	/* 5.config rdma from memory mode to directlink mode */
	data_config_dl->rdma_config = decouple_rdma_config;
	data_config_dl->rdma_config.address = 0;
	data_config_dl->rdma_config.pitch = 0;
	data_config_dl->rdma_config.security = DISP_NORMAL_BUFFER;
	/* no need ioctl because of rdma_dirty */
	set_is_dc(0);

	if (disp_helper_get_option(DISP_OPT_GMO_OPTIMIZE))
		pd_release_dc_buffer();

	/* release output buffer */
	layer = disp_sync_get_output_interface_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	/* 9.enable event for new path */
	if (primary_display_is_video_mode()) {
		if (_need_lfr_check()) {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_DSI0_FRAME_DONE);
		} else {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_RDMA0_DONE);
		}
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	if (!primary_display_is_video_mode())
		_cmdq_build_trigger_loop();

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, 3, 1);

	return ret;
}

static int DC_switch_to_DL_fast(int sw_only)
{
	int ret = 0;

	if (!sw_only)
		ret = _DC_switch_to_DL_fast();
	else
		ret = _DC_switch_to_DL_sw_only();

	return ret;
}

static int DL_switch_to_rdma_mode(cmdqRecHandle handle, int block)
{
	int ret;
	DDP_SCENARIO_ENUM old_scenario, new_scenario;
	int need_flush = 0;
	struct ddp_io_golden_setting_arg gset_arg;

	if (!handle) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		if (ret) {
			DISPERR("%s:%d, create cmdq handle fail!ret=%d\n", __func__, __LINE__, ret);
			return -1;
		}
		_cmdq_insert_wait_frame_done_token_mira(handle);
		need_flush = 1;
	}

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP;
	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, handle,
			primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 1;
	dpmgr_path_ioctl(pgc->dpmgr_handle, handle, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	if (need_flush) {
		if (block)
			cmdqRecFlush(handle);
		else
			cmdqRecFlushAsync(handle);
		cmdqRecDestroy(handle);
	}

	return 0;
}

static int rdma_mode_switch_to_DL(cmdqRecHandle handle, int block)
{
	int ret;
	DDP_SCENARIO_ENUM old_scenario, new_scenario;
	int need_flush = 0;
	disp_ddp_path_config *pconfig;
	struct ddp_io_golden_setting_arg gset_arg;

	if (!handle) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		if (ret) {
			DISPERR("%s:%d, create cmdq handle fail!ret=%d\n", __func__, __LINE__, ret);
			return -1;
		}
		_cmdq_insert_wait_frame_done_token_mira(handle);
		need_flush = 1;
	}

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_DISP;
	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, handle,
			primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	/* set rdma to DL mode */
	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->rdma_config.address = 0;
	pconfig->rdma_config.pitch = 0;
	pconfig->rdma_config.width = pconfig->dst_w;
	pconfig->rdma_config.height = pconfig->dst_h;
	pconfig->rdma_config.security = DISP_NORMAL_BUFFER;
	pconfig->rdma_dirty = 1;
	pconfig->dst_dirty = 1;
	if (need_flush) {
		/* re-config ovl because ovl is new-coming */
		pconfig->ovl_dirty = 1;
	}

	/* no need ioctl because of rdma_dirty */
	set_is_dc(0);

	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, handle);
	/* clear dirty set by this func */
	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 0;
	dpmgr_path_ioctl(pgc->dpmgr_handle, handle, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	if (need_flush) {
		if (block)
			cmdqRecFlush(handle);
		else
			cmdqRecFlushAsync(handle);
		cmdqRecDestroy(handle);
	}

	return 0;
}

static int config_display_m4u_port(void)
{
	int ret = 0;
	M4U_PORT_STRUCT sPort;
	char *m4u_usage =
	    disp_helper_get_option(DISP_OPT_USE_M4U) ? "virtual" : "physical";

	sPort.ePortID = M4U_PORT_DISP_OVL0;
	sPort.Virtuality = disp_helper_get_option(DISP_OPT_USE_M4U);
	sPort.Security = 0;
	sPort.Distance = 1;
	sPort.Direction = 0;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPERR("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_OVL0), m4u_usage, ret);
		return -1;
	}

	sPort.ePortID = M4U_PORT_DISP_2L_OVL0;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPERR("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_OVL0_2L), m4u_usage, ret);
		return -1;
	}

	sPort.ePortID = M4U_PORT_DISP_2L_OVL1;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPERR("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_OVL0_2L), m4u_usage, ret);
		return -1;
	}

	sPort.ePortID = M4U_PORT_DISP_RDMA0;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPERR("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_RDMA0), m4u_usage, ret);
		return -1;
	}

	sPort.ePortID = M4U_PORT_DISP_WDMA0;
	ret = m4u_config_port(&sPort);
	if (ret) {
		DISPERR("config M4U Port %s to %s FAIL(ret=%d)\n",
			  ddp_get_module_name(DISP_MODULE_WDMA0), m4u_usage, ret);
		return -1;
	}
	return ret;
}

static disp_internal_buffer_info *allocat_decouple_buffer(int size)
{
	void *buffer_va = NULL;
	unsigned int buffer_mva = 0;
	unsigned int mva_size = 0;
	struct ion_mm_data mm_data;
	struct ion_client *client = NULL;
	struct ion_handle *handle = NULL;
	disp_internal_buffer_info *buf_info = NULL;

	memset((void *)&mm_data, 0, sizeof(struct ion_mm_data));
	client = ion_client_create(g_ion_device, "disp_decouple");

	buf_info = kzalloc(sizeof(disp_internal_buffer_info), GFP_KERNEL);
	if (buf_info) {
		handle = ion_alloc(client, size, 0, ION_HEAP_MULTIMEDIA_MASK, 0);
		if (IS_ERR(handle)) {
			DISPERR("Fatal Error, ion_alloc for size %d failed\n", size);
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		buffer_va = ion_map_kernel(client, handle);
		if (buffer_va == NULL) {
			DISPERR("ion_map_kernrl failed\n");
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}
		mm_data.config_buffer_param.kernel_handle = handle;
		mm_data.mm_cmd = ION_MM_CONFIG_BUFFER;
		if (ion_kernel_ioctl(client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data) < 0) {
			DISPERR("ion_test_drv: Config buffer failed.\n");
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		ion_phys(client, handle, (ion_phys_addr_t *) &buffer_mva, (size_t *) &mva_size);
		if (buffer_mva == 0) {
			DISPERR("Fatal Error, get mva failed\n");
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}
		buf_info->handle = handle;
		buf_info->mva = buffer_mva;
		buf_info->size = mva_size;
		buf_info->va = buffer_va;
		buf_info->client = client;
	} else {
		DISPERR("Fatal error, kzalloc internal buffer info failed!!\n");
		kfree(buf_info);
		return NULL;
	}
	return buf_info;
}

static int init_decouple_buffers(void)
{
	int i = 0;
	enum UNIFIED_COLOR_FMT fmt = UFMT_RGB888;
	int height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	int width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	int Bpp = UFMT_GET_Bpp(fmt);
	int buffer_size;

	buffer_size = width * height * Bpp;

	for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++) {	/* INTERNAL Buf 3 frames */
		decouple_buffer_info[i] = allocat_decouple_buffer(buffer_size);
		if (decouple_buffer_info[i] != NULL)
			pgc->dc_buf[i] = decouple_buffer_info[i]->mva;

	}

	/*initialize rdma config */
	decouple_rdma_config.height = height;
	decouple_rdma_config.width = width;
	decouple_rdma_config.idx = 0;
	decouple_rdma_config.inputFormat = fmt;
	decouple_rdma_config.pitch = width * Bpp;
	decouple_rdma_config.security = DISP_NORMAL_BUFFER;
	decouple_rdma_config.dst_x = 0;
	decouple_rdma_config.dst_y = 0;
	decouple_rdma_config.dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	decouple_rdma_config.dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);

	/*initialize wdma config */
	decouple_wdma_config.srcHeight = height;
	decouple_wdma_config.srcWidth = width;
	decouple_wdma_config.clipX = 0;
	decouple_wdma_config.clipY = 0;
	decouple_wdma_config.clipHeight = height;
	decouple_wdma_config.clipWidth = width;
	decouple_wdma_config.outputFormat = fmt;
	decouple_wdma_config.useSpecifiedAlpha = 1;
	decouple_wdma_config.alpha = 0xFF;
	decouple_wdma_config.dstPitch = width * Bpp;
	decouple_wdma_config.security = DISP_NORMAL_BUFFER;

	return 0;

}

static int _allocate_dc_buffer(void)
{
	int ret = 0;
	int i = 0;
	enum UNIFIED_COLOR_FMT fmt = UFMT_RGB888;
	int height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	int width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	int Bpp = UFMT_GET_Bpp(fmt);

	int buffer_size = width * height * Bpp;

	gmo_decouple_buffer_info = allocat_decouple_buffer(buffer_size);
	if (gmo_decouple_buffer_info != NULL)
		for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++)
			pgc->dc_buf[i] = gmo_decouple_buffer_info->mva;
	else {
		DISPERR("_allocate_dc_buffer fail\n");
		return -EFAULT;
	}
	DISPDBG("_allocate_dc_buffer mva 0x%x\n", gmo_decouple_buffer_info->mva);
	/*initialize rdma config */
	decouple_rdma_config.height = height;
	decouple_rdma_config.width = width;
	decouple_rdma_config.idx = 0;
	decouple_rdma_config.inputFormat = fmt;
	decouple_rdma_config.pitch = width * Bpp;
	decouple_rdma_config.security = DISP_NORMAL_BUFFER;
	decouple_rdma_config.dst_x = 0;
	decouple_rdma_config.dst_y = 0;
	decouple_rdma_config.dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	decouple_rdma_config.dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);

	/*initialize wdma config */
	decouple_wdma_config.srcHeight = height;
	decouple_wdma_config.srcWidth = width;
	decouple_wdma_config.clipX = 0;
	decouple_wdma_config.clipY = 0;
	decouple_wdma_config.clipHeight = height;
	decouple_wdma_config.clipWidth = width;
	decouple_wdma_config.outputFormat = fmt;
	decouple_wdma_config.useSpecifiedAlpha = 1;
	decouple_wdma_config.alpha = 0xFF;
	decouple_wdma_config.dstPitch = width * Bpp;
	decouple_wdma_config.security = DISP_NORMAL_BUFFER;
	return ret;
}

static int _release_dc_buffer(void)
{
	int ret = 0;
	int i = 0;

	if (gmo_decouple_buffer_info) {
		ion_free(gmo_decouple_buffer_info->client, gmo_decouple_buffer_info->handle);
		ion_client_destroy(gmo_decouple_buffer_info->client);
		kfree(gmo_decouple_buffer_info);
		gmo_decouple_buffer_info = NULL;
		for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++)
			pgc->dc_buf[i] = 0;
		DISPDBG("_release_dc_buffer\n");
	}

	return ret;
}

int pd_release_dc_buffer(void)
{
	int ret = 0;

	ret = _release_dc_buffer();
	return ret;
}

int pd_allocate_dc_buffer(void)
{
	int ret = 0;

	ret = _allocate_dc_buffer();
	return ret;
}

static int _build_path_direct_link(void)
{
	int ret = 0;


	DISPFUNC();
	pgc->mode = DIRECT_LINK_MODE;

	pgc->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_PRIMARY_DISP, pgc->cmdq_handle_config);
	if (pgc->dpmgr_handle) {
		DISPDBG("dpmgr create path SUCCESS(%p)\n", pgc->dpmgr_handle);
	} else {
		DISPERR("dpmgr create path FAIL\n");
		return -1;
	}
	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	return ret;
}


static int _convert_disp_input_to_ovl(OVL_CONFIG_STRUCT *dst, disp_input_config *src)
{
	int ret = 0;
	int force_disable_alpha = 0;
	enum UNIFIED_COLOR_FMT tmp_fmt;
	unsigned int Bpp = 0;

	if (!src || !dst) {
		disp_aee_print("%s src(0x%p) or dst(0x%p) is null\n", __func__, src, dst);
		return -1;
	}

	dst->layer = src->layer_id;
	dst->isDirty = 1;
	dst->buff_idx = src->next_buff_idx;
	dst->layer_en = src->layer_enable;

	/* if layer is disable, we just needs config above params. */
	if (!src->layer_enable)
		return 0;

	tmp_fmt = disp_fmt_to_unified_fmt(src->src_fmt);
	/* display don't support X channel, like XRGB8888
	 * we need to enable const_bld*/
	ufmt_disable_X_channel(tmp_fmt, &dst->fmt, &dst->const_bld);
#if 0
	if (tmp_fmt != dst->fmt)
		force_disable_alpha = 1;
#endif
	Bpp = UFMT_GET_Bpp(dst->fmt);

	dst->addr = (unsigned long)src->src_phy_addr;
	dst->vaddr = (unsigned long)src->src_base_addr;
	dst->src_x = src->src_offset_x;
	dst->src_y = src->src_offset_y;
	dst->src_w = src->src_width;
	dst->src_h = src->src_height;
	dst->src_pitch = src->src_pitch * Bpp;
	dst->dst_x = src->tgt_offset_x;
	dst->dst_y = src->tgt_offset_y;

	/* dst W/H should <= src W/H */
	dst->dst_w = min(src->src_width, src->tgt_width);
	dst->dst_h = min(src->src_height, src->tgt_height);

	dst->keyEn = src->src_use_color_key;
	dst->key = src->src_color_key;

	dst->aen = force_disable_alpha ? 0 : src->alpha_enable;
	dst->sur_aen = force_disable_alpha ? 0 : src->sur_aen;

	dst->alpha = src->alpha;
	dst->src_alpha = src->src_alpha;
	dst->dst_alpha = src->dst_alpha;

	dst->identity = src->identity;
	dst->connected_type = src->connected_type;
	dst->security = src->security;
	dst->yuv_range = src->yuv_range;

	if (src->buffer_source == DISP_BUFFER_ALPHA) {
		dst->source = OVL_LAYER_SOURCE_RESERVED;	/* dim layer, constant alpha */
	} else if (src->buffer_source == DISP_BUFFER_ION || src->buffer_source == DISP_BUFFER_MVA) {
		dst->source = OVL_LAYER_SOURCE_MEM;	/* from memory */
	} else {
		DISPERR("unknown source = %d", src->buffer_source);
		dst->source = OVL_LAYER_SOURCE_MEM;
	}

	return ret;
}

static int _convert_disp_input_to_rdma(RDMA_CONFIG_STRUCT *dst,
				       disp_session_input_config *session_input)
{
	pr_err("%s not implement yet!!!!!!!!!!!!!!!\n", __func__);
	return -1;
}

int _trigger_display_interface(int blocking, void *callback, unsigned int userdata)
{
	int ret;

	DISPFUNC();

	if (_should_wait_path_idle()) {
		ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		if (ret <= 0)
			primary_display_diagnose();
	}

	if (_should_update_lcm()) {
		int x = disp_helper_get_option(DISP_OPT_FAKE_LCM_X);
		int y = disp_helper_get_option(DISP_OPT_FAKE_LCM_Y);
		int width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
		int height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
		disp_lcm_update(pgc->plcm, x, y, width, height, 0);
	}
	if (_should_start_path())
		dpmgr_path_start(pgc->dpmgr_handle, primary_display_cmdq_enabled());

	if (_should_trigger_path()) {
		if (islcmconnected)
			dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, primary_display_cmdq_enabled());
	}
	if (_should_set_cmdq_dirty())
		_cmdq_set_config_handle_dirty();

	if (_should_flush_cmdq_config_handle())
		_cmdq_flush_config_handle(blocking, callback, userdata);

	if (_should_reset_cmdq_config_handle())
		_cmdq_reset_config_handle();

	/* clear cmdq dirty incase trigger loop starts here */
	if (_should_set_cmdq_dirty())
		_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);

	if (_should_insert_wait_frame_done_token())
		_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	if (_need_lfr_check())
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config, CMDQ_DSI_LFR_MODE, 0);

	return 0;
}

int _trigger_ovl_to_memory(disp_path_handle disp_handle,
				  cmdqRecHandle cmdq_handle,
				  CmdqAsyncFlushCB callback, unsigned int data)
{
	int layer = 0;
	unsigned int rdma_pitch_sec;

	MMProfileLogEx(ddp_mmp_get_events()->ovl_trigger, MMProfileFlagStart, 0, data);

	dpmgr_path_trigger(disp_handle, cmdq_handle, CMDQ_ENABLE);

	cmdqRecWaitNoClear(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);

	layer = disp_sync_get_output_timeline_id();
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer, mem_config.buff_idx);

	layer = disp_sync_get_output_interface_timeline_id();
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer,
				mem_config.interface_idx);

	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 0, (unsigned int)mem_config.addr);

	/*rdma pitch only use bit[15..0], we use bit[31:30] to store secure information */
	rdma_pitch_sec = mem_config.pitch | (mem_config.security << 30);
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 1, rdma_pitch_sec);
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 2, (unsigned int)mem_config.fmt);

	cmdqRecFlushAsyncCallback(cmdq_handle, callback, data);
	cmdqRecReset(cmdq_handle);
	cmdqRecWait(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);
	MMProfileLogEx(ddp_mmp_get_events()->ovl_trigger, MMProfileFlagEnd, 0, data);
	return 0;
}

int _trigger_overlay_engine(void)
{
	/* maybe we need a simple merge mechanism for CPU config. */
	dpmgr_path_trigger(pgc->ovl2mem_path_handle, NULL,
			   disp_helper_get_option(DISP_OPT_USE_CMDQ));
	return 0;
}


static unsigned int _need_lfr_check(void)
{
	unsigned int ret = 0;
#ifdef CONFIG_OF
	if ((pgc->plcm->params->dsi.lfr_enable == 1) && (islcmconnected == 1))
		ret = 1;

#else
	if (pgc->plcm->params->dsi.lfr_enable == 1)
		ret = 1;

#endif
	return ret;
}

static int _disp_primary_path_check_trigger(void *data)
{
	int ret = 0;

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
		cmdqRecHandle handle = NULL;

		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
		while (1) {
			dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
			MMProfileLogEx(ddp_mmp_get_events()->primary_display_aalod_trigger,
				       MMProfileFlagPulse, 0, 0);
			dprec_logger_trigger(DPREC_LOGGER_PQ_TRIGGER_1SECOND, 0, 0);

			_primary_path_lock(__func__);

			if (pgc->state == DISP_ALIVE) {
				primary_display_idlemgr_kick((char *)__func__, 0);
				cmdqRecReset(handle);
				_cmdq_insert_wait_frame_done_token_mira(handle);
				_cmdq_set_config_handle_dirty_mira(handle);
				_cmdq_flush_config_handle_mira(handle, 0);
			}

			_primary_path_unlock(__func__);

			if (kthread_should_stop())
				break;
		}

		cmdqRecDestroy(handle);
	} else {
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
		while (1) {
			dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
			dprec_logger_trigger(DPREC_LOGGER_PQ_TRIGGER_1SECOND, 0, 0);
			DISPMSG("Force Trigger Display Path\n");
			primary_display_trigger(1, NULL, 0);

			if (kthread_should_stop())
				break;
		}
	}
	return 0;
}


int32_t cmdqDdpClockOn(uint64_t engineFlag)
{
	return 0;
}

int32_t cmdqDdpClockOff(uint64_t engineFlag)
{
	return 0;
}

unsigned int cmdqDdpDumpInfo(uint64_t engineFlag, char *pOutBuf, unsigned int bufSize)
{
	DISPERR("cmdq timeout:%llu\n", engineFlag);
	primary_display_diagnose();

	if (primary_display_is_decouple_mode())
		ddp_dump_analysis(DISP_MODULE_OVL0);

	ddp_dump_analysis(DISP_MODULE_WDMA0);

	/* try to set event by CPU to avoid blocking auto test such as Monkey/MTBF */
	/* cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF); */
	/* cmdqCoreSetEvent(CMDQ_EVENT_DISP_RDMA0_EOF); */

	if (disp_helper_get_option(DISP_OPT_DETECT_RECOVERY)) {
		atomic_set(&primary_recovery_task_wakeup, 1);
		wake_up_interruptible(&primary_recovery_task_wq);
	}

	return 0;
}

uint32_t cmdqDdpResetEng(uint64_t engineFlag)
{
	return 0;
}

static void _RDMA0_INTERNAL_IRQ_Handler(DISP_MODULE_ENUM module, unsigned int param)
{
	if (param & 0x2) {
		/* RDMA Start */
		spm_sodi_mempll_pwr_mode(1);
		MMProfileLogEx(ddp_mmp_get_events()->sodi_disable, MMProfileFlagPulse, 0, 0);
	} else if (param & 0x4) {
		/* RDMA Done */
		spm_sodi_mempll_pwr_mode(0);
		MMProfileLogEx(ddp_mmp_get_events()->sodi_enable, MMProfileFlagPulse, 0, 0);
	}
}

#if 0
void primary_display_sodi_rule_init(void)
{
	/* enable sodi when display driver is ready */
	if (disp_helper_get_option(DISP_HELPER_OPTION_SODI_SUPPORT)) {
		if (primary_display_is_video_mode())
			spm_enable_sodi(1);
		else
			spm_enable_sodi(1);

		return;
	}
}
#endif
int primary_display_change_lcm_resolution(unsigned int width, unsigned int height)
{
	if (pgc->plcm) {
		DISPMSG("LCM Resolution will be changed, original: %dx%d, now: %dx%d\n",
			pgc->plcm->params->width, pgc->plcm->params->height, width, height);
		/* align with 4 is the minimal check, to ensure we can boot up into kernel,
		 * and could modify dfo setting again using meta tool */
		/* otherwise we will have a panic in lk(root cause unknown). */
		if (width > pgc->plcm->params->width || height > pgc->plcm->params->height
		    || width == 0 || height == 0 || width % 4 || height % 4) {
			DISPERR("Invalid resolution: %dx%d\n", width, height);
			return -1;
		}

		if (primary_display_is_video_mode()) {
			DISPERR("Warning!!!Video Mode can't support multiple resolution!\n");
			return -1;
		}

		pgc->plcm->params->width = width;
		pgc->plcm->params->height = height;

		return 0;
	} else {
		return -1;
	}
}

static int _wdma_fence_release_callback(unsigned long userdata)
{
	int fence_idx, layer;
	layer = disp_sync_get_output_timeline_id();

	cmdqBackupReadSlot(pgc->cur_config_fence, layer, &fence_idx);
	mtkfb_release_fence(primary_session_id, layer, fence_idx);
	MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_fence_release, MMProfileFlagPulse, layer,
		       fence_idx);

	return 0;
}

static int _Interface_fence_release_callback(unsigned long userdata)
{
	int layer = disp_sync_get_output_interface_timeline_id();

#ifdef _DEBUG_DITHER_HANG_
	int ret = 0;
	if (primary_display_is_video_mode()) {
		unsigned int status;
		cmdqBackupReadSlot(pgc->dither_status_info, 0, &status);
		if ((status) != 0x10001) {
			/* dither is not idle !! */
			DISPERR("disp dither status error!! stat=0x%x\n", status);
			/* disp_aee_print("dither_stat 0x%x\n", status); */
			MMProfileLogEx(ddp_mmp_get_events()->primary_error, MMProfileFlagPulse, status, 1);
			primary_display_diagnose();
			ret = -1;
		}
	}
#endif

	if (userdata > 0) {
		mtkfb_release_fence(primary_session_id, layer, userdata);
		MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_fence_release, MMProfileFlagPulse,
			       layer, userdata);
	}

	return 0;
}

int _decouple_update_rdma_config_nolock(void)
{
	uint32_t interface_fence = 0;
	int layer = 0;
	int ret = 0;

	if (primary_display_is_decouple_mode()) {
		static cmdqRecHandle cmdq_handle;
		unsigned int rdma_pitch_sec;

		layer = disp_sync_get_output_interface_timeline_id();
		cmdqBackupReadSlot(pgc->cur_config_fence, layer, &interface_fence);

		if (primary_get_state() != DISP_ALIVE) {
			/* don't trigger rdma */
			/* release interface fence */
			_Interface_fence_release_callback(interface_fence > 1 ? interface_fence - 1 : 0);

			return -1;
		}

		if (cmdq_handle == NULL)
			ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret == 0) {
			RDMA_CONFIG_STRUCT tmpConfig = decouple_rdma_config;
			cmdqRecReset(cmdq_handle);
			_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
			cmdqBackupReadSlot(pgc->rdma_buff_info, 0, (uint32_t *)(&(tmpConfig.address)));

			/*rdma pitch only use bit[15..0], we use bit[31:30] to store secure information*/
			cmdqBackupReadSlot(pgc->rdma_buff_info, 1, &(rdma_pitch_sec));
			tmpConfig.pitch = rdma_pitch_sec & ~(3<<30);
			tmpConfig.security = rdma_pitch_sec >> 30;

			cmdqBackupReadSlot(pgc->rdma_buff_info, 2, &(tmpConfig.inputFormat));

			tmpConfig.height = primary_display_get_height();
			tmpConfig.width = primary_display_get_width();
			tmpConfig.yuv_range = DISP_YUV_BT601;
#ifdef _DEBUG_DITHER_HANG_
			if (primary_display_is_video_mode()) {
				cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->dither_status_info,
							    0, disp_addr_convert(DISP_REG_DITHER_OUT_CNT));
			}
#endif
			_config_rdma_input_data(&tmpConfig, pgc->dpmgr_handle, cmdq_handle);
			_cmdq_set_config_handle_dirty_mira(cmdq_handle);

			cmdqRecFlushAsyncCallback(cmdq_handle, _Interface_fence_release_callback,
					interface_fence > 1 ? interface_fence - 1 : 0);

			dprec_mmp_dump_rdma_layer(&tmpConfig, 0);
			MMProfileLogEx(ddp_mmp_get_events()->primary_rdma_config, MMProfileFlagPulse,
					interface_fence, tmpConfig.address);
		} else {
			DISPERR("fail to create cmdq\n");
		}
	}
	return 0;
}


static int decouple_update_rdma_config(void)
{
	int ret;
	_primary_path_lock(__func__);
	ret = _decouple_update_rdma_config_nolock();
	_primary_path_unlock(__func__);
	return ret;
}

static int _requestCondition(int overlap_layers)
{
	int ret = 0;
	int iwidth = 0;
	int ihight = 0;

	ihight = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	iwidth = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);

	if (ihight * iwidth < DISP_HW_HRT_PERF_LCM_AREA_THRESHOLD) {
		if (overlap_layers <= DISP_HW_HRT_PERF_FOR_LCM_SMALL)
			return -1;
	} else {
		if (overlap_layers > DISP_HW_HRT_PERF_FOR_LCM_BIG_MAX)
			DISPWARN("overlayed layer num is %d > %d\n", overlap_layers,
			DISP_HW_HRT_PERF_FOR_LCM_BIG_MAX);
		if (overlap_layers <= DISP_HW_HRT_PERF_FOR_LCM_BIG)
			return -1;
	}

	return ret;
}

static int _request_dvfs_perf(int req)
{
	if (is_vcorefs_can_work() != 1)
		return 0;

	if (atomic_read(&dvfs_ovl_req_status) != req) {
		vcorefs_request_dvfs_opp(KIR_OVL, req);
		atomic_set(&dvfs_ovl_req_status, req);
	}

	return 0;
}

static int _ovl_fence_release_callback(unsigned long userdata)
{
	int i = 0;
	unsigned int addr = 0;
	int ret = 0;
	int real_overlap_layers = 0;
	MMProfileLogEx(ddp_mmp_get_events()->session_release, MMProfileFlagStart, 1, userdata);

	/* check overlap layer */
	cmdqBackupReadSlot(pgc->subtractor_when_free, i, &real_overlap_layers);
	real_overlap_layers >>= 16;

	_primary_path_lock(__func__);

	/*if (real_overlap_layers > DISP_HW_HRT_LYAERS_FOR_LOW_POWER) {*/
	if (!_requestCondition(real_overlap_layers)) {
		_request_dvfs_perf(OPPI_PERF);
	} else {
		/* be carefull for race condition !! because callback may delay */
		/* so we need to check last request when ovl_config */
		if (dvfs_last_ovl_req == OPPI_UNREQ)
			_request_dvfs_perf(OPPI_UNREQ);
	}
	_primary_path_unlock(__func__);

	/*check last ovl status: should be idle when config */
	if (primary_display_is_video_mode() && !primary_display_is_decouple_mode()) {
		unsigned int status;
		cmdqBackupReadSlot(pgc->ovl_status_info, 0, &status);
#ifdef DEBUG_OVL_CONFIG_TIME
		unsigned int time_event = 0;
		unsigned int time_event1 = 0;
		unsigned int time_event2 = 0;

		cmdqBackupReadSlot(pgc->ovl_config_time, 0, &time_event);
		cmdqBackupReadSlot(pgc->ovl_config_time, 1, &time_event1);
		cmdqBackupReadSlot(pgc->ovl_config_time, 2, &time_event2);
		DISPMSG("ovl config time_event %d time_event1 %d time_event2 %d time1_diff  %d  time2_diff %d\n",
			time_event, time_event1, time_event2, time_event1 - time_event, time_event2 - time_event1);
#endif
		if ((status & 0x1) != 0) {
			/* ovl is not idle !! */
			DISPERR("disp ovl status error!! stat=0x%x\n", status);
			/* disp_aee_print("ovl_stat 0x%x\n", status); */
			MMProfileLogEx(ddp_mmp_get_events()->primary_error, MMProfileFlagPulse, status, 0);
			primary_display_diagnose();
			ret = -1;
		}

        #ifdef VENDOR_EDIT
        /* pdl@oppo.com,2016/05/05 add for debug system hang */
		unsigned int status_ovl0;
		unsigned int status_ovl0_2;
		unsigned int status_ovl1_2;
		cmdqBackupReadSlot(pgc->ovl_status_info_debug, 0, &status_ovl0);
		cmdqBackupReadSlot(pgc->ovl_status_info_debug, 1, &status_ovl0_2);
		cmdqBackupReadSlot(pgc->ovl_status_info_debug, 2, &status_ovl1_2);

		if (((status_ovl0 & 0x1) != 0) ||((status_ovl0_2 & 0x1) != 0)||((status_ovl1_2 & 0x1) != 0)){
			printk("------------fencecallbackovl0_status_info_debug  0x%x-----------\n", status_ovl0);
			printk("------------fencecallbackovl0_2_status_info_debug  0x%x-----------\n", status_ovl0_2);
			printk("------------fencecallbackovl1_2_status_info_debug  0x%x-----------\n", status_ovl1_2);
			primary_display_diagnose();
		}
		#endif
	}

	for (i = 0; i < PRIMARY_SESSION_INPUT_LAYER_COUNT; i++) {
		int fence_idx = 0;
		int subtractor = 0;
		if (i == primary_display_get_option("ASSERT_LAYER") && is_DAL_Enabled()) {
			mtkfb_release_layer_fence(primary_session_id, i);
		} else {
			cmdqBackupReadSlot(pgc->cur_config_fence, i, &fence_idx);
			cmdqBackupReadSlot(pgc->subtractor_when_free, i, &subtractor);
			subtractor &= 0xFFFF;
			mtkfb_release_fence(primary_session_id, i, fence_idx - subtractor);
		}
		MMProfileLogEx(ddp_mmp_get_events()->primary_ovl_fence_release, MMProfileFlagPulse,
			       i, fence_idx - subtractor);
	}

	addr = ddp_ovl_get_cur_addr(!_should_config_ovl_input(), 0);
	if ((primary_display_is_decouple_mode() == 0))
		update_frm_seq_info(addr, 0, 2, FRM_START);

	MMProfileLogEx(ddp_mmp_get_events()->session_release, MMProfileFlagEnd, 1, userdata);
	return ret;
}

/* #define UPDATE_RDMA_CONFIG_USING_CMDQ_CALLBACK */
static int _ovl_wdma_fence_release_callback(unsigned long userdata)
{
	int ret = 0;

	ret = _ovl_fence_release_callback(userdata);
	ret |= _wdma_fence_release_callback(userdata);

#ifdef UPDATE_RDMA_CONFIG_USING_CMDQ_CALLBACK
	ret |= decouple_update_rdma_config();
#endif

	return ret;
}


#ifdef UPDATE_RDMA_CONFIG_USING_CMDQ_CALLBACK
static int decouple_mirror_update_rdma_config_thread(void *data)
{
	return 0;
}
#else
static void decouple_mirror_irq_callback(DISP_MODULE_ENUM module, unsigned int reg_value)
{
	#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT) && defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	/* In TEE, we have to protect WDMA registers, so we can't enable WDMA interrupt */
	/* here we use ovl frame done interrupt instead */
	if ((module == DISP_MODULE_OVL0) && (primary_display_is_decouple_mode())) {
		if (reg_value & 0x2) {/*OVL0 frame done*/
			atomic_set(&decouple_update_rdma_event, 1);
			wake_up_interruptible(&decouple_update_rdma_wq);
	    }
	}

#else
	if ((module == DISP_MODULE_WDMA0) && (primary_display_is_decouple_mode())) {
		if (reg_value & 0x1) { /*wdma0 frame done*/
			atomic_set(&decouple_update_rdma_event, 1);
			wake_up_interruptible(&decouple_update_rdma_wq);
	    }
	}
#endif

	return;
}

static int decouple_mirror_update_rdma_config_thread(void *data)
{
	struct sched_param param = {.sched_priority = RTPM_PRIO_SCRN_UPDATE };
	sched_setscheduler(current, SCHED_RR, &param);

	disp_register_module_irq_callback(DISP_MODULE_WDMA0, decouple_mirror_irq_callback);
	disp_register_module_irq_callback(DISP_MODULE_OVL0, decouple_mirror_irq_callback);
	while (1) {
		wait_event_interruptible(decouple_update_rdma_wq,
					 atomic_read(&decouple_update_rdma_event));
		atomic_set(&decouple_update_rdma_event, 0);
		decouple_update_rdma_config();
		if (kthread_should_stop())
			break;
	}

	return 0;
}
#endif

static int primary_display_remove_output(CmdqAsyncFlushCB callback, unsigned int userdata)
{
	int ret = 0;
	static cmdqRecHandle cmdq_handle;
	static cmdqRecHandle cmdq_wait_handle;

	/*create config thread */
	if (cmdq_handle == NULL)
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);

	if (ret == 0) {
		/* capture thread wait wdma sof */
		if (cmdq_wait_handle == NULL)
			ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);

		if (ret == 0) {
			cmdqRecReset(cmdq_wait_handle);
			cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
			cmdqRecFlush(cmdq_wait_handle);
			/* cmdqRecDestroy(cmdq_wait_handle); */
		} else {
			DISPERR("fail to create  wait handle\n");
		}
		cmdqRecReset(cmdq_handle);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

		/* update output fence */
		cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence,
					disp_sync_get_output_timeline_id(), mem_config.buff_idx);

		dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);

		cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
		_cmdq_set_config_handle_dirty_mira(cmdq_handle);
		cmdqRecFlushAsyncCallback(cmdq_handle, callback, 0);
		pgc->need_trigger_ovl1to2 = 0;
		/* cmdqRecDestroy(cmdq_handle); */
	} else {
		ret = -1;
		DISPERR("fail to remove memout out\n");
	}
	return ret;
}

static void primary_display_frame_update_irq_callback(DISP_MODULE_ENUM module, unsigned int param)
{
	/* /if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) */
	/* /    return; */

	if (module == DISP_MODULE_RDMA0) {
		if (param & 0x2) {	/* rdma0 frame start  0x20 */
			if (pgc->session_id > 0 && primary_display_is_decouple_mode())
				update_frm_seq_info(ddp_ovl_get_cur_addr(1, 0), 0, 1, FRM_START);
		}

		if (param & 0x4) {	/* rdma0 frame done */
			atomic_set(&primary_display_frame_update_event, 1);
			wake_up_interruptible(&primary_display_frame_update_wq);
		}
	}

	if ((module == DISP_MODULE_OVL0) && (primary_display_is_decouple_mode() == 0)) {
		if (param & 0x2) {	/* ov0 frame done */
			atomic_set(&primary_display_frame_update_event, 1);
			wake_up_interruptible(&primary_display_frame_update_wq);
		}
	}

}

static int primary_display_frame_update_kthread(void *data)
{
	struct sched_param param = {.sched_priority = RTPM_PRIO_SCRN_UPDATE };

	sched_setscheduler(current, SCHED_RR, &param);

	for (;;) {
		wait_event_interruptible(primary_display_frame_update_wq,
					 atomic_read(&primary_display_frame_update_event));
		atomic_set(&primary_display_frame_update_event, 0);

		if (pgc->session_id > 0)
			update_frm_seq_info(0, 0, 0, FRM_END);

		if (kthread_should_stop())
			break;

	}

	return 0;
}

static int _present_fence_release_worker_thread(void *data)
{
	disp_sync_info *layer_info;
	static int count;
	struct sched_param param = {.sched_priority = RTPM_PRIO_FB_THREAD };
	sched_setscheduler(current, SCHED_RR, &param);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);

	while (1) {
		int fence_increment = 0;
		int timeline_id;
		wait_event_interruptible(primary_display_present_fence_wq,
				atomic_read(&primary_display_present_fence_update_event));
		atomic_set(&primary_display_present_fence_update_event, 0);
		if (!islcmconnected && !primary_display_is_video_mode()) {
			if ((count++%3) == 0)
				DISPCHECK("LCM Not Connected && CMD Mode\n");
			msleep(20);
		} else {
			dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
			/* dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE); */
		}

		timeline_id = disp_sync_get_present_timeline_id();

		layer_info = _get_sync_info(primary_session_id, timeline_id);
		if (layer_info == NULL) {
			MMProfileLogEx(ddp_mmp_get_events()->present_fence_release,
				       MMProfileFlagPulse, -1, 0x5a5a5a5a);
			continue;
		}

		_primary_path_lock(__func__);
		fence_increment = gPresentFenceIndex - layer_info->timeline->value;
		if (fence_increment > 0) {
			timeline_inc(layer_info->timeline, fence_increment);
			DISPPR_FENCE("R+/%s%d/L%d/id%d\n",
				     disp_session_mode_spy(primary_session_id),
				     DISP_SESSION_DEV(primary_session_id), timeline_id,
				     gPresentFenceIndex);
		}
		MMProfileLogEx(ddp_mmp_get_events()->present_fence_release, MMProfileFlagPulse,
			       gPresentFenceIndex, fence_increment);
		_primary_path_unlock(__func__);
	}

	return 0;
}

int primary_display_set_frame_buffer_address(unsigned long va, unsigned long mva)
{

	DISPDBG("framebuffer va 0x%lx, mva 0x%lx\n", va, mva);
	pgc->framebuffer_va = va;
	pgc->framebuffer_mva = mva;

	return 0;
}

unsigned long primary_display_get_frame_buffer_mva_address(void)
{
	return pgc->framebuffer_mva;
}

unsigned long primary_display_get_frame_buffer_va_address(void)
{
	return pgc->framebuffer_va;
}

int is_dim_layer(unsigned long mva)
{
	if (mva == get_dim_layer_mva_addr())
		return 1;
	return 0;
}

unsigned long get_dim_layer_mva_addr(void)
{
	static unsigned long dim_layer_mva;
	if (dim_layer_mva == 0) {
		int frame_buffer_size = ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) *
		    DISP_GetScreenHeight() * 4;
		dim_layer_mva = pgc->framebuffer_mva + (DISP_GetPages() - 1) * frame_buffer_size;
		DISPMSG("init dim layer mva %lu, size %d", dim_layer_mva, frame_buffer_size);
	}
	return dim_layer_mva;
}

static int init_cmdq_slots(cmdqBackupSlotHandle *pSlot, int count, int init_val)
{
	int i;
	cmdqBackupAllocateSlot(pSlot, count);

	for (i = 0; i < count; i++)
		cmdqBackupWriteSlot(*pSlot, i, init_val);

	return 0;
}

static int update_primary_intferface_module(void)
{
	/*update interface module, it may be: dsi0/dsi1/dsi_dual */
	DISP_MODULE_ENUM interface_module;
	interface_module = _get_dst_module_by_lcm(pgc->plcm);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_DISP, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_RDMA0_DISP, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_BYPASS_RDMA, interface_module);

	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_ALL, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_DITHER_1TO2, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_UFOE_1TO2, interface_module);

	return 0;
}

#if defined(ILI9881C_FHD720_DSI_VDO_BOE)
/* liping-m@PhoneSW.Multimedia, 2016/02/26	ADD for lcm */
unsigned int oppo_lcm_is_which = 0;

void  oppo_disp_lcm_is_which(char *lcm_name)
{
	char *plcm_name_boe ="ili9881c_fhd720_dsi_vdo_boe_drv";
	char *plcm_name_truly ="nt35521s_fhd720_dsi_vdo_truly";
	char *plcm_name_tianma ="oppo_nt35521s_fhd720_dsi_vdo_tianma";

	//lcm_drv = lcm_driver_list[0];
	if (!strcmp(lcm_name, plcm_name_boe)){
		DISPCHECK("oppo_disp_lcm_is_which lcm_drv is ili9881c_fhd720_dsi_vdo_boe_drv \n");
		oppo_lcm_is_which=1;
	}else if(!strcmp(lcm_name, plcm_name_truly)){
		DISPCHECK("oppo_disp_lcm_is_which lcm_drv is nt35521s_fhd720_dsi_vdo_truly \n");
		oppo_lcm_is_which = 2;
	}else{
		DISPCHECK("oppo_disp_lcm_is_which lcm_drv is oppo_nt35521s_fhd720_dsi_vdo_tianma \n");
		oppo_lcm_is_which = 3;
	}

}
#endif

int primary_display_init(char *lcm_name, unsigned int lcm_fps, int is_lcm_inited)
{
	DISP_STATUS ret = DISP_STATUS_OK;
	DISP_MODULE_ENUM dst_module = 0;
	LCM_PARAMS *lcm_param = NULL;
	int use_cmdq = disp_helper_get_option(DISP_OPT_USE_CMDQ);
	struct ddp_io_golden_setting_arg gset_arg;
	disp_ddp_path_config *data_config = NULL;

	DISPCHECK("primary_display_init begin lcm=%s, inited=%d\n", lcm_name, is_lcm_inited);

	#if defined(ILI9881C_FHD720_DSI_VDO_BOE)
	/* liping-m@PhoneSW.Multimedia, 2016/02/26	ADD for lcm */
		oppo_disp_lcm_is_which(lcm_name);
	#endif

	dprec_init();
	dpmgr_init();

	init_cmdq_slots(&(pgc->ovl_config_time), 3, 0);
	init_cmdq_slots(&(pgc->cur_config_fence), DISP_SESSION_TIMELINE_COUNT, 0);
	init_cmdq_slots(&(pgc->subtractor_when_free), DISP_SESSION_TIMELINE_COUNT, 0);
	init_cmdq_slots(&(pgc->rdma_buff_info), 3, 0);
	init_cmdq_slots(&(pgc->ovl_status_info), 4, 0);
    init_cmdq_slots(&(pgc->dither_status_info), 1, 0x10001);

    #ifdef VENDOR_EDIT
    /* pdl@oppo.com,2016/04/28 add for debug system hang */
    init_cmdq_slots(&(pgc->ovl_status_info_debug), 4, 0);
    #endif

	mutex_init(&(pgc->capture_lock));
	mutex_init(&(pgc->lock));
	mutex_init(&(pgc->switch_dst_lock));

	fps_ctx_init(fps_get_ctx(), disp_helper_get_option(DISP_OPT_FPS_CALC_WND));

	_primary_path_lock(__func__);

	pgc->plcm = disp_lcm_probe(lcm_name, LCM_INTERFACE_NOTDEFINED, is_lcm_inited);

	if (pgc->plcm == NULL) {
		DISPDBG("disp_lcm_probe returns null\n");
		ret = DISP_STATUS_ERROR;
		goto done;
	} else {
		DISPCHECK("disp_lcm_probe SUCCESS\n");
	}

	lcm_param = disp_lcm_get_params(pgc->plcm);

	if (lcm_param == NULL) {
		DISPERR("get lcm params FAILED\n");
		ret = DISP_STATUS_ERROR;
		goto done;
	}

	update_primary_intferface_module();

	if (use_cmdq) {
		ret = cmdqCoreRegisterCB(CMDQ_GROUP_DISP, cmdqDdpClockOn,
					 cmdqDdpDumpInfo, cmdqDdpResetEng, cmdqDdpClockOff);
		if (ret) {
			DISPERR("cmdqCoreRegisterCB failed, ret=%d\n", ret);
			ret = DISP_STATUS_ERROR;
			goto done;
		}

		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &(pgc->cmdq_handle_config));
		if (ret) {
			DISPDBG("cmdqRecCreate FAIL, ret=%d\n", ret);
			ret = DISP_STATUS_ERROR;
			goto done;
		} else {
			DISPDBG("cmdqRecCreate SUCCESS, g_cmdq_handle=%p\n",
				  pgc->cmdq_handle_config);
		}
		/*create ovl2mem path cmdq handle */
		ret =
		    cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_MEMOUT, &(pgc->cmdq_handle_ovl1to2_config));
		if (ret != 0) {
			DISPERR("cmdqRecCreate FAIL, ret=%d\n", ret);
			ret = DISP_STATUS_ERROR;
			goto done;
		}
	} else {
		pgc->cmdq_handle_config = NULL;
		pgc->cmdq_handle_ovl1to2_config = NULL;
	}

	if (primary_display_mode == DIRECT_LINK_MODE) {
		_build_path_direct_link();
		pgc->session_mode = DISP_SESSION_DIRECT_LINK_MODE;
		DISPCHECK("primary display is DIRECT LINK MODE\n");
	} else if (primary_display_mode == DECOUPLE_MODE) {
		_build_path_decouple();
		pgc->session_mode = DISP_SESSION_DECOUPLE_MODE;

		DISPCHECK("primary display is DECOUPLE MODE\n");
	} else if (primary_display_mode == SINGLE_LAYER_MODE) {
		_build_path_single_layer();

		DISPCHECK("primary display is SINGLE LAYER MODE\n");
	} else if (primary_display_mode == DEBUG_RDMA1_DSI0_MODE) {
		_build_path_debug_rdma1_dsi0();

		DISPCHECK("primary display is DEBUG RDMA1 DSI0 MODE\n");
	} else {
		DISPCHECK("primary display mode is WRONG\n");
	}

	if (use_cmdq && is_lcm_inited) {
		/* if lcm is not inited (no LK),
		 * the first config should not wait frame done
		 * because there's no frame done for vdo mode */
		_cmdq_reset_config_handle();
		_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
	}

	config_display_m4u_port();
	primary_display_set_max_layer(PRIMARY_SESSION_INPUT_LAYER_COUNT);
	if (!disp_helper_get_option(DISP_OPT_GMO_OPTIMIZE))
		init_decouple_buffers();

	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
	DISPDBG("primary_display_init->dpmgr_path_init\n");
	dpmgr_path_init(pgc->dpmgr_handle, use_cmdq);

	/* use fake timer to generate vsync signal for cmd mode w/o LCM(originally using LCM TE Signal as VSYNC) */
	/* so we don't need to modify display driver's behavior. */
	if (disp_helper_get_option(DISP_OPT_NO_LCM_FOR_LOW_POWER_MEASUREMENT)) {
		/* only for low power measurement */
		DISPWARN("WARNING!!!!!! FORCE NO LCM MODE!!!\n");
		islcmconnected = 0;

		/* no need to change video mode vsync behavior */
		if (!primary_display_is_video_mode()) {
			_init_vsync_fake_monitor(lcm_fps);

			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_UNKNOWN);
		}
	}

	if (use_cmdq) {
		_cmdq_build_trigger_loop();
		_cmdq_start_trigger_loop();
	}

	DISPCHECK("primary_display_init->dpmgr_path_config\n");


	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	memcpy(&(data_config->dispif_config), lcm_param, sizeof(LCM_PARAMS));
	data_config->dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	data_config->dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	data_config->p_golden_setting_context = get_golden_setting_pgc();

	if (lcm_param->type == LCM_TYPE_DSI) {
		if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888)
			data_config->lcm_bpp = 24;
		else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565)
			data_config->lcm_bpp = 16;
		else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666)
			data_config->lcm_bpp = 18;
	} else if (lcm_param->type == LCM_TYPE_DPI) {
		if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888)
			data_config->lcm_bpp = 24;
		else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565)
			data_config->lcm_bpp = 16;
		if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666)
			data_config->lcm_bpp = 18;
	}

	data_config->fps = lcm_fps;
	data_config->dst_dirty = 1;
	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, pgc->cmdq_handle_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 0;
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	dpmgr_path_start(pgc->dpmgr_handle, use_cmdq);

	if (use_cmdq) {
		_cmdq_flush_config_handle(0, NULL, 0);
		_cmdq_reset_config_handle();
		_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
	}

	if (is_lcm_inited) {
		ret = disp_lcm_init(pgc->plcm, 0);	/* no need lcm power on,because lk power on lcm */
	} else {
		ret = disp_lcm_init(pgc->plcm, 1);

		if (primary_display_is_video_mode())
			dpmgr_path_trigger(pgc->dpmgr_handle, NULL, 0);
	}

	if (disp_helper_get_option(DISP_OPT_MET_LOG))
		set_enterulps(0);

	primary_display_check_recovery_init();

	if (disp_helper_get_option(DISP_OPT_SWITCH_DST_MODE)) {
		primary_display_switch_dst_mode_task =
		    kthread_create(_disp_primary_path_switch_dst_mode_thread, NULL,
				   "display_switch_dst_mode");
		wake_up_process(primary_display_switch_dst_mode_task);
	}

	if (decouple_update_rdma_config_thread == NULL) {
		decouple_update_rdma_config_thread =
		    kthread_create(decouple_mirror_update_rdma_config_thread, NULL,
				   "decouple_update_rdma_cfg");
		wake_up_process(decouple_update_rdma_config_thread);
	}

	if (decouple_trigger_thread == NULL) {
		decouple_trigger_thread =
		    kthread_create(decouple_trigger_worker_thread, NULL, "decouple_trigger");
		wake_up_process(decouple_trigger_thread);
	}

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		primary_path_aal_task =
		    kthread_create(_disp_primary_path_check_trigger, NULL, "display_check_aal");
		wake_up_process(primary_path_aal_task);
	}

	if (disp_helper_get_option(DISP_OPT_PRESENT_FENCE)) {
		init_waitqueue_head(&primary_display_present_fence_wq);
		present_fence_release_worker_task =
		    kthread_create(_present_fence_release_worker_thread, NULL,
				   "present_fence_worker");
		wake_up_process(present_fence_release_worker_task);
	}

	if (disp_helper_get_option(DISP_OPT_PERFORMANCE_DEBUG)) {
		if (primary_display_frame_update_task == NULL) {
			init_waitqueue_head(&primary_display_frame_update_wq);
			disp_register_module_irq_callback(DISP_MODULE_RDMA0,
							  primary_display_frame_update_irq_callback);
			disp_register_module_irq_callback(DISP_MODULE_OVL0,
							  primary_display_frame_update_irq_callback);
			primary_display_frame_update_task =
			    kthread_create(primary_display_frame_update_kthread, NULL,
					   "frame_update_worker");
			wake_up_process(primary_display_frame_update_task);
		}
	}

	if (primary_display_is_video_mode()) {
		/*
		if (disp_helper_get_option(DISP_HELPER_OPTION_IDLE_MGR))
			primary_display_idlemgr_init();
			*/

		if (disp_helper_get_option(DISP_OPT_SWITCH_DST_MODE)) {
			primary_display_cur_dst_mode = 1;	/* video mode */
			primary_display_def_dst_mode = 1;	/* default mode is video mode */
		}
		if (_need_lfr_check()) {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_DSI0_FRAME_DONE);
		} else {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_RDMA0_DONE);
		}
	}

	if (disp_helper_get_option(DISP_OPT_DETECT_RECOVERY)) {
		/*thread: path recovery */
		primary_display_Recovery_task =
		    kthread_create(primary_display_recovery_thread, NULL,
				   "display_recovery");
		init_waitqueue_head(&primary_recovery_task_wq);
		if (disp_helper_get_option(DISP_OPT_BYPASS_OVL))
			wake_up_process(primary_display_Recovery_task);
	}
	pgc->lcm_fps = lcm_fps;
	/* keep lowpower init after setting lcm_fps */
	primary_display_lowpower_init();

	pgc->state = DISP_ALIVE;

/*
	primary_display_sodi_rule_init();

	if (disp_helper_get_option(DISP_HELPER_OPTION_DYNAMIC_SWITCH_MMSYSCLK))
		register_mmclk_switch_cb(primary_display_switch_mmsys_clk);
*/

done:
	primary_display_diagnose();
	dst_module = _get_dst_module_by_lcm(pgc->plcm);
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_deinit(void)
{
	_primary_path_lock(__func__);

	_cmdq_stop_trigger_loop();
	dpmgr_path_deinit(pgc->dpmgr_handle, CMDQ_DISABLE);
	_primary_path_unlock(__func__);
	return 0;
}

/* register rdma done event */
int primary_display_wait_for_idle(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();

	_primary_path_lock(__func__);


	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_wait_for_dump(void)
{
	return 0;
}

int primary_display_release_fence_fake(void)
{
	unsigned int layer_en = 0;
	unsigned int addr = 0;
	unsigned int fence_idx = -1;
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	int i = 0;

	DISPFUNC();

	for (i = 0; i < PRIMARY_SESSION_INPUT_LAYER_COUNT; i++) {
		if (i == primary_display_get_option("ASSERT_LAYER") && is_DAL_Enabled()) {
			mtkfb_release_layer_fence(session_id, 3);
		} else {
			disp_sync_get_cached_layer_info(session_id, i, &layer_en,
							(unsigned long *)&addr, &fence_idx);
			if (fence_idx == -1) {
				DISPPR_ERROR("find fence for layer %d,addr 0x%08x fail, unregistered\n", i, addr);
			} else if (fence_idx < 0) {
				DISPPR_ERROR("find fence idx for layer %d,addr 0x%08x fail,unknown\n", i, addr);
			} else {
				if (layer_en)
					mtkfb_release_fence(session_id, i, fence_idx - 1);
				else
					mtkfb_release_fence(session_id, i, fence_idx);
			}
		}
	}

	return 0;
}

int primary_display_wait_for_vsync(void *config)
{
	disp_session_vsync_config *c = (disp_session_vsync_config *) config;
	int ret = 0, has_vsync = 1;
	unsigned long long ts = 0ULL;
	static int count;
	/* kick idle manager here to ensure sodi is disabled when screen update begin(not 100% ensure) */
	primary_display_idlemgr_kick((char *)__func__, 1);

#ifdef CONFIG_MTK_FPGA
	if (!primary_display_is_video_mode())
		has_vsync = 0;	/* fpga has no TE signal */
#endif

	if (!islcmconnected || !has_vsync) {
		if ((count++%3) == 0)
			DISPCHECK("use fake vsync: lcm_connect=%d, has_vsync=%d\n",
			  islcmconnected, has_vsync);
		msleep(20);
		return 0;
	}

	if (pgc->force_fps_keep_count && pgc->force_fps_skip_count) {
		g_keep++;
		DISPMSG("vsync|keep %d\n", g_keep);
		if (g_keep == pgc->force_fps_keep_count) {
			g_keep = 0;

			while (g_skip != pgc->force_fps_skip_count) {
				g_skip++;
				DISPMSG("vsync|skip %d\n", g_skip);
				ret =
				    dpmgr_wait_event_timeout(pgc->dpmgr_handle,
							     DISP_PATH_EVENT_IF_VSYNC, HZ / 10);
				if (ret == -2) {
					DISPPR_ERROR("vsync for primary display path not enabled yet\n");
#ifdef VENDOR_EDIT
                    /* liping-m@Mobile Phone Software Dept.Driver, 2016/01/12  Add for except log */
                    mm_keylog_write("wait for vsync exception\n", "vsync for primary display path not enabled yet\n", TYPE_VSYNC_EXCEPTION);
#endif /*VENDOR_EDIT*/
					return -1;
				} else if (ret == 0) {
					primary_display_release_fence_fake();
				}
			}
			g_skip = 0;
		}
	} else {
		g_keep = 0;
		g_skip = 0;
	}

	ret = dpmgr_wait_event_ts(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, &ts);

	if (ret == -2) {
		DISPPR_ERROR("vsync for primary display path not enabled yet\n");
#ifdef VENDOR_EDIT
        /* liping-m@Mobile Phone Software Dept.Driver, 2016/01/12  Add for except log */
        mm_keylog_write("wait for vsync exception\n", "vsync for primary display path not enabled yet\n", TYPE_VSYNC_EXCEPTION);
#endif /*VENDOR_EDIT*/
		goto out;
	} else if (ret == 0) {
		/* primary_display_release_fence_fake(); */
	}

	if (pgc->vsync_drop) {
		/* ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, HZ/10); */
		ret = dpmgr_wait_event_ts(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, &ts);
	}

out:
	c->vsync_ts = ts;
	c->vsync_cnt++;

	return ret;
}

unsigned int primary_display_get_ticket(void)
{
	return dprec_get_vsync_count();
}

int primary_suspend_release_fence(void)
{
	unsigned int session = (unsigned int)((DISP_SESSION_PRIMARY) << 16 | (0));
	unsigned int i = 0;
	for (i = 0; i < DISP_SESSION_TIMELINE_COUNT; i++) {
		DISPDBG("mtkfb_release_layer_fence  session=0x%x,layerid=%d\n", session, i);
		mtkfb_release_layer_fence(session, i);
	}
	return 0;
}

#ifdef VENDOR_EDIT
//guoling@MultiMedia.Display.LCD, 2016/10/14,add for NT IC sleep in power down.
#if defined(OPPO_HX8394_TRULY_HD720_DSI_VDO)
extern void  boe_nt_suspend_power(void);
extern void  truly_nt_suspend_power(void);
#endif
#endif/*VENDOR_EDIT*/
int primary_display_suspend(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;
	int event_ret;

	DISPCHECK("primary_display_suspend begin\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagStart, 0, 0);
	primary_display_idlemgr_kick((char *)__func__, 1);

	if (disp_helper_get_option(DISP_OPT_SWITCH_DST_MODE))
		primary_display_switch_dst_mode(primary_display_def_dst_mode);

	_primary_path_switch_dst_lock();
	disp_sw_mutex_lock(&(pgc->capture_lock));
	_primary_path_lock(__func__);

	while (primary_get_state() == DISP_BLANK) {
		_primary_path_unlock(__func__);
		DISPCHECK("primary_display_suspend wait tui finish!!\n");
		primary_display_wait_state(DISP_ALIVE, MAX_SCHEDULE_TIMEOUT);
		_primary_path_lock(__func__);
		DISPCHECK("primary_display_suspend wait tui done stat=%d\n", primary_get_state());
	}

	if (pgc->state == DISP_SLEPT) {
		DISPWARN("primary display path is already sleep, skip\n");
		goto done;
	}
	primary_display_idlemgr_kick((char *)__func__, 0);

	if (pgc->session_mode == DISP_SESSION_RDMA_MODE
	    || pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
		/* switch back to DL mode before suspend */
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE,
					pgc->session_id, 0, NULL, 1);
	}

	/* need leave share sram for suspend */
	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		leave_share_sram(CMDQ_SYNC_RESOURCE_WROT0);

	/* switch to vencpll before disable mmsys clk */
	if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK))
		mmdvfs_notify_mmclk_switch_request(MMDVFS_EVENT_OVL_SINGLE_LAYER_EXIT);
	/* blocking flush before stop trigger loop */
	_blocking_flush();
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 1);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 1, 2);
		event_ret =
		    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 2, 2);
		DISPCHECK
		    ("[POWER]primary display path is busy now, wait frame done, event_ret=%d\n",
		     event_ret);
		if (event_ret <= 0) {
			DISPERR("wait frame done in suspend timeout\n");
			MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 3,
				       2);
			primary_display_diagnose();
			ret = -1;
		}
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 2);

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
		DISPCHECK("[POWER]display cmdq trigger loop stop\n");
		_cmdq_stop_trigger_loop();
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 3);

	DISPDBG("[POWER]primary display path stop[begin]\n");
	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("[POWER]primary display path stop[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 4);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 1, 4);
		DISPERR("[POWER]stop display path failed, still busy\n");
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		ret = -1;
		/* even path is busy(stop fail), we still need to continue power off other module/devices */
		/* goto done; */
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 5);

	DISPCHECK("[POWER]lcm suspend[begin]\n");
	disp_lcm_suspend(pgc->plcm);
	DISPCHECK("[POWER]lcm suspend[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 6);
	DISPDBG("[POWER]primary display path Release Fence[begin]\n");
	primary_suspend_release_fence();
	DISPCHECK("[POWER]primary display path Release Fence[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 7);

	DISPDBG("[POWER]dpmanager path power off[begin]\n");
	dpmgr_path_power_off(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (primary_display_is_decouple_mode())
		dpmgr_path_power_off(pgc->ovl2mem_path_handle, CMDQ_DISABLE);
	if (disp_helper_get_option(DISP_OPT_MET_LOG))
		set_enterulps(1);

	DISPCHECK("[POWER]dpmanager path power off[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagPulse, 0, 8);
#ifdef VENDOR_EDIT
//guoling@MultiMedia.Display.LCD, 2016/10/14,add for NT IC sleep in power down.
#if defined(OPPO_HX8394_TRULY_HD720_DSI_VDO)
	if(!strcmp(pgc->plcm->drv->name, "oppo_nt35521s_boe_hd720_dsi_vdo")){
	    boe_nt_suspend_power();
	}else if(!strcmp(pgc->plcm->drv->name, "oppo_nt35521s_truly_g5d_hd720_dsi_vdo")){
	    truly_nt_suspend_power();
	}
#endif
#endif/*VENDOR_EDIT*/

done:
	primary_set_state(DISP_SLEPT);
	_primary_path_unlock(__func__);
	disp_sw_mutex_unlock(&(pgc->capture_lock));
	_primary_path_switch_dst_unlock();

	aee_kernel_wdt_kick_Powkey_api("mtkfb_early_suspend", WDT_SETBY_Display);
	primary_trigger_cnt = 0;
	MMProfileLogEx(ddp_mmp_get_events()->primary_suspend, MMProfileFlagEnd, 0, 0);
	DISPCHECK("primary_display_suspend end\n");
	return ret;
}

int primary_display_get_lcm_index(void)
{
	int index = 0;
	DISPFUNC();

	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	index = pgc->plcm->index;
	return index;
}

int primary_display_resume(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;
	struct ddp_io_golden_setting_arg gset_arg;
	int use_cmdq, i;

	DISPCHECK("primary_display_resume begin\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagStart, 0, 0);

	_primary_path_lock(__func__);
	use_cmdq = disp_helper_get_option(DISP_OPT_USE_CMDQ);
	if (pgc->state == DISP_ALIVE) {
		DISPCHECK("primary display path is already resume, skip\n");
		goto done;
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 1);

	if (is_ipoh_bootup) {
		DISPCHECK("[primary display path] leave primary_display_resume -- IPOH\n");
		DISPCHECK("ESD check start[begin]\n");
		primary_display_esd_check_enable(1);
		DISPCHECK("ESD check start[end]\n");
		is_ipoh_bootup = false;
		DISPDBG("[POWER]start cmdq[begin]--IPOH\n");
		if (use_cmdq)
			_cmdq_start_trigger_loop();
		enable_idlemgr(1);
		DISPDBG("[POWER]start cmdq[end]--IPOH\n");
		goto done;
	}
	/* c/v switch by suspend start*/
	if (disp_helper_get_option(DISP_OPT_CV_BYSUSPEND)) {
		if (lcm_mode_status != 0) {
			static LCM_DSI_MODE_CON vdo_mode_type;
			LCM_PARAMS *lcm_param_cv = NULL;

			lcm_param_cv = disp_lcm_get_params(pgc->plcm);
			DISPCHECK("lcm_mode_status=%d, lcm_param_cv->dsi.mode %d\n",
					lcm_mode_status, lcm_param_cv->dsi.mode);
			if (lcm_param_cv->dsi.mode != CMD_MODE)
				vdo_mode_type = lcm_param_cv->dsi.mode;
			if (lcm_mode_status == 1) {
				lcm_dsi_mode = CMD_MODE;
				spm_sodi_mempll_pwr_mode(0);
				spm_enable_sodi(1);
			} else if (lcm_mode_status == 2) {
				if (vdo_mode_type)
					lcm_dsi_mode = vdo_mode_type;
				else
					lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
				spm_sodi_mempll_pwr_mode(1);
				spm_enable_sodi(1);
			} else {
				lcm_dsi_mode = lcm_param_cv->dsi.mode;
			}

			DSI_ForceConfig(1);
			lcm_param_cv->dsi.mode = lcm_dsi_mode;
			MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, lcm_mode_status);
			lcm_mode_status = 0;
			if (disp_helper_get_option(DISP_OPT_CV_BYSUSPEND))
				primary_display_check_recovery_init();
		}
		DISPDBG("lcm_mode_status=%d, lcm_dsi_mode=%d\n", lcm_mode_status, lcm_dsi_mode);
	}
	/* c/v switch by suspend end*/
	DISPDBG("dpmanager path power on[begin]\n");
	dpmgr_path_power_on(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (disp_helper_get_option(DISP_OPT_MET_LOG))
		set_enterulps(0);

	DISPCHECK("dpmanager path power on[end]\n");
	DISPCHECK("dpmanager path reset[begin]\n");
	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
	DISPCHECK("dpmanager path reset[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 2);

	DISPDBG("[POWER]dpmanager re-init[begin]\n");

	{
		LCM_PARAMS *lcm_param = NULL;
		disp_ddp_path_config *data_config = NULL;
		/* disconnect primary path first *
		 * because MMsys config register may not power off during early suspend
		 * BUT session mode may change in primary_display_switch_mode() */
		ddp_disconnect_path(DDP_SCENARIO_PRIMARY_ALL, NULL);
		ddp_disconnect_path(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, NULL);
		DISPCHECK("cmd/video mode=%d\n", primary_display_is_video_mode());
		dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());

		dpmgr_path_connect(pgc->dpmgr_handle, CMDQ_DISABLE);
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 2);
		lcm_param = disp_lcm_get_params(pgc->plcm);

		data_config =
		    dpmgr_path_get_last_config(pgc->dpmgr_handle);
		memcpy(&(data_config->dispif_config), lcm_param, sizeof(LCM_PARAMS));

		data_config->dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
		data_config->dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
		if (lcm_param->type == LCM_TYPE_DSI) {
			if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888)
				data_config->lcm_bpp = 24;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565)
				data_config->lcm_bpp = 16;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666)
				data_config->lcm_bpp = 18;
		} else if (lcm_param->type == LCM_TYPE_DPI) {
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888)
				data_config->lcm_bpp = 24;
			else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565)
				data_config->lcm_bpp = 16;
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666)
				data_config->lcm_bpp = 18;
		}

		data_config->fps = pgc->lcm_fps;
		data_config->dst_dirty = 1;

		/* disable all ovl layers to show black screen */
		/* note that if WFD is connected, we may miss the black setting before the last suspend */
		for (i = 0; i < ARRAY_SIZE(data_config->ovl_config); i++) {
			if (is_DAL_Enabled() &&
				data_config->ovl_config[i].layer == primary_display_get_option("ASSERT_LAYER"))
				continue;
			data_config->ovl_config[i].layer_en = 0;
		}

		data_config->ovl_dirty = 1;

		ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, NULL);
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 2, 2);
		data_config->dst_dirty = 0;

		memset(&gset_arg, 0, sizeof(gset_arg));
		gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
		gset_arg.is_decouple_mode = primary_display_is_decouple_mode();
		dpmgr_path_ioctl(pgc->dpmgr_handle, NULL, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	}
	DISPCHECK("[POWER]dpmanager re-init[end]\n");
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 3);

	DISPDBG("[POWER]lcm resume[begin]\n");
	disp_lcm_resume(pgc->plcm);
	DISPCHECK("[POWER]lcm resume[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 4);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 4);
		DISPERR("[POWER]Fatal error, we didn't start display path but it's already busy\n");
		ret = -1;
		/* goto done; */
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 5);
	DISPCHECK("[POWER]dpmgr path start[begin]\n");
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (primary_display_is_decouple_mode())
		dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

	DISPCHECK("[POWER]dpmgr path start[end]\n");

	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 6);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 6);
		DISPERR
		    ("[POWER]Fatal error, we didn't trigger display path but it's already busy\n");
		ret = -1;
		/* goto done; */
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 7);

	if (use_cmdq) {
	    DISPCHECK("[POWER]build cmdq trigger loop[begin]\n");
		_cmdq_build_trigger_loop();
		DISPCHECK("[POWER]build cmdq trigger loop[end]\n");
	}
	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 1, 7);
		/* for video mode, we need to force trigger here */
		/* for cmd mode, just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
		if (_should_reset_cmdq_config_handle())
			_cmdq_reset_config_handle();
		if (_should_insert_wait_frame_done_token())
			_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 8);

	if (use_cmdq) {
		DISPDBG("[POWER]start cmdq[begin]\n");
		_cmdq_start_trigger_loop();
		DISPDBG("[POWER]start cmdq[end]\n");
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 9);

	/* primary_display_diagnose(); */
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 10);

	if (!primary_display_is_video_mode()) {
		DISPCHECK("[POWER]triggger cmdq[begin] cmd mode\n");
		if (_should_reset_cmdq_config_handle())
			_cmdq_reset_config_handle();
		if (_should_insert_wait_frame_done_token())
			_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_EXT_TE);
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);

		/*refresh black picture of ovl bg */
		_trigger_display_interface(1, NULL, 0);
		DISPCHECK("[POWER]triggger cmdq[end] cmd mode\n");
		mdelay(16);	/* wait for one frame for pms workarround!!!! */
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagPulse, 0, 11);

	/* (in suspend) when we stop trigger loop
	 * if no other thread is runing, cmdq may disable its clock
	 * all cmdq event will be cleared after suspend */
	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

	/* reinit fake timer for debug, we can enable option then press powerkey to enable thsi feature. */
	/* use fake timer to generate vsync signal for cmd mode w/o LCM(originally using LCM TE Signal as VSYNC) */
	/* so we don't need to modify display driver's behavior. */
	if (disp_helper_get_option(DISP_OPT_NO_LCM_FOR_LOW_POWER_MEASUREMENT)) {
		/* only for low power measurement */
		DISPWARN("WARNING!!!!!! FORCE NO LCM MODE!!!\n");
		islcmconnected = 0;

		/* no need to change video mode vsync behavior */
		if (!primary_display_is_video_mode()) {
			_init_vsync_fake_monitor(pgc->lcm_fps);

			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_UNKNOWN);
		}
	}

	/* need enter share sram for resume */
	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		enter_share_sram(CMDQ_SYNC_RESOURCE_WROT0);
	if (disp_helper_get_option(DISP_OPT_CV_BYSUSPEND))
		DSI_ForceConfig(0);
done:
	primary_set_state(DISP_ALIVE);
	_primary_path_unlock(__func__);

	aee_kernel_wdt_kick_Powkey_api("mtkfb_late_resume", WDT_SETBY_Display);
	MMProfileLogEx(ddp_mmp_get_events()->primary_resume, MMProfileFlagEnd, 0, 0);
	return 0;
}

int primary_display_ipoh_restore(void)
{
	DISPMSG("primary_display_ipoh_restore In\n");
	DISPDBG("ESD check stop[begin]\n");
	enable_idlemgr(0);
	primary_display_esd_check_enable(0);
	DISPCHECK("ESD check stop[end]\n");
	if (NULL != pgc->cmdq_handle_trigger) {
		struct TaskStruct *pTask = pgc->cmdq_handle_trigger->pRunningTask;
		if (NULL != pTask) {
			DISPCHECK("[Primary_display]display cmdq trigger loop stop[begin]\n");
			_cmdq_stop_trigger_loop();
			DISPCHECK("[Primary_display]display cmdq trigger loop stop[end]\n");
			ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), NULL, 0);
		}
	}
	DISPMSG("primary_display_ipoh_restore Out\n");
	return 0;
}

int primary_display_start(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();

	_primary_path_lock(__func__);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPERR("Fatal error, we didn't trigger display path but it's already busy\n");
		ret = -1;
		goto done;
	}

done:
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_stop(void)
{
	DISP_STATUS ret = DISP_STATUS_OK;

	DISPFUNC();
	_primary_path_lock(__func__);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle))
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);


	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		DISPERR("stop display path failed, still busy\n");
		ret = -1;
		goto done;
	}

done:
	_primary_path_unlock(__func__);
	return ret;
}

void primary_display_update_present_fence(unsigned int fence_idx)
{
	gPresentFenceIndex = fence_idx;
	atomic_set(&primary_display_present_fence_update_event, 1);
	wake_up_interruptible(&primary_display_present_fence_wq);
}

/* the function will trigger ovl->wdma */
static int trigger_decouple_mirror(void)
{
	if (pgc->need_trigger_dcMirror_out == 0) {
		/* DISPPR_ERROR("There is no output config when decouple mirror!!\n"); */
	} else {
		pgc->need_trigger_dcMirror_out = 0;

		/* check if still in decouple mirror mode: *
		 * DC->DL switch may happen before vsync, it may free ovl2mem_handle ! */
		if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
			_trigger_ovl_to_memory(pgc->ovl2mem_path_handle,
						      pgc->cmdq_handle_ovl1to2_config,
						      _ovl_wdma_fence_release_callback,
						      DISP_SESSION_DECOUPLE_MIRROR_MODE);
			dprec_logger_trigger(DPREC_LOGGER_PRIMARY_TRIGGER, 0xffffffff, 0);
		} else {
			dprec_logger_trigger(DPREC_LOGGER_PRIMARY_TRIGGER, 0xffffffff, 0xaaaaaaaa);
		}
	}
	return 0;
}

static int primary_display_trigger_nolock(int blocking, void *callback, int need_merge)
{
	int ret = 0;
	/* DISPFUNC(); */

	last_primary_trigger_time = sched_clock();
	if (is_switched_dst_mode) {
		primary_display_switch_dst_mode(1);	/* swith to vdo mode if trigger disp */
		is_switched_dst_mode = false;
	}

	primary_trigger_cnt++;

	if (pgc->state == DISP_SLEPT) {
		DISPERR("%s, skip because primary dipslay is sleep\n", __func__);
		goto done;
	}
	primary_display_idlemgr_kick((char *)__func__, 0);

	dprec_logger_start(DPREC_LOGGER_PRIMARY_TRIGGER, 0, 0);

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE ||
		pgc->session_mode == DISP_SESSION_RDMA_MODE) {
		_trigger_display_interface(blocking, _ovl_fence_release_callback,
					   DISP_SESSION_DIRECT_LINK_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) {
		_trigger_display_interface(0, _ovl_fence_release_callback,
					   DISP_SESSION_DIRECT_LINK_MIRROR_MODE);

		if (pgc->need_trigger_ovl1to2 == 0) {
			DISPPR_ERROR("There is no output config when directlink mirror!!\n");
		} else {
			primary_display_remove_output(_wdma_fence_release_callback,
						      DISP_SESSION_DIRECT_LINK_MIRROR_MODE);
			pgc->need_trigger_ovl1to2 = 0;
		}
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
		_trigger_ovl_to_memory(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config,
				       _ovl_fence_release_callback, DISP_SESSION_DECOUPLE_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		if (need_merge == 0 || primary_is_sec()) {
			trigger_decouple_mirror();
		} else {
			/* because only one of MM/UI thread will set output */
			/* we have to merge it up, then trigger at next vsync */
			/* see decouple_trigger_worker_thread below */
			atomic_set(&decouple_trigger_event, 1);
			wake_up(&decouple_trigger_wq);
		}
	}

	dprec_logger_done(DPREC_LOGGER_PRIMARY_TRIGGER, 0, 0);

	smart_ovl_try_switch_mode_nolock();
done:
	if ((primary_trigger_cnt > 1) && aee_kernel_Powerkey_is_press()) {
		aee_kernel_wdt_kick_Powkey_api("primary_display_trigger", WDT_SETBY_Display);
		primary_trigger_cnt = 0;
	}

	if (pgc->session_id > 0)
		update_frm_seq_info(0, 0, 0, FRM_TRIGGER);

	return ret;
}

int primary_display_trigger(int blocking, void *callback, int need_merge)
{
	int ret;

	_primary_path_lock(__func__);
	ret = primary_display_trigger_nolock(blocking, callback, need_merge);
	_primary_path_unlock(__func__);
	return ret;
}

/* the function will trigger dc and wake up dc thread for merge status; */
/* decouple_trigger thread->trigger mirror->decouple_update_rdma_config_thread */
static int decouple_trigger_worker_thread(void *data)
{
	struct sched_param param = {.sched_priority = RTPM_PRIO_SCRN_UPDATE };
	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		wait_event(decouple_trigger_wq, atomic_read(&decouple_trigger_event));
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);

		_primary_path_lock(__func__);
		trigger_decouple_mirror();
		atomic_set(&decouple_trigger_event, 0);
		wake_up(&decouple_trigger_wq);
		_primary_path_unlock(__func__);
		if (kthread_should_stop()) {
			DISPERR("error: stop %s as demond\n", __func__);
			break;
		}
	}
	return 0;
}

static int config_wdma_output(disp_path_handle disp_handle,
			      cmdqRecHandle cmdq_handle, disp_output_config *output)
{
	disp_ddp_path_config *pconfig = NULL;

	ASSERT(output != NULL);
	pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->wdma_config.dstAddress = (unsigned long)output->pa;
	pconfig->wdma_config.srcHeight = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	pconfig->wdma_config.srcWidth = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	pconfig->wdma_config.clipX = output->x;
	pconfig->wdma_config.clipY = output->y;
	pconfig->wdma_config.clipHeight = output->height;
	pconfig->wdma_config.clipWidth = output->width;
	pconfig->wdma_config.outputFormat = disp_fmt_to_unified_fmt(output->fmt);
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch = output->pitch * UFMT_GET_Bpp(pconfig->wdma_config.outputFormat);
	pconfig->wdma_config.security = output->security;
	pconfig->wdma_dirty = 1;

	return dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
}

static int _convert_disp_output_to_memout(disp_output_config *src, disp_mem_output_config *dst)
{
	dst->fmt = disp_fmt_to_unified_fmt(src->fmt);

	dst->vaddr = (unsigned long)src->va;
	dst->security = src->security;
	dst->w = src->width;
	dst->h = src->height;

	dst->addr = (unsigned long)src->pa;

	dst->buff_idx = src->buff_idx;
	dst->interface_idx = src->interface_idx;

	dst->x = src->x;
	dst->y = src->y;
	dst->pitch = src->pitch * UFMT_GET_Bpp(dst->fmt);
	return 0;
}

static int primary_frame_cfg_output(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;
	disp_path_handle disp_handle;
	cmdqRecHandle cmdq_handle = NULL;

	if (pgc->state == DISP_SLEPT) {
		DISPERR("mem out is already slept or mode wrong(%d)\n", pgc->session_mode);
		goto done;
	}

	if (!primary_display_is_mirror_mode()) {
		DISPERR("should not config output if not mirror mode!!\n");
		goto done;
	}

	if (primary_display_is_decouple_mode()) {
		disp_handle = pgc->ovl2mem_path_handle;
		cmdq_handle = pgc->cmdq_handle_ovl1to2_config;
	} else {
		disp_handle = pgc->dpmgr_handle;
		cmdq_handle = pgc->cmdq_handle_config;
	}

	if (primary_display_is_decouple_mode()) {
		pgc->need_trigger_dcMirror_out = 1;
	} else {
		/*direct link mirror mode should add memout first */
		dpmgr_path_add_memout(pgc->dpmgr_handle, DISP_MODULE_OVL0, cmdq_handle);
		pgc->need_trigger_ovl1to2 = 1;
	}

	ret = config_wdma_output(disp_handle, cmdq_handle, &cfg->output_cfg);

	if ((pgc->session_id > 0) && primary_display_is_decouple_mode()) {
		update_frm_seq_info((unsigned long)cfg->output_cfg.pa, 0,
				mtkfb_query_frm_seq_by_addr(pgc->session_id, 0, 0), FRM_CONFIG);
	}

	_convert_disp_output_to_memout(&cfg->output_cfg, &mem_config);

	MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_config, MMProfileFlagPulse,
		       cfg->output_cfg.buff_idx, (unsigned long)cfg->output_cfg.pa);

done:
	return ret;
}


static int can_bypass_ovl(disp_ddp_path_config *data_config, int *bypass_layer_id)
{
	int total_layer = 0;
	int i;
	unsigned int w, h;

#ifdef CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW
	/* rdma don't support rotation */
	return 0;
#endif

	if (!disp_helper_get_option(DISP_OPT_BYPASS_OVL))
		return 0;

	for (i = 0; i < ARRAY_SIZE(data_config->ovl_config); i++) {
		if (data_config->ovl_config[i].layer_en) {
			total_layer++;
			*bypass_layer_id = i;
		}
	}

	if (total_layer != 1)
		return 0;

	/* rdma cannot process dim layer */
	if (data_config->ovl_config[*bypass_layer_id].source != OVL_LAYER_SOURCE_MEM)
		return 0;

	/* now we have only 1 layer */
	/* we need to check layer size, because rdma has output_valid_thres setting
	 * if (size < output_valid_thres) RDMA will hang !!*/
	h = data_config->ovl_config[*bypass_layer_id].dst_h;
	w = data_config->ovl_config[*bypass_layer_id].dst_w;
	if (w * h <= 512 * 16 / 2)
		return 0;

    #ifdef VENDOR_EDIT
    /* pdl@oppo.com, 2016/02/04,add for fix system hang issue */
    if((w != 1080) || (h != 1920))
    {
        return 0;
    }
    #endif /*VENDOR_EDIT*/

	return 1;
}

static int _config_ovl_input(struct disp_frame_cfg_t *cfg,
			     disp_path_handle disp_handle, cmdqRecHandle cmdq_handle)
{
	int ret = 0, i = 0, layer = 0;
	disp_ddp_path_config *data_config = NULL;
	int max_layer_id_configed = 0;
	int bypass, bypass_layer_id = 0;
	int overlap_layers;

#ifdef DEBUG_OVL_CONFIG_TIME
	cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_config_time, 0, 0x10008028);
#endif
	/*=== create new data_config for ovl input ===*/
	data_config = dpmgr_path_get_last_config(disp_handle);
	for (i = 0; i < cfg->input_layer_num; i++) {
		disp_input_config *input_cfg = &cfg->input_cfg[i];
		OVL_CONFIG_STRUCT *ovl_cfg;

		layer = input_cfg->layer_id;
		ovl_cfg = &(data_config->ovl_config[layer]);
		if (cfg->setter != SESSION_USER_AEE) {
			if (is_DAL_Enabled() && layer == primary_display_get_option("ASSERT_LAYER")) {
				DISPMSG("skip AEE layer %d\n", layer);
				continue;
			}
		} else {
			DISPMSG("set AEE layer %d\n", layer);
		}
		_convert_disp_input_to_ovl(ovl_cfg, input_cfg);

		dprec_logger_start(DPREC_LOGGER_PRIMARY_CONFIG,
				   ovl_cfg->layer | (ovl_cfg->layer_en << 16), ovl_cfg->addr);
		dprec_logger_done(DPREC_LOGGER_PRIMARY_CONFIG, ovl_cfg->src_x, ovl_cfg->src_y);

		dprec_mmp_dump_ovl_layer(ovl_cfg, layer, 1);

		if ((ovl_cfg->layer == 0) && (!primary_display_is_decouple_mode()))
			update_frm_seq_info(ovl_cfg->addr,
					    ovl_cfg->src_x * 4 +
					    ovl_cfg->src_y * ovl_cfg->src_pitch,
					    mtkfb_query_frm_seq_by_addr(pgc->session_id, 0, 0),
					    FRM_CONFIG);

		if (max_layer_id_configed < layer)
			max_layer_id_configed = layer;

		data_config->ovl_layer_dirty |= (1 << i);
	}
	overlap_layers = cfg->overlap_layer_num;
	data_config->overlap_layer_num = overlap_layers;

	if (!_requestCondition(overlap_layers)) {
		_request_dvfs_perf(OPPI_PERF);
		dvfs_last_ovl_req = OPPI_PERF;
	} else {
		dvfs_last_ovl_req = OPPI_UNREQ;
	}
	if (disp_helper_get_option(DISP_OPT_SHOW_VISUAL_DEBUG_INFO)) {
		char msg[10];

		snprintf(msg, sizeof(msg), "HRT=%d,", overlap_layers);
		screen_logger_add_message("HRT", MESSAGE_REPLACE, msg);
	}

	if (_should_wait_path_idle())
		dpmgr_wait_event_timeout(disp_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	if (cmdq_handle)
		setup_disp_sec(data_config, cmdq_handle, 1);

    #ifdef VENDOR_EDIT
    /* pdl@oppo.com,2016/04/28 add for debug system hang */
	unsigned long ovl_base0 = ovl_base_addr(DISP_MODULE_OVL0);
	unsigned long ovl_base0_2 = ovl_base_addr(DISP_MODULE_OVL0_2L);
	unsigned long ovl_base1_2 = ovl_base_addr(DISP_MODULE_OVL1_2L);
	cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_status_info_debug,
			0, disp_addr_convert(DISP_REG_OVL_STA + ovl_base0));
	cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_status_info_debug,
			1, disp_addr_convert(DISP_REG_OVL_STA + ovl_base0_2));
	cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_status_info_debug,
			2, disp_addr_convert(DISP_REG_OVL_STA + ovl_base1_2));
    #endif

	/* check bypass ovl */
	bypass = can_bypass_ovl(data_config, &bypass_layer_id);
	/* pdl@oppo.com,2016/01/13 modify from "if (0)" to "if (bypass)", for optimize power*/
	if (bypass) {
		/* switch to rdma mode */
		if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE) {
			do_primary_display_switch_mode(DISP_SESSION_RDMA_MODE, pgc->session_id, 0,
				cmdq_handle, 0);
		}
	} else {
		if (pgc->session_mode == DISP_SESSION_RDMA_MODE) {
			do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0,
				cmdq_handle, 0);
		}
	}


	if (pgc->session_mode != DISP_SESSION_RDMA_MODE) {
		data_config->ovl_dirty = 1;
	} else {
		ret = ddp_convert_ovl_input_to_rdma(&data_config->rdma_config,
							&data_config->ovl_config[bypass_layer_id],
							data_config->dst_w, data_config->dst_h);
		data_config->rdma_dirty = 1;

		/* no need ioctl because of rdma_dirty */
		set_is_dc(1);

		dynamic_debug_msg_print(data_config->rdma_config.address, data_config->rdma_config.width,
				data_config->rdma_config.height, data_config->rdma_config.pitch,
				UFMT_GET_Bpp(data_config->rdma_config.inputFormat));

	}

	if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK)) {
		if (bypass) {
			if (set_one_layer(1))
				; /* mmdvfs_notify_mmclk_switch_request(MMDVFS_EVENT_OVL_SINGLE_LAYER_ENTER); */
		} else {
			if (set_one_layer(0))
				; /* mmdvfs_notify_mmclk_switch_request(MMDVFS_EVENT_OVL_SINGLE_LAYER_EXIT); */
		}
	}

	ret = dpmgr_path_config(disp_handle, data_config, cmdq_handle);

#ifdef DEBUG_OVL_CONFIG_TIME
	cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_config_time, 1, 0x10008028);
#endif
	if (!cmdq_handle)
		goto done;

	/* write fence_id/enable to DRAM using cmdq
	 * it will be used when release fence (put these after config registers done)*/
	for (i = 0; i < cfg->input_layer_num; i++) {
		unsigned int last_fence, cur_fence, sub;
		disp_input_config *input_cfg = &cfg->input_cfg[i];
		layer = input_cfg->layer_id;

		cmdqBackupReadSlot(pgc->cur_config_fence, layer, &last_fence);
		cur_fence = input_cfg->next_buff_idx;

		if (cur_fence != -1 && cur_fence > last_fence)
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer, cur_fence);

		/* for dim_layer/disable_layer/no_fence_layer, just release all fences configured */
		/* for other layers, release current_fence-1 */
		if (input_cfg->buffer_source == DISP_BUFFER_ALPHA
		    || input_cfg->layer_enable == 0 || cur_fence == -1)
			sub = 0;
		else
			sub = 1;

		/* store overlap layer to layer0's subtractor_when_free : bit[31:16] */
		if (layer == 0)
			sub |= overlap_layers << 16;
		cmdqRecBackupUpdateSlot(cmdq_handle, pgc->subtractor_when_free, layer, sub);
	}

	if (primary_display_is_video_mode() && !primary_display_is_decouple_mode()) {
		unsigned long ovl_base = ovl_base_addr(DISP_MODULE_OVL1_2L);
		cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_status_info,
					    0, disp_addr_convert(DISP_REG_OVL_STA + ovl_base));
	}

done:
#ifdef DEBUG_OVL_CONFIG_TIME
	cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_config_time, 2, 0x10008028);
#endif
	return ret;
}

/* notes: primary lock should be held when call this func */
static int primary_frame_cfg_input(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;
	unsigned int wdma_mva = 0;
	disp_path_handle disp_handle;
	cmdqRecHandle cmdq_handle;

	primary_display_idlemgr_kick((char *)__func__, 0);

	if (primary_display_is_decouple_mode()) {
		disp_handle = pgc->ovl2mem_path_handle;
		cmdq_handle = pgc->cmdq_handle_ovl1to2_config;
	} else {
		disp_handle = pgc->dpmgr_handle;
		cmdq_handle = pgc->cmdq_handle_config;
	}

	if (pgc->state == DISP_SLEPT) {
		DISPERR("%s, skip because primary dipslay is slept\n", __func__);

		if (is_DAL_Enabled() &&
			cfg->setter == SESSION_USER_AEE &&
			cfg->input_cfg[0].layer_id == primary_display_get_option("ASSERT_LAYER")) {
			disp_ddp_path_config *data_config = dpmgr_path_get_last_config(disp_handle);
			int layer = cfg->input_cfg[0].layer_id;

			ret = _convert_disp_input_to_ovl(&(data_config->ovl_config[layer]),
					&cfg->input_cfg[0]);
		}

		goto done;
	}

/* yan.chen@swdp.shanghai: add frame cnt variable to summarize all frame updates on MTK platform
 * The function names are difference across MT6752, MT6795 and MT6755.
 * On MT6755, it is primary_frame_cfg_input().
 * On MT6795, it is primary_display_config_input().
 * On MT6752, it is primary_display_config_interface_input()
 * What the hell is MTK doing?
 */
#ifdef VENDOR_EDIT
        pgc->frame_cnt++;
#endif

	fps_ctx_update(fps_get_ctx());

	if (disp_helper_get_option(DISP_OPT_SHOW_VISUAL_DEBUG_INFO))
		primary_show_basic_debug_info(cfg);

	_config_ovl_input(cfg, disp_handle, cmdq_handle);

	if (primary_display_is_decouple_mode() && !primary_display_is_mirror_mode()) {
		pgc->dc_buf_id++;
		pgc->dc_buf_id %= DISP_INTERNAL_BUFFER_COUNT;
		wdma_mva = pgc->dc_buf[pgc->dc_buf_id];
		decouple_wdma_config.dstAddress = wdma_mva;
		_config_wdma_output(&decouple_wdma_config, pgc->ovl2mem_path_handle,
				    pgc->cmdq_handle_ovl1to2_config);
		mem_config.addr = wdma_mva;
		mem_config.buff_idx = -1;
		mem_config.interface_idx = -1;
		mem_config.security = DISP_NORMAL_BUFFER;
		mem_config.pitch = decouple_wdma_config.dstPitch;
		mem_config.fmt = decouple_wdma_config.outputFormat;
		MMProfileLogEx(ddp_mmp_get_events()->primary_wdma_config, MMProfileFlagPulse,
			       pgc->dc_buf_id, wdma_mva);
	}
done:
	return ret;
}

int primary_display_config_input_multiple(disp_session_input_config *session_input)
{
	int ret = 0;
	struct disp_frame_cfg_t *frame_cfg;
	BUG_ON(sizeof(session_input->config) != sizeof(frame_cfg->input_cfg));

	frame_cfg = kzalloc(sizeof(struct disp_frame_cfg_t), GFP_KERNEL);
	if (frame_cfg == NULL) {
		pr_err("error: kzalloc %zu memory fail!\n", sizeof(*frame_cfg));
		return -ENOMEM;
	}

	frame_cfg->session_id = session_input->session_id;
	frame_cfg->setter = session_input->setter;
	frame_cfg->input_layer_num = session_input->config_layer_num;
	frame_cfg->overlap_layer_num = 4;
	memcpy(frame_cfg->input_cfg, session_input->config, sizeof(frame_cfg->input_cfg));

	_primary_path_lock(__func__);

	ret = primary_frame_cfg_input(frame_cfg);

	_primary_path_unlock(__func__);

	kfree(frame_cfg);
	return ret;
}

int primary_display_frame_cfg(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;
	disp_session_sync_info *session_info = disp_get_session_sync_info_for_debug(cfg->session_id);
	dprec_logger_event *input_event, *output_event, *trigger_event;

	if (session_info) {
		input_event = &session_info->event_setinput;
		output_event = &session_info->event_setoutput;
		trigger_event = &session_info->event_trigger;
	} else {
		input_event = output_event = trigger_event = NULL;
	}

	_primary_path_lock(__func__);

	/* set input */
	dprec_start(input_event, cfg->overlap_layer_num, cfg->input_layer_num);
	primary_frame_cfg_input(cfg);
	dprec_done(input_event, 0, 0);

	if (cfg->output_en) {
		dprec_start(output_event, cfg->output_cfg.buff_idx, 0);
		primary_frame_cfg_output(cfg);
		dprec_done(output_event, 0, 0);
	}

	if (trigger_event) {
		/* to debug UI thread or MM thread */
		unsigned int proc_name = (current->comm[0] << 24) |
		    (current->comm[1] << 16) | (current->comm[2] << 8) | (current->comm[3] << 0);
		dprec_start(trigger_event, proc_name, 0);
	}

	if (cfg->present_fence_idx != (unsigned int)-1)
		primary_display_update_present_fence(cfg->present_fence_idx);

	primary_display_trigger_nolock(0, NULL, 0);

	dprec_done(trigger_event, 0, 0);

	_primary_path_unlock(__func__);
	return ret;
}


int primary_display_user_cmd(unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	cmdqRecHandle handle = NULL;
	int cmdqsize = 0;

	MMProfileLogEx(ddp_mmp_get_events()->primary_display_cmd,
		MMProfileFlagStart, (unsigned long)handle, 0);

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		cmdqRecReset(handle);
		_cmdq_insert_wait_frame_done_token_mira(handle);
		cmdqsize = cmdqRecGetInstructionCount(handle);
	}

	if (cmd == DISP_IOCTL_AAL_GET_HIST) {
		_primary_path_lock(__func__);
		if (pgc->state == DISP_SLEPT && handle) {
			cmdqRecDestroy(handle);
			handle = NULL;
		}
		_primary_path_unlock(__func__);

		/* only cmd mode & with disable mmsys clk will kick */
		if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS) &&
			!primary_display_is_video_mode())
			primary_display_idlemgr_kick((char *)__func__, 1);
		ret = dpmgr_path_user_cmd(pgc->dpmgr_handle, cmd, arg, handle);

		if (handle) {
			if (cmdqRecGetInstructionCount(handle) > cmdqsize) {
				_primary_path_lock(__func__);
				if (pgc->state == DISP_ALIVE) {
					/* do not set dirty here, just write register. */
					/* if set dirty needed, will be implemented by dpmgr_module_notify() */
					/* _cmdq_set_config_handle_dirty_mira(handle); */
					/* use non-blocking flush here to avoid primary path is locked for too long */
					_cmdq_flush_config_handle_mira(handle, 0);
				}
				_primary_path_unlock(__func__);
			}

			cmdqRecDestroy(handle);
		}
	} else {
		_primary_path_switch_dst_lock();
		_primary_path_lock(__func__);
		if (pgc->state == DISP_SLEPT && handle) {
			cmdqRecDestroy(handle);
			handle = NULL;
			goto user_cmd_unlock;
		}
		/* only cmd mode & with disable mmsys clk will kick */
		if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS) &&
				!primary_display_is_video_mode())
			primary_display_idlemgr_kick((char *)__func__, 0);
		ret = dpmgr_path_user_cmd(pgc->dpmgr_handle, cmd, arg, handle);

		if (handle) {
			if (cmdqRecGetInstructionCount(handle) > cmdqsize) {
				if (pgc->state == DISP_ALIVE) {
					/* do not set dirty here, just write register. */
					/* if set dirty needed, will be implemented by dpmgr_module_notify() */
					/* _cmdq_set_config_handle_dirty_mira(handle); */
					/* use non-blocking flush here to avoid primary path is locked for too long */
					_cmdq_flush_config_handle_mira(handle, 0);
				}
			}

			cmdqRecDestroy(handle);
		}
user_cmd_unlock:
		_primary_path_unlock(__func__);
		_primary_path_switch_dst_unlock();

	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_display_cmd,
		MMProfileFlagEnd, (unsigned long)handle, cmdqsize);

	return ret;
}

int do_primary_display_switch_mode(int sess_mode, unsigned int session, int need_lock,
					cmdqRecHandle handle, int block)
{
	int ret = 0, sw_only = 0;
	if (need_lock)
		_primary_path_lock(__func__);

	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagStart,
		       pgc->session_mode, sess_mode);

	if (pgc->session_mode == sess_mode)
		goto done;


	if (pgc->state == DISP_SLEPT) {
		DISPERR("primary display switch from %s to %s in suspend state!!!\n",
			p_session_mode_spy(pgc->session_mode), p_session_mode_spy(sess_mode));
		sw_only = 1;
	}

	DISPMSG("primary display will switch from %s to %s\n", p_session_mode_spy(pgc->session_mode),
		p_session_mode_spy(sess_mode));

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
	    && sess_mode == DISP_SESSION_DECOUPLE_MODE) {
		/* dl to dc */
		DL_switch_to_DC_fast(sw_only);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/* dc to dl */
		DC_switch_to_DL_fast(sw_only);
		/* primary_display_diagnose(); */
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) {
		/* dl to dl mirror */
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/*dl mirror to dl */
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE
		   && sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		if (disp_helper_get_option(DISP_OPT_GMO_OPTIMIZE))
			ret = pd_allocate_dc_buffer();
		if (ret)
			goto err;
		/* dl to dc mirror  mirror */
		DL_switch_to_DC_fast(sw_only);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE
		   && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		/*dc mirror  to dl */
		DC_switch_to_DL_fast(sw_only);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE &&
			sess_mode == DISP_SESSION_DECOUPLE_MODE){
		/* do nothing */
		/* just switch mode */
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE &&
			sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE){
		/* do nothing */
		/* just switch mode */
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE && sess_mode == DISP_SESSION_RDMA_MODE) {
		ret = DL_switch_to_rdma_mode(handle, block);
		if (ret)
			goto err;
	} else if (pgc->session_mode == DISP_SESSION_RDMA_MODE && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		ret = rdma_mode_switch_to_DL(handle, block);
		if (ret)
			goto err;
	} else if (pgc->session_mode == DISP_SESSION_RDMA_MODE && sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		/* switch to DL mode first */
		ret = rdma_mode_switch_to_DL(NULL, 0);
		if (ret)
			goto err;
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, pgc->session_mode, 0);
		DL_switch_to_DC_fast(0);
	} else if (pgc->session_mode == DISP_SESSION_RDMA_MODE && sess_mode == DISP_SESSION_DECOUPLE_MODE) {
		/* switch to DL mode first */
		ret = rdma_mode_switch_to_DL(NULL, 0);
		if (ret)
			goto err;
		MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, pgc->session_mode, 0);
		DL_switch_to_DC_fast(0);
	} else {
		DISPERR("invalid mode switch from %s to %s\n", p_session_mode_spy(pgc->session_mode),
			p_session_mode_spy(sess_mode));
		BUG();
	}
done:
	MMProfileLogEx(ddp_mmp_get_events()->primary_mode[pgc->session_mode],
				MMProfileFlagEnd, pgc->session_mode, sess_mode);
	MMProfileLogEx(ddp_mmp_get_events()->primary_mode[sess_mode],
				MMProfileFlagStart, pgc->session_mode, sess_mode);

	pgc->session_mode = sess_mode;
	DISPMSG("primary display is %s mode now\n", p_session_mode_spy(pgc->session_mode));
	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagPulse, pgc->session_mode, sess_mode);
	pgc->session_id = session;
	screen_logger_add_message("sess_mode", MESSAGE_REPLACE, (char *)p_session_mode_spy(sess_mode));
err:
	MMProfileLogEx(ddp_mmp_get_events()->primary_switch_mode, MMProfileFlagEnd, pgc->session_mode, sess_mode);

	if (need_lock)
		_primary_path_unlock(__func__);
	pgc->session_id = session;
	return ret;
}

int primary_display_switch_mode(int sess_mode, unsigned int session, int force)
{
	int ret = 0;

	_primary_path_lock(__func__);
	if (bypass_blank && !force) {
		DISPCHECK("%s DISPLAY UT Return!\n", __func__);
		goto done;
	}

	primary_display_idlemgr_kick((char *)__func__, 0);

	if (!force && primary_display_is_mirror_mode() == _is_mirror_mode(sess_mode)) {
		/* HWC only needs to control mirror/not mirror
		 * it doesn't need to control DL/DC */
		 goto done;
	}

	if (pgc->session_mode == sess_mode)
		goto done;

	while (primary_get_state() == DISP_BLANK) {
		_primary_path_unlock(__func__);
		DISPMSG("%s wait for leave TUI\n", __func__);
		primary_display_wait_not_state(DISP_BLANK, MAX_SCHEDULE_TIMEOUT);
		_primary_path_lock(__func__);
	}

	ret = do_primary_display_switch_mode(sess_mode, session, 0, NULL, 0);

done:
	_primary_path_unlock(__func__);

	return ret;
}

int primary_display_is_alive(void)
{
	unsigned int temp = 0;
	/* DISPFUNC(); */
	_primary_path_lock(__func__);

	if (pgc->state == DISP_ALIVE)
		temp = 1;


	_primary_path_unlock(__func__);

	return temp;
}

int primary_display_is_sleepd(void)
{
	unsigned int temp = 0;
	/* DISPFUNC(); */
	_primary_path_lock(__func__);

	if (pgc->state == DISP_SLEPT)
		temp = 1;


	_primary_path_unlock(__func__);

	return temp;
}

int primary_display_get_width(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->params->width;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

int primary_display_get_height(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->params->height;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

int primary_display_get_virtual_width(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->params->virtual_width;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

int primary_display_get_virtual_height(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->params->virtual_height;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}
int primary_display_get_original_width(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->lcm_original_width;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

int primary_display_get_original_height(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->lcm_original_height;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

int primary_display_get_bpp(void)
{
	return 32;
}

void primary_display_set_max_layer(int maxlayer)
{
	pgc->max_layer = maxlayer;
}

int primary_display_get_max_layer(void)
{
	return pgc->max_layer;
}

int primary_display_get_info(disp_session_info *info)
{
	disp_session_info *dispif_info = (disp_session_info *) info;

	LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);
	if (lcm_param == NULL) {
		DISPERR("lcm_param is null\n");
		return -1;
	}

	memset((void *)dispif_info, 0, sizeof(disp_session_info));

	if (is_DAL_Enabled())
		dispif_info->maxLayerNum = pgc->max_layer - 1;
	else
		dispif_info->maxLayerNum = pgc->max_layer;

	dispif_info->const_layer_num = 0;

	switch (lcm_param->type) {
	case LCM_TYPE_DBI:
		{
			dispif_info->displayType = DISP_IF_TYPE_DBI;
			dispif_info->displayMode = DISP_IF_MODE_COMMAND;
			dispif_info->isHwVsyncAvailable = 1;
			/* DISPMSG("DISP Info: DBI, CMD Mode, HW Vsync enable\n"); */
			break;
		}
	case LCM_TYPE_DPI:
		{
			dispif_info->displayType = DISP_IF_TYPE_DPI;
			dispif_info->displayMode = DISP_IF_MODE_VIDEO;
			dispif_info->isHwVsyncAvailable = 1;
			/* DISPMSG("DISP Info: DPI, VDO Mode, HW Vsync enable\n"); */
			break;
		}
	case LCM_TYPE_DSI:
		{
			dispif_info->displayType = DISP_IF_TYPE_DSI0;
			if (lcm_param->dsi.mode == CMD_MODE) {
				dispif_info->displayMode = DISP_IF_MODE_COMMAND;
				dispif_info->isHwVsyncAvailable = 1;
				/* DISPMSG("DISP Info: DSI, CMD Mode, HW Vsync enable\n"); */
			} else {
				dispif_info->displayMode = DISP_IF_MODE_VIDEO;
				dispif_info->isHwVsyncAvailable = 1;
				/* DISPMSG("DISP Info: DSI, VDO Mode, HW Vsync enable\n"); */
			}

			break;
		}
	default:
		break;
	}


	dispif_info->displayFormat = DISP_IF_FORMAT_RGB888;

	dispif_info->displayWidth = primary_display_get_width();
	dispif_info->displayHeight = primary_display_get_height();

	dispif_info->physicalWidth = DISP_GetActiveWidth();
	dispif_info->physicalHeight = DISP_GetActiveHeight();

	dispif_info->vsyncFPS = pgc->lcm_fps;
	dispif_info->isConnected = 1;

	fps_ctx_get_fps(fps_get_ctx(), &dispif_info->updateFPS, &dispif_info->is_updateFPS_stable);
	dispif_info->updateFPS *= 100;

	return 0;
}

int primary_display_get_pages(void)
{
	return 3;
}

int primary_display_is_video_mode(void)
{
	/* TODO: we should store the video/cmd mode in runtime, because ROME will support cmd/vdo dynamic switch */
	return disp_lcm_is_video_mode(pgc->plcm);
}

int primary_display_diagnose(void)
{
	int ret = 0;
	DISPMSG("==== %s ===>\n", __func__);

	dpmgr_check_status(pgc->dpmgr_handle);

	if (primary_display_is_decouple_mode()) {
		if (pgc->ovl2mem_path_handle)
			dpmgr_check_status(pgc->ovl2mem_path_handle);
	}
	DISPMSG("==== %s ===<\n", __func__);
	return ret;
}

CMDQ_SWITCH primary_display_cmdq_enabled(void)
{
	return disp_helper_get_option(DISP_OPT_USE_CMDQ);
}

int primary_display_manual_lock(void)
{
	_primary_path_lock(__func__);
	return 0;
}

int primary_display_manual_unlock(void)
{
	_primary_path_unlock(__func__);
	return 0;
}

void primary_display_reset(void)
{
	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
}

unsigned int primary_display_get_fps_nolock(void)
{
	return pgc->lcm_fps;
}
unsigned int primary_display_get_fps(void)
{
	unsigned int fps = 0;
	_primary_path_lock(__func__);
	fps = pgc->lcm_fps;
	_primary_path_unlock(__func__);

	return fps;
}

int primary_display_force_set_fps(unsigned int keep, unsigned int skip)
{
	int ret = 0;
	DISPMSG("force set fps to keep %d, skip %d\n", keep, skip);
	_primary_path_lock(__func__);

	pgc->force_fps_keep_count = keep;
	pgc->force_fps_skip_count = skip;

	g_keep = 0;
	g_skip = 0;
	_primary_path_unlock(__func__);

	return ret;
}

int primary_display_force_set_vsync_fps(unsigned int fps)
{
	int ret = 0;
	DISPMSG("force set fps to %d\n", fps);
	_primary_path_lock(__func__);

	if (fps == pgc->lcm_fps) {
		pgc->vsync_drop = 0;
		ret = 0;
	} else if (fps == 30) {
		pgc->vsync_drop = 1;
		ret = 0;
	} else {
		ret = -1;
	}

	_primary_path_unlock(__func__);

	return ret;
}

int primary_display_vsync_switch(int method)
{
	int ret = 0;
	if (method == 0) {
		pr_debug("Vsync map RDMA %d\n", method);
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_RDMA0_DONE);
	} else if (method == 1) {
		pr_debug("Vsync map DSI TE %d\n", method);
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_DSI0_EXT_TE);
	} else if (method == 2) {
		pr_debug("Vsync map DSI FRAME DONE %d\n", method);
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_DSI0_FRAME_DONE);
	}

	return ret;
}

int _set_backlight_by_cmdq(unsigned int level)
{
	int ret = 0;

	cmdqRecHandle cmdq_handle_backlight = NULL;

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 1);
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_backlight);
	DISPDBG("primary backlight, handle=%p\n", cmdq_handle_backlight);
	if (ret != 0) {
		DISPERR("fail to create primary cmdq handle for backlight\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_backlight);
		disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		_cmdq_flush_config_handle_mira(cmdq_handle_backlight, 1);
		DISPCHECK("[BL]_set_backlight_by_cmdq ret=%d\n", ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_backlight);
		cmdqRecWait(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_handle_clear_dirty(cmdq_handle_backlight);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_backlight);
		disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		cmdqRecSetEventToken(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CABC_EOF);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_backlight, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[BL]_set_backlight_by_cmdq ret=%d\n", ret);
	}
	cmdqRecDestroy(cmdq_handle_backlight);
	cmdq_handle_backlight = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 5);

	return ret;
}

int _set_backlight_by_cpu(unsigned int level)
{
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 1);
	if (primary_display_is_video_mode()) {
		disp_lcm_set_backlight(pgc->plcm, NULL, level);
	} else {
		DISPCHECK("[BL]display cmdq trigger loop stop[begin]\n");
		if (primary_display_cmdq_enabled())
			_cmdq_stop_trigger_loop();

		DISPCHECK("[BL]display cmdq trigger loop stop[end]\n");

		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			DISPCHECK("[BL]primary display path is busy\n");
			ret =
			    dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						     HZ * 1);
			DISPCHECK("[BL]wait frame done ret:%d\n", ret);
		}

		DISPCHECK("[BL]stop dpmgr path[begin]\n");
		dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[BL]stop dpmgr path[end]\n");
		if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
			DISPCHECK("[BL]primary display path is busy after stop\n");
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE,
						 HZ * 1);
			DISPCHECK("[BL]wait frame done ret:%d\n", ret);
		}
		DISPCHECK("[BL]reset display path[begin]\n");
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[BL]reset display path[end]\n");

		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 2);

		disp_lcm_set_backlight(pgc->plcm, NULL, level);

		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 3);

		DISPCHECK("[BL]start dpmgr path[begin]\n");
		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
		DISPCHECK("[BL]start dpmgr path[end]\n");

		if (primary_display_cmdq_enabled()) {
			DISPCHECK("[BL]start cmdq trigger loop[begin]\n");
			_cmdq_start_trigger_loop();
		}
		DISPCHECK("[BL]start cmdq trigger loop[end]\n");
	}
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 0, 7);
	return ret;
}

int primary_display_setbacklight(unsigned int level)
{
	int ret = 0;

	DISPFUNC();

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagStart, 0, 0);
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
#endif
	if (pgc->state == DISP_SLEPT) {
		DISPERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick((char *)__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl,
					       MMProfileFlagPulse, 0, 7);
				disp_lcm_set_backlight(pgc->plcm, NULL, level);
			} else {
				_set_backlight_by_cmdq(level);
			}
		} else {
			_set_backlight_by_cpu(level);
		}
	}
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagEnd, 0, 0);
	return ret;
}

int _set_lcm_cmd_by_cmdq(unsigned int *lcm_cmd, unsigned int *lcm_count, unsigned int *lcm_value)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle_lcm_cmd = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 1);
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_lcm_cmd);
	DISPDBG("primary set lcm cmd, handle=%p\n", cmdq_handle_lcm_cmd);
	if (ret != 0) {
		DISPERR("fail to create primary cmdq handle for setlcmcmd\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_lcm_cmd);
		disp_lcm_set_lcm_cmd(pgc->plcm, cmdq_handle_lcm_cmd, lcm_cmd, lcm_count, lcm_value);
		_cmdq_flush_config_handle_mira(cmdq_handle_lcm_cmd, 1);
		DISPCHECK("[CMD]_set_lcm_cmd_by_cmdq ret=%d\n", ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_lcm_cmd);
		_cmdq_handle_clear_dirty(cmdq_handle_lcm_cmd);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_lcm_cmd);

		disp_lcm_set_lcm_cmd(pgc->plcm, cmdq_handle_lcm_cmd, lcm_cmd, lcm_count, lcm_value);
		cmdqRecSetEventToken(cmdq_handle_lcm_cmd, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_lcm_cmd, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[CMD]_set_lcm_cmd_by_cmdq ret=%d\n", ret);
	}
	cmdqRecDestroy(cmdq_handle_lcm_cmd);
	cmdq_handle_lcm_cmd = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 5);
	return ret;

}

#ifdef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Multimedia, 2015/05/26  Add for S6E3FAE's HBM mode */
#if defined(S6E3FA3_FHD_DSI_CMD)
extern int lcm_set_HBM_mode(void *handle,int mode);

int _set_HBM_mode_by_cmdq(unsigned int level)
{
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 1);
	int ret = 0;

	cmdqRecHandle cmdq_handle_HBM_mode = NULL;
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP,&cmdq_handle_HBM_mode);
	//DISPCHECK("primary HBM mode, handle=0x%08x\n", cmdq_handle_HBM_mode);
	if(ret!=0)
	{
		DISPCHECK("fail to create primary cmdq handle for HBM mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_HBM_mode);
		//disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		if((!is_project(OPPO_16031))&&(!is_project(OPPO_16034))){
		ret = lcm_set_HBM_mode(cmdq_handle_HBM_mode,level);
		}
		_cmdq_flush_config_handle_mira(cmdq_handle_HBM_mode, 1);
		DISPCHECK("[BL]_set_HBM_mode_by_cmdq ret=%d\n",ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_HBM_mode);
		cmdqRecWait(cmdq_handle_HBM_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_handle_clear_dirty(cmdq_handle_HBM_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_HBM_mode);
		//disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		if((!is_project(OPPO_16031))&&(!is_project(OPPO_16034))){
		ret = lcm_set_HBM_mode(cmdq_handle_HBM_mode,level);
		}
		cmdqRecSetEventToken(cmdq_handle_HBM_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_HBM_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_HBM_mode, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[BL]_set_HBM_mode_by_cmdq ret=%d\n",ret);
	}
	cmdqRecDestroy(cmdq_handle_HBM_mode);
	cmdq_handle_HBM_mode = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 5);

	return ret;
}

int primary_display_set_HBM_mode(unsigned int level)
{
	DISPFUNC();
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagStart, 0, 0);
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
#endif
	if (pgc->state == DISP_SLEPT) {
		DISPERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl,
					       MMProfileFlagPulse, 0, 7);
				//disp_lcm_set_backlight(pgc->plcm, NULL, level);
			} else {
				//_set_backlight_by_cmdq(level);
				_set_HBM_mode_by_cmdq(level);
			}
		} else {
			//_set_backlight_by_cpu(level);
		}
	}
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagEnd, 0, 0);
	return ret;
}
#endif
#if defined(OPPO_JDI_R63452_FHD1080_DSI_CMD)
extern int parse_input(void *handle,const char *buf);
extern int parse_reg_output(void *handle,const char *buf);


int _set_JDI_reg_by_cmdq(const char *buf)
{
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 1);
	int ret = 0;

	cmdqRecHandle cmdq_handle_reg_mode = NULL;
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP,&cmdq_handle_reg_mode);
	//DISPCHECK("primary HBM mode, handle=0x%08x\n", cmdq_handle_HBM_mode);
	if(ret!=0)
	{
		DISPCHECK("fail to create primary cmdq handle for HBM mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_reg_mode);
		//disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		if(is_project(OPPO_16031)||is_project(OPPO_16034)){
		ret = parse_input(cmdq_handle_reg_mode,buf);
		}
		_cmdq_flush_config_handle_mira(cmdq_handle_reg_mode, 1);
		DISPCHECK("[BL]_set_HBM_mode_by_cmdq ret=%d\n",ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_reg_mode);
		cmdqRecWait(cmdq_handle_reg_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_handle_clear_dirty(cmdq_handle_reg_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_reg_mode);
		//disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		if(is_project(OPPO_16031)||is_project(OPPO_16034)){
		ret = parse_input(cmdq_handle_reg_mode,buf);
		}
		cmdqRecSetEventToken(cmdq_handle_reg_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_reg_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_reg_mode, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[BL]_set_JDI_reg_by_cmdq ret=%d\n",ret);
	}
	cmdqRecDestroy(cmdq_handle_reg_mode);
	cmdq_handle_reg_mode = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 5);

	return ret;
}

int primary_display_set_jdi_reg(const char *buf)
{
	DISPFUNC();
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagStart, 0, 0);
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
#endif
	if (pgc->state == DISP_SLEPT) {
		DISPERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl,
					       MMProfileFlagPulse, 0, 7);
				//disp_lcm_set_backlight(pgc->plcm, NULL, level);
			} else {
				//_set_backlight_by_cmdq(level);
				_set_JDI_reg_by_cmdq(buf);
			}
		} else {
			//_set_backlight_by_cpu(level);
		}
	}
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagEnd, 0, 0);
	return ret;
}

int _read_JDI_reg_by_cmdq(const char *buf)
{
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 1);
	int ret = 0;

	cmdqRecHandle cmdq_handle_read_reg= NULL;
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP,&cmdq_handle_read_reg);
	//DISPCHECK("primary HBM mode, handle=0x%08x\n", cmdq_handle_HBM_mode);
	if(ret!=0)
	{
		DISPCHECK("fail to create primary cmdq handle for HBM mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_read_reg);
		//disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		if(is_project(OPPO_16031)||is_project(OPPO_16034)){
		ret = parse_reg_output(cmdq_handle_read_reg,buf);
		}
		_cmdq_flush_config_handle_mira(cmdq_handle_read_reg, 1);
		DISPCHECK("[BL]_set_HBM_mode_by_cmdq ret=%d\n",ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_read_reg);
		cmdqRecWait(cmdq_handle_read_reg, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_handle_clear_dirty(cmdq_handle_read_reg);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_read_reg);
		//disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		if(is_project(OPPO_16031)||is_project(OPPO_16034)){
		ret = parse_input(cmdq_handle_read_reg,buf);
		}
		cmdqRecSetEventToken(cmdq_handle_read_reg, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_read_reg, CMDQ_SYNC_TOKEN_CABC_EOF);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_read_reg, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[BL]_set_JDI_reg_by_cmdq ret=%d\n",ret);
	}
	cmdqRecDestroy(cmdq_handle_read_reg);
	cmdq_handle_read_reg = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 5);

	return ret;
}

int primary_display_read_jdi_reg(const char *buf)
{
	DISPFUNC();
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagStart, 0, 0);
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
#endif
	if (pgc->state == DISP_SLEPT) {
		DISPERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl,
					       MMProfileFlagPulse, 0, 7);
				//disp_lcm_set_backlight(pgc->plcm, NULL, level);
			} else {
				//_set_backlight_by_cmdq(level);
				_read_JDI_reg_by_cmdq(buf);
			}
		} else {
			//_set_backlight_by_cpu(level);
		}
	}
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagEnd, 0, 0);
	return ret;
}

#endif

#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
/* liping-m@PhoneSW.Multimedia, 2016/02/29  Add for BOE's CABC mode */
#if defined(ILI9881C_FHD720_DSI_VDO_BOE)
extern unsigned int oppo_lcm_is_which;
extern int boe_lcm_set_cabc_mode(void *handle,unsigned int level);
extern int truly_lcm_set_cabc_mode(void *handle,unsigned int level);
extern int tianma_lcm_set_cabc_mode(void *handle,unsigned int level);
#endif

#if defined(OPPO_HX8394_TRULY_HD720_DSI_VDO)  || defined(OPPO16021_HX8394_TRULY_HD720_DSI_VDO)
extern int truly_himax_set_cabc_mode(void *handle,unsigned int level);
extern int truly_g5d_himax_set_cabc_mode(void *handle,unsigned int level);
extern int truly_g5d_nt35521s_set_cabc_mode(void *handle,unsigned int level);
extern int tianma_himax_set_cabc_mode(void *handle,unsigned int level);
extern int truly_cpt_himax_set_cabc_mode(void *handle,unsigned int level);
extern int boe_nt35521s_set_cabc_mode(void *handle,unsigned int level);
extern int boe_lcm_set_cabc_mode(void *handle,unsigned int level);
#endif
#if defined(OPPO_JDI_R63452_FHD1080_DSI_CMD)
extern int jdi_lcm_set_cabc_mode(void *handle,int mode);
#endif
int _set_cabc_mode_by_cmdq(unsigned int level)
{
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagPulse, 1, 1);
	int ret = 0;

	cmdqRecHandle cmdq_handle_CABC_mode = NULL;
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP,&cmdq_handle_CABC_mode);
	if(ret!=0)
	{
		DISPCHECK("fail to create primary cmdq handle for HBM mode\n");
		return -1;
	}

	if (primary_display_is_video_mode()) {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 2);
		cmdqRecReset(cmdq_handle_CABC_mode);
		#if defined(ILI9881C_FHD720_DSI_VDO_BOE)
		if(oppo_lcm_is_which==1){
			ret = boe_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		}else if(oppo_lcm_is_which==2){
			ret = truly_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		}else if(oppo_lcm_is_which==3){
			ret = tianma_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		}
		#endif
		#if defined(OPPO_JDI_R63452_FHD1080_DSI_CMD)
		if(!strcmp(pgc->plcm->drv->name, "oppo_jdi_r63452_fhd1080_dsi_cmd")){
			ret = jdi_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		 }
		#endif
		_cmdq_flush_config_handle_mira(cmdq_handle_CABC_mode, 1);
		DISPCHECK("[CABC]_set_cabc_mode_by_cmdq ret=%d\n",ret);
	} else {
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 3);
		cmdqRecReset(cmdq_handle_CABC_mode);
		cmdqRecWait(cmdq_handle_CABC_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_handle_clear_dirty(cmdq_handle_CABC_mode);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_CABC_mode);
		#if defined(ILI9881C_FHD720_DSI_VDO_BOE)
		if(oppo_lcm_is_which==1){
			ret = boe_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		}else if(oppo_lcm_is_which==2){
			ret = truly_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		}else if(oppo_lcm_is_which==3){
			ret = tianma_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		}
		#endif

		#if defined(OPPO_JDI_R63452_FHD1080_DSI_CMD)
		if(!strcmp(pgc->plcm->drv->name, "oppo_jdi_r63452_fhd1080_dsi_cmd")){
			ret = jdi_lcm_set_cabc_mode(cmdq_handle_CABC_mode,level);
		 }
		#endif
		cmdqRecSetEventToken(cmdq_handle_CABC_mode, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_CABC_mode, CMDQ_SYNC_TOKEN_CABC_EOF);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 4);
		_cmdq_flush_config_handle_mira(cmdq_handle_CABC_mode, 1);
		MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 6);
		DISPCHECK("[cabc]_set_cabc_mode_by_cmdq ret=%d\n",ret);
	}
	cmdqRecDestroy(cmdq_handle_CABC_mode);
	cmdq_handle_CABC_mode = NULL;
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagPulse, 1, 5);

	return ret;
}

int primary_display_set_cabc_mode(unsigned int level)
{
	DISPFUNC();
	int ret = 0;

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagStart, 0, 0);
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);
#endif
	if (pgc->state == DISP_SLEPT) {
		DISPERR("Sleep State set backlight invald\n");
	} else {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl,
					       MMProfileFlagPulse, 0, 7);
				//disp_lcm_set_backlight(pgc->plcm, NULL, level);
				#if defined(ILI9881C_FHD720_DSI_VDO_BOE)
				if(oppo_lcm_is_which==1){
					ret = boe_lcm_set_cabc_mode(NULL,level);
				}else if(oppo_lcm_is_which==2){
					ret = truly_lcm_set_cabc_mode(NULL,level);
				}else if(oppo_lcm_is_which==3){
					ret = tianma_lcm_set_cabc_mode(NULL,level);
				}
				#endif
				#if defined(OPPO_HX8394_TRULY_HD720_DSI_VDO)
				if(!strcmp(pgc->plcm->drv->name, "oppo_hx8394_truly_hd720_dsi_vdo")){
				    ret = truly_himax_set_cabc_mode(NULL,level);
				    DISPERR("set truly cabc level %d\n",level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo_hx8394_tianma_hd720_dsi_vdo")){
				    ret = tianma_himax_set_cabc_mode(NULL,level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo_hx8394_truly_g5d_hd720_dsi_vdo")){
				    ret = truly_g5d_himax_set_cabc_mode(NULL,level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo_nt35521s_truly_g5d_hd720_dsi_vdo")){
				    ret = truly_g5d_nt35521s_set_cabc_mode(NULL,level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo_hx8394_truly_cpt_hd720_dsi_vdo")){
				    ret = truly_cpt_himax_set_cabc_mode(NULL,level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo_nt35521s_boe_hd720_dsi_vdo")){
				    ret = boe_nt35521s_set_cabc_mode(NULL,level);
				}
				#endif
				#if defined(OPPO16021_HX8394_TRULY_HD720_DSI_VDO)
				if(!strcmp(pgc->plcm->drv->name, "oppo16021_hx8394_truly_hd720_dsi_vdo")){
				    ret = truly_himax_set_cabc_mode(NULL,level);
				    DISPERR("set truly cabc level %d\n",level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo16021_hx8394_boe_hd720_dsi_vdo")){
				    ret = boe_lcm_set_cabc_mode(NULL,level);
				}else if(!strcmp(pgc->plcm->drv->name, "oppo16021_hx8394_tianma_hd720_dsi_vdo")){
				    ret = tianma_himax_set_cabc_mode(NULL,level);
				}
				#endif
			} else {
				//_set_backlight_by_cmdq(level);
				_set_cabc_mode_by_cmdq(level);
			}
		} else {
			//_set_backlight_by_cpu(level);
		}
	}
#ifndef CONFIG_MTK_AAL_SUPPORT
	_primary_path_unlock(__func__);
	_primary_path_switch_dst_unlock();
#endif
	MMProfileLogEx(ddp_mmp_get_events()->primary_set_bl, MMProfileFlagEnd, 0, 0);
	return ret;
}
#endif /*VENDOR_EDIT*/
int primary_display_setlcm_cmd(unsigned int *lcm_cmd, unsigned int *lcm_count,
			       unsigned int *lcm_value)
{
	int ret = 0;

	DISPFUNC();
	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) {
		DISPMSG("%s skip due to stage %s\n", __func__, disp_helper_stage_spy());
		return 0;
	}

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagStart, 0, 0);

	_primary_path_switch_dst_lock();

	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Sleep State set backlight invald\n");
	} else {
		if (primary_display_cmdq_enabled()) {
			if (primary_display_is_video_mode()) {
				MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd,
					       MMProfileFlagPulse, 0, 7);
				_set_lcm_cmd_by_cmdq(lcm_cmd, lcm_count, lcm_value);
			} else {
				_set_lcm_cmd_by_cmdq(lcm_cmd, lcm_count, lcm_value);
			}
		} else {
			/* cpu */
		}
	}
	_primary_path_unlock(__func__);

	_primary_path_switch_dst_lock();

	MMProfileLogEx(ddp_mmp_get_events()->primary_set_cmd, MMProfileFlagEnd, 0, 0);

	return ret;
}

int primary_display_mipi_clk_change(unsigned int clk_value)
{
	cmdqRecHandle cmdq_handle = NULL;

	if (pgc->state == DISP_SLEPT) {
		DISPCHECK("Sleep State clk change invald\n");
		return 0;
	}

	_primary_path_lock(__func__);

	if (!primary_display_is_video_mode()) {
		DISPCHECK("clk change CMD Mode return\n");
		return 0;
	}

	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
	cmdqRecReset(cmdq_handle);

	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
	pgc->plcm->params->dsi.PLL_CLOCK = clk_value;

	dpmgr_path_build_cmdq(pgc->dpmgr_handle,
			cmdq_handle, CMDQ_STOP_VDO_MODE, 0);

	dpmgr_path_ioctl(primary_get_dpmgr_handle(), cmdq_handle,
		DDP_PHY_CLK_CHANGE,
		&clk_value);

	dpmgr_path_build_cmdq(pgc->dpmgr_handle,
		cmdq_handle, CMDQ_START_VDO_MODE, 0);

	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_RDMA0_EOF);

	dpmgr_path_trigger(pgc->dpmgr_handle, cmdq_handle, CMDQ_ENABLE);
	ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), pgc->cmdq_handle_config_esd, 0);
	_cmdq_flush_config_handle_mira(cmdq_handle, 1);

	cmdqRecDestroy(cmdq_handle);
	cmdq_handle = NULL;

	DISPCHECK("primary_display_mipi_clk_change return\n");

	_primary_path_unlock(__func__);

	return 0;
}

/***********************/
/*****Legacy DISP API*****/
/***********************/
uint32_t DISP_GetScreenWidth(void)
{
	return primary_display_get_width();
}

uint32_t DISP_GetScreenHeight(void)
{
	return primary_display_get_height();
}

uint32_t DISP_GetActiveHeight(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->params->physical_height;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

uint32_t DISP_GetActiveWidth(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return 0;
	}

	if (pgc->plcm->params) {
		return pgc->plcm->params->physical_width;
	} else {
		DISPERR("lcm_params is null!\n");
		return 0;
	}
}

LCM_PARAMS *DISP_GetLcmPara(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return NULL;
	}

	if (pgc->plcm->params)
		return pgc->plcm->params;
	else
		return NULL;
}

LCM_DRIVER *DISP_GetLcmDrv(void)
{
	if (pgc->plcm == NULL) {
		DISPERR("lcm handle is null\n");
		return NULL;
	}

	if (pgc->plcm->drv)
		return pgc->plcm->drv;
	else
		return NULL;
}

static int _screen_cap_by_cmdq(unsigned int mva, enum UNIFIED_COLOR_FMT ufmt, DISP_MODULE_ENUM after_eng)
{
	int ret = 0;
	cmdqRecHandle cmdq_handle = NULL;
	cmdqRecHandle cmdq_wait_handle = NULL;
	disp_ddp_path_config *pconfig = NULL;
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();

	/*create config thread */
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
	if (ret != 0) {
		DISPCHECK("primary capture:Fail to create primary cmdq handle for capture\n");
		ret = -1;
		goto out;
	}
	cmdqRecReset(cmdq_handle);

	/*create wait thread */
	ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);
	if (ret != 0) {
		DISPCHECK
		    ("primary capture:Fail to create primary cmdq wait handle for capture\n");
		ret = -1;
		goto out;
	}
	cmdqRecReset(cmdq_wait_handle);

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 1);

	_cmdq_handle_clear_dirty(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	_primary_path_lock(__func__);

	primary_display_idlemgr_kick(__func__, 0);
	dpmgr_path_add_memout(pgc->dpmgr_handle, after_eng, cmdq_handle);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->wdma_dirty = 1;
	pconfig->wdma_config.dstAddress = mva;
	pconfig->wdma_config.srcHeight = h_yres;
	pconfig->wdma_config.srcWidth = w_xres;
	pconfig->wdma_config.clipX = 0;
	pconfig->wdma_config.clipY = 0;
	pconfig->wdma_config.clipHeight = h_yres;
	pconfig->wdma_config.clipWidth = w_xres;
	pconfig->wdma_config.outputFormat = ufmt;
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch = w_xres * UFMT_GET_bpp(ufmt) / 8;
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);
	pconfig->wdma_dirty = 0;

	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);
	DISPMSG("primary capture:Flush add memout mva(0x%x)\n", mva);
	/*wait wdma0 sof */
	cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	cmdqRecFlush(cmdq_wait_handle);
	DISPMSG("primary capture:Flush wait wdma sof\n");
	cmdqRecReset(cmdq_handle);
	_cmdq_handle_clear_dirty(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);

	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	/* flush remove memory to cmdq */
	_cmdq_flush_config_handle_mira(cmdq_handle, 1);
	DISPMSG("primary capture: Flush remove memout\n");

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);
	_primary_path_unlock(__func__);

out:
	cmdqRecDestroy(cmdq_handle);
	cmdqRecDestroy(cmdq_wait_handle);
	return 0;
}

static int _screen_cap_by_cpu(unsigned int mva, enum UNIFIED_COLOR_FMT ufmt, DISP_MODULE_ENUM after_eng)
{
	int ret = 0;
	disp_ddp_path_config *pconfig = NULL;
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 1);

	if (_should_wait_path_idle()) {
		ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		if (ret <= 0)
			primary_display_diagnose();
	}

	_primary_path_lock(__func__);
	primary_display_idlemgr_kick(__func__, 1);

	dpmgr_path_add_memout(pgc->dpmgr_handle, after_eng, NULL);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->wdma_dirty = 1;
	pconfig->wdma_config.dstAddress = mva;
	pconfig->wdma_config.srcHeight = h_yres;
	pconfig->wdma_config.srcWidth = w_xres;
	pconfig->wdma_config.clipX = 0;
	pconfig->wdma_config.clipY = 0;
	pconfig->wdma_config.clipHeight = h_yres;
	pconfig->wdma_config.clipWidth = w_xres;
	pconfig->wdma_config.outputFormat = ufmt;
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch = w_xres * UFMT_GET_bpp(ufmt) / 8;
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, NULL);
	pconfig->wdma_dirty = 0;

	_trigger_display_interface(1, NULL, 0);
	msleep(20);
	if (_should_wait_path_idle()) {
		ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
		if (ret <= 0)
			primary_display_diagnose();
	}

	dpmgr_path_remove_memout(pgc->dpmgr_handle, NULL);

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);
	_primary_path_unlock(__func__);
	return 0;
}

int primary_display_capture_framebuffer_ovl(unsigned long pbuf, enum UNIFIED_COLOR_FMT ufmt)
{

	int ret = 0;

	m4u_client_t *m4uClient = NULL;
	unsigned int mva = 0;
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();
	unsigned int pixel_byte = primary_display_get_bpp() / 8;
	int tmp;
	int buffer_size = h_yres * w_xres * pixel_byte;
	DISP_MODULE_ENUM after_eng = DISP_MODULE_OVL0;
	DISPMSG("primary capture: begin\n");

	disp_sw_mutex_lock(&(pgc->capture_lock));

	if (primary_display_is_sleepd()) {
		memset((void *)pbuf, 0, buffer_size);
		DISPMSG("primary capture: Fail black End\n");
		goto out;
	}

	m4uClient = m4u_create_client();
	if (m4uClient == NULL) {
		DISPERR("primary capture:Fail to alloc  m4uClient\n");
		ret = -1;
		goto out;
	}

	ret = m4u_alloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, pbuf, NULL, buffer_size,
			  M4U_PROT_READ | M4U_PROT_WRITE, 0, &mva);
	if (ret != 0) {
		DISPERR("primary capture:Fail to allocate mva\n");
		ret = -1;
		goto out;
	}

	ret = m4u_cache_sync(m4uClient, M4U_PORT_DISP_WDMA0, pbuf, buffer_size, mva,
			   M4U_CACHE_FLUSH_ALL);
	if (ret != 0) {
		DISPERR("primary capture:Fail to cach sync\n");
		ret = -1;
		goto out;
	}
	tmp = disp_helper_get_option(DISP_OPT_SCREEN_CAP_FROM_DITHER);
	if (tmp == 0)
		after_eng = DISP_MODULE_OVL0;
	else if (tmp == 1)
		after_eng = DISP_MODULE_DITHER;
	else if (tmp == 2)
		after_eng = DISP_MODULE_UFOE;

	if (primary_display_cmdq_enabled())
		_screen_cap_by_cmdq(mva, ufmt, after_eng);
	else
		_screen_cap_by_cpu(mva, ufmt, after_eng);

	ret = m4u_cache_sync(m4uClient, M4U_PORT_DISP_WDMA0, pbuf, buffer_size, mva,
			   M4U_CACHE_INVALID_BY_RANGE);
out:
	if (mva > 0)
		m4u_dealloc_mva(m4uClient, M4U_PORT_DISP_WDMA0, mva);

	if (m4uClient != 0)
		m4u_destroy_client(m4uClient);

	disp_sw_mutex_unlock(&(pgc->capture_lock));
	DISPMSG("primary capture: end\n");
	return ret;
}

int primary_display_capture_framebuffer(unsigned long pbuf)
{
	unsigned int fb_layer_id = primary_display_get_option("FB_LAYER");
	unsigned int w_xres = primary_display_get_width();
	unsigned int h_yres = primary_display_get_height();
	unsigned int pixel_bpp = primary_display_get_bpp() / 8;	/* bpp is either 32 or 16, can not be other value */
	unsigned int w_fb = ALIGN_TO(w_xres, MTK_FB_ALIGNMENT);
	unsigned int fbsize = w_fb * h_yres * pixel_bpp;	/* frame buffer size */
	unsigned long fbaddress =
	    dpmgr_path_get_last_config(pgc->dpmgr_handle)->ovl_config[fb_layer_id].addr;
	unsigned int i;
	unsigned long ttt;
	void *fbv = 0;
	DISPMSG("w_res=%d, h_yres=%d, pixel_bpp=%d, w_fb=%d, fbsize=%d, fbaddress=0x%lx\n", w_xres,
		h_yres, pixel_bpp, w_fb, fbsize, fbaddress);
	fbv = ioremap(fbaddress, fbsize);
	DISPMSG
	    ("w_xres = %d, h_yres = %d, w_fb = %d, pixel_bpp = %d, fbsize = %d, fbaddress = 0x%08lx\n",
	     w_xres, h_yres, w_fb, pixel_bpp, fbsize, fbaddress);
	if (!fbv) {
		DISPMSG
		    ("[FB Driver], Unable to allocate memory for frame buffer: address=0x%lx, size=0x%08x\n",
		     fbaddress, fbsize);
		return -1;
	}

	ttt = get_current_time_us();
	for (i = 0; i < h_yres; i++) {
		memcpy((void *)(pbuf + i * w_xres * pixel_bpp),
		       (void *)(fbv + i * w_fb * pixel_bpp), w_xres * pixel_bpp);
	}
	DISPMSG("capture framebuffer cost %ld us\n", get_current_time_us() - ttt);
	iounmap((void *)fbv);
	return -1;
}

static uint32_t disp_fb_bpp = 32;
static uint32_t disp_fb_pages = 3;

uint32_t DISP_GetScreenBpp(void)
{
	return disp_fb_bpp;
}

uint32_t DISP_GetPages(void)
{
	return disp_fb_pages;
}

uint32_t DISP_GetFBRamSize(void)
{
	return ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) *
	    DISP_GetScreenHeight() *
	    ((DISP_GetScreenBpp() + 7) >> 3) * DISP_GetPages();
}

uint32_t DISP_GetVRamSize(void)
{
#if 0
	/* Use a local static variable to cache the calculated vram size */
	/*  */
	static uint32_t vramSize;

	if (0 == vramSize) {
		disp_drv_init_context();

		/* /get framebuffer size */
		vramSize = DISP_GetFBRamSize();

		/* /get DXI working buffer size */
		vramSize += disp_if_drv->get_working_buffer_size();

		/* get assertion layer buffer size */
		vramSize += DAL_GetLayerSize();

		/* Align vramSize to 1MB */
		/*  */
		vramSize = ALIGN_TO_POW_OF_2(vramSize, 0x100000);

		DISP_LOG("DISP_GetVRamSize: %u bytes\n", vramSize);
	}

	return vramSize;
#endif
	return 0;
}

uint32_t DISP_GetVRamSizeBoot(char *cmdline)
{
	unsigned int vramsize;
	vramsize = mtkfb_get_fb_size();
	BUG_ON(!vramsize);
	DISPCHECK("[DT]display vram size = 0x%08x|%d\n", vramsize, vramsize);
	return vramsize;
}

int disp_hal_allocate_framebuffer(phys_addr_t pa_start, phys_addr_t pa_end, unsigned long *va,
				  unsigned long *mva)
{
	int ret = 0;
	*va = (unsigned long)ioremap_nocache(pa_start, pa_end - pa_start + 1);
	pr_debug("disphal_allocate_fb, pa_start=0x%pa, pa_end=0x%pa, va=0x%lx\n", &pa_start,
		 &pa_end, *va);

	if (disp_helper_get_option(DISP_OPT_USE_M4U)) {
		m4u_client_t *client;

		struct sg_table *sg_table = &table;
		sg_alloc_table(sg_table, 1, GFP_KERNEL);

		sg_dma_address(sg_table->sgl) = pa_start;
		sg_dma_len(sg_table->sgl) = (pa_end - pa_start + 1);
		client = m4u_create_client();
		if (IS_ERR_OR_NULL(client))
			DISPERR("create client fail!\n");


		*mva = pa_start & 0xffffffffULL;
		ret =
		    m4u_alloc_mva(client, M4U_PORT_DISP_OVL0, 0, sg_table, (pa_end - pa_start + 1),
				  M4U_PROT_READ | M4U_PROT_WRITE, M4U_FLAGS_FIX_MVA, (unsigned int *)mva);
		/* m4u_alloc_mva(M4U_PORT_DISP_OVL0, pa_start, (pa_end - pa_start + 1), 0, 0, mva); */
		if (ret)
			DISPERR("m4u_alloc_mva returns fail: %d\n", ret);

		pr_debug("[DISPHAL] FB MVA is 0x%lx PA is 0x%pa\n", *mva, &pa_start);

	} else {
		*mva = pa_start & 0xffffffffULL;
	}

	return 0;
}


unsigned int primary_display_get_option(const char *option)
{
	if (!strcmp(option, "FB_LAYER"))
		return 0;
	if (!strcmp(option, "ASSERT_LAYER"))
		return PRIMARY_SESSION_INPUT_LAYER_COUNT - 1;
	if (!strcmp(option, "M4U_ENABLE"))
		return disp_helper_get_option(DISP_OPT_USE_M4U);
	ASSERT(0);
	return 0; /* avoid build warning */
}



/*
mode: 0, switch to cmd mode; 1, switch to vdo mode
*/
int primary_display_switch_dst_mode(int mode)
{
	DISP_STATUS ret = DISP_STATUS_ERROR;
	void *lcm_cmd = NULL;

	pr_debug
	    ("[ERROR: primary_display_switch_dst_mode]this function not enable in disp driver\n");

	DISPFUNC();
	_primary_path_switch_dst_lock();
	disp_sw_mutex_lock(&(pgc->capture_lock));
	if (pgc->plcm->params->type != LCM_TYPE_DSI) {
		pr_debug("[primary_display_switch_dst_mode] Error, only support DSI IF\n");
		goto done;
	}
	if (pgc->state == DISP_SLEPT) {
		DISPCHECK
		    ("[primary_display_switch_dst_mode], primary display path is already sleep, skip\n");
		goto done;
	}

	if (mode == primary_display_cur_dst_mode) {
		DISPCHECK
		    ("[primary_display_switch_dst_mode]not need switch,cur_mode:%d, switch_mode:%d\n",
		     primary_display_cur_dst_mode, mode);
		goto done;
	}
	lcm_cmd = disp_lcm_switch_mode(pgc->plcm, mode);
	if (lcm_cmd == NULL) {
		DISPCHECK
		    ("[primary_display_switch_dst_mode]get lcm cmd fail primary_display_cur_dst_mode=%d mode=%d\n",
		     primary_display_cur_dst_mode, mode);
		goto done;
	} else {
		int temp_mode = 0;
		if (0 != dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config,
				     DDP_SWITCH_LCM_MODE, lcm_cmd)) {
			pr_err("switch lcm mode fail, return directly\n");
			goto done;
		}
		_primary_path_lock(__func__);
		temp_mode = (int)(pgc->plcm->params->dsi.mode);
		pgc->plcm->params->dsi.mode = pgc->plcm->params->dsi.switch_mode;
		pgc->plcm->params->dsi.switch_mode = temp_mode;
		dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
		if (0 != dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config,
				     DDP_SWITCH_DSI_MODE, lcm_cmd)) {
			pr_err("switch dsi mode fail, return directly\n");
			_primary_path_unlock(__func__);
			goto done;
		}
	}
	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT))
		primary_display_sodi_rule_init();
	_cmdq_stop_trigger_loop();
	_cmdq_build_trigger_loop();
	_cmdq_start_trigger_loop();
	_cmdq_reset_config_handle();	/* must do this */
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	primary_display_cur_dst_mode = mode;

	if (primary_display_is_video_mode()) {
		if (_need_lfr_check()) {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_DSI0_FRAME_DONE);
		} else {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
					       DDP_IRQ_RDMA0_DONE);
		}
	} else {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC,
				       DDP_IRQ_DSI0_EXT_TE);
	}
	_primary_path_unlock(__func__);
	ret = DISP_STATUS_OK;
done:
/* dprec_handle_option(0x0); */
	disp_sw_mutex_unlock(&(pgc->capture_lock));
	_primary_path_switch_dst_unlock();
	return ret;
}


int primary_fps_ctx_set_wnd_sz(unsigned int wnd_sz)
{
	return fps_ctx_set_wnd_sz(fps_get_ctx(), wnd_sz);
}

int primary_display_signal_recovery(void)
{
	DISPCHECK("[Primary Recovery] signal\n");
	atomic_set(&primary_recovery_task_wakeup, 1);
	wake_up_interruptible(&primary_recovery_task_wq);
	return 0;
}

static int primary_display_recovery_thread(void *data)
{
	int ret = 0;

	struct sched_param param = {.sched_priority = 87 };

	sched_setscheduler(current, SCHED_RR, &param);
	DISPCHECK("[primary_display_recovery_thread]\n");
	while (1) {
		msleep(1500);
		ret =
		    wait_event_interruptible(primary_recovery_task_wq,
					     atomic_read
					     (&primary_recovery_task_wakeup));
		atomic_set(&primary_recovery_task_wakeup, 0);
		pr_debug("[Primary Recovery] begin ret %d\n", ret);
		_primary_path_lock(__func__);
		if (pgc->state != DISP_SLEPT) {
			primary_display_idlemgr_kick((char *)__func__, 0);
			if (ret == 0) {
				DISPCHECK("[Primary Recovery]stop cmdq trigger loop\n");
				_cmdq_stop_trigger_loop();

				if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
					pr_debug("[ESD]primary display path is busy\n");
					ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle,
							DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
					pr_debug("[ESD]wait frame done ret:%d\n", ret);
				}

				if (0 != ret) {
					pr_debug("[Primary Recovery]stop path\n");
					dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
				}

				DISPCHECK("[Primary Recovery] path reset\n");
				dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);

				DISPCHECK("[Primary Recovery] path start\n");
				dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);

				DISPCHECK("[Primary Recovery]start cmdq trigger loop\n");
				_cmdq_start_trigger_loop();

				/* (in suspend) when we stop trigger loop
				* if no other thread is running, cmdq may disable its clock
				* all cmdq event will be cleared after suspend */
				cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

				if (primary_display_is_video_mode()) {
					/* for video mode, need to force trigger here */
					/* for cmd mode, */
					/* just set DPREC_EVENT_CMDQ_SET_EVENT_ALLOW when trigger loop start */
					dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
				} else {
					cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
					mdelay(40);
				}
			}
		} else {
			pr_debug
			    ("[Primary Recovery] can not recovery path in suspend\n");
		}
		_primary_path_unlock(__func__);
		pr_debug("[Primary Recovery]  end\n");

		if (kthread_should_stop())
			break;
	}
	return ret;

}
#ifdef VENDOR_EDIT
/* liping-m@PhoneSW.Multimedia, 2016/06/20 Add for MTK dynamic frame rate*/
int primary_display_vdo_porch_change(DSI_PORCH_TYPE porch, unsigned int val)
{
	/* this function is only for vfp change at present */
	int ret = 0;
	int cmd = DDP_DSI_VFP_CHANGE;
	cmdqRecHandle handle = NULL;

	if (porch == DSI_HFP)
		cmd = DDP_DSI_HFP_CHANGE;
	else if (porch == DSI_VFP)
		cmd = DDP_DSI_VFP_CHANGE;
	else {
		ret = -1;
		pr_err("primary_display_vdo_porch_change error \n");
		goto done;
	}

	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	cmdqRecReset(handle);

	/* make sure token rdma_sof is clear */
	cmdqRecClearEventToken(handle, CMDQ_EVENT_DISP_RDMA0_SOF);
	/* wait rdma0_sof: only used for video mode & trigger loop need wait and clear rdma0 sof */
	cmdqRecWaitNoClear(handle, CMDQ_EVENT_DISP_RDMA0_SOF);

	dpmgr_path_ioctl(primary_get_dpmgr_handle(), handle, cmd, &val);
	cmdqRecFlushAsync(handle);
	cmdqRecDestroy(handle);
done:
	return ret;
}

unsigned int _fps_to_vfp(int fps){
	unsigned int vfp = 0;
	DSI_REGS *regs = NULL;
	unsigned int DSI_VSA_NL, DSI_VBP_NL, DSI_VACT_NL;
	unsigned int DSI_HSA_WC, DSI_HBP_WC, DSI_HFP_WC, DSI_BLLP_WC;
	unsigned int DSI_PHY_TIMECON0, LPX, DA_HS_PREP, DA_HS_ZERO;
	unsigned int clk_per_line;

	unsigned int lane_num = pgc->plcm->params->dsi.LANE_NUM;
	unsigned int width = pgc->plcm->params->width;
	unsigned int height = pgc->plcm->params->height;
	unsigned int pll = pgc->plcm->params->dsi.PLL_CLOCK;

	DSI_BackupRegisters(DISP_MODULE_DSI0, NULL);
	regs = DSI_get_backup_regs(DISP_MODULE_DSI0);

	DSI_VSA_NL = AS_UINT32(&regs->DSI_VSA_NL);
	DSI_VBP_NL = AS_UINT32(&regs->DSI_VBP_NL);
	DSI_VACT_NL = AS_UINT32(&regs->DSI_VACT_NL);

	DSI_HSA_WC = AS_UINT32(&regs->DSI_HSA_WC);
	DSI_HBP_WC = AS_UINT32(&regs->DSI_HBP_WC);
	DSI_HFP_WC = AS_UINT32(&regs->DSI_HFP_WC);
	DSI_BLLP_WC = AS_UINT32(&regs->DSI_BLLP_WC);

	DSI_PHY_TIMECON0 = AS_UINT32(&regs->DSI_PHY_TIMECON0);
	LPX = DSI_PHY_TIMECON0 & (0xFF);
	DA_HS_PREP = (DSI_PHY_TIMECON0 >> 8) & 0xFF;
	DA_HS_ZERO = (DSI_PHY_TIMECON0 >> 16) & 0xFF;

	clk_per_line = ((DSI_HSA_WC + DSI_HBP_WC + DSI_HFP_WC + width * 3 + 6 * 4) /
		      lane_num + LPX + DA_HS_PREP + DA_HS_ZERO + 1);
	vfp = pll * 1000000 / 4 / fps / clk_per_line - DSI_VSA_NL - DSI_VBP_NL - height;

	printk("%s, vfp = %d\n", __func__, vfp);
	return vfp;
}
#endif