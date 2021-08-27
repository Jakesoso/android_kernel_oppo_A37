#ifndef __DISP_DRV_LOG_H__
#define __DISP_DRV_LOG_H__
#include "display_recorder.h"
#include <linux/xlog.h>
#include "ddp_debug.h"
#define DISP_LOG_PRINT(level, sub_module, fmt, arg...)      \
	    pr_debug(fmt, ##arg)

#define DISPMSG(string, args...) pr_debug("[DISP]"string, ##args)
#define DISPCHECK(string, args...) pr_debug("[DISPCHECK]"string, ##args)

#define DISPDBG(string, args...)                   \
	do {                                       \
		if (ddp_debug_dbg_log_level())          \
			pr_debug("disp/"string, ##args);            \
	} while (0)
#define DISPPR_FENCE(string, args...)                    \
	do { \
		if (ddp_debug_dbg_log_level()) {         \
			dprec_logger_pr(DPREC_LOGGER_FENCE, string, ##args);\
			pr_debug("fence/"string, ##args);\
		} \
	} while (0)

#define DISPERR	DISPPR_ERROR

#define DISPFUNC() DISPDBG("[DISP]func|%s\n", __func__)	/* default on, err msg */
#define DISPDBGFUNC() DISPDBG("[DISP]func|%s\n", __func__)	/* default on, err msg */
/*#define DISPCHECK(string, args...) pr_debug("[DISPCHECK]"string, ##args)*/

#define DISPPR_HWOP(string, args...)	/* dprec_logger_pr(DPREC_LOGGER_HWOP, string, ##args); */
#define DISPPR_ERROR(string, args...)  do {\
		dprec_logger_pr(DPREC_LOGGER_ERROR, string, ##args);\
		pr_err("[DISP][%s #%d]ERROR:"string, __func__, __LINE__, ##args);\
	} while (0)

#define DISPWARN(string, args...)  do {\
		dprec_logger_pr(DPREC_LOGGER_ERROR, string, ##args);\
		pr_warn("[DISP][%s #%d]warn:"string, __func__, __LINE__, ##args);\
	} while (0)

/*#define DISPPR_FENCE(string, args...)  do { \
		dprec_logger_pr(DPREC_LOGGER_FENCE, string, ##args);\
		pr_debug("fence/"string, ##args);\
	} while (0)*/
#define disp_aee_print(string, args...) do {\
	char disp_name[100];\
	snprintf(disp_name, 100, "[DISP]"string, ##args); \
	aee_kernel_warning_api(__FILE__, __LINE__, \
		DB_OPT_DEFAULT | DB_OPT_MMPROFILE_BUFFER | DB_OPT_DISPLAY_HANG_DUMP | DB_OPT_DUMP_DISPLAY, \
		disp_name, "[DISP] error"string, ##args);  \
	pr_err("DISP error: "string, ##args);  \
} while (0)
#endif				/* __DISP_DRV_LOG_H__ */
