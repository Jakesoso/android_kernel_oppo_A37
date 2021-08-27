#ifndef ___FUSE_H___
#define ___FUSE_H___
extern void fuse_request_send_background_ex(struct fuse_conn *fc,
    struct fuse_req *req, __u32 size);
extern void fuse_request_send_ex(struct fuse_conn *fc, struct fuse_req *req,
    __u32 size);

//#ifndef VENDOR_EDIT
//Modified by Tong.han@Bsp.Group.Boot Added for debug mmcqd take cpu resources,2016-4-22
//#if defined(CONFIG_MT_ENG_BUILD)  /* IO log is only enabled in eng load */
//#define FUSEIO_TRACE
//#endif
//#else
#define FUSEIO_TRACE
//#endif /*VENDOR_EDIT*/

#ifdef FUSEIO_TRACE
#include <linux/sched.h>
#include <linux/xlog.h>
#include <linux/kthread.h>

extern void fuse_time_diff(struct timespec *start,
    struct timespec *end,
    struct timespec *diff);

extern void fuse_iolog_add(__u32 io_bytes, int type,
    struct timespec *start,
    struct timespec *end);

extern __u32 fuse_iolog_timeus_diff(struct timespec *start,
    struct timespec *end);

extern void fuse_iolog_exit(void);
extern void fuse_iolog_init(void);

struct fuse_rw_info
{
    __u32  count;
    __u32  bytes;
    __u32  us;
};

struct fuse_proc_info
{
    pid_t pid;
    __u32 valid;
    int misc_type;
    struct fuse_rw_info read;
    struct fuse_rw_info write;
    struct fuse_rw_info misc;
};

#define FUSE_IOLOG_MAX     12
#define FUSE_IOLOG_BUFLEN  512
#define FUSE_IOLOG_LATENCY 1

#define FUSE_IOLOG_INIT()   struct timespec _tstart, _tend
#define FUSE_IOLOG_START()  get_monotonic_boottime(&_tstart)
#define FUSE_IOLOG_END()    get_monotonic_boottime(&_tend)
#define FUSE_IOLOG_US()     fuse_iolog_timeus_diff(&_tstart, &_tend)
#define FUSE_IOLOG_PRINT(iobytes, type)  fuse_iolog_add(iobytes, type, &_tstart, &_tend)

#else

#define FUSE_IOLOG_INIT(...)
#define FUSE_IOLOG_START(...)
#define FUSE_IOLOG_END(...)
#define FUSE_IOLOG_PRINT(...)
#define fuse_iolog_init(...)
#define fuse_iolog_exit(...)

#endif

#endif
