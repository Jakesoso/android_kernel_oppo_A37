#include <linux/module.h>
#include <linux/slab.h>
#include <linux/soc/oppo/mmkey_log.h>
#include <linux/switch.h>

static char logname[1024]="N/A";
static struct mutex keylog_mutex;

static ssize_t mm_keylog_printname(struct switch_dev *sdev, char *buf) {
    return sprintf(buf, logname);
}

static int keylog_registered = 0;
static struct switch_dev keylog_swthdev;

void mm_keylog_write(char *logmessage, char *cause, int id) {
    char *mergelog;
    pr_err("enter %s\n", __func__);
    if(!keylog_registered ){
        pr_err("keylog device not registered %s\n", __func__);
        return;
    }
    mutex_lock(&keylog_mutex);

    mergelog = (char *)kzalloc(strlen("log:") + strlen(logmessage) + strlen(" cause:") + strlen(cause) + strlen("\n"), GFP_KERNEL);
    strcat(mergelog, "log:");
    strcat(mergelog, logmessage);
    strcat(mergelog, " cause:");
    strcat(mergelog, cause);
    strcat(mergelog, "\n");
    pr_err("logname lenth = %d\n", (int)strlen(mergelog));
    strcpy(logname, mergelog);
    switch_set_state(&keylog_swthdev, id);
    switch_set_state(&keylog_swthdev, -id);
    kfree(mergelog);
    mutex_unlock(&keylog_mutex);
}

EXPORT_SYMBOL_GPL(mm_keylog_write);

static int __init oppo_criticallog_init(void)
{
    int ret = 0;
    mutex_init(&keylog_mutex);
    keylog_swthdev.name = "oppo_critical_log";
    keylog_swthdev.print_name = mm_keylog_printname;
    ret = switch_dev_register(&keylog_swthdev);
    if(ret){
        goto keylog_err;
    }
    keylog_registered = 1;
    return 0;
keylog_err:
    switch_dev_unregister(&keylog_swthdev);
    keylog_registered = 0;
    return ret;
}
arch_initcall(oppo_criticallog_init);

MODULE_DESCRIPTION("OPPO critical log");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("John.Xu <xuzhaoan@oppo.com>");
