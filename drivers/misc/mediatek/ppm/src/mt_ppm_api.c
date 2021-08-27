
#include "mt_ppm_internal.h"

#ifdef VENDOR_EDIT
//xiaocheng@Swdp.shanghai, 2016/3/30, Add API to enable/disable ppm.
int mt_ppm_set_enabled(unsigned int enabled)
{
	int i;
	struct ppm_client_req *c_req = &(ppm_main_info.client_req);
	struct ppm_client_req *last_req = &(ppm_main_info.last_req);

	ppm_lock(&ppm_main_info.lock);
	ppm_main_info.is_enabled = (enabled) ? true : false;

	if (!ppm_main_info.is_enabled) {
		/* send default limit to client */
		ppm_main_clear_client_req(c_req);
		for (i = 0; i < NR_PPM_CLIENTS; i++) {
			if (ppm_main_info.client_info[i].limit_cb)
				ppm_main_info.client_info[i].limit_cb(*c_req);
		}
		memcpy(last_req->cpu_limit, c_req->cpu_limit,
			ppm_main_info.cluster_num * sizeof(*c_req->cpu_limit));

		ppm_info("send no limit to client since ppm is disabled!\n");
	}
	ppm_unlock(&ppm_main_info.lock);
	return 0;
}
EXPORT_SYMBOL(mt_ppm_set_enabled);
#endif

/* APIs */
void mt_ppm_set_dvfs_table(unsigned int cpu, struct cpufreq_frequency_table *tbl,
	unsigned int num, enum dvfs_table_type type)
{
	int i, j;

	FUNC_ENTER(FUNC_LV_API);

	for (i = 0; i < ppm_main_info.cluster_num; i++) {
		if (ppm_main_info.cluster_info[i].cpu_id == cpu) {
			/* return if table is existed */
			if (ppm_main_info.cluster_info[i].dvfs_tbl)
				return;

			ppm_lock(&ppm_main_info.lock);

			ppm_main_info.dvfs_tbl_type = type;
			ppm_main_info.cluster_info[i].dvfs_tbl = tbl;
			ppm_main_info.cluster_info[i].dvfs_opp_num = num;
			/* dump dvfs table */
			ppm_info("DVFS table type = %d\n", type);
			ppm_info("DVFS table of cluster %d:\n", ppm_main_info.cluster_info[i].cluster_id);
			for (j = 0; j < num; j++)
				ppm_info("%d: %d KHz\n", j, ppm_main_info.cluster_info[i].dvfs_tbl[j].frequency);

			ppm_unlock(&ppm_main_info.lock);

			FUNC_EXIT(FUNC_LV_API);
			return;
		}
	}

	if (i == ppm_main_info.cluster_num)
		ppm_err("@%s: cpu_id not found!\n", __func__);

	FUNC_EXIT(FUNC_LV_API);
}

void mt_ppm_register_client(enum ppm_client client, void (*limit)(struct ppm_client_req req))
{
	FUNC_ENTER(FUNC_LV_API);

	ppm_lock(&ppm_main_info.lock);

	/* init client */
	ppm_main_info.client_info[client].client = client;
	ppm_main_info.client_info[client].limit_cb = limit;

	ppm_unlock(&ppm_main_info.lock);

	FUNC_EXIT(FUNC_LV_API);
}

met_set_ppm_state_funcMET g_pSet_PPM_State;

void mt_set_ppm_state_registerCB(met_set_ppm_state_funcMET pCB)
{
	g_pSet_PPM_State = pCB;
}
EXPORT_SYMBOL(mt_set_ppm_state_registerCB);

void mt_ppm_set_5A_limit_throttle(bool enable)
{
	FUNC_ENTER(FUNC_LV_API);
#ifdef PPM_VPROC_5A_LIMIT_CHECK
	ppm_lock(&ppm_main_info.lock);
	if (!ppm_main_info.is_5A_limit_enable) {
		ppm_unlock(&ppm_main_info.lock);
		goto end;
	}
	ppm_main_info.is_5A_limit_on = enable;
	ppm_info("is_5A_limit_on = %d\n", ppm_main_info.is_5A_limit_on);
	ppm_unlock(&ppm_main_info.lock);

	ppm_task_wakeup();
end:
#endif
	FUNC_EXIT(FUNC_LV_API);
}

