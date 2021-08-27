#ifndef __DDP_CLK_MGR_H__
#define __DDP_CLK_MGR_H__

#ifndef CONFIG_MTK_CLKMGR
#include <linux/clk.h>
#endif


typedef enum disp_clk_id {
	DISP0_SMI_COMMON = 0,
	DISP0_SMI_LARB0,
	DISP0_DISP_OVL0,
	DISP0_DISP_OVL1,
	DISP0_DISP_RDMA0,
	DISP0_DISP_RDMA1,
	DISP0_DISP_WDMA0,
	DISP0_DISP_COLOR,
	DISP0_DISP_CCORR,
	DISP0_DISP_AAL,
	DISP0_DISP_GAMMA,
	DISP0_DISP_DITHER,
	DISP0_DISP_UFOE_MOUT,
	DISP0_DISP_WDMA1,
	DISP0_DISP_2L_OVL0,
	DISP0_DISP_2L_OVL1,
	DISP0_DISP_OVL0_MOUT,
	DISP1_DSI_ENGINE,
	DISP1_DSI_DIGITAL,
	DISP1_DPI_ENGINE,
	DISP1_DPI_PIXEL,
	DISP_PWM,
	DISP_MTCMOS_CLK,
	MUX_DPI0,
	TVDPLL_D2,
	TVDPLL_D4,
	TVDPLL_D8,
	TVDPLL_D16,
	DPI_CK,
	MUX_DISPPWM,
	OSC_D2,
	OSC_D8,
	MUX_MM,
	MM_VENCPLL,
	SYSPLL2_D2,
	MAX_DISP_CLK_CNT
} eDDP_CLK_ID;

#ifndef CONFIG_MTK_CLKMGR

int ddp_clk_prepare(eDDP_CLK_ID id);
int ddp_clk_unprepare(eDDP_CLK_ID id);
int ddp_clk_enable(eDDP_CLK_ID id);
int ddp_clk_disable(eDDP_CLK_ID id);
int ddp_clk_prepare_enable(eDDP_CLK_ID id);
int ddp_clk_disable_unprepare(eDDP_CLK_ID id);
int ddp_clk_set_parent(eDDP_CLK_ID id, eDDP_CLK_ID parent);
int ddp_set_clk_handle(struct clk *pclk, unsigned int n);
int ddp_parse_apmixed_base(void);
int ddp_set_mipi26m(int en);

#endif				/* CONFIG_MTK_CLKMGR */

#endif
