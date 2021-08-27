#define LOG_TAG "dump"
#include <linux/slab.h>
#include "ddp_reg.h"
#include "ddp_log.h"
#include "ddp_dump.h"
#include "ddp_ovl.h"
#include "ddp_wdma.h"
#include "ddp_rdma.h"
#include "ddp_dsi.h"

static char *ddp_signal_0(int bit)
{
	switch (bit) {
	case 31:
		return "dpi0_sel_to_dpi0";
	case 30:
		return "dis0_sel_to_dsi0";
	case 29:
		return "rdma1_sout1_to_dpi0_sin2";
	case 28:
		return "rdma1_sout0_to_dsi0_sin2";
	case 27:
		return "rdma1_to_rdma1_sout";
		/* case 26: return "bit26-unused"; */
	case 25:
		return "ovl1_mout1_to_wdma1";
	case 24:
		return "ovl1_mout0_to_rdma1";
	case 23:
		return "ovl1_to_ovl1_mout";
	case 22:
		return "wdma0_sel_to_wdma0";
	case 21:
		return "ufoe_mout2_to_wdma0_sin2";
	case 20:
		return "ufoe_mout1_to_dpi0_sin0";
	case 19:
		return "ufoe_mout0_to_dsi0_sin0";
	case 18:
		return "ufoe_to_ufoe_mout";
	case 17:
		return "ufoe_sel_to_ufoe";
	case 16:
		return "rdma0_sout3_to_dpi0_sin1";
	case 15:
		return "rdma0_sout2_to_dsi0_sin1";
	case 14:
		return "rdma0_sout1_to_color_sin0";
	case 13:
		return "rdma0_sout0_to_ufoe_sin0";
	case 12:
		return "rdma0_to_rdma0_sout";
	case 11:
		return "dither_mout2_to_wdma0_sin1";
	case 10:
		return "dither_mout1_to_ufoe_sin1";
	case 9:
		return "dither_mout0_to_rdma0";
	case 8:
		return "dither_to_dither_mout";
		/* case 7:  return "bit7-unused"; */
	case 6:
		return "aal_to_gamma";
	case 5:
		return "ccorr_to_aal";
	case 4:
		return "color_to_ccorr";
	case 3:
		return "color_sel_to_color";
	case 2:
		return "ovl0_mout1_to_wdma0_sin0";
	case 1:
		return "ovl0_mout0_to_color_sin1";
		/* case 0:  return "bit0-unused"; */
	default:
		return NULL;
	}
}

static char *ddp_signal_1(int bit)
{
	switch (bit) {
	case 15:
		return "ovl0_sel_to_ovl0_mout";
	case 14:
		return "ovl1_mout2_to_ovl0_sel";
	case 13:
		return "ovl1_sout1_to_ovl0_sel";
	case 12:
		return "ovl1_sout0_to_ovl1_4L";
	case 11:
		return "ovl1_2L_to_ovl1_sout";
	case 10:
		return "ovl0_sout1_to_ovl1_2L";
	case 9:
		return "ovl0_sout0_to_ovl0_sel";
	case 8:
		return "ovl0_to_ovl0_sout";
	default:
		return NULL;
	}
}

static char *ddp_greq_name(int bit)
{
	switch (bit) {
	case 10:
		return "mdp_wrot";
	case 9:
		return "mdp_wdma";
	case 8:
		return "mdp_rdma";
	case 7:
		return "ovl1_2L";
	case 6:
		return "ovl0_2L";
	case 5:
		return "wdma1";
	case 4:
		return "rdma1";
	case 3:
		return "ovl1";
	case 2:
		return "wdma0";
	case 1:
		return "rdma0";
	case 0:
		return "ovl0";
	default:
		return NULL;
	}
}

static char *ddp_get_mutex_module_name(unsigned int bit)
{
	switch (bit) {
	case 7:
		return "ovl0";
	case 8:
		return "ovl1";
	case 9:
		return "rdma0";
	case 10:
		return "rdma1";
	case 11:
		return "wdma0";
	case 12:
		return "color0";
	case 13:
		return "ccorr";
	case 14:
		return "aal";
	case 15:
		return "gamma";
	case 16:
		return "dither";
	case 17:
		return "wdma1";
	case 18:
		return "pwm0";
	case 19:
		return "ovl0_2l";
	case 20:
		return "ovl1_2l";
	default:
		return "mutex-unknown";
	}
}

char *ddp_get_fmt_name(DISP_MODULE_ENUM module, unsigned int fmt)
{
	if (module == DISP_MODULE_WDMA0) {
		switch (fmt) {
		case 0:
			return "rgb565";
		case 1:
			return "rgb888";
		case 2:
			return "rgba8888";
		case 3:
			return "argb8888";
		case 4:
			return "uyvy";
		case 5:
			return "yuy2";
		case 7:
			return "y-only";
		case 8:
			return "iyuv";
		case 12:
			return "nv12";
		default:
			DDPDUMP("ddp_get_fmt_name, unknown fmt=%d, module=%d\n", fmt, module);
			return "unknown";
		}
	} else if (module == DISP_MODULE_OVL0) {
		switch (fmt) {
		case 0:
			return "rgb565";
		case 1:
			return "rgb888";
		case 2:
			return "rgba8888";
		case 3:
			return "argb8888";
		case 4:
			return "uyvy";
		case 5:
			return "yuyv";
		default:
			DDPDUMP("ddp_get_fmt_name, unknown fmt=%d, module=%d\n", fmt, module);
			return "unknown";
		}
	} else if (module == DISP_MODULE_RDMA0 || module == DISP_MODULE_RDMA1 || module == DISP_MODULE_RDMA2) {
		switch (fmt) {
		case 0:
			return "rgb565";
		case 1:
			return "rgb888";
		case 2:
			return "rgba8888";
		case 3:
			return "argb8888";
		case 4:
			return "uyvy";
		case 5:
			return "yuyv";
		default:
			DDPDUMP("ddp_get_fmt_name, unknown fmt=%d, module=%d\n", fmt, module);
			return "unknown";
		}
	} else {
		DDPDUMP("ddp_get_fmt_name, unknown module=%d\n", module);
	}

	return "unknown";
}

static char *ddp_clock_0(int bit)
{
	switch (bit) {
	case 0:
		return "smi_common, ";
	case 1:
		return "smi_larb0, ";
	case 10:
		return "ovl0, ";
	case 11:
		return "ovl1, ";
	case 12:
		return "rdma0, ";
	case 13:
		return "rdma1, ";
	case 14:
		return "wdma0, ";
	case 15:
		return "color, ";
	case 16:
		return "ccorr, ";
	case 17:
		return "aal, ";
	case 18:
		return "gamma, ";
	case 19:
		return "dither, ";
	case 21:
		return "ufoe_mout, ";
	case 22:
		return "wdma1, ";
	case 23:
		return "ovl0_2L, ";
	case 24:
		return "ovl1_2L, ";
	case 25:
		return "ovl0_mout, ";
	default:
		return NULL;
	}
}

static char *ddp_clock_1(int bit)
{
	switch (bit) {
	case 0:
		return "dsi_engine, ";
	case 1:
		return "dsi_digital, ";
	case 2:
		return "dpi_pixel, ";
	case 3:
		return "dpi_engine, ";
	default:
		return NULL;
	}
}

static void mutex_dump_reg(void)
{
	DDPDUMP("== START: DISP MUTEX REGS ==\n");
	DDPDUMP("(0x000)M_INTEN   =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN));
	DDPDUMP("(0x004)M_INTSTA  =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA));
	DDPDUMP("(0x020)M0_EN     =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_EN));
	DDPDUMP("(0x028)M0_RST    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_RST));
	DDPDUMP("(0x02c)M0_MOD    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_MOD));
	DDPDUMP("(0x030)M0_SOF    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX0_SOF));
	DDPDUMP("(0x040)M1_EN     =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_EN));
	DDPDUMP("(0x048)M1_RST    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_RST));
	DDPDUMP("(0x04c)M1_MOD    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_MOD));
	DDPDUMP("(0x050)M1_SOF    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX1_SOF));
	DDPDUMP("(0x060)M2_EN     =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_EN));
	DDPDUMP("(0x068)M2_RST    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_RST));
	DDPDUMP("(0x06c)M2_MOD    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_MOD));
	DDPDUMP("(0x070)M2_SOF    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX2_SOF));
	DDPDUMP("(0x080)M3_EN     =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_EN));
	DDPDUMP("(0x088)M3_RST    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_RST));
	DDPDUMP("(0x08c)M3_MOD    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_MOD));
	DDPDUMP("(0x090)M3_SOF    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX3_SOF));
	DDPDUMP("(0x0a0)M4_EN     =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_EN));
	DDPDUMP("(0x0a8)M4_RST    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_RST));
	DDPDUMP("(0x0ac)M4_MOD    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_MOD));
	DDPDUMP("(0x0b0)M4_SOF    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX4_SOF));
	DDPDUMP("(0x0c0)M5_EN     =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_EN));
	DDPDUMP("(0x0c8)M5_RST    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_RST));
	DDPDUMP("(0x0cc)M5_MOD    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_MOD));
	DDPDUMP("(0x0d0)M5_SOF    =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MUTEX5_SOF));
	DDPDUMP("(0x200)DEBUG_OUT_SEL =0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_DEBUG_OUT_SEL));
	DDPDUMP("-- END: DISP MUTEX REGS --\n");
}


static void mutex_dump_analysis(void)
{
	int i = 0;
	int j = 0;
	char *mutex_module;
	char *p = NULL;
	int len = 0;
	unsigned int val;
	unsigned int regval;

	mutex_module = kzalloc(512, GFP_KERNEL);
	if (!mutex_module) {
		DDPDUMP("%s fail because of no memory\n", __func__);
		return;
	}

	DDPDUMP("==DISP Mutex Analysis==\n");
	for (i = 0; i < 5; i++) {
		p = mutex_module;
		len = 0;
		if (DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(i)) == 0)
			continue;

		val = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(i));
		len = sprintf(p, "MUTEX%d :SOF=%s,EOF=%s,WAIT=%d,module=(", i,
				ddp_get_mutex_sof_name(REG_FLD_VAL_GET(SOF_FLD_MUTEX0_SOF, val)),
				ddp_get_mutex_sof_name(REG_FLD_VAL_GET(SOF_FLD_MUTEX0_EOF, val)),
				REG_FLD_VAL_GET(SOF_FLD_MUTEX0_SOF_WAIT, val));

		p += len;
		regval = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(i));
		for (j = 0; j < 32; j++) {
			if ((regval & (1 << j))) {
				len = sprintf(p, "%s,", ddp_get_mutex_module_name(j));
				p += len;
			}
		}
		DDPDUMP("%s)\n", mutex_module);
	}
	kfree(mutex_module);
}

static void mmsys_config_dump_reg(void)
{
	DDPDUMP("== START: DISP MMSYS_CONFIG REGS ==\n");
	DDPDUMP("MMSYS:0x000=0x%08x,0x004=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_INTEN),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_INTSTA));
	DDPDUMP("MMSYS:0x030=0x%08x,0x034=0x%08x,0x038=0x%08x,0x03C=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL0_MOUT_EN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL1_MOUT_EN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DITHER_MOUT_EN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_UFOE_MOUT_EN));
	DDPDUMP("MMSYS:0x040=0x%08x,0x058=0x%08x,0x05C=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MOUT_RST),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_COLOR0_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_WDMA0_SEL_IN));
	DDPDUMP("MMSYS:0x060=0x%08x,0x064=0x%08x,0x068=0x%08x,0x06C=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_DISP_UFOE_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DSI0_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DPI0_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_RDMA0_SOUT_SEL_IN));
	DDPDUMP("MMSYS:0x070=0x%08x,0x074=0x%08x,0x078=0x%08x,0x07C=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_DISP_RDMA1_SOUT_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL0_SOUT_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL0_SEL_IN),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL1_SOUT_SEL_IN));
	DDPDUMP("MMSYS:0x0F0=0x%08x,0x100=0x%08x,0x110=0x%08x,0x120=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_MISC),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS0));
	DDPDUMP("MMSYS:0x130=0x%08x,0x140=0x%08x,0x144=0x%08x,0x150=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_HW_DCM_DIS1),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW0_RST_B),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_SW1_RST_B),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_LCM_RST_B));
	DDPDUMP("MMSYS:0x880=0x%08x,0x890=0x%08x,0x894=0x%08x,0x898=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DEBUG_OUT_SEL),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DUMMY0),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DUMMY1),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DUMMY2));
	DDPDUMP("MMSYS:0x89c=0x%08x,0x8a0=0x%08x,0x8a4=0x%08x,0x8a8=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_DUMMY3),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_1),
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0));
	DDPDUMP("MMSYS:0x8ac=0x%08x\n",
		DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_1));
	DDPDUMP("-- END: DISP MMSYS_CONFIG REGS --\n");
}

/* ------ clock:
Before power on mmsys:
CLK_CFG_0_CLR (address is 0x10000048) = 0x80000000 (bit 31).
Before using DISP_PWM0 or DISP_PWM1:
CLK_CFG_1_CLR(address is 0x10000058)=0x80 (bit 7).
Before using DPI pixel clock:
CLK_CFG_6_CLR(address is 0x100000A8)=0x80 (bit 7).

Only need to enable the corresponding bits of MMSYS_CG_CON0 and MMSYS_CG_CON1 for the modules:
smi_common, larb0, mdp_crop, fake_eng, mutex_32k, pwm0, pwm1, dsi0, dsi1, dpi.
Other bits could keep 1. Suggest to keep smi_common and larb0 always clock on.

--------valid & ready
example:
ovl0 -> ovl0_mout_ready=1 means engines after ovl_mout are ready for receiving data
	ovl0_mout_ready=0 means ovl0_mout can not receive data, maybe ovl0_mout or after engines config error
ovl0 -> ovl0_mout_valid=1 means engines before ovl0_mout is OK,
	ovl0_mout_valid=0 means ovl can not transfer data to ovl0_mout, means ovl0 or before engines are not ready.
*/

static void mmsys_config_dump_analysis(void)
{
	unsigned int i = 0;
	unsigned int reg = 0;
	char *clock_on;
	char *pos = NULL;
	char *name;

	unsigned int valid0 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_0);
	unsigned int valid1 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_VALID_1);
	unsigned int ready0 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_0);
	unsigned int ready1 = DISP_REG_GET(DISP_REG_CONFIG_DISP_DL_READY_1);
	unsigned int greq = DISP_REG_GET(DISP_REG_CONFIG_SMI_LARB0_GREQ);

	DDPDUMP("==DISP MMSYS_CONFIG ANALYSIS==\n");
	DDPDUMP("mmsys clock=0x%x, CG_CON0=0x%x, CG_CON1=0x%x\n",
		DISP_REG_GET(DISP_REG_CLK_CFG_0_MM_CLK),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0),
		DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1));
	if ((DISP_REG_GET(DISP_REG_CLK_CFG_0_MM_CLK) >> 31) & 0x1)
		DDPERR("mmsys clock abnormal!!\n");

#define CLOCK_ON_SIZE 512
	clock_on = kzalloc(CLOCK_ON_SIZE, GFP_KERNEL);
	if (!clock_on)
		return;

	reg = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0);
	for (i = 0; i < 32; i++) {
		if ((reg & (1 << i)) == 0) {
			name = ddp_clock_0(i);
			if (name)
				strncat(clock_on, name, CLOCK_ON_SIZE);
		}
	}

	reg = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON1);
	for (i = 0; i < 32; i++) {
		if ((reg & (1 << i)) == 0) {
			name = ddp_clock_1(i);
			if (name)
				strncat(clock_on, name, CLOCK_ON_SIZE);
		}
	}
	DDPDUMP("clock on modules:%s\n", clock_on);

	DDPDUMP("valid0=0x%x, valid1=0x%x, ready0=0x%x, ready1=0x%x, greq=0x%x\n",
		valid0, valid1, ready0, ready1, greq);
	for (i = 0; i < 32; i++) {
		name = ddp_signal_0(i);
		if (!name)
			continue;

		pos = clock_on;
		pos += sprintf(pos, "%25s: ", name);

		if ((valid0 & (1 << i)))
			pos += sprintf(pos, "%10s,", "valid");
		else
			pos += sprintf(pos, "%10s,", "not valid");

		if ((ready0 & (1 << i)))
			pos += sprintf(pos, "%10s", "ready");
		else
			pos += sprintf(pos, "%10s", "not ready");

		DDPDUMP("%s\n", clock_on);
	}

	for (i = 0; i < 32; i++) {
		name = ddp_signal_1(i);
		if (!name)
			continue;

		pos = clock_on;
		pos += sprintf(pos, "%25s: ", name);

		if ((valid1 & (1 << i)))
			pos += sprintf(pos, "%10s,", "valid");
		else
			pos += sprintf(pos, "%10s,", "not valid");

		if ((ready1 & (1 << i)))
			pos += sprintf(pos, "%10s", "ready");
		else
			pos += sprintf(pos, "%10s", "not ready");

		DDPDUMP("%s\n", clock_on);
	}

	/* greq: 1 means SMI dose not grant, maybe SMI hang */
	if (greq)
		DDPMSG("smi greq not grant module: (greq: 1 means SMI dose not grant, maybe SMI hang)");

	clock_on[0] = '\0';
	for (i = 0; i < 32; i++) {
		if (greq & (1 << i)) {
			name = ddp_greq_name(i);
			if (!name)
				continue;
			strncat(clock_on, name, CLOCK_ON_SIZE);
		}
	}
	DDPDUMP("%s\n", clock_on);

	kfree(clock_on);
	return;
}

static void gamma_dump_reg(void)
{
	DDPDUMP("== START: DISP GAMMA REGS ==\n");
	DDPDUMP("(0x000)GA_EN        =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_EN));
	DDPDUMP("(0x004)GA_RESET     =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_RESET));
	DDPDUMP("(0x008)GA_INTEN     =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_INTEN));
	DDPDUMP("(0x00c)GA_INTSTA    =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_INTSTA));
	DDPDUMP("(0x010)GA_STATUS    =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_STATUS));
	DDPDUMP("(0x020)GA_CFG       =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_CFG));
	DDPDUMP("(0x024)GA_IN_COUNT  =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_INPUT_COUNT));
	DDPDUMP("(0x028)GA_OUT_COUNT =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_OUTPUT_COUNT));
	DDPDUMP("(0x02c)GA_CHKSUM    =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_CHKSUM));
	DDPDUMP("(0x030)GA_SIZE      =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_SIZE));
	DDPDUMP("(0x0c0)GA_DUMMY_REG =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_DUMMY_REG));
	DDPDUMP("(0x800)GA_LUT       =0x%x\n", DISP_REG_GET(DISP_REG_GAMMA_LUT));
	DDPDUMP("-- END: DISP GAMMA REGS --\n");
}

static void gamma_dump_analysis(void)
{
	DDPDUMP("==DISP GAMMA ANALYSIS==\n");
	DDPDUMP("gamma: en=%d, w=%d, h=%d, in_p_cnt=%d, in_l_cnt=%d, out_p_cnt=%d, out_l_cnt=%d\n",
		DISP_REG_GET(DISP_REG_GAMMA_EN),
		(DISP_REG_GET(DISP_REG_GAMMA_SIZE) >> 16) & 0x1fff,
		DISP_REG_GET(DISP_REG_GAMMA_SIZE) & 0x1fff,
		DISP_REG_GET(DISP_REG_GAMMA_INPUT_COUNT) & 0x1fff,
		(DISP_REG_GET(DISP_REG_GAMMA_INPUT_COUNT) >> 16) & 0x1fff,
		DISP_REG_GET(DISP_REG_GAMMA_OUTPUT_COUNT) & 0x1fff,
		(DISP_REG_GET(DISP_REG_GAMMA_OUTPUT_COUNT) >> 16) & 0x1fff);
}

static void merge_dump_reg(void)
{
	DDPDUMP("== START: DISP MERGE REGS ==\n");
	DDPDUMP("(0x000)MERGE_EN       =0x%x\n", DISP_REG_GET(DISP_REG_MERGE_ENABLE));
	DDPDUMP("(0x004)MERGE_SW_RESET =0x%x\n", DISP_REG_GET(DISP_REG_MERGE_SW_RESET));
	DDPDUMP("(0x008)MERGE_DEBUG    =0x%x\n", DISP_REG_GET(DISP_REG_MERGE_DEBUG));
	DDPDUMP("-- END: DISP MERGE REGS --\n");
}

static void merge_dump_analysis(void)
{
	DDPDUMP("==DISP MERGE ANALYSIS==\n");
	DDPDUMP("merge: en=%d, debug=0x%x\n", DISP_REG_GET(DISP_REG_MERGE_ENABLE),
		DISP_REG_GET(DISP_REG_MERGE_DEBUG));
}

static void split_dump_reg(DISP_MODULE_ENUM module)
{
	DDPDUMP("error: disp_split dose not exist! module=%d\n", module);
}

static void split_dump_analysis(DISP_MODULE_ENUM module)
{
	DDPDUMP("error: disp_split dose not exist! module=%d\n", module);
}

static void color_dump_reg(DISP_MODULE_ENUM module)
{
	int index = 0;

	if (DISP_MODULE_COLOR0 == module) {
		index = 0;
	} else if (DISP_MODULE_COLOR1 == module) {
		DDPDUMP("error: DISP COLOR%d dose not exist!\n", index);
		return;
	}
	DDPDUMP("== START: DISP COLOR%d REGS ==\n", index);
	DDPDUMP("(0x400)COLOR_CFG_MAIN   =0x%x\n", DISP_REG_GET(DISP_COLOR_CFG_MAIN));
	DDPDUMP("(0x404)COLOR_PXL_CNT_MAIN   =0x%x\n", DISP_REG_GET(DISP_COLOR_PXL_CNT_MAIN));
	DDPDUMP("(0x408)COLOR_LINE_CNT_MAIN   =0x%x\n", DISP_REG_GET(DISP_COLOR_LINE_CNT_MAIN));
	DDPDUMP("(0xc00)COLOR_START      =0x%x\n", DISP_REG_GET(DISP_COLOR_START));
	DDPDUMP("(0xc28)DISP_COLOR_CK_ON      =0x%x\n", DISP_REG_GET(DISP_COLOR_CK_ON));
	DDPDUMP("(0xc50)COLOR_INTER_IP_W =0x%x\n", DISP_REG_GET(DISP_COLOR_INTERNAL_IP_WIDTH));
	DDPDUMP("(0xc54)COLOR_INTER_IP_H =0x%x\n", DISP_REG_GET(DISP_COLOR_INTERNAL_IP_HEIGHT));
	DDPDUMP("-- END: DISP COLOR%d REGS --\n", index);
}

static void color_dump_analysis(DISP_MODULE_ENUM module)
{
	int index = 0;
	if (DISP_MODULE_COLOR0 == module) {
		index = 0;
	} else if (DISP_MODULE_COLOR1 == module) {
		DDPDUMP("error: DISP COLOR%d dose not exist!\n", index);
		return;
	}
	DDPDUMP("==DISP COLOR%d ANALYSIS==\n", index);
	DDPDUMP("color%d: bypass=%d, w=%d, h=%d, pixel_cnt=%d, line_cnt=%d,\n",
		index,
		(DISP_REG_GET(DISP_COLOR_CFG_MAIN) >> 7) & 0x1,
		DISP_REG_GET(DISP_COLOR_INTERNAL_IP_WIDTH),
		DISP_REG_GET(DISP_COLOR_INTERNAL_IP_HEIGHT),
		DISP_REG_GET(DISP_COLOR_PXL_CNT_MAIN) & 0xffff,
		(DISP_REG_GET(DISP_COLOR_LINE_CNT_MAIN) >> 16) & 0x1fff);

}

static void aal_dump_reg(void)
{
	DDPDUMP("== START: DISP AAL REGS ==\n");
	DDPDUMP("(0x000)AAL_EN           =0x%x\n", DISP_REG_GET(DISP_AAL_EN));
	DDPDUMP("(0x008)AAL_INTEN        =0x%x\n", DISP_REG_GET(DISP_AAL_INTEN));
	DDPDUMP("(0x00c)AAL_INTSTA       =0x%x\n", DISP_REG_GET(DISP_AAL_INTSTA));
	DDPDUMP("(0x020)AAL_CFG          =0x%x\n", DISP_REG_GET(DISP_AAL_CFG));
	DDPDUMP("(0x024)AAL_IN_CNT       =0x%x\n", DISP_REG_GET(DISP_AAL_IN_CNT));
	DDPDUMP("(0x028)AAL_OUT_CNT      =0x%x\n", DISP_REG_GET(DISP_AAL_OUT_CNT));
	DDPDUMP("(0x030)AAL_SIZE         =0x%x\n", DISP_REG_GET(DISP_AAL_SIZE));
	DDPDUMP("(0x20c)AAL_CABC_00      =0x%x\n", DISP_REG_GET(DISP_AAL_CABC_00));
	DDPDUMP("(0x214)AAL_CABC_02      =0x%x\n", DISP_REG_GET(DISP_AAL_CABC_02));
	DDPDUMP("(0x20c)AAL_STATUS_00    =0x%x\n", DISP_REG_GET(DISP_AAL_STATUS_00));
	DDPDUMP("(0x210)AAL_STATUS_01    =0x%x\n", DISP_REG_GET(DISP_AAL_STATUS_00 + 0x4));
	DDPDUMP("(0x2a0)AAL_STATUS_31    =0x%x\n", DISP_REG_GET(DISP_AAL_STATUS_32 - 0x4));
	DDPDUMP("(0x2a4)AAL_STATUS_32    =0x%x\n", DISP_REG_GET(DISP_AAL_STATUS_32));
	DDPDUMP("(0x3b0)AAL_DRE_MAPPING_00     =0x%x\n", DISP_REG_GET(DISP_AAL_DRE_MAPPING_00));
	DDPDUMP("-- END: DISP AAL REGS --\n");
}

static void aal_dump_analysis(void)
{
	DDPDUMP("==DISP AAL ANALYSIS==\n");
	DDPDUMP("aal: bypass=%d, relay=%d, en=%d, w=%d, h=%d, in(%d,%d),out(%d,%d)\n",
		DISP_REG_GET(DISP_AAL_EN) == 0x0,
		DISP_REG_GET(DISP_AAL_CFG) & 0x01,
		DISP_REG_GET(DISP_AAL_EN),
		(DISP_REG_GET(DISP_AAL_SIZE) >> 16) & 0x1fff,
		DISP_REG_GET(DISP_AAL_SIZE) & 0x1fff,
		DISP_REG_GET(DISP_AAL_IN_CNT) & 0x1fff,
		(DISP_REG_GET(DISP_AAL_IN_CNT) >> 16) & 0x1fff,
		DISP_REG_GET(DISP_AAL_OUT_CNT) & 0x1fff,
		(DISP_REG_GET(DISP_AAL_OUT_CNT) >> 16) & 0x1fff);
}

static void pwm_dump_reg(DISP_MODULE_ENUM module)
{
	int index = 0;
	unsigned long reg_base = 0;
	if (module == DISP_MODULE_PWM0) {
		index = 0;
		reg_base = DISPSYS_PWM0_BASE;
	} else {
		index = 1;
		reg_base = DISPSYS_PWM1_BASE;
	}
	DDPDUMP("== START: DISP PWM%d REGS ==\n", index);
	DDPDUMP("(0x000)PWM_EN           =0x%x\n", DISP_REG_GET(reg_base + DISP_PWM_EN_OFF));
	DDPDUMP("(0x008)PWM_CON_0        =0x%x\n", DISP_REG_GET(reg_base + DISP_PWM_CON_0_OFF));
	DDPDUMP("(0x010)PWM_CON_1        =0x%x\n", DISP_REG_GET(reg_base + DISP_PWM_CON_1_OFF));
	DDPDUMP("(0x028)PWM_DEBUG        =0x%x\n", DISP_REG_GET(reg_base + 0x28));
	DDPDUMP("-- END: DISP PWM%d REGS --\n", index);
}

static void pwm_dump_analysis(DISP_MODULE_ENUM module)
{
	int index = 0;
	unsigned int reg_base = 0;
	if (module == DISP_MODULE_PWM0) {
		index = 0;
		reg_base = DISPSYS_PWM0_BASE;
	} else {
		index = 1;
		reg_base = DISPSYS_PWM1_BASE;
	}
	DDPDUMP("==DISP PWM%d ANALYSIS==\n", index);
	DDPDUMP("pwm clock=%d\n", (DISP_REG_GET(DISP_REG_CLK_CFG_1_CLR) >> 7) & 0x1);

}

static void od_dump_reg(void)
{
	DDPDUMP("== START: DISP OD REGS ==\n");
	DDPDUMP("(00)EN           =0x%x\n", DISP_REG_GET(DISP_REG_OD_EN));
	DDPDUMP("(04)RESET        =0x%x\n", DISP_REG_GET(DISP_REG_OD_RESET));
	DDPDUMP("(08)INTEN        =0x%x\n", DISP_REG_GET(DISP_REG_OD_INTEN));
	DDPDUMP("(0C)INTSTA       =0x%x\n", DISP_REG_GET(DISP_REG_OD_INTSTA));
	DDPDUMP("(10)STATUS       =0x%x\n", DISP_REG_GET(DISP_REG_OD_STATUS));
	DDPDUMP("(20)CFG          =0x%x\n", DISP_REG_GET(DISP_REG_OD_CFG));
	DDPDUMP("(24)INPUT_COUNT =0x%x\n", DISP_REG_GET(DISP_REG_OD_INPUT_COUNT));
	DDPDUMP("(28)OUTPUT_COUNT =0x%x\n", DISP_REG_GET(DISP_REG_OD_OUTPUT_COUNT));
	DDPDUMP("(2C)CHKSUM       =0x%x\n", DISP_REG_GET(DISP_REG_OD_CHKSUM));
	DDPDUMP("(30)SIZE         =0x%x\n", DISP_REG_GET(DISP_REG_OD_SIZE));
	DDPDUMP("(40)HSYNC_WIDTH  =0x%x\n", DISP_REG_GET(DISP_REG_OD_HSYNC_WIDTH));
	DDPDUMP("(44)VSYNC_WIDTH  =0x%x\n", DISP_REG_GET(DISP_REG_OD_VSYNC_WIDTH));
	DDPDUMP("(48)MISC         =0x%x\n", DISP_REG_GET(DISP_REG_OD_MISC));
	DDPDUMP("(C0)DUMMY_REG    =0x%x\n", DISP_REG_GET(DISP_REG_OD_DUMMY_REG));
	DDPDUMP("-- END: DISP OD REGS --\n");
}

static void od_dump_analysis(void)
{
	DDPDUMP("==DISP OD ANALYSIS==\n");
	DDPDUMP("od: w=%d, h=%d, bypass=%d\n",
		(DISP_REG_GET(DISP_REG_OD_SIZE) >> 16) & 0xffff,
		DISP_REG_GET(DISP_REG_OD_SIZE) & 0xffff, DISP_REG_GET(DISP_REG_OD_CFG) & 0x1);

}

static void ccorr_dump_reg(void)
{
	DDPDUMP("== START: DISP CCORR REGS ==\n");
	DDPDUMP("(00)EN   =0x%x\n", DISP_REG_GET(DISP_REG_CCORR_EN));
	DDPDUMP("(20)CFG  =0x%x\n", DISP_REG_GET(DISP_REG_CCORR_CFG));
	DDPDUMP("(24)IN_CNT =0x%x\n", DISP_REG_GET(DISP_REG_CCORR_IN_CNT));
	DDPDUMP("(28)OUT_CNT =0x%x\n", DISP_REG_GET(DISP_REG_CCORR_OUT_CNT));
	DDPDUMP("(30)SIZE =0x%x\n", DISP_REG_GET(DISP_REG_CCORR_SIZE));
	DDPDUMP("-- END: DISP CCORR REGS --\n");
}

static void ccorr_dump_analyze(void)
{
	DDPDUMP("ccorr: en=%d, config=%d, w=%d, h=%d, in_p_cnt=%d, in_l_cnt=%d, out_p_cnt=%d, out_l_cnt=%d\n",
	     DISP_REG_GET(DISP_REG_CCORR_EN), DISP_REG_GET(DISP_REG_CCORR_CFG),
	     (DISP_REG_GET(DISP_REG_CCORR_SIZE) >> 16) & 0x1fff,
	     DISP_REG_GET(DISP_REG_CCORR_SIZE) & 0x1fff,
	     DISP_REG_GET(DISP_REG_CCORR_IN_CNT) & 0x1fff,
	     (DISP_REG_GET(DISP_REG_CCORR_IN_CNT) >> 16) & 0x1fff,
	     DISP_REG_GET(DISP_REG_CCORR_IN_CNT) & 0x1fff,
	     (DISP_REG_GET(DISP_REG_CCORR_IN_CNT) >> 16) & 0x1fff);
}

static void dither_dump_reg(void)
{
	DDPDUMP("== START: DISP DITHER REGS ==\n");
	DDPDUMP("(00)EN   =0x%x\n", DISP_REG_GET(DISP_REG_DITHER_EN));
	DDPDUMP("(20)CFG  =0x%x\n", DISP_REG_GET(DISP_REG_DITHER_CFG));
	DDPDUMP("(24)IN_CNT =0x%x\n", DISP_REG_GET(DISP_REG_DITHER_IN_CNT));
	DDPDUMP("(28)OUT_CNT =0x%x\n", DISP_REG_GET(DISP_REG_DITHER_OUT_CNT));
	DDPDUMP("(30)SIZE =0x%x\n", DISP_REG_GET(DISP_REG_DITHER_SIZE));
	DDPDUMP("-- END: DISP DITHER REGS --\n");
}

static void dither_dump_analyze(void)
{
	DDPDUMP
	    ("dither: en=%d, config=%d, w=%d, h=%d, in_p_cnt=%d, in_l_cnt=%d, out_p_cnt=%d, out_l_cnt=%d\n",
	     DISP_REG_GET(DISPSYS_DITHER_BASE + 0x000), DISP_REG_GET(DISPSYS_DITHER_BASE + 0x020),
	     (DISP_REG_GET(DISP_REG_DITHER_SIZE) >> 16) & 0x1fff,
	     DISP_REG_GET(DISP_REG_DITHER_SIZE) & 0x1fff,
	     DISP_REG_GET(DISP_REG_DITHER_IN_CNT) & 0x1fff,
	     (DISP_REG_GET(DISP_REG_DITHER_IN_CNT) >> 16) & 0x1fff,
	     DISP_REG_GET(DISP_REG_DITHER_OUT_CNT) & 0x1fff,
	     (DISP_REG_GET(DISP_REG_DITHER_OUT_CNT) >> 16) & 0x1fff);
}

static void ufoe_dump_reg(void)
{
	DDPDUMP("==DISP UFOE REGS==\n");
	return;
}

static void ufoe_dump_analysis(void)
{
	DDPDUMP("==DISP UFOE ANALYSIS==\n");
	return;
}

static void dsi_dump_reg(DISP_MODULE_ENUM module)
{
	int i = 0;
	if (DISP_MODULE_DSI0) {
		DDPDUMP("== START: DISP DSI0 REGS ==\n");
		for (i = 0; i < 25 * 16; i += 16) {
			DDPDUMP("DSI0: 0x%04x=0x%08x,0x%04x=0x%08x,0x%04x=0x%08x,0x%04x=0x%08x\n",
				i, INREG32(DISPSYS_DSI0_BASE + i),
				i + 0x4, INREG32(DISPSYS_DSI0_BASE + i + 0x4),
				i + 0x8, INREG32(DISPSYS_DSI0_BASE + i + 0x8),
				i + 0xc, INREG32(DISPSYS_DSI0_BASE + i + 0xc));
		}
		DDPDUMP("DSI0 CMDQ+0x200 : 0x%08x  0x%08x  0x%08x  0x%08x\n",
		       INREG32(DISPSYS_DSI0_BASE + 0x200), INREG32(DISPSYS_DSI0_BASE + 0x200 + 0x4),
		       INREG32(DISPSYS_DSI0_BASE + 0x200 + 0x8),
		       INREG32(DISPSYS_DSI0_BASE + 0x200 + 0xc));
		DDPDUMP("-- END: DISP DSI0 REGS --\n");
	}
}

static void dpi_dump_reg(void)
{
	int i;

	DDPDUMP("---------- Start dump DPI registers ----------\n");

	for (i = 0; i <= 0x40; i += 4)
		DDPDUMP("DPI+%04x : 0x%08x\n", i, INREG32(DISPSYS_DPI_BASE + i));

	for (i = 0x68; i <= 0x7C; i += 4)
		DDPDUMP("DPI+%04x : 0x%08x\n", i, INREG32(DISPSYS_DPI_BASE + i));

	DDPDUMP("DPI+Color Bar    : 0x%04x : 0x%08x\n", 0xF00, INREG32(DISPSYS_DPI_BASE + 0xF00));
	DDPDUMP("DPI MMSYS_CG_CON0: 0x%08x\n", INREG32(DISP_REG_CONFIG_MMSYS_CG_CON0));
	DDPDUMP("DPI MMSYS_CG_CON1: 0x%08x\n", INREG32(DISP_REG_CONFIG_MMSYS_CG_CON1));
}

static void dpi_dump_analysis(void)
{
	DDPDUMP("==DISP DPI ANALYSIS==\n");
	DDPDUMP("DPI clock=0x%x\n", DISP_REG_GET(DISP_REG_CLK_CFG_6_DPI));
	DDPDUMP("DPI  clock_clear=%d\n", (DISP_REG_GET(DISP_REG_CLK_CFG_6_CLR) >> 7) & 0x1);
}

int ddp_dump_reg(DISP_MODULE_ENUM module)
{
	switch (module) {
	case DISP_MODULE_WDMA0:
		wdma_dump_reg(module);
		break;
	case DISP_MODULE_RDMA0:
	case DISP_MODULE_RDMA1:
	case DISP_MODULE_RDMA2:
		rdma_dump_reg(module);
		break;
	case DISP_MODULE_OVL0:
	case DISP_MODULE_OVL1:
	case DISP_MODULE_OVL0_2L:
	case DISP_MODULE_OVL1_2L:
		ovl_dump_reg(module);
		break;
	case DISP_MODULE_GAMMA:
		gamma_dump_reg();
		break;
	case DISP_MODULE_CONFIG:
		mmsys_config_dump_reg();
		break;
	case DISP_MODULE_MUTEX:
		mutex_dump_reg();
		break;
	case DISP_MODULE_MERGE:
		merge_dump_reg();
		break;
	case DISP_MODULE_SPLIT0:
	case DISP_MODULE_SPLIT1:
		split_dump_reg(module);
		break;
	case DISP_MODULE_COLOR0:
	case DISP_MODULE_COLOR1:
		color_dump_reg(module);
		break;
	case DISP_MODULE_AAL:
		aal_dump_reg();
		break;
	case DISP_MODULE_PWM0:
	case DISP_MODULE_PWM1:
		pwm_dump_reg(module);
		break;
	case DISP_MODULE_OD:
		od_dump_reg();
		break;
	case DISP_MODULE_DSI0:
	case DISP_MODULE_DSI1:
		dsi_dump_reg(module);
		break;
	case DISP_MODULE_DPI:
		dpi_dump_reg();
		break;
	case DISP_MODULE_CCORR:
		ccorr_dump_reg();
		break;
	case DISP_MODULE_DITHER:
		dither_dump_reg();
		break;
	default:
		DDPDUMP("no dump_reg for module %s(%d)\n", ddp_get_module_name(module), module);
	}
	return 0;
}

int ddp_dump_analysis(DISP_MODULE_ENUM module)
{
	switch (module) {
	case DISP_MODULE_WDMA0:
		wdma_dump_analysis(module);
		break;
	case DISP_MODULE_RDMA0:
	case DISP_MODULE_RDMA1:
	case DISP_MODULE_RDMA2:
		rdma_dump_analysis(module);
		break;
	case DISP_MODULE_OVL0:
	case DISP_MODULE_OVL1:
	case DISP_MODULE_OVL0_2L:
	case DISP_MODULE_OVL1_2L:
		ovl_dump_analysis(module);
		break;
	case DISP_MODULE_GAMMA:
		gamma_dump_analysis();
		break;
	case DISP_MODULE_CONFIG:
		mmsys_config_dump_analysis();
		break;
	case DISP_MODULE_MUTEX:
		mutex_dump_analysis();
		break;
	case DISP_MODULE_MERGE:
		merge_dump_analysis();
		break;
	case DISP_MODULE_SPLIT0:
	case DISP_MODULE_SPLIT1:
		split_dump_analysis(module);
		break;
	case DISP_MODULE_COLOR0:
	case DISP_MODULE_COLOR1:
		color_dump_analysis(module);
		break;
	case DISP_MODULE_AAL:
		aal_dump_analysis();
		break;
	case DISP_MODULE_OD:
		od_dump_analysis();
		break;
	case DISP_MODULE_PWM0:
	case DISP_MODULE_PWM1:
		pwm_dump_analysis(module);
		break;
	case DISP_MODULE_DSI0:
	case DISP_MODULE_DSI1:
	case DISP_MODULE_DSIDUAL:
		dsi_analysis(module);
		break;
	case DISP_MODULE_DPI:
		dpi_dump_analysis();
		break;
	case DISP_MODULE_CCORR:
		ccorr_dump_analyze();
		break;
	case DISP_MODULE_DITHER:
		dither_dump_analyze();
		break;
	default:
		DDPDUMP("no dump_analysis for module %s(%d)\n", ddp_get_module_name(module),
			module);
	}
	return 0;
}
