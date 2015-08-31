/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <nexell/platform.h>
#include <nexell/display.h>
#include <nexell/soc-s5pxx18.h>

/*
#define pr_debug(msg...)		printk(msg);
*/

#define	DEV_NAME_LVDS	"nxp-lvds"

static int  lvds_set_vsync(struct disp_process_dev *pdev, struct disp_vsync_info *psync)
{
	RET_ASSERT_VAL(pdev && psync, -EINVAL);
	pr_debug("%s: %s\n", __func__, dev_to_str(pdev->dev_id));

	pdev->status |= PROC_STATUS_READY;
	memcpy(&pdev->vsync, psync , sizeof(*psync));

	return 0;
}

static int lvds_get_vsync(struct disp_process_dev *pdev, struct disp_vsync_info *psync)
{
	printk("%s: %s\n", __func__, dev_to_str(pdev->dev_id));
	RET_ASSERT_VAL(pdev, -EINVAL);

	if (psync)
		memcpy(psync, &pdev->vsync, sizeof(*psync));

	return 0;
}

static int  lvds_prepare(struct disp_process_dev *pdev)
{
	struct disp_vsync_info *psync = &pdev->vsync;
	struct disp_lvds_param *plvds = pdev->dev_param;
	int input = pdev->dev_in;
	int clkid = DISP_CLOCK_LVDS;
	unsigned int val;
	int format = 1;	// 1: JEiDA, 0: VESA, 2:User

	//-------- 미리 정의된 형식.
	// VESA에서 iTA만 iTE로 교체한  형식으로 넣어준다.
	// wire [34:0] loc_VideoIn = {4'hf, 4'h0, i_VDEN, i_VSYNC, i_HSYNC, i_VD[23:0] };
	U32 VSYNC = 25;
	U32 HSYNC = 24;
	U32 VDEN  = 26; // bit 위치.
	U32 ONE   = 34;
	U32 ZERO  = 27;

	//====================================================
	// 일단 현재는 location은 사용하고 있지 않다.
	//====================================================
	U32 LOC_A[7] = {ONE,ONE,ONE,ONE,ONE,ONE,ONE};
	U32 LOC_B[7] = {ONE,ONE,ONE,ONE,ONE,ONE,ONE};
	U32 LOC_C[7] = {VDEN,VSYNC,HSYNC,ONE, HSYNC, VSYNC, VDEN};
	U32 LOC_D[7] = {ZERO,ZERO,ZERO,ZERO,ZERO,ZERO,ZERO};
	U32 LOC_E[7] = {ZERO,ZERO,ZERO,ZERO,ZERO,ZERO,ZERO};

	RET_ASSERT_VAL(DISP_DEVICE_END > pdev->dev_id, -EINVAL);
	RET_ASSERT_VAL(pdev->dev_in == DISP_DEVICE_SYNCGEN0 ||
				   pdev->dev_in == DISP_DEVICE_SYNCGEN1 ||
				   pdev->dev_in == DISP_DEVICE_RESCONV, -EINVAL);

	if (plvds)
		format = plvds->lcd_format;

	pr_debug("%s: [%d]=%s, in[%d]=%s, %s\n",
		__func__, pdev->dev_id, dev_to_str(pdev->dev_id), input, dev_to_str(input),
		(format==0?"VESA":(format==1?"JEIDA":"LOC")));

	switch (input) {
	case DISP_DEVICE_SYNCGEN0:	input = 0; break;
	case DISP_DEVICE_SYNCGEN1:	input = 1; break;
	case DISP_DEVICE_RESCONV  :	input = 2; break;
	default:
		return -EINVAL;
	}

	/*
	if (LVDS_PCLK_L_MIN > psync->pixel_clock_hz) {
		printk(KERN_ERR "Fail, invalid pixel clock %u (min: %u)\n",
			psync->pixel_clock_hz, LVDS_PCLK_L_MIN);
		return -EINVAL;
	}
	if (psync->pixel_clock_hz > LVDS_PCLK_H_MAX) {
		printk(KERN_ERR  "Fail, invalid pixel clock %u (max: %u)\n",
			psync->pixel_clock_hz, LVDS_PCLK_H_MAX);
		return -EINVAL;
	}
	*/

	/*
	 * select TOP MUX
	 */
	NX_DISPTOP_CLKGEN_SetClockPClkMode(clkid, NX_PCLKMODE_ALWAYS);
	NX_DISPTOP_CLKGEN_SetClockDivisorEnable (clkid, CFALSE);
	NX_DISPTOP_CLKGEN_SetClockSource (clkid, 0, psync->clk_src_lv0);
	NX_DISPTOP_CLKGEN_SetClockDivisor(clkid, 0, psync->clk_div_lv0);
	NX_DISPTOP_CLKGEN_SetClockSource (clkid, 1, psync->clk_src_lv1);  // CLKSRC_PLL0
	NX_DISPTOP_CLKGEN_SetClockDivisor(clkid, 1, psync->clk_div_lv1);
//	NX_DISPTOP_CLKGEN_SetClockDivisorEnable(clkid, CTRUE);

	pr_debug("%s: clkid=%d, src0=%d, div0=%d, src1=%d, div1=%d\n",
		__func__, clkid, psync->clk_src_lv0, psync->clk_div_lv0, psync->clk_src_lv1, psync->clk_div_lv1);

	//===============
	// LVDS Control Pin Setting
	//===============
	val =	(0<<30)  // CPU_I_VBLK_FLAG_SEL
			|	(0<<29)  // CPU_I_BVLK_FLAG
			|	(1<<28)  // SKINI_BST
			|	(1<<27)  // DLYS_BST
			|	(0<<26)  // I_AUTO_SEL
			|	(format<<19)  // JEiDA data packing
			|	(0x1B<<13)  // I_LOCK_PPM_SET, PPM setting for PLL lock
			|	(0x638<<1)  // I_DESKEW_CNT_SEL, period of de-skew region
			;
	NX_LVDS_SetLVDSCTRL0(    0, val );

	val =	(0<<28) // I_ATE_MODE, funtion mode
			|	(0<<27) // I_TEST_CON_MODE, DA (test ctrl mode)
			|	(0<<24) // I_TX4010X_DUMMY
			|	(0<<15) // SKCCK 0
			|	(0<<12) // SKC4 (TX output skew control pin at ODD ch4)
			|	(0<<9)  // SKC3 (TX output skew control pin at ODD ch3)
			|	(0<<6)  // SKC2 (TX output skew control pin at ODD ch2)
			|	(0<<3)  // SKC1 (TX output skew control pin at ODD ch1)
			|	(0<<0)  // SKC0 (TX output skew control pin at ODD ch0)
				;
	NX_LVDS_SetLVDSCTRL1(    0, val );

	val =	(0<<15) // CK_POL_SEL, Input clock, bypass
			|	(0<<14) // VSEL, VCO Freq. range. 0: Low(40MHz~90MHz), 1:High(90MHz~160MHz)
			|	(0x1<<12) // S (Post-scaler)
			|	(0xA<<6) // M (Main divider)
			|	(0xA<<0) // P (Pre-divider)
			;
	NX_LVDS_SetLVDSCTRL2(    0, val );

	val =	(0x03<<6) // SK_BIAS, Bias current ctrl pin
			|	(0<<5) // SKEWINI, skew selection pin, 0 : bypass, 1 : skew enable
			|	(0<<4) // SKEW_EN_H, skew block power down, 0 : power down, 1 : operating
			|	(1<<3) // CNTB_TDLY, delay control pin
			|	(0<<2) // SEL_DATABF, input clock 1/2 division control pin
			|	(0x3<<0) // SKEW_REG_CUR, regulator bias current selection in in SKEW block
			;
	NX_LVDS_SetLVDSCTRL3(    0, val );

	val =	(0<<28) // FLT_CNT, filter control pin for PLL
			|	(0<<27) // VOD_ONLY_CNT, the pre-emphasis's pre-diriver control pin (VOD only)
			|	(0<<26) // CNNCT_MODE_SEL, connectivity mode selection, 0:TX operating, 1:con check
			|	(0<<24) // CNNCT_CNT, connectivity ctrl pin, 0:tx operating, 1: con check
			|	(0<<23) // VOD_HIGH_S, VOD control pin, 1 : Vod only
			|	(0<<22) // SRC_TRH, source termination resistor select pin
			|	(0x3F<<14) // CNT_VOD_H, TX driver output differential volatge level control pin
			|	(0x01<<6) // CNT_PEN_H, TX driver pre-emphasis level control
			|	(0x4<<3) // FC_CODE, vos control pin
			|	(0<<2) // OUTCON, TX Driver state selectioin pin, 0:Hi-z, 1:Low
			|	(0<<1) // LOCK_CNT, Lock signal selection pin, enable
			|	(0<<0) // AUTO_DSK_SEL, auto deskew selection pin, normal
			;
	NX_LVDS_SetLVDSCTRL4(    0, val );

	val =	(0<<24)	// I_BIST_RESETB
			|	(0<<23)	// I_BIST_EN
			|	(0<<21)	// I_BIST_PAT_SEL
			|	(0<<14) // I_BIST_USER_PATTERN
			|	(0<<13)	// I_BIST_FORCE_ERROR
			|	(0<<7)	// I_BIST_SKEW_CTRL
			|	(0<<5)	// I_BIST_CLK_INV
			|	(0<<3)	// I_BIST_DATA_INV
			|	(0<<0)	// I_BIST_CH_SEL
			;
	NX_LVDS_SetLVDSTMODE0( 0, val );

	// user do not need to modify this codes.
	val = (LOC_A[4]  <<24) | (LOC_A[3]  <<18) | (LOC_A[2]  <<12) | (LOC_A[1]  <<6) | (LOC_A[0]  <<0);
	NX_LVDS_SetLVDSLOC0( 0, val );

	val = (LOC_B[2]  <<24) | (LOC_B[1]  <<18) | (LOC_B[0]  <<12) | (LOC_A[6]  <<6) | (LOC_A[5]  <<0);
	NX_LVDS_SetLVDSLOC1( 0, val );

	val = (LOC_C[0]  <<24) | (LOC_B[6]  <<18) | (LOC_B[5]  <<12) | (LOC_B[4]  <<6) | (LOC_B[3]  <<0);
	NX_LVDS_SetLVDSLOC2( 0, val );

	val = (LOC_C[5]  <<24) | (LOC_C[4]  <<18) | (LOC_C[3]  <<12) | (LOC_C[2]  <<6) | (LOC_C[1]  <<0);
	NX_LVDS_SetLVDSLOC3( 0, val );

	val = (LOC_D[3]  <<24) | (LOC_D[2]  <<18) | (LOC_D[1]  <<12) | (LOC_D[0]  <<6) | (LOC_C[6]  <<0);
	NX_LVDS_SetLVDSLOC4( 0, val );

	val = (LOC_E[1]  <<24) | (LOC_E[0]  <<18) | (LOC_D[6]  <<12) | (LOC_D[5]  <<6) | (LOC_D[4]  <<0);
	NX_LVDS_SetLVDSLOC5( 0, val );

	val = (LOC_E[6]  <<24) | (LOC_E[5]  <<18) | (LOC_E[4]  <<12) | (LOC_E[3]  <<6) | (LOC_E[2]  <<0);
	NX_LVDS_SetLVDSLOC6( 0, val );

	NX_LVDS_SetLVDSLOCMASK0( 0, 0xffffffff );
	NX_LVDS_SetLVDSLOCMASK1( 0, 0xffffffff );

	NX_LVDS_SetLVDSLOCPOL0( 0, (0<<19) | ( 0<<18) );

	/*
	 * select TOP MUX
	 */
	NX_DISPLAYTOP_SetLVDSMUX(CTRUE, input);

	return 0;
}

static int  lvds_enable(struct disp_process_dev *pdev, int enable)
{
	int clkid = DISP_CLOCK_LVDS;
	int rstn = NX_LVDS_GetResetNumber(0);
	pr_debug("%s %s, %s\n", __func__, dev_to_str(pdev->dev_id), enable?"ON":"OFF");

	if (!enable) {
  		NX_DISPTOP_CLKGEN_SetClockDivisorEnable(clkid, CFALSE);
  		pdev->status &= ~PROC_STATUS_ENABLE;
	} else {
        nxp_soc_peri_reset_enter(rstn);	/* power down */

		/* prepare */
		lvds_prepare(pdev);

		/* START: CLKGEN, LVDS is started in setup function*/
  		NX_DISPTOP_CLKGEN_SetClockDivisorEnable(clkid, CTRUE);
		/* LVDS PHY Reset, make sure last. */
		nxp_soc_peri_reset_exit(rstn);
		pdev->status |=  PROC_STATUS_ENABLE;
  	}
  	return 0;
}

static int  lvds_stat_enable(struct disp_process_dev *pdev)
{
	int clkid = DISP_CLOCK_LVDS;
	CBOOL ret = NX_DISPTOP_CLKGEN_GetClockDivisorEnable(clkid);

	if (CTRUE == ret)
		pdev->status |=  PROC_STATUS_ENABLE;
	else
		pdev->status &= ~PROC_STATUS_ENABLE;

	return pdev->status & PROC_STATUS_ENABLE ? 1 : 0;
}

static int  lvds_suspend(struct disp_process_dev *pdev)
{
	pm_dbgout("%s\n", __func__);
	return lvds_enable(pdev, 0);
}

static void lvds_resume(struct disp_process_dev *pdev)
{
	pm_dbgout("%s\n", __func__);
	lvds_enable(pdev, 1);
}

static struct disp_process_ops lvds_ops = {
	.set_vsync 	= lvds_set_vsync,
	.get_vsync  = lvds_get_vsync,
	.enable 	= lvds_enable,
	.stat_enable= lvds_stat_enable,
	.suspend	= lvds_suspend,
	.resume	  	= lvds_resume,
};

static void lvds_initialize(void)
{
	int clkid = DISP_CLOCK_LVDS;
	int i;

	/* BASE : CLKGEN, LVDS */
	NX_DISPTOP_CLKGEN_SetBaseAddress(clkid,
		(void*)IO_ADDRESS(NX_DISPTOP_CLKGEN_GetPhysicalAddress(clkid)));
	NX_DISPTOP_CLKGEN_SetClockPClkMode(clkid, NX_PCLKMODE_ALWAYS);

	/* BASE : LVDS */
	NX_LVDS_Initialize();
	for (i = 0; NX_LVDS_GetNumberOfModule() > i; i++)
		NX_LVDS_SetBaseAddress(i, (void*)IO_ADDRESS(NX_LVDS_GetPhysicalAddress(i)));
}

#ifdef CONFIG_OF
static const struct of_device_id lvds_dt_match[];

static int lvds_get_dt_data(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *np = node;
	const struct of_device_id *match;
	struct nxp_lcd_plat_data *plat;
	struct disp_lvds_param *lvds;
	struct disp_vsync_info *psync;

	match = of_match_node(lvds_dt_match, pdev->dev.of_node);
	RET_ASSERT_VAL(match, -EINVAL);

	plat = (struct nxp_lcd_plat_data *)match->data;
	lvds = (struct disp_lvds_param *)plat->dev_param;
	psync = plat->vsync;

	of_property_read_u32(np, "display_in", &plat->display_in);
	of_property_read_u32(np, "lvds_format", &lvds->lcd_format);

	np = of_get_child_by_name(node, "vsync");
	RET_ASSERT_VAL(np, -EINVAL);

	of_property_read_u32(np, "interlace", &psync->interlace);
	of_property_read_u32(np, "hor_active_size", &psync->h_active_len);
	of_property_read_u32(np, "hor_sync_width", &psync->h_sync_width);
	of_property_read_u32(np, "hor_back_porch", &psync->h_back_porch);
	of_property_read_u32(np, "hor_front_porch", &psync->h_front_porch);
	of_property_read_u32(np, "hor_sync_invert", &psync->h_sync_invert);
	of_property_read_u32(np, "ver_active_size", &psync->v_active_len);
	of_property_read_u32(np, "ver_sync_width", &psync->v_sync_width);
	of_property_read_u32(np, "ver_back_porch", &psync->v_back_porch);
	of_property_read_u32(np, "ver_front_porch", &psync->v_front_porch);
	of_property_read_u32(np, "ver_sync_invert", &psync->v_sync_invert);
	of_property_read_u32(np, "pixel_clock_hz", &psync->pixel_clock_hz);

	np = of_get_child_by_name(node, "clock");
	RET_ASSERT_VAL(np, -EINVAL);

	of_property_read_u32(np, "clock,src_div0", &psync->clk_src_lv0);
	of_property_read_u32(np, "clock,div_val0", &psync->clk_div_lv0);
	of_property_read_u32(np, "clock,src_div1", &psync->clk_src_lv1);
	of_property_read_u32(np, "clock,div_val1", &psync->clk_div_lv1);

	pdev->dev.platform_data = plat;

	return 0;
}
#endif

static int lvds_probe(struct platform_device *pdev)
{
	struct nxp_lcd_plat_data *plat = pdev->dev.platform_data;
	struct disp_lvds_param *plvds;
	int device = DISP_DEVICE_LVDS;
	int ret = 0, input;

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		ret = lvds_get_dt_data(pdev);
		RET_ASSERT_VAL(0 == ret, -EINVAL);
	}
	plat = pdev->dev.platform_data;
#endif

	RET_ASSERT_VAL(plat, -EINVAL);
	RET_ASSERT_VAL(plat->display_in == DISP_DEVICE_SYNCGEN0 ||
				   plat->display_in == DISP_DEVICE_SYNCGEN1 ||
				   plat->display_dev == DISP_DEVICE_LVDS ||
				   plat->display_in == DISP_DEVICE_RESCONV, -EINVAL);
	RET_ASSERT_VAL(plat->vsync, -EINVAL);
	RET_ASSERT_VAL(plat->dev_param, -EINVAL);

	plvds = kzalloc(sizeof(*plvds), GFP_KERNEL);
	RET_ASSERT_VAL(plvds, -EINVAL);
	memcpy(plvds, plat->dev_param, sizeof(*plvds));

	input = plat->display_in;

	lvds_initialize();

	nxp_soc_disp_register_proc_ops(device, &lvds_ops);
	nxp_soc_disp_device_connect_to(device, input, plat->vsync);
	nxp_soc_disp_device_set_dev_param(device, plvds);

	if (plat->sync_par && (input == DISP_DEVICE_SYNCGEN0 ||
		input == DISP_DEVICE_SYNCGEN1))
		nxp_soc_disp_device_set_sync_param(input, plat->sync_par);

	printk("LVDS: [%d]=%s connect to [%d]=%s\n",
		device, dev_to_str(device), input, dev_to_str(input));

	return 0;
}

#ifdef CONFIG_OF
static struct disp_vsync_info lvds_vsync;
static struct disp_lvds_param lvds_param;

static struct nxp_lcd_plat_data lvds_dt_data = {
	.display_dev = DISP_DEVICE_LVDS,
	.vsync = &lvds_vsync,
	.dev_param = (union disp_dev_param*)&lvds_param,
};

static const struct of_device_id lvds_dt_match[] = {
	{
	.compatible = "nexell,nxp-lvds",
	.data = (void*)&lvds_dt_data,
	}, {},
};
MODULE_DEVICE_TABLE(of, lvds_dt_match);
#else
#define lvds_dt_match NULL
#endif

static struct platform_driver lvds_driver = {
	.probe	= lvds_probe,
	.driver	= {
		.name	= DEV_NAME_LVDS,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(lvds_dt_match),
	},
};

static int __init lvds_initcall(void)
{
	return platform_driver_register(&lvds_driver);
}
subsys_initcall(lvds_initcall);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Display LVDS driver for the Nexell");
MODULE_LICENSE("GPL");
