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

#define	DEV_NAME_MIPI	"nxp-mipi-dsi"

static int  mipi_set_vsync(struct disp_process_dev *pdev, struct disp_vsync_info *psync)
{
	RET_ASSERT_VAL(pdev && psync, -EINVAL);
	pr_debug("%s: %s\n", __func__, dev_to_str(pdev->dev_id));

	pdev->status |= PROC_STATUS_READY;
	memcpy(&pdev->vsync, psync , sizeof(*psync));

	return 0;
}

static int mipi_get_vsync(struct disp_process_dev *pdev, struct disp_vsync_info *psync)
{
	printk("%s: %s\n", __func__, dev_to_str(pdev->dev_id));
	RET_ASSERT_VAL(pdev, -EINVAL);

	if (psync)
		memcpy(psync, &pdev->vsync, sizeof(*psync));

	return 0;
}

static int  mipi_prepare(struct disp_process_dev *pdev)
{
	struct disp_vsync_info *psync = &pdev->vsync;
	struct disp_mipi_param *pmipi = pdev->dev_param;
	int input = pdev->dev_in;
	int index = 0;
	int clkid = DISP_CLOCK_MIPI;
	int width  = psync->h_active_len;
	int height = psync->v_active_len;
	int ret = 0;

	int HFP = psync->h_front_porch;
	int HBP = psync->h_back_porch;
	int HS  = psync->h_sync_width;
	int VFP = psync->v_front_porch;
	int VBP = psync->v_back_porch;
	int VS  = psync->v_sync_width;
	unsigned int pllpms, bandctl, pllctl, phyctl;

	RET_ASSERT_VAL(pmipi, -EINVAL);
	RET_ASSERT_VAL(DISP_DEVICE_END > pdev->dev_id, -EINVAL);
	RET_ASSERT_VAL(pdev->dev_in == DISP_DEVICE_SYNCGEN0 ||
				   pdev->dev_in == DISP_DEVICE_SYNCGEN1 ||
				   pdev->dev_in == DISP_DEVICE_RESCONV, -EINVAL);

	pr_debug("%s: [%d]=%s, in[%d]=%s\n",
		__func__, pdev->dev_id, dev_to_str(pdev->dev_id), input, dev_to_str(input));

	pllpms  = pmipi->pllpms;
	bandctl = pmipi->bandctl;
	pllctl  = pmipi->pllctl;
	phyctl  = pmipi->phyctl;

	switch (input) {
	case DISP_DEVICE_SYNCGEN0:	input = 0; break;
	case DISP_DEVICE_SYNCGEN1:	input = 1; break;
	case DISP_DEVICE_RESCONV  :	input = 2; break;
	default:
		return -EINVAL;
	}

	NX_MIPI_DSI_SetPLL(index
			,CTRUE      // CBOOL Enable      ,
            ,0xFFFFFFFF // U32 PLLStableTimer,
            ,pllpms     // 19'h033E8: 1Ghz  // Use LN28LPP_MipiDphyCore1p5Gbps_Supplement.
            ,bandctl    // 4'hF     : 1Ghz  // Use LN28LPP_MipiDphyCore1p5Gbps_Supplement.
            ,pllctl     // U32 M_PLLCTL      , // Refer to 10.2.2 M_PLLCTL of MIPI_D_PHY_USER_GUIDE.pdf  Default value is all "0". If you want to change register values, it need to confirm from IP Design Team
            ,phyctl		// U32 B_DPHYCTL       // Refer to 10.2.3 M_PLLCTL of MIPI_D_PHY_USER_GUIDE.pdf or NX_MIPI_PHY_B_DPHYCTL enum or LN28LPP_MipiDphyCore1p5Gbps_Supplement. default value is all "0". If you want to change register values, it need to confirm from IP Design Team
			);

	if (pmipi->lcd_init) {
		NX_MIPI_DSI_SoftwareReset(index);
	    NX_MIPI_DSI_SetClock (index
	    		,0  // CBOOL EnableTXHSClock    ,
	            ,0  // CBOOL UseExternalClock   , // CFALSE: PLL clock CTRUE: External clock
	            ,1  // CBOOL EnableByteClock    , // ByteClock means (D-PHY PLL clock / 8)
	            ,1  // CBOOL EnableESCClock_ClockLane,
	            ,1  // CBOOL EnableESCClock_DataLane0,
	            ,0  // CBOOL EnableESCClock_DataLane1,
	            ,0  // CBOOL EnableESCClock_DataLane2,
	            ,0  // CBOOL EnableESCClock_DataLane3,
	            ,1  // CBOOL EnableESCPrescaler , // ESCClock = ByteClock / ESCPrescalerValue
	            ,5  // U32   ESCPrescalerValue
	   			);

		NX_MIPI_DSI_SetPhy( index
				,0 // U32   NumberOfDataLanes , // 0~3
	            ,1 // CBOOL EnableClockLane   ,
	            ,1 // CBOOL EnableDataLane0   ,
	            ,0 // CBOOL EnableDataLane1   ,
	            ,0 // CBOOL EnableDataLane2   ,
	            ,0 // CBOOL EnableDataLane3   ,
	            ,0 // CBOOL SwapClockLane     ,
	            ,0 // CBOOL SwapDataLane      )
				);

		ret = pmipi->lcd_init(width, height, pmipi->private_data);
		if (0 > ret)
			return ret;
	}

	NX_MIPI_DSI_SoftwareReset(index);
    NX_MIPI_DSI_SetClock (index
    		,1  // CBOOL EnableTXHSClock    ,
            ,0  // CBOOL UseExternalClock   , // CFALSE: PLL clock CTRUE: External clock
            ,1  // CBOOL EnableByteClock    , // ByteClock means (D-PHY PLL clock / 8)
            ,1  // CBOOL EnableESCClock_ClockLane,
            ,1  // CBOOL EnableESCClock_DataLane0,
            ,1  // CBOOL EnableESCClock_DataLane1,
            ,1  // CBOOL EnableESCClock_DataLane2,
            ,1  // CBOOL EnableESCClock_DataLane3,
            ,1  // CBOOL EnableESCPrescaler , // ESCClock = ByteClock / ESCPrescalerValue
            ,5  // U32   ESCPrescalerValue
   			);

	NX_MIPI_DSI_SetPhy( index
			,3 // U32   NumberOfDataLanes , // 0~3
            ,1 // CBOOL EnableClockLane   ,
            ,1 // CBOOL EnableDataLane0   ,
            ,1 // CBOOL EnableDataLane1   ,
            ,1 // CBOOL EnableDataLane2   ,
            ,1 // CBOOL EnableDataLane3   ,
            ,0 // CBOOL SwapClockLane     ,
            ,0 // CBOOL SwapDataLane      )
			);

	NX_MIPI_DSI_SetConfigVideoMode  (index
			,1   // CBOOL EnableAutoFlushMainDisplayFIFO ,
			,0   // CBOOL EnableAutoVerticalCount        ,
			,1,NX_MIPI_DSI_SYNCMODE_EVENT // CBOOL EnableBurst, NX_MIPI_DSI_SYNCMODE SyncMode,
			//,0,NX_MIPI_DSI_SYNCMODE_PULSE // CBOOL EnableBurst, NX_MIPI_DSI_SYNCMODE SyncMode,
			,1   // CBOOL EnableEoTPacket                ,
			,1   // CBOOL EnableHsyncEndPacket           , // Set HSEMode=1
			,1   // CBOOL EnableHFP                      , // Set HFPMode=0
			,1   // CBOOL EnableHBP                      , // Set HBPMode=0
			,1   // CBOOL EnableHSA                      , // Set HSAMode=0
			,0   // U32   NumberOfVirtualChannel         , // 0~3
			,NX_MIPI_DSI_FORMAT_RGB888   // NX_MIPI_DSI_FORMAT Format            ,
			,HFP  // U32   NumberOfWordsInHFP             , // ~65535
			,HBP  // U32   NumberOfWordsInHBP             , // ~65535
			,HS   // U32   NumberOfWordsInHSYNC           , // ~65535
			,VFP  // U32   NumberOfLinesInVFP             , // ~2047
			,VBP   // U32   NumberOfLinesInVBP             , // ~2047
			,VS    // U32   NumberOfLinesInVSYNC           , // ~1023
			,0 // U32   NumberOfLinesInCommandAllow
    		);

	NX_MIPI_DSI_SetSize(index, width, height);

	NX_DISPLAYTOP_SetMIPIMUX(CTRUE, input);

	// 0 is spdif, 1 is mipi vclk
	NX_DISPTOP_CLKGEN_SetClockSource (clkid, 0, psync->clk_src_lv0);
	NX_DISPTOP_CLKGEN_SetClockDivisor(clkid, 0, psync->clk_div_lv0);
	NX_DISPTOP_CLKGEN_SetClockSource (clkid, 1, psync->clk_src_lv1);  // CLKSRC_PLL0
	NX_DISPTOP_CLKGEN_SetClockDivisor(clkid, 1, psync->clk_div_lv1);

	return 0;
}

static int  mipi_enable(struct disp_process_dev *pdev, int enable)
{
	int clkid = DISP_CLOCK_MIPI;
	CBOOL on = (enable ? CTRUE : CFALSE);
	pr_debug("%s %s, %s\n", __func__, dev_to_str(pdev->dev_id), enable?"ON":"OFF");

	if (enable)
		mipi_prepare(pdev);

	/* SPDIF and MIPI */
    NX_DISPTOP_CLKGEN_SetClockDivisorEnable(clkid, CTRUE);

	/* START: CLKGEN, MIPI is started in setup function*/
  	NX_DISPTOP_CLKGEN_SetClockDivisorEnable(clkid, on);
	NX_MIPI_DSI_SetEnable(0, on);
  	return 0;
}

static int  mipi_stat_enable(struct disp_process_dev *pdev)
{
	return pdev->status & PROC_STATUS_ENABLE ? 1 : 0;
}

static int  mipi_suspend(struct disp_process_dev *pdev)
{
	pm_dbgout("%s\n", __func__);
	return mipi_enable(pdev, 0);
}

static void mipi_resume(struct disp_process_dev *pdev)
{
	int index = 0;
	pm_dbgout("%s\n", __func__);

	NX_TIEOFF_Set(TIEOFFINDEX_OF_MIPI0_NX_DPSRAM_1R1W_EMAA, 3);
	NX_TIEOFF_Set(TIEOFFINDEX_OF_MIPI0_NX_DPSRAM_1R1W_EMAB, 3);
	if (! nxp_soc_peri_reset_status(NX_MIPI_GetResetNumber(index, NX_MIPI_RST))) {
	    nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST));
    	nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_DSI_I));
    	nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_S));
    	nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_M));
    	nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST));
    	nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_DSI_I));
		nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_S));
    	nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_M));
    }
	mipi_enable(pdev, 1);
}

static struct disp_process_ops mipi_ops = {
	.set_vsync 	= mipi_set_vsync,
	.get_vsync  = mipi_get_vsync,
	.enable 	= mipi_enable,
	.stat_enable= mipi_stat_enable,
	.suspend	= mipi_suspend,
	.resume	  	= mipi_resume,
};

static void mipi_initialize(void)
{
	int clkid = DISP_CLOCK_MIPI;
	int index = 0;

	NX_TIEOFF_Set(TIEOFFINDEX_OF_MIPI0_NX_DPSRAM_1R1W_EMAA, 3);
	NX_TIEOFF_Set(TIEOFFINDEX_OF_MIPI0_NX_DPSRAM_1R1W_EMAB, 3);

	if (! nxp_soc_peri_reset_status(NX_MIPI_GetResetNumber(index, NX_MIPI_RST))) {
	    nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST));
    	nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_DSI_I));
    	nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_S));
    	nxp_soc_peri_reset_enter(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_M));
    	nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST));
    	nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_DSI_I));
		nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_S));
    	nxp_soc_peri_reset_exit(NX_MIPI_GetResetNumber(index, NX_MIPI_RST_PHY_M));
    }

	/* BASE : CLKGEN, MIPI */
	NX_DISPTOP_CLKGEN_SetBaseAddress(clkid, (void*)IO_ADDRESS(NX_DISPTOP_CLKGEN_GetPhysicalAddress(clkid)));
	NX_DISPTOP_CLKGEN_SetClockPClkMode(clkid, NX_PCLKMODE_ALWAYS);

	/* BASE : MIPI */
	NX_MIPI_Initialize();
    NX_MIPI_SetBaseAddress(0, (void*)IO_ADDRESS(NX_MIPI_GetPhysicalAddress(0)));
	NX_MIPI_OpenModule(0);
}

#ifdef CONFIG_OF
static const struct of_device_id mipi_dt_match[];

static int mipi_get_dt_data(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *np = node;
	const struct of_device_id *match;
	struct nxp_lcd_plat_data *plat;
	struct disp_vsync_info *psync;

	match = of_match_node(mipi_dt_match, pdev->dev.of_node);
	RET_ASSERT_VAL(match, -EINVAL);

	plat = (struct nxp_lcd_plat_data *)match->data;
	psync = plat->vsync;

	of_property_read_u32(np, "display_in", &plat->display_in);

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

	/* TO DO for MIPI parameters
		np = of_get_child_by_name(node, "mipi");
	*/

	pdev->dev.platform_data = plat;

	return 0;
}
#endif

static int mipi_probe(struct platform_device *pdev)
{
	struct nxp_lcd_plat_data *plat = pdev->dev.platform_data;
	struct disp_mipi_param *pmipi;
	int device = DISP_DEVICE_MIPI;
	int ret = 0, input;

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		ret = mipi_get_dt_data(pdev);
		RET_ASSERT_VAL(0 == ret, -EINVAL);
	}
	plat = pdev->dev.platform_data;
#endif

	RET_ASSERT_VAL(plat, -EINVAL);
	RET_ASSERT_VAL(plat->display_in == DISP_DEVICE_SYNCGEN0 ||
				   plat->display_in == DISP_DEVICE_SYNCGEN1 ||
				   plat->display_dev == DISP_DEVICE_MIPI ||
				   plat->display_in == DISP_DEVICE_RESCONV, -EINVAL);
	RET_ASSERT_VAL(plat->vsync, -EINVAL);
	RET_ASSERT_VAL(plat->dev_param, -EINVAL);

	pmipi = kzalloc(sizeof(*pmipi), GFP_KERNEL);
	RET_ASSERT_VAL(pmipi, -EINVAL);
	memcpy(pmipi, plat->dev_param, sizeof(*pmipi));

	input = plat->display_in;

	mipi_initialize();

	nxp_soc_disp_register_proc_ops(device, &mipi_ops);
	nxp_soc_disp_device_connect_to(device, input, plat->vsync);
	nxp_soc_disp_device_set_dev_param(device, pmipi);

	if (plat->sync_par &&
		(input == DISP_DEVICE_SYNCGEN0 ||
		 input == DISP_DEVICE_SYNCGEN1))
		nxp_soc_disp_device_set_sync_param(input, plat->sync_par);

	printk("MIPI: [%d]=%s connect to [%d]=%s\n",
		device, dev_to_str(device), input, dev_to_str(input));

	return 0;
}

#ifdef CONFIG_OF
static struct disp_vsync_info mipi_vsync;
static struct disp_mipi_param mipi_param;

static struct nxp_lcd_plat_data mipi_dt_data = {
	.display_dev = DISP_DEVICE_MIPI,
	.vsync = &mipi_vsync,
	.dev_param = (union disp_dev_param*)&mipi_param,
};

static const struct of_device_id mipi_dt_match[] = {
	{
	.compatible = "nexell,nxp-mipi-dsi",
	.data = (void*)&mipi_dt_data,
	}, {},
};
MODULE_DEVICE_TABLE(of, mipi_dt_match);
#else
#define mipi_dt_match NULL
#endif

static struct platform_driver mipi_driver = {
	.probe	= mipi_probe,
	.driver	= {
		.name	= DEV_NAME_MIPI,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(mipi_dt_match),
	},
};

static int __init mipi_initcall(void)
{
	return platform_driver_register(&mipi_driver);
}
subsys_initcall(mipi_initcall);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Display MiPi-DSI driver for the Nexell");
MODULE_LICENSE("GPL");
