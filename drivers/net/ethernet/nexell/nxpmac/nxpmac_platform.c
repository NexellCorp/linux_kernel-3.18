/*******************************************************************************
  This contains the functions to handle the platform driver.

  Copyright (C) 2007-2011  STMicroelectronics Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_net.h>

#include <linux/phy.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

#include "nxpmac.h"



#ifdef CONFIG_OF
static uint32_t CFG_ETHER_GMAC_PHY_RST_NUM = -1;

static void __gmac_phy_reset(void)
{
	if (CFG_ETHER_GMAC_PHY_RST_NUM == -1) {
		pr_err("GMAC PHY reset number is not initialized.\n");
		return;
	}

	// Set GPIO nReset
	nxp_soc_gpio_set_io_dir(CFG_ETHER_GMAC_PHY_RST_NUM, 1);
	nxp_soc_gpio_set_out_value(CFG_ETHER_GMAC_PHY_RST_NUM, 1);
	udelay( 100 );
	nxp_soc_gpio_set_out_value(CFG_ETHER_GMAC_PHY_RST_NUM, 0);
	udelay( 100 );
	nxp_soc_gpio_set_out_value(CFG_ETHER_GMAC_PHY_RST_NUM, 1);
	msleep( 30 );

	//gpio_free(CFG_ETHER_GMAC_PHY_RST_NUM);
	pr_debug(" MDIO phy reset !!\n");
}

int gmac_phy_reset(void *priv)
{
	__gmac_phy_reset();

	return 0;
}

int  nxpmac_init(struct platform_device *pdev)
{
	// Clock control
	NX_CLKGEN_Initialize();
	NX_CLKGEN_SetBaseAddress( CLOCKINDEX_OF_DWC_GMAC_MODULE, (void *)__io_address(NX_CLKGEN_GetPhysicalAddress(CLOCKINDEX_OF_DWC_GMAC_MODULE)) );

	NX_CLKGEN_SetClockSource( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, 4);     // Sync mode for 100 & 10Base-T : External RX_clk
	NX_CLKGEN_SetClockDivisor( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, 1);    // Sync mode for 100 & 10Base-T

	NX_CLKGEN_SetClockOutInv( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, CFALSE);    // TX Clk invert off : 100 & 10Base-T

	NX_CLKGEN_SetClockDivisorEnable( CLOCKINDEX_OF_DWC_GMAC_MODULE, CTRUE);

	// Reset control
	NX_RSTCON_Initialize();
	NX_RSTCON_SetBaseAddress((void *)__io_address(NX_RSTCON_GetPhysicalAddress()) );
	NX_RSTCON_SetRST(RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i, RSTCON_NEGATE);
	udelay(100);
	NX_RSTCON_SetRST(RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i, RSTCON_ASSERT);
	udelay(100);
	NX_RSTCON_SetRST(RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i, RSTCON_NEGATE);
	udelay(100);

	// nReset MDIO PHY
	//__gmac_phy_reset();

	printk("NXP mac init .................. %s\n", phy_loopback_test ? "(Loopback)" : "");
	return 0;
}

static int nxpmac_probe_config_dt(struct platform_device *pdev,
				  struct plat_stmmacenet_data *plat,
				  const char **mac)
{
	struct device_node *np = pdev->dev.of_node;
	uint32_t phy_addr;
	uint32_t phy_irq;
	struct device *dev = &pdev->dev;

	if (!np)
		return -ENODEV;

	if (dev && !dev->dma_mask) {
		dev->dma_mask = &dev->coherent_dma_mask;
		dev->coherent_dma_mask = DMA_BIT_MASK(32);
	}

	*mac = of_get_mac_address(np);
	plat->interface = of_get_phy_mode(np);
	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(struct stmmac_mdio_bus_data),
					   GFP_KERNEL);

	if (of_property_read_u32(np, "phy-addr", &phy_addr)) {
		pr_warn("incorrect phy address\n");
		phy_addr = 0;
	}
	if (of_property_read_u32(np, "probed-phy-irq", &phy_irq)) {
		pr_warn("incorrect phy irq\n");
		phy_irq = 0;
	}
	if (of_property_read_u32(np, "reset_gpio", &CFG_ETHER_GMAC_PHY_RST_NUM)) {
		pr_err("incorrect phy reset number\n");
	}
	if (of_property_read_u32(np, "loopback_mode", &phy_loopback_test))
		phy_loopback_test = 0;

	plat->mdio_bus_data->probed_phy_irq = phy_irq;
	plat->mdio_bus_data->phy_reset = gmac_phy_reset;
	plat->mdio_bus_data->phy_mask = 0;
	plat->phy_addr = (int)phy_addr;


	if (of_device_is_compatible(np, "nexell,nxp-gmac")) {
		plat->has_gmac = 1;
		plat->pmt = 1;
		plat->init = nxpmac_init;
	}

	return 0;
}
#else
static int nxpmac_probe_config_dt(struct platform_device *pdev,
				  struct plat_stmmacenet_data *plat,
				  const char **mac)
{
	return -ENOSYS;
}
#endif /* CONFIG_OF */

/**
 * stmmac_pltfr_probe
 * @pdev: platform device pointer
 * Description: platform_device probe function. It allocates
 * the necessary resources and invokes the main to init
 * the net device, register the mdio bus etc.
 */
static int stmmac_pltfr_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct device *dev = &pdev->dev;
	void __iomem *addr = NULL;
	struct stmmac_priv *priv = NULL;
	struct plat_stmmacenet_data *plat_dat = NULL;
	const char *mac = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);

	if (pdev->dev.of_node) {
		plat_dat = devm_kzalloc(&pdev->dev,
					sizeof(struct plat_stmmacenet_data),
					GFP_KERNEL);
		if (!plat_dat) {
			pr_err("%s: ERROR: no memory", __func__);
			return  -ENOMEM;
		}

		ret = nxpmac_probe_config_dt(pdev, plat_dat, &mac);
		if (ret) {
			pr_err("%s: main dt probe failed", __func__);
			return ret;
		}
	} else {
		plat_dat = pdev->dev.platform_data;
	}

	/* Custom initialisation (if needed)*/
	if (plat_dat->init) {
		ret = plat_dat->init(pdev);
		if (unlikely(ret))
			return ret;
	}

	priv = stmmac_dvr_probe(&(pdev->dev), plat_dat, addr);
	if (!priv) {
		pr_err("%s: main driver probe failed", __func__);
		return -ENODEV;
	}

	/* Get MAC address if available (DT) */
	if (mac)
		memcpy(priv->dev->dev_addr, mac, ETH_ALEN);

	/* Get the MAC information */
	priv->dev->irq = platform_get_irq_byname(pdev, "macirq");
	if (priv->dev->irq == -ENXIO) {
		pr_err("%s: ERROR: MAC IRQ configuration "
		       "information not found\n", __func__);
		return -ENXIO;
	}

	/*
	 * On some platforms e.g. SPEAr the wake up irq differs from the mac irq
	 * The external wake up irq can be passed through the platform code
	 * named as "eth_wake_irq"
	 *
	 * In case the wake up interrupt is not passed from the platform
	 * so the driver will continue to use the mac irq (ndev->irq)
	 */
	priv->wol_irq = platform_get_irq_byname(pdev, "eth_wake_irq");
	if (priv->wol_irq == -ENXIO)
		priv->wol_irq = priv->dev->irq;

	priv->lpi_irq = platform_get_irq_byname(pdev, "eth_lpi");

	platform_set_drvdata(pdev, priv->dev);

	pr_debug("STMMAC platform driver registration completed");

	return 0;
}

/**
 * stmmac_pltfr_remove
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and calls the platforms hook and release the resources (e.g. mem).
 */
static int stmmac_pltfr_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret = stmmac_dvr_remove(ndev);

	if (priv->plat->exit)
		priv->plat->exit(pdev);

	platform_set_drvdata(pdev, NULL);

	return ret;
}

#ifdef CONFIG_PM
static int stmmac_pltfr_suspend(struct device *dev)
{
	int ret;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct platform_device *pdev = to_platform_device(dev);

	ret = stmmac_suspend(ndev);
	if (priv->plat->exit)
		priv->plat->exit(pdev);

	return ret;
}

static int stmmac_pltfr_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct platform_device *pdev = to_platform_device(dev);

	if (priv->plat->init)
		priv->plat->init(pdev);

	return stmmac_resume(ndev);
}

int stmmac_pltfr_freeze(struct device *dev)
{
	int ret;
	struct plat_stmmacenet_data *plat_dat = dev_get_platdata(dev);
	struct net_device *ndev = dev_get_drvdata(dev);
	struct platform_device *pdev = to_platform_device(dev);

	ret = stmmac_freeze(ndev);
	if (plat_dat->exit)
		plat_dat->exit(pdev);

	return ret;
}

int stmmac_pltfr_restore(struct device *dev)
{
	struct plat_stmmacenet_data *plat_dat = dev_get_platdata(dev);
	struct net_device *ndev = dev_get_drvdata(dev);
	struct platform_device *pdev = to_platform_device(dev);

	if (plat_dat->init)
		plat_dat->init(pdev);

	return stmmac_restore(ndev);
}

static const struct dev_pm_ops stmmac_pltfr_pm_ops = {
	.suspend = stmmac_pltfr_suspend,
	.resume = stmmac_pltfr_resume,
	.freeze = stmmac_pltfr_freeze,
	.thaw = stmmac_pltfr_restore,
	.restore = stmmac_pltfr_restore,
};
#else
static const struct dev_pm_ops stmmac_pltfr_pm_ops;
#endif /* CONFIG_PM */

static const struct of_device_id nxpmac_dt_ids[] = {
	{ .compatible = "nexell,nxp-gmac"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nxpmac_dt_ids);

struct platform_driver stmmac_pltfr_driver = {
	.probe = stmmac_pltfr_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		   .name = NXPMAC_RESOURCE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &stmmac_pltfr_pm_ops,
		   .of_match_table = of_match_ptr(nxpmac_dt_ids),
		   },
};

MODULE_DESCRIPTION("STMMAC 10/100/1000 Ethernet PLATFORM driver");
MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_LICENSE("GPL");
