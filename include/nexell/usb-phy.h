/*
 * Copyright (C) 2013 Nexell Co.Ltd
 * Author: BongKwan Kook <kook@nexell.co.kr>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __PLAT_USB_OTG_PHY_H
#define __PLAT_USB_OTG_PHY_H __FILE__

/*      
 *  USB HOST (ehci/ohci)    
 */ 
struct nxp_ehci_platdata {
    int (*phy_init)(struct platform_device *pdev, int type);
    int (*phy_exit)(struct platform_device *pdev, int type);
    int (*hsic_phy_pwr_on)(struct platform_device *pdev, bool on);
    int resume_delay_time;  /* unit ms, more than 100 ms */
};

struct nxp_ohci_platdata {
    int (*phy_init)(struct platform_device *pdev, int type);
    int (*phy_exit)(struct platform_device *pdev, int type);
};

enum nxp_usb_phy_type {
	NXP_USB_PHY_OTG,
	NXP_USB_PHY_OHCI,
	NXP_USB_PHY_EHCI,
	NXP_USB_PHY_HSIC,
};

extern int  nxp_usb_phy_init(struct platform_device *pdev, int type);
extern int  nxp_usb_phy_exit(struct platform_device *pdev, int type);
extern int	nxp_hsic_phy_pwr_on(struct platform_device *pdev, bool on);


#endif /* __PLAT_USB_OTG_PHY_H */
