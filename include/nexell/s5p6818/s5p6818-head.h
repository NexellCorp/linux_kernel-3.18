/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __S5P6818_HEAD_H__
#define __S5P6818_HEAD_H__

#ifndef __LINUX__
#define	__LINUX__
#endif
#ifndef __PRINTK__
#define	__PRINTK__
#endif
#if   defined (CONFIG_S5P6818_PROTO_RELEASE) && !defined (NX_RELEASE)
#define	NX_RELEASE
#elif defined (CONFIG_S5P6818_PROTO_DEBUG)   && !defined (NX_DEBUG)
#define	NX_DEBUG
#endif

#ifndef __ASSEMBLY__
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_clkpwr.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_clkgen.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_rstcon.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_tieoff.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_mcus.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_timer.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_alive.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_gpio.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_displaytop.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_disptop_clkgen.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_dualdisplay.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_mlc.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_dpc.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_lcdif.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_resconv.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_lvds.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_hdmi.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_mipi.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_i2c.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_i2s.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_sdmmc.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_ssp.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_rtc.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_mpegtsi.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_vip.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_adc.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_ecid.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_tmu.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_ppm.h>
#include <../../drivers/platform/nexell/s5p6818/prototype/module/nx_pdm.h>
#endif /* 	__ASSEMBLY__ */

#endif /*	__S5P6818_H__ */

