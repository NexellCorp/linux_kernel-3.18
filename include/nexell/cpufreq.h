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
#ifndef __ARCH_CPUFREQ_H__
#define __ARCH_CPUFREQ_H__

/*
 *    CPU Freq platform data
 */
struct nxp_cpufreq_plat_data {
    int pll_dev;                    /* core pll : 0, 1, 2, 3 */
    unsigned long (*dvfs_table)[2]; /* [freq KHz].[u volt] */
    int  table_size;
    long max_cpufreq;       /* unit Khz */
    long max_retention;     /* unit msec */
    long rest_cpufreq;      /* unit Khz */
    long rest_retention;    /* unit msec */
	char *supply_name;		/* voltage regulator name */
	long supply_delay_us;
};

#endif