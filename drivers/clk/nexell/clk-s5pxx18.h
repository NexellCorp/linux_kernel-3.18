#ifndef _CLK_S5PXX18_H
#define _CLK_S5PXX18_H

#include <nexell/platform.h>

#define	I_PLL0_BIT		(0)
#define	I_PLL1_BIT		(1)
#define	I_PLL2_BIT		(2)
#define	I_PLL3_BIT		(3)
#define	I_EXT1_BIT		(4)
#define	I_EXT2_BIT		(5)
#define	I_CLKn_BIT		(7)

#define	I_CLOCK_NUM		6		/* PLL0, PLL1, PLL2, PLL3, EXT1, EXT2 */

#ifdef  CONFIG_ARM_NXP_CPUFREQ
#define I_EXECEPT_CLK	(1<<CONFIG_NXP_CPUFREQ_PLLDEV)
#else
#define I_EXECEPT_CLK	(0)
#endif

#define	I_CLOCK_MASK	(((1<<I_CLOCK_NUM) - 1) & ~I_EXECEPT_CLK)

#endif
