#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

extern int __init early_asv_margin(char *str);

int __init svt_early_asv_margin(void)
{
#if defined(CONFIG_S5P6818_MARGIN_SVT_MINUS_3) || defined(CONFIG_S5P6818_MARGIN_SVT_MINUS_5)

	#ifdef CONFIG_S5P6818_MARGIN_SVT_MINUS_3
	char *margin = "-3%";
	#endif

	#ifdef CONFIG_S5P6818_MARGIN_SVT_MINUS_5
	char *margin = "-5%";
	#endif

	early_asv_margin(margin);
#endif

	return 0;
}

core_initcall(svt_early_asv_margin);