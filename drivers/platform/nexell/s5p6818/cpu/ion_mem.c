#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <asm/io.h>

#if defined CONFIG_CMA && defined CONFIG_ION
#include <linux/memblock.h>
#include <linux/cma.h>

static char *my_cma_name = "ion";
static struct cma_region reg;
static int __init ion_cma_setup(struct reserved_mem *rmem,
						unsigned long node, const char *uname)
{
    static const char map[] __initconst =
        "*=ion;"
        "ion-nxp=ion;"
        "nx_vpu=ion;";
    phys_addr_t start = rmem->base;
    phys_addr_t size = rmem->size;
	int ret = 0;

	printk("ion: start 0x%lx, size %lu\n", (long)start, (long)size);

	reg.name = my_cma_name;
    reg.start = start;
    reg.size = size;
    reg.alignment = PAGE_SIZE;
    reg.reserved = 1;

	ret = cma_early_region_register(&reg);
    if(0 > ret) {
    	printk("FATAL ERROR, failed to cma_early_region_register!!!\n");
		return ret;
	}

	cma_set_defaults(NULL, map);
	printk("ion: cma register success!!!\n");

	return 0;
}

RESERVEDMEM_OF_DECLARE(my_cma, "cma_for_ion", ion_cma_setup);
#endif