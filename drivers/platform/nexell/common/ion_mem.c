#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <asm/io.h>

#if defined(CONFIG_CMA) && defined(CONFIG_ION)
#include <linux/memblock.h>
#include <linux/cma.h>

static struct cma *res_cma;
struct cma *get_global_cma(void)
{
    return res_cma;
}

static int __init ion_cma_setup(struct reserved_mem *rmem)
{
    phys_addr_t start = rmem->base;
    phys_addr_t size = rmem->size;
	int ret = 0;

	printk("ion: start 0x%lx, size %lu\n", (long)start, (long)size);

    ret = cma_init_reserved_mem(start, size, 0, &res_cma);
    if (ret) {
        printk(KERN_ERR "CRITICAL ERROR: %s, failed to cma_init_reserved_mem(), ret %d\n", __func__, ret);
        return ret;
    }

	printk("ion: cma register success!!!\n");

	return 0;
}

RESERVEDMEM_OF_DECLARE(my_cma, "cma_for_ion", ion_cma_setup);
#endif
