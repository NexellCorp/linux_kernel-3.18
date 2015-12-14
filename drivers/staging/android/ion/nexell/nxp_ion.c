/*
 * (C) Copyright 2009
 * sung woo park, Nexell Co, <swpark@nexell.co.kr>
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
#include <linux/err.h>
/*#include <linux/ion.h>*/
#include "../ion.h"
#include <linux/export.h>
#include <linux/nxp_ion.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/cma.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/bitops.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/of.h>

#include <asm/pgtable.h>

#include "../ion_priv.h"

/**
 * variables
 * start "s_" : static
 * start "g_" : global
 */
struct ion_device *g_ion_nxp = NULL;

static int s_num_heaps = 0;
static struct ion_heap **s_heaps = NULL;
static struct device *s_nxp_ion_dev = NULL;

/**
 * for global cma handler
 * defined in ion_mem.c
 */
extern struct cma *get_global_cma(void);

/*
 * external support functions
 */
struct ion_device *get_global_ion_device(void)
{
    return g_ion_nxp;
}
EXPORT_SYMBOL(get_global_ion_device);

/**
 * nxp ion contig heap ops
 */
#define CONVERT_SIZE_TO_PAGES(size) (size) % PAGE_SIZE ? (size) / PAGE_SIZE + 1 : (size) / PAGE_SIZE

static int ion_nxp_contig_heap_allocate(struct ion_heap *heap,
                    struct ion_buffer *buffer,
                    unsigned long len,
                    unsigned long align,
                    unsigned long flags)
{
    struct cma *cma = get_global_cma();
    if (!cma) {
        pr_err("CRITICAL ERROR: %s, get_global_cma() return NULL\n", __func__);
        return -ENOMEM;
    } else {
        int pages = CONVERT_SIZE_TO_PAGES(len);
        struct page *page = cma_alloc(cma, pages, 0);
        if (!page) {
            pr_err("CRITICAL ERROR: %s, failed to cma_alloc(), pages %d\n", __func__, pages);
            return -ENOMEM;
        }
        buffer->priv_phys = page_to_phys(page);
        // for debugging
        printk("%s: phys 0x%lx, pages %d\n", __func__, buffer->priv_phys, pages);
    }

    return 0;
}

static int ion_nxp_reserve_heap_allocate(struct ion_heap *heap,
                    struct ion_buffer *buffer,
                    unsigned long len,
                    unsigned long align,
                    unsigned long flags)
{
    pr_err("%s: not implemented reserve heap!!!\n", __func__);
    return -1;
}


static void ion_nxp_heap_free(struct ion_buffer *buffer)
{
    struct cma *cma = get_global_cma();
    if (!cma) {
        pr_err("CRITICAL ERROR: %s, get_global_cma() return NULL\n", __func__);
        return;
    } else {
        struct page *page = phys_to_page(buffer->priv_phys);
        int pages = CONVERT_SIZE_TO_PAGES(buffer->size);
        bool ret = cma_release(cma, page, pages);
        if (!ret) {
            printk(KERN_ERR "CRITICAL ERROR: %s, failed to cma_release for page %p, pages %d\n", __func__, page, pages);
        }
        // for debugging
        printk("%s: phys 0x%lx, pages %d\n", __func__, buffer->priv_phys, pages);
    }
}

static int ion_nxp_heap_phys(struct ion_heap *heap,
                    struct ion_buffer *buffer,
                    ion_phys_addr_t *addr, size_t *len)
{
    *addr = buffer->priv_phys;
    *len  = buffer->size;
    return 0;
}

static struct sg_table *ion_nxp_heap_map_dma(struct ion_heap *heap,
                    struct ion_buffer *buffer)
{
    struct sg_table *table;
    int ret;

    table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
    if (!table) {
        pr_err("%s error: fail to kzalloc size(%d)\n", __func__, sizeof(struct sg_table));
        return ERR_PTR(-ENOMEM);
    }
    ret = sg_alloc_table(table, 1, GFP_KERNEL);
    if (ret) {
        pr_err("%s error: fail to sg_alloc_table\n", __func__);
        return ERR_PTR(ret);
    }
    sg_init_one(table->sgl, phys_to_virt(buffer->priv_phys), buffer->size);
    return table;
}

static void ion_nxp_heap_unmap_dma(struct ion_heap *heap,
                    struct ion_buffer *buffer)
{
    if (buffer->sg_table) {
        sg_free_table(buffer->sg_table);
    }
}

static void *ion_nxp_heap_map_kernel(struct ion_heap *heap,
                    struct ion_buffer *buffer)
{
    return phys_to_virt(buffer->priv_phys);
}

static void ion_nxp_heap_unmap_kernel(struct ion_heap *heap,
                    struct ion_buffer *buffer)
{
}

static int ion_nxp_heap_map_user(struct ion_heap *heap,
                    struct ion_buffer *buffer,
                    struct vm_area_struct *vma)
{
    unsigned long pfn = __phys_to_pfn(buffer->priv_phys);
    return remap_pfn_range(vma, vma->vm_start, pfn + vma->vm_pgoff,
                    vma->vm_end - vma->vm_start,
                    vma->vm_page_prot);
}

static struct ion_heap_ops contig_heap_ops = {
    .allocate   = ion_nxp_contig_heap_allocate,
    .free       = ion_nxp_heap_free,
    .phys       = ion_nxp_heap_phys,
    .map_dma    = ion_nxp_heap_map_dma,
    .unmap_dma  = ion_nxp_heap_unmap_dma,
    .map_kernel = ion_nxp_heap_map_kernel,
    .unmap_kernel = ion_nxp_heap_unmap_kernel,
    .map_user   = ion_nxp_heap_map_user,
};

static struct ion_heap_ops reserve_heap_ops = {
    .allocate   = ion_nxp_reserve_heap_allocate,
    .free       = ion_nxp_heap_free,
    .phys       = ion_nxp_heap_phys,
    .map_dma    = ion_nxp_heap_map_dma,
    .unmap_dma  = ion_nxp_heap_unmap_dma,
    .map_kernel = ion_nxp_heap_map_kernel,
    .unmap_kernel = ion_nxp_heap_unmap_kernel,
    .map_user   = ion_nxp_heap_map_user,
};

static struct ion_heap *ion_nxp_heap_create(int type)
{
    struct ion_heap *heap;
    heap = kzalloc(sizeof(struct ion_heap), GFP_KERNEL);
    if (!heap) {
        pr_err("%s: fail to kzalloc size(%d)\n", __func__, sizeof(struct ion_heap));
        return ERR_PTR(-ENOMEM);
    }

    switch (type) {
    case ION_HEAP_TYPE_NXP_CONTIG:
        heap->ops  = &contig_heap_ops;
        heap->type = ION_HEAP_TYPE_NXP_CONTIG;
        break;
    case ION_HEAP_TYPE_NXP_RESERVE:
        heap->ops  = &reserve_heap_ops;
        heap->type = ION_HEAP_TYPE_NXP_RESERVE;
        break;
    default:
        printk("%s: invalid type 0x%x\n", __func__, type);
        kfree(heap);
        return NULL;
    }

    return heap;
}

static void ion_nxp_heap_destroy(struct ion_heap *heap)
{
    kfree(heap);
}

/**
 * local helper functions
 * start _
 */
static struct ion_heap *_ion_heap_create(struct ion_platform_heap *heap_data)
{
    struct ion_heap *heap = NULL;
    int heap_type = heap_data->type;

    /*
     * current supported custom heap type
     *   - ION_HEAP_TYPE_NXP_CONTIG
     */
    /*switch (heap_data->type) {*/
    switch (heap_type) {
    case ION_HEAP_TYPE_NXP_CONTIG:
    case ION_HEAP_TYPE_NXP_RESERVE:
        heap = ion_nxp_heap_create(heap_data->type);
        break;
    default:
        return ion_heap_create(heap_data);
    }

    if (IS_ERR_OR_NULL(heap)) {
        pr_err("%s: error creating heap %s type %d base %lu size %u\n",
                __func__, heap_data->name, heap_data->type,
                heap_data->base, heap_data->size);
        return ERR_PTR(-EINVAL);
    }

    heap->name = heap_data->name;
    heap->id   = heap_data->id;

    return heap;
}

static void _ion_heap_destroy(struct ion_heap *heap)
{
    int heap_type;

    if (!heap)
        return;

    heap_type = heap->type;

    switch (heap_type) {
    case ION_HEAP_TYPE_NXP_CONTIG:
    case ION_HEAP_TYPE_NXP_RESERVE:
        ion_nxp_heap_destroy(heap);
        break;
    default:
        ion_heap_destroy(heap);
    }
}

static int ion_sync_from_device(struct ion_client *client, int fd)
{
    struct dma_buf *dmabuf;
    struct ion_buffer *buffer;

    /*printk("%s: fd %d\n", __func__, fd);*/
    dmabuf = dma_buf_get(fd);
    if (IS_ERR(dmabuf))
        return PTR_ERR(dmabuf);

    buffer = dmabuf->priv;

    dma_sync_sg_for_cpu(NULL, buffer->sg_table->sgl, buffer->sg_table->nents, DMA_FROM_DEVICE);
    dma_buf_put(dmabuf);

    return 0;
}
/* custom ioctl */
static long nxp_ion_custom_ioctl(struct ion_client *client,
        unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch (cmd) {
    case NXP_ION_GET_PHY_ADDR:
        {
            struct nxp_ion_physical data;
            struct dma_buf *dmabuf;
            struct ion_buffer *buffer;
            if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
                pr_err("%s error: failed to copy_from_user()\n", __func__);
                return -EFAULT;
            }
            dmabuf = dma_buf_get(data.ion_buffer_fd);
            if (IS_ERR_OR_NULL(dmabuf)) {
                pr_err("%s: can't get dmabuf\n", __func__);
                return -EINVAL;
            }
            buffer = dmabuf->priv;
            data.phys = (unsigned long)buffer->priv_phys;
            if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
                pr_err("%s error: failed to copy_to_user()\n", __func__);
                return -EFAULT;
            }
            dma_buf_put(dmabuf);
        }
        break;
    case NXP_ION_SYNC_FROM_DEVICE:
        {
            struct ion_fd_data data;
            if (copy_from_user(&data, (void __user *)arg, sizeof(struct ion_fd_data)))
                return -EFAULT;
            ion_sync_from_device(client, data.fd);
            break;
        }
    default:
        return -EINVAL;
    }

    return ret;
}

#ifdef CONFIG_OF
static struct ion_platform_data *nxp_ion_parse_dt(struct device *dev)
{
    struct device_node *of_node = dev->of_node;
    struct ion_platform_data *pdata;
    int i;

    printk("===========> %s entered\n", __func__);

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        dev_err(dev, "failed to devm_kzalloc() for struct ion_platform_data\n");
        return ERR_PTR(-ENOMEM);
    }

    if (of_property_read_u32(of_node, "nr", &pdata->nr)) {
         dev_err(dev, "failed to read nr\n");
         return ERR_PTR(-EINVAL);
    }
    printk("%s: nr %d\n", __func__, pdata->nr);

    pdata->heaps = devm_kzalloc(dev, pdata->nr * sizeof(struct ion_platform_heap), GFP_KERNEL);
    if (!pdata->heaps) {
        dev_err(dev, "failed to devm_kzalloc() for struct ion_platform_heap\n");
        kfree(pdata);
        return ERR_PTR(-ENOMEM);
    }

    for (i = 0; i < pdata->nr; i++) {
        if (of_property_read_u32_index(of_node, "types", i, &pdata->heaps[i].type)) {
            dev_err(dev, "failed to read types index %d\n", i);
            kfree(pdata->heaps);
            kfree(pdata);
            return ERR_PTR(-EINVAL);
        }
        printk("%s: index %d types 0x%x\n", __func__, i, pdata->heaps[i].type);
        pdata->heaps[i].id = pdata->heaps[i].type;
        if (of_property_read_string_index(of_node, "names", i, &pdata->heaps[i].name)) {
            dev_err(dev, "failed to read names index %d\n", i);
            kfree(pdata->heaps);
            kfree(pdata);
            return ERR_PTR(-EINVAL);
        }
        printk("%s: index %d name %s\n", __func__, i, pdata->heaps[i].name);
    }

    printk("<=========== %s exit\n", __func__);
    return pdata;
}
#endif

/**
 * platform driver
 */
static int nxp_ion_probe(struct platform_device *pdev)
{
    int error;
    int i;
    struct ion_device *ion_dev;
    struct ion_heap **heaps;

#ifdef CONFIG_OF
    struct ion_platform_data *pdata = nxp_ion_parse_dt(&pdev->dev);
    if (IS_ERR(pdata)) {
        printk(KERN_ERR "%s: failed to nxp_ion_parse_dt()\n", __func__);
        return PTR_ERR(pdata);
    }
#else
    struct ion_platform_data *pdata = pdev->dev.platform_data;
    if (pdata->nr <= 0) {
        pr_err("%s error: invalid platform_data, nr(%d)\n", __func__, pdata->nr);
        return -EINVAL;
    }
#endif

    ion_dev = ion_device_create(nxp_ion_custom_ioctl);
    if (IS_ERR_OR_NULL(ion_dev)) {
        pr_err("%s error: fail to ion_device_create\n", __func__);
        return -EINVAL;
    }

    heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);
    if (!heaps) {
        pr_err("%s error: fail to kzalloc struct ion_heap(size: %d)\n",
                __func__, sizeof(struct ion_heap *) * pdata->nr);
        ion_device_destroy(ion_dev);
        return -ENOMEM;
    }

    for (i = 0; i < pdata->nr; i++) {
        struct ion_platform_heap *heap_data = &pdata->heaps[i];

        heaps[i] = _ion_heap_create(heap_data);
        if (IS_ERR_OR_NULL(heaps[i])) {
            error = PTR_ERR(heaps[i]);
            pr_err("%s error: fail to _ion_heap_create(error: %d)\n", __func__, error);
            goto err;
        }
        ion_device_add_heap(ion_dev, heaps[i]);
    }

    s_num_heaps     = pdata->nr;
    s_heaps         = heaps;
    g_ion_nxp       = ion_dev;
    s_nxp_ion_dev   = &pdev->dev;

    platform_set_drvdata(pdev, g_ion_nxp);

#ifdef CONFIG_OF
    if (pdata)
        devm_kfree(&pdev->dev, pdata);
#endif

    printk("%s success!!!\n", __func__);
    return 0;

err:
    for (i = 0; i < pdata->nr; i++) {
        if (heaps[i]) {
            ion_heap_destroy(heaps[i]);
        }
    }
    kfree(heaps);
    ion_device_destroy(ion_dev);
#ifdef CONFIG_OF
    if (pdata)
        devm_kfree(&pdev->dev, pdata);
#endif
    return error;
}

static int nxp_ion_remove(struct platform_device *pdev)
{
    struct ion_device *idev = platform_get_drvdata(pdev);
    int i;

    ion_device_destroy(idev);
    for (i = 0; i < s_num_heaps; i++)
        _ion_heap_destroy(s_heaps[i]);
    kfree(s_heaps);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nxp_ion_dt_match[] = {
    { .compatible = "nexell,nxp-ion" },
    {},
};
MODULE_DEVICE_TABLE(of, nxp_ion_dt_match);
#endif

static struct platform_driver ion_driver = {
    .probe  = nxp_ion_probe,
    .remove = nxp_ion_remove,
    .driver = {
        .name = "ion-nxp",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(nxp_ion_dt_match),
#endif
    },
};

static int __init ion_init(void)
{
    return platform_driver_register(&ion_driver);
}

subsys_initcall(ion_init);

MODULE_AUTHOR("swpark <swpark@nexell.co.kr>");
MODULE_DESCRIPTION("ION Platform Driver for Nexell");
MODULE_LICENSE("GPL");
