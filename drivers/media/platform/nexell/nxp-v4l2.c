/*#define DEBUG 1*/

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/of.h>

#include <media/v4l2-ctrls.h>
#include <media/media-device.h>
#include <media/videobuf2-ion-nxp.h>

/*#include <mach/nxp-v4l2-platformdata.h>*/
/*#include <mach/platform.h>*/
#include <nxp-v4l2-platformdata.h>
#include <platform.h>

#include "nxp-capture.h"
#include "nxp-scaler.h"
#include "nxp-out.h"
#include "nxp-v4l2.h"
#include "loopback-sensor.h"

// TODO
#if 1
extern void			nxp_soc_gpio_set_out_value(unsigned int io, int high);
extern void 		nxp_soc_gpio_set_io_dir(unsigned int io, int out);
extern void 		nxp_soc_gpio_set_io_func(unsigned int io, unsigned int func);
#endif
/**
 * static variable for public api
 */
static struct nxp_v4l2 *__me = NULL;

struct media_device *nxp_v4l2_get_media_device(void)
{
    if (likely(__me))
        return &__me->media_dev;
    return NULL;
}

struct v4l2_device *nxp_v4l2_get_v4l2_device(void)
{
    if (likely(__me))
        return &__me->v4l2_dev;
    return NULL;
}

void *nxp_v4l2_get_alloc_ctx(void)
{
    if (likely(__me))
        return &__me->v4l2_dev;
    return NULL;
}

#ifdef CONFIG_OF
#include <linux/regulator/driver.h>
#include <linux/delay.h>
#include <linux/i2c.h>

extern long nxp_soc_pwm_set_frequency(int ch, unsigned int request, unsigned int duty);
static int general_set_clock(struct nxp_vin_platformdata *pdata, bool enable)
{
    printk("%s enable %d\n", __func__, enable);
    if (pdata->pwm_number >= 0 && pdata->clock_rate > 0) {
        if (enable) {
            printk("%s: pwm %d, set clock %d\n", __func__, pdata->pwm_number, pdata->clock_rate);
            nxp_soc_pwm_set_frequency(pdata->pwm_number, pdata->clock_rate, 50);
        } else {
            nxp_soc_pwm_set_frequency(pdata->pwm_number, 0, 0);
        }
    }
    return 0;
}

static int general_power_enable(struct nxp_vin_platformdata *pdata, bool enable)
{
    printk("%s: %d, regulator nr %d\n", __func__, enable, pdata->regulator_nr);
    if (pdata->regulator_nr > 0) {
        int i;
        struct regulator *power;
        int ret;
        for (i = 0; i < pdata->regulator_nr; i++) {
            power = regulator_get(NULL, pdata->regulator_names[i]);
            if (IS_ERR(power)) {
                printk(KERN_ERR "%s: failed to regulator_get() for %s\n", __func__, pdata->regulator_names[i]);
                return -EINVAL;
            }
            if (enable)
                ret = regulator_enable(power);
            else
                regulator_disable(power);
            regulator_put(power);
        }
    }

    printk("%s: enable_io %d, enable_invert %d, enable_ms %d, reset_io %d, reset_invert %d, reset_ms %d\n", __func__,
            pdata->enable_io, pdata->enable_invert, pdata->enable_delay_ms, pdata->reset_io, pdata->reset_invert, pdata->reset_delay_ms);
    if (enable) {
        if (pdata->enable_io >= 0) {
            nxp_soc_gpio_set_out_value(pdata->enable_io, pdata->enable_invert);
            nxp_soc_gpio_set_io_dir(pdata->enable_io, 1);
            nxp_soc_gpio_set_io_func(pdata->enable_io, 0);
            general_set_clock(pdata, enable);
            if (pdata->enable_delay_ms > 0)
                mdelay(pdata->enable_delay_ms);
            nxp_soc_gpio_set_out_value(pdata->enable_io, !pdata->enable_invert);
        }

        if (pdata->reset_io >= 0) {
            nxp_soc_gpio_set_out_value(pdata->reset_io, !pdata->reset_invert);
            nxp_soc_gpio_set_io_dir(pdata->reset_io, 1);
            nxp_soc_gpio_set_io_func(pdata->reset_io, 0);
            if (pdata->reset_delay_ms > 0)
                mdelay(pdata->reset_delay_ms);
            nxp_soc_gpio_set_out_value(pdata->reset_io, pdata->reset_invert);
            if (pdata->reset_delay_ms > 0)
                mdelay(pdata->reset_delay_ms);
        }
    } else {
        if (pdata->enable_io >= 0)
            nxp_soc_gpio_set_out_value(pdata->enable_io, pdata->enable_invert);
        if (pdata->reset_io >= 0)
            nxp_soc_gpio_set_out_value(pdata->reset_io, !pdata->reset_invert);
        general_set_clock(pdata, enable);
    }

    return 0;
}

static bool general_power_state_changed(struct nxp_vin_platformdata *pdata)
{
    return 0;
}

static const u_int port_table[][11][2] = {
    {
        { PAD_GPIO_E +  4, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_E +  5, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_E +  6, NX_GPIO_PADFUNC_1 },

        { PAD_GPIO_D + 28, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_D + 29, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_D + 30, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_D + 31, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_E +  0, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_E +  1, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_E +  2, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_E +  3, NX_GPIO_PADFUNC_1 },
    },
    {
        { PAD_GPIO_A + 28, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_E + 13, NX_GPIO_PADFUNC_2 },
        { PAD_GPIO_E +  7, NX_GPIO_PADFUNC_2 },

        { PAD_GPIO_A + 30, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_B +  0, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_B +  2, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_B +  4, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_B +  6, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_B +  8, NX_GPIO_PADFUNC_1 },
        { PAD_GPIO_B +  9, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_B + 10, NX_GPIO_PADFUNC_1 },
    },
    {

        { PAD_GPIO_C + 14, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 15, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 16, NX_GPIO_PADFUNC_3 },

        { PAD_GPIO_C + 17, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 18, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 19, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 20, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 21, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 22, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 23, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 24, NX_GPIO_PADFUNC_3 },
    }
};

static void general_setup_io(struct nxp_vin_platformdata *pdata, bool force)
{
    if (!pdata->is_mipi) {
        u_int *pad;
        int i, len;
        u_int io, fn;

        printk("%s: vid %d\n", __func__, pdata->vid);
        pad = (u_int *)port_table[pdata->vid];
        len = sizeof(port_table[pdata->vid])/sizeof(port_table[pdata->vid][0]);

        for (i = 0; i < len; i++) {
            io = *pad++;
            fn = *pad++;
            nxp_soc_gpio_set_io_dir(io, 0);
            nxp_soc_gpio_set_io_func(io, fn);
        }
    }
}

static struct nxp_v4l2_i2c_board_info *make_i2c_board_info(const char *name, u32 i2c_adapter, u32 i2c_addr)
{
    struct nxp_v4l2_i2c_board_info *nxp_i2c_board_info = NULL;
    struct i2c_board_info *board_info = NULL;

    nxp_i2c_board_info = kzalloc(sizeof(struct nxp_v4l2_i2c_board_info), GFP_KERNEL);
    if (!nxp_i2c_board_info) {
        printk(KERN_ERR "%s: failed to alloc nxp_i2c_board_info\n", __func__);
        return NULL;
    }

    board_info = kzalloc(sizeof(struct i2c_board_info), GFP_KERNEL);
    if (!board_info) {
        printk(KERN_ERR "%s: failed to alloc i2c_board_info\n", __func__);
        kfree(nxp_i2c_board_info);
        return NULL;
    }

    strcpy(board_info->type, name);
    board_info->addr = i2c_addr;
    nxp_i2c_board_info->i2c_adapter_id = i2c_adapter;
    nxp_i2c_board_info->board_info = board_info;

    return nxp_i2c_board_info;
}

static int nxp_v4l2_parse_capture_sensor_dt(struct device_node *node, struct nxp_capture_platformdata *pcapture)
{
    u32 type;
    if (of_property_read_u32(node, "type", &type)) {
        printk(KERN_ERR "%s: failed to read type", __func__);
        return -EINVAL;
    }

    if (type == NXP_CAPTURE_I2C) {
        const char *name;
        u32 i2c_adapter;
        u32 i2c_addr;

        if (of_property_read_string(node, "sensor_name", &name)) {
            printk(KERN_ERR "%s: failed to read sensor name\n", __func__);
            return -EINVAL;
        }

        if (of_property_read_u32(node, "i2c_adapter", &i2c_adapter)) {
             printk(KERN_ERR "%s: failed to read i2c_adapter\n", __func__);
             return -EINVAL;
        }

        if (of_property_read_u32(node, "addr", &i2c_addr)) {
             printk(KERN_ERR "%s: failed to read i2c addr\n", __func__);
             return -EINVAL;
        }

        pcapture->sensor = make_i2c_board_info(name, i2c_adapter, i2c_addr);
        if (!pcapture->sensor) {
            printk(KERN_ERR "%s: failed to make_i2c_board_info()\n", __func__);
            return -ENOMEM;
        }
    }
		else if(type == NXP_CAPTURE_LOOPBACK)
		{
			pcapture->sensor = NULL;
			printk(KERN_INFO "[%s] NXP_CAPTURE_LOOPBACK\n", __func__);
		}

    return 0;
}

static int nxp_v4l2_parse_capture_power_dt(struct device_node *node, struct nxp_vin_platformdata *pvin)
{
    printk("%s entered\n", __func__);
    if (of_property_read_u32(node, "enable_io", &pvin->enable_io))
        pvin->enable_io = -1;
    printk("enable_io %d\n", pvin->enable_io);
    if (of_property_read_u32(node, "enable_invert", (u32 *)&pvin->enable_invert))
        pvin->enable_invert = false;
    printk("enable_invert %d\n", pvin->enable_invert);
    if (of_property_read_u32(node, "enable_delay_ms", &pvin->enable_delay_ms))
        pvin->enable_delay_ms = -1;
    printk("enable_delay_ms %d\n", pvin->enable_delay_ms);

    if (of_property_read_u32(node, "reset_io", &pvin->reset_io))
        pvin->reset_io = -1;
    if (of_property_read_u32(node, "reset_invert", (u32 *)&pvin->reset_invert))
        pvin->reset_invert = false;
    if (of_property_read_u32(node, "reset_delay_ms", &pvin->reset_delay_ms))
        pvin->reset_delay_ms = -1;

    pvin->regulator_nr = of_property_count_strings(node, "regulator_names");
    if (pvin->regulator_nr > 0) {
        int i;
        const char *name;
        pvin->regulator_names = kmalloc(sizeof(char *) * pvin->regulator_nr, GFP_KERNEL);
        if (!pvin->regulator_names) {
             printk(KERN_ERR "%s: failed to alloc regulator names\n", __func__);
             return -ENOMEM;
        }
        for (i = 0; i < pvin->regulator_nr; i++) {
            if (of_property_read_string_index(node, "regulator_names", i, &name)) {
                printk(KERN_ERR "%s: failed to read regulator_names index %d\n", __func__, i);
                return -EINVAL;
            }
            pvin->regulator_names[i] = (char *)name;
        }
    }

    printk("%s exit\n", __func__);
    return 0;
}

static int nxp_v4l2_parse_capture_clock_dt(struct device_node *node, struct nxp_vin_platformdata *pvin)
{
    printk("%s entered\n", __func__);
    if (of_property_read_u32(node, "pwm_number", &pvin->pwm_number)) {
        pvin->pwm_number = -1;
    } else {
        if (of_property_read_u32(node, "rate", &pvin->clock_rate)) {
            printk(KERN_ERR "%s: failed to read clock_rate\n", __func__);
            return -EINVAL;
        }
    }

    printk("%s exit\n", __func__);
    return 0;
}

static int nxp_v4l2_parse_capture_sub_dt(struct device_node *node, struct nxp_capture_platformdata *pcapture)
{
    struct device_node *child_node = NULL;
    int ret;

    printk("%s %p\n", __func__, pcapture);

    if (of_property_read_u32(node, "type", &pcapture->type)) {
        printk(KERN_ERR "failed to read type\n");
        return -EINVAL;
    }

    if (pcapture->type == NXP_CAPTURE_INF_CSI)
        pcapture->parallel.is_mipi = true;
    else
        pcapture->parallel.is_mipi = false;

    if (pcapture->parallel.is_mipi) {
        // mipi use always same platdata
        pcapture->module = 0;
        pcapture->parallel.port = 1;
        pcapture->parallel.h_frontporch = 4;
        pcapture->parallel.h_syncwidth = 4;
        pcapture->parallel.h_backporch = 4;
        pcapture->parallel.v_frontporch = 1;
        pcapture->parallel.v_syncwidth = 1;
        pcapture->parallel.v_backporch = 1;
        pcapture->parallel.clock_invert = false;
        pcapture->parallel.data_order = NXP_VIN_CBY0CRY1;
        pcapture->parallel.interlace = false;
    } else {
        if (of_property_read_u32(node, "module", &pcapture->module)) {
            printk(KERN_ERR "failed to read module\n");
            return -EINVAL;
        }

        if (of_property_read_u32(node, "port", &pcapture->parallel.port)) {
            printk(KERN_ERR "failed to read port\n");
            return -EINVAL;
        }

        if (of_property_read_u32(node, "external_sync", (u32 *)&pcapture->parallel.external_sync)) {
            printk(KERN_ERR "failed to read external_sync\n");
            return -EINVAL;
        }

        // if use 656 interface, porch value is always same...
        if (pcapture->parallel.external_sync == false) {
            pcapture->parallel.h_frontporch = 7;
            pcapture->parallel.h_syncwidth = 1;
            pcapture->parallel.h_backporch = 10;
            pcapture->parallel.v_frontporch = 0;
            pcapture->parallel.v_syncwidth = 2;
            pcapture->parallel.v_backporch = 3;
        } else {
            if (of_property_read_u32(node, "h_frontporch", &pcapture->parallel.h_frontporch)) {
                printk(KERN_ERR "failed to read h_frontporch\n");
                return -EINVAL;
            }
            if (of_property_read_u32(node, "h_syncwidth", &pcapture->parallel.h_syncwidth)) {
                printk(KERN_ERR "failed to read h_syncwidth\n");
                return -EINVAL;
            }
            if (of_property_read_u32(node, "h_backporch", &pcapture->parallel.h_backporch)) {
                printk(KERN_ERR "failed to read h_backporch\n");
                return -EINVAL;
            }
            if (of_property_read_u32(node, "v_frontporch", &pcapture->parallel.v_frontporch)) {
                printk(KERN_ERR "failed to read v_frontporch\n");
                return -EINVAL;
            }
            if (of_property_read_u32(node, "v_syncwidth", &pcapture->parallel.v_syncwidth)) {
                printk(KERN_ERR "failed to read v_syncwidth\n");
                return -EINVAL;
            }
            if (of_property_read_u32(node, "v_backporch", &pcapture->parallel.v_backporch)) {
                printk(KERN_ERR "failed to read v_backporch\n");
                return -EINVAL;
            }
        }

        if (of_property_read_u32(node, "data_order", &pcapture->parallel.data_order)) {
            printk(KERN_ERR "failed to read data_order\n");
            return -EINVAL;
        }

        if (of_property_read_u32(node, "vid", &pcapture->parallel.vid)) {
            printk(KERN_ERR "failed to read vid\n");
            return -EINVAL;
        }

        if (of_property_read_u32(node, "interlace", (u32 *)&pcapture->parallel.interlace))
            pcapture->parallel.interlace = false;

        if (of_property_read_u32(node, "late_power_down", (u32 *)&pcapture->parallel.late_power_down))
            pcapture->parallel.late_power_down = false;
    }

    child_node = of_find_node_by_name(node, "sensor");
    if (!child_node) {
        printk(KERN_ERR "%s: can't find sensor dt node, you must specify sensor dt!!!\n", __func__);
        return -EINVAL;
    }
    if (child_node) {
         ret = nxp_v4l2_parse_capture_sensor_dt(child_node, pcapture);
         if (ret) {
             printk(KERN_ERR "%s: failed to nxp_v4l2_parse_capture_sensor_dt()\n", __func__);
             return ret;
         }
    }

    child_node = of_find_node_by_name(node, "power");
    if (child_node) {
         ret = nxp_v4l2_parse_capture_power_dt(child_node, &pcapture->parallel);
         if (ret) {
             printk(KERN_ERR "%s: failed to nxp_v4l2_parse_capture_power_dt()\n", __func__);
             return ret;
         }
    }

    child_node = of_find_node_by_name(node, "clock");
    if (child_node) {
         ret = nxp_v4l2_parse_capture_clock_dt(child_node, &pcapture->parallel);
         if (ret) {
             printk(KERN_ERR "%s: failed to nxp_v4l2_parse_capture_clock_dt()\n", __func__);
             return ret;
         }
    }

    // set callback
    pcapture->parallel.power_enable = general_power_enable;
    pcapture->parallel.power_state_changed = general_power_state_changed;
    pcapture->parallel.set_clock = general_set_clock;
    pcapture->parallel.setup_io = general_setup_io;

    printk("%s: power_enable %p, power_state_changed %p, set_clock %p, setup_ion %p\n", __func__,
            pcapture->parallel.power_enable,
            pcapture->parallel.power_state_changed,
            pcapture->parallel.set_clock,
            pcapture->parallel.setup_io);

    return 0;
}

static int nxp_v4l2_parse_capture_dt(struct device_node *node, struct nxp_v4l2_platformdata *pdata)
{
    int i;
    struct device_node *child_node = NULL;
    int ret;
    char sub_name[64] = {0, };

    if (of_property_read_u32(node, "nr", &pdata->capture_nr)) {
        printk(KERN_ERR "failed to read capture nr\n");
        return -EINVAL;
    }

    pdata->captures = kzalloc(sizeof(struct nxp_capture_platformdata) * pdata->capture_nr, GFP_KERNEL);
    if (!pdata->captures) {
         printk(KERN_ERR "failed to alloc capture platformdata\n");
         return -ENOMEM;
    }

    for (i = 0; i < pdata->capture_nr; i++) {
        sprintf(sub_name, "%s%d", "capture", i);
        child_node = of_find_node_by_name(node, sub_name);
        if (!child_node) {
            printk(KERN_ERR "can't find child node %s\n", sub_name);
            kfree(pdata->captures);
            return -EINVAL;
        }
        ret = nxp_v4l2_parse_capture_sub_dt(child_node, &pdata->captures[i]);
        if (ret) {
            printk(KERN_ERR "failed to nxp_v4l2_parse_capture_sub_dt()\n");
            kfree(pdata->captures);
            return -EINVAL;
        }
    }

    return 0;
}

static int nxp_v4l2_parse_out_dt(struct device_node *node, struct nxp_v4l2_platformdata *pdata)
{
    return 0;
}

static struct nxp_v4l2_platformdata *nxp_v4l2_parse_dt(struct device *dev)
{
    struct device_node *of_node = dev->of_node;
    struct device_node *child_node = NULL;
    struct nxp_v4l2_platformdata *pdata;
    int ret;

    printk("%s entered\n", __func__);
    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        dev_err(dev, "failed to devm_kzalloc() for struct nxp_v4l2_platformdata\n");
        return ERR_PTR(-ENOMEM);
    }

    child_node = of_find_node_by_name(of_node, "capture");
    printk("%s: %p\n", __func__, child_node);
    if (child_node) {
        ret = nxp_v4l2_parse_capture_dt(child_node, pdata);
        if (ret) {
            printk(KERN_ERR "%s: failed to nxp_v4l2_parse_capture_dt()\n", __func__);
            kfree(pdata);
            return ERR_PTR(-EINVAL);
        }
    }

    child_node = of_find_node_by_name(of_node, "out");
    if (child_node) {
         ret = nxp_v4l2_parse_out_dt(child_node, pdata);
         if (ret) {
             printk(KERN_ERR "%s: failed to nxp_v4l2_parse_out_dt()\n", __func__);
             if (pdata->captures)
                 kfree(pdata->captures);
             kfree(pdata);
             return ERR_PTR(-EINVAL);
         }
    }

    printk("%s exit\n", __func__);
    return pdata;
}
#endif

static int nxp_v4l2_probe(struct platform_device *pdev)
{
    struct v4l2_device *v4l2_dev;
    struct nxp_v4l2 *nxp_v4l2;
    struct nxp_v4l2_platformdata *pdata;
#ifdef CONFIG_VIDEO_NXP_CAPTURE
    struct nxp_capture_platformdata *capture_pdata;
    struct nxp_capture *capture;
    int i;
#endif
#ifdef CONFIG_NXP_M2M_SCALER
    struct nxp_scaler *scaler;
#endif
#ifdef CONFIG_LOOPBACK_SENSOR_DRIVER
    struct nxp_loopback_sensor *loopback_sensor = NULL;
#endif

    int ret;

    pr_debug("%s entered\n", __func__);
    printk("%s entered\n", __func__);

#ifndef CONFIG_OF
    pdata = pdev->dev.platform_data;
    if (!pdata) {
        dev_err(&pdev->dev, "can't get platformdata\n");
        return -EINVAL;
    }
#else
    pdata = nxp_v4l2_parse_dt(&pdev->dev);
    if (IS_ERR(pdata)) {
         printk(KERN_ERR "%s: failed to nxp_v4l2_parse_det()\n", __func__);
         return PTR_ERR(pdata);
    }
#endif

    nxp_v4l2 = kzalloc(sizeof(*nxp_v4l2), GFP_KERNEL);
    if (!nxp_v4l2) {
        pr_err("%s error: fail to kzalloc(size %lu)\n", __func__, sizeof(struct nxp_v4l2));
        return -ENOMEM;
    }

    nxp_v4l2->pdev = pdev;
    nxp_v4l2->pdata = pdata;

    snprintf(nxp_v4l2->media_dev.model, sizeof(nxp_v4l2->media_dev.model), "%s",
            dev_name(&pdev->dev));

    nxp_v4l2->media_dev.dev = &pdev->dev;

    v4l2_dev       = &nxp_v4l2->v4l2_dev;
    v4l2_dev->mdev = &nxp_v4l2->media_dev;
    snprintf(v4l2_dev->name, sizeof(v4l2_dev->name), "%s",
            dev_name(&pdev->dev));

    /* alloc context : use uncached area */
    nxp_v4l2->alloc_ctx =
        vb2_ion_create_context(&pdev->dev, SZ_4K, VB2ION_CTX_UNCACHED);
    if (!nxp_v4l2->alloc_ctx) {
        pr_err("%s: failed to ion alloc context\n", __func__);
        ret = -ENOMEM;
        goto err_alloc_ctx;
    }

    ret = v4l2_device_register(&pdev->dev, &nxp_v4l2->v4l2_dev);
    if (ret < 0) {
        pr_err("%s: failed to register v4l2_device: %d\n", __func__, ret);
        goto err_v4l2_reg;
    }

    ret = media_device_register(&nxp_v4l2->media_dev);
    if (ret < 0) {
        pr_err("%s: failed to register media_device: %d\n", __func__, ret);
        goto err_media_reg;
    }

    __me = nxp_v4l2;

#ifdef CONFIG_LOOPBACK_SENSOR_DRIVER
    loopback_sensor = create_nxp_loopback_sensor(pdata->captures);
    if (!loopback_sensor) {
        pr_err("%s: failed to create_nxp_loopback_sensor()\n", __func__);
        ret = -EINVAL;
        goto err_loopback_sensor_create;
    }

    ret = register_nxp_loopback_sensor(loopback_sensor);
    if (ret < 0) {
        pr_err("%s: failed to register_nxp_loopback_sensor()\n", __func__);
        goto err_loopback_sensor_create;
    }

    nxp_v4l2->loopback_sensor = loopback_sensor;
#endif

#ifdef CONFIG_VIDEO_NXP_CAPTURE
    /* capture */
#ifdef CONFIG_OF
    for (capture_pdata = pdata->captures, i = 0;
            i < pdata->capture_nr;
            capture_pdata++, i++) {
#else
    for (capture_pdata = pdata->captures, i = 0;
            capture_pdata->sensor;
            capture_pdata++, i++) {
#endif
        capture = create_nxp_capture(i, capture_pdata->module, capture_pdata);
        if (!capture) {
            pr_err("%s: failed to %dth create_nxp_capture()\n", __func__, i);
            ret = -EINVAL;
            goto err_capture_create;
        }
        ret = register_nxp_capture(capture);
        if (ret < 0) {
            pr_err("%s: failed to %dth register_nxp_capture()\n", __func__, i);
            goto err_capture_create;
        }
        nxp_v4l2->capture[i] = capture;
    }
#endif

#ifdef CONFIG_NXP_M2M_SCALER
    /* m2m */
    scaler = create_nxp_scaler();
    if (!scaler) {
        pr_err("%s: failed to create_nxp_scaler()\n", __func__);
        ret = -ENOMEM;
#ifdef CONFIG_VIDEO_NXP_CAPTURE
        goto err_capture_create;
#else
        goto err_media_reg;
#endif
    }

    ret = register_nxp_scaler(scaler);
    if (ret < 0) {
        pr_err("%s: failed to nxp_scaler_register()\n", __func__);
        goto err_register_scaler;
    }
    nxp_v4l2->scaler = scaler;
#endif

#ifdef CONFIG_VIDEO_NXP_OUT
    /* out */
    nxp_v4l2->out = create_nxp_out(pdata->out);
    if (!nxp_v4l2->out) {
        pr_err("%s: failed to create_nxp_out()\n", __func__);
        goto err_create_out;
    }

    ret = register_nxp_out(nxp_v4l2->out);
    if (ret < 0) {
        pr_err("%s: failed to register_nxp_out()\n", __func__);
        goto err_register_out;
    }
#endif

    ret = v4l2_device_register_subdev_nodes(&nxp_v4l2->v4l2_dev);
    if (ret < 0) {
        pr_err("%s: failed to v4l2_device_register_subdev_nodes()\n", __func__);
        goto err_register_out_subdev;
    }

    platform_set_drvdata(pdev, nxp_v4l2);
    printk("%s success!!!\n", __func__);

    return 0;

err_register_out_subdev:
#ifdef CONFIG_VIDEO_NXP_OUT
    unregister_nxp_out(nxp_v4l2->out);
err_register_out:
    release_nxp_out(nxp_v4l2->out);
err_create_out:
#endif
#ifdef CONFIG_NXP_M2M_SCALER
    unregister_nxp_scaler(scaler);
err_register_scaler:
    release_nxp_scaler(scaler);
#endif
#ifdef CONFIG_VIDEO_NXP_CAPTURE
err_capture_create:
    for (i = 0; i < NXP_MAX_CAPTURE_NUM; ++i) {
        capture = nxp_v4l2->capture[i];
        if (capture) {
            unregister_nxp_capture(capture);
            release_nxp_capture(capture);
        }
    }
    media_device_unregister(&nxp_v4l2->media_dev);
#endif
#ifdef CONFIG_LOOPBACK_SENSOR_DRIVER
err_loopback_sensor_create:
    if( loopback_sensor )
    {
      unregister_nxp_loopback_sensor(loopback_sensor);
      release_nxp_loopback_sensor(loopback_sensor);
    }
#endif

err_media_reg:
    v4l2_device_unregister(&nxp_v4l2->v4l2_dev);
err_v4l2_reg:
    vb2_ion_destroy_context(nxp_v4l2->alloc_ctx);
err_alloc_ctx:
    kfree(nxp_v4l2);
    __me = NULL;
    return ret;
}

static int nxp_v4l2_remove(struct platform_device *pdev)
{
    struct nxp_v4l2 *nxp_v4l2 = platform_get_drvdata(pdev);
#ifdef CONFIG_VIDEO_NXP_CAPTURE
    int i;
#endif

    if (!nxp_v4l2)
        return 0;

#ifdef CONFIG_VIDEO_NXP_OUT
    unregister_nxp_out(nxp_v4l2->out);
    release_nxp_out(nxp_v4l2->out);
#endif

#ifdef CONFIG_NXP_M2M_SCALER
    if (nxp_v4l2->scaler) {
        unregister_nxp_scaler(nxp_v4l2->scaler);
        release_nxp_scaler(nxp_v4l2->scaler);
    }
#endif

#ifdef CONFIG_VIDEO_NXP_CAPTURE
    for (i = 0; i < NXP_MAX_CAPTURE_NUM; ++i) {
        struct nxp_capture *capture = nxp_v4l2->capture[i];
        if (capture) {
            unregister_nxp_capture(capture);
            release_nxp_capture(capture);
        }
    }
#endif

    media_device_unregister(&nxp_v4l2->media_dev);
    v4l2_device_unregister(&nxp_v4l2->v4l2_dev);
    vb2_ion_destroy_context(nxp_v4l2->alloc_ctx);
    kfree(nxp_v4l2);

    __me = NULL;
    return 0;
}

#ifdef CONFIG_PM
static int nxp_v4l2_suspend(struct device *dev)
{
    int ret;
    int i;

    /*PM_DBGOUT("+%s\n", __func__);*/

    if (!__me) {
        /*PM_DBGOUT("%s: No Exist\n", __func__);*/
        return 0;
    }

#ifdef CONFIG_VIDEO_NXP_CAPTURE
    for (i = 0; i < NXP_MAX_CAPTURE_NUM; i++) {
        if (__me->capture[i]) {
            ret = suspend_nxp_capture(__me->capture[i]);
            if (ret) {
                /*PM_DBGOUT("failed to suspend capture %d\n", i);*/
                return ret;
            }
        }
    }
#endif

#ifdef CONFIG_NXP_M2M_SCALER
    if (__me->scaler) {
        ret = suspend_nxp_scaler(__me->scaler);
        if (ret) {
            /*PM_DBGOUT("failed to suspend scaler\n");*/
            return ret;
        }
    }
#endif

#ifdef CONFIG_VIDEO_NXP_OUT
    if (__me->out) {
        ret = suspend_nxp_out(__me->out);
        if (ret) {
            /*PM_DBGOUT("failed to suspend out\n");*/
            return ret;
        }
    }
#endif

    /*PM_DBGOUT("-%s\n", __func__);*/
    return 0;
}

static int nxp_v4l2_resume(struct device *dev)
{
    int ret;
    int i;

    /*PM_DBGOUT("+%s\n", __func__);*/
#ifdef CONFIG_VIDEO_NXP_CAPTURE
    for (i = 0; i < NXP_MAX_CAPTURE_NUM; i++) {
        if (__me->capture[i]) {
            ret = resume_nxp_capture(__me->capture[i]);
            if (ret) {
                /*PM_DBGOUT("failed to suspend capture %d\n", i);*/
                return ret;
            }
        }
    }
#endif

#ifdef CONFIG_NXP_M2M_SCALER
    if (__me->scaler) {
        ret = resume_nxp_scaler(__me->scaler);
        if (ret) {
            /*PM_DBGOUT(KERN_ERR "failed to suspend scaler\n");*/
            return ret;
        }
    }
#endif

#ifdef CONFIG_VIDEO_NXP_OUT
    if (__me->out) {
        ret = resume_nxp_out(__me->out);
        if (ret) {
            /*PM_DBGOUT(KERN_ERR "failed to suspend out\n");*/
            return ret;
        }
    }
#endif

    /*PM_DBGOUT("-%s\n", __func__);*/
    return 0;
}
#endif

/* #ifdef CONFIG_PM_RUNTIME */
/* static int nxp_v4l2_runtime_suspend(struct device *dev) */
/* { */
/*     printk("%s entered\n", __func__); */
/*     printk("%s exit\n", __func__); */
/* } */
/*  */
/* static int nxp_v4l2_runtime_resume(struct device *dev) */
/* { */
/*     printk("%s entered\n", __func__); */
/*     printk("%s exit\n", __func__); */
/* } */
/* #endif */

#ifdef CONFIG_PM
static const struct dev_pm_ops nxp_v4l2_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(nxp_v4l2_suspend, nxp_v4l2_resume)
    /* SET_RUNTIME_PM_OPS(nxp_v4l2_runtime_suspend, nxp_v4l2_runtime_resume, NULL) */
};

#define NXP_V4L2_PMOPS (&nxp_v4l2_pm_ops)
#else
#define NXP_V4L2_PMOPS NULL
#endif

static struct platform_device_id nxp_v4l2_id_table[] = {
    { NXP_V4L2_DEV_NAME, 0 },
    { },
};

#ifdef CONFIG_OF
static const struct of_device_id nxp_v4l2_dt_match[] = {
    { .compatible = "nexell,nxp-v4l2" },
    {},
};
MODULE_DEVICE_TABLE(of, nxp_v4l2_dt_match);
#endif

static struct platform_driver nxp_v4l2_driver = {
    .probe      = nxp_v4l2_probe,
    .remove     = nxp_v4l2_remove,
    .id_table   = nxp_v4l2_id_table,
    .driver     = {
        .name   = NXP_V4L2_DEV_NAME,
        .owner  = THIS_MODULE,
        .pm = NXP_V4L2_PMOPS,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(nxp_v4l2_dt_match),
#endif
    },
};

#ifdef CONFIG_NXP_OUT_HDMI
struct nxp_hdmi *get_nxp_hdmi(void) {
     return __me->out->hdmi;
}
#endif

module_platform_driver(nxp_v4l2_driver);

MODULE_AUTHOR("swpark <swpark@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell NXP series V4L2/MEDIA top device driver");
MODULE_LICENSE("GPL");
