#ifndef _NXP_V4L2_PLATFORMDATA_H
#define _NXP_V4L2_PLATFORMDATA_H __FILE__

#include <linux/types.h>

/* device name */
#define NXP_V4L2_DEV_NAME           "nxp-v4l2"

struct platform_device;

enum nxp_capture_interface_type {
    NXP_CAPTURE_INF_PARALLEL,
    NXP_CAPTURE_INF_CSI,
};

struct nxp_mipi_csi_platformdata {
    int     module;
    ulong   clk_rate;
    u32     lanes;
    u32     alignment;
    u32     hs_settle;
    u32     width;
    u32     height;
    bool    fixed_phy_vdd;
    int     irq;
    void __iomem *base;
    int     (*phy_enable)(bool);
};

enum nxp_vin_data_order {
    NXP_VIN_CBY0CRY1,
    NXP_VIN_CRY1CBY0,
    NXP_VIN_Y0CBY1CR,
    NXP_VIN_Y1CRY0CB,
};

enum nxp_capture_sensor_type {
    NXP_CAPTURE_I2C,
    NXP_CAPTURE_SPI,
    NXP_CAPTURE_LOOPBACK,
    NXP_CAPTURE_MAX
};

struct nxp_vin_platformdata {
    bool    is_mipi;
    bool    external_sync;
    u32     h_active;
    u32     h_frontporch;
    u32     h_syncwidth;
    u32     h_backporch;
    u32     v_active;
    u32     v_frontporch;
    u32     v_backporch;
    u32     v_syncwidth;
    bool    clock_invert;
    u32     port;
    enum nxp_vin_data_order data_order;
    bool    interlace;
    bool    late_power_down;
#ifndef CONFIG_OF
    ulong   clk_rate;
    int     (*power_enable)(bool);
    bool    (*power_state_changed)(void);
    int     (*set_clock)(ulong);
    void    (*setup_io)(int, bool);
#else
    u32     vid;
    int     enable_io;
    bool    enable_invert;
    int     enable_delay_ms;
    int     reset_io;
    bool    reset_invert;
    int     reset_delay_ms;
    int     regulator_nr;
    char    **regulator_names;
    int     pwm_number;
    u32     clock_rate;
    // args : regulator_nr, regulator_names, enable_io,
    int     (*power_enable)(struct nxp_vin_platformdata *, bool);
    bool    (*power_state_changed)(struct nxp_vin_platformdata *);
    // args : pwm_number, clock_rate
    int     (*set_clock)(struct nxp_vin_platformdata *, bool);
    // args : vid
    void    (*setup_io)(struct nxp_vin_platformdata *, bool);
#endif
};

struct nxp_decimator_platformdata {
    u32     start_delay_ms;
    u32     stop_delay_ms;
};

struct nxp_v4l2_i2c_board_info {
    struct i2c_board_info *board_info;
    int     i2c_adapter_id;
};

struct nxp_capture_platformdata {
    int module;
    struct nxp_v4l2_i2c_board_info *sensor;
    enum nxp_capture_interface_type   type;
    struct nxp_vin_platformdata   parallel; /* always */
    struct nxp_decimator_platformdata deci;
    struct nxp_mipi_csi_platformdata *csi;  /* only csi */
};

struct nxp_hdmi_platformdata {
    int internal_irq; /* hdmi irq */
    int external_irq; /* gpio */
    void __iomem *base;
    void (*set_int_external)(int gpio);
    void (*set_int_internal)(int gpio);
    int  (*read_hpd_gpio)(int gpio);
    struct nxp_v4l2_i2c_board_info *edid;
    struct nxp_v4l2_i2c_board_info *hdcp;
};

struct nxp_out_platformdata {
    struct nxp_hdmi_platformdata hdmi;
};

struct nxp_v4l2_platformdata {
#ifdef CONFIG_OF
    int capture_nr;
#endif
    struct nxp_capture_platformdata *captures;
    struct nxp_out_platformdata     *out;
};

#endif
