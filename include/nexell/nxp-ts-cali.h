#ifndef __NXP_TS_CALI_H__
#define __NXP_TS_CALI_H__
struct nxp_ts_cali_plat_data {
    int touch_points;    /* support touch points num when multi touch */
    int x_resol;
    int y_resol;
    int rotate;          /* 0, 90, 180, 270 */
	int ptr_cnt;
    long pointercal[10];     /* calibration value (tslib) */
	int touch_irq;
};

#endif /*__NXP_TS_CALI_H__*/
