#ifndef __THERMAL_HW_H__
#define __THERMAL_HW_H__

struct nxp_thermal_trigger {
	int   trig_type;		/* DTS: not use, passive=1, hot=2, critical=3 */
	int	  trig_temp;
	long  trig_duration;	/* ms */
	long  trig_frequency;
	int   triggered;
};

struct nxp_theramal_plat_data {
	int channel;
	int poll_duration;						/* DTS: "polling-delay" */
	struct nxp_thermal_trigger triggers[3];	/* DTS: triggers */
	int trigger_size;						/* DTS: not use */
	int trig_step_up;						/* DTS: "trigger-step-up" */
};
#endif
