/*
 * Copyright (c) 2016 Nexell Co., Ltd.
 *		http://www.nexell.co.kr
 *
 * Author: KOO Bon-gyu <freestyle@nexell.co.kr>
 */


#ifndef __PINCTRL_NXP_H__
#define __PINCTRL_NXP_H__

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>

#include <linux/gpio.h>

/* pinmux function number for pin as gpio output line */
#define FUNC_INPUT	0x0
#define FUNC_OUTPUT	0x1

/**
 * enum pincfg_type - possible pin configuration types supported.
 * @PINCFG_TYPE_FUNC: Function configuration.
 * @PINCFG_TYPE_DAT: Pin value configuration.
 * @PINCFG_TYPE_PULL: Pull up/down configuration.
 * @PINCFG_TYPE_STRENGTH: Drive strength configuration.
 */
enum pincfg_type {
	PINCFG_TYPE_FUNC,
	PINCFG_TYPE_DAT,
	PINCFG_TYPE_PULL,
	PINCFG_TYPE_STRENGTH,

	PINCFG_TYPE_NUM
};

/*
 * pin configuration (pull up/down and drive strength) type and its value are
 * packed together into a 16-bits. The upper 8-bits represent the configuration
 * type and the lower 8-bits hold the value of the configuration type.
 */
#define PINCFG_TYPE_MASK		0xFF
#define PINCFG_VALUE_SHIFT		8
#define PINCFG_VALUE_MASK		(0xFF << PINCFG_VALUE_SHIFT)
#define PINCFG_PACK(type, value)	(((value) << PINCFG_VALUE_SHIFT) | type)
#define PINCFG_UNPACK_TYPE(cfg)		((cfg) & PINCFG_TYPE_MASK)
#define PINCFG_UNPACK_VALUE(cfg)	(((cfg) & PINCFG_VALUE_MASK) >> \
						PINCFG_VALUE_SHIFT)
/**
 * enum eint_type - possible external interrupt types.
 * @EINT_TYPE_NONE: bank does not support external interrupts
 * @EINT_TYPE_GPIO: bank supportes external gpio interrupts
 * @EINT_TYPE_WKUP: bank supportes external wakeup interrupts
 * @EINT_TYPE_WKUP_MUX: bank supports multiplexed external wakeup interrupts
 *
 * Samsung GPIO controller groups all the available pins into banks. The pins
 * in a pin bank can support external gpio interrupts or external wakeup
 * interrupts or no interrupts at all. From a software perspective, the only
 * difference between external gpio and external wakeup interrupts is that
 * the wakeup interrupts can additionally wakeup the system if it is in
 * suspended state.
 */
enum eint_type {
	EINT_TYPE_NONE,
	EINT_TYPE_GPIO,
	EINT_TYPE_WKUP,
	EINT_TYPE_WKUP_MUX,
};

/* maximum length of a pin in pin descriptor (example: "gpioa-30") */
#define PIN_NAME_LENGTH	10

#define PIN_GROUP(n, p, f)				\
	{						\
		.name		= n,			\
		.pins		= p,			\
		.num_pins	= ARRAY_SIZE(p),	\
		.func		= f			\
	}

#define PMX_FUNC(n, g)					\
	{						\
		.name		= n,			\
		.groups		= g,			\
		.num_groups	= ARRAY_SIZE(g),	\
	}

struct nexell_pinctrl_drv_data;

/**
 * struct nexell_pin_bank_type: pin bank type description
 * @fld_width: widths of configuration bitfields (0 if unavailable)
 * @reg_offset: offsets of configuration registers (don't care of width is 0)
 */
struct nexell_pin_bank_type {
	u8 fld_width[PINCFG_TYPE_NUM];
	u8 reg_offset[PINCFG_TYPE_NUM];
};

/**
 * struct nexell_pin_bank: represent a controller pin-bank.
 * @type: type of the bank (register offsets and bitfield widths)
 * @pctl_offset: starting offset of the pin-bank registers.
 * @pin_base: starting pin number of the bank.
 * @nr_pins: number of pins included in this bank.
 * @eint_func: function to set in CON register to configure pin as EINT.
 * @eint_type: type of the external interrupt supported by the bank.
 * @eint_mask: bit mask of pins which support EINT function.
 * @name: name to be prefixed for each pin in this pin bank.
 * @of_node: OF node of the bank.
 * @drvdata: link to controller driver data
 * @irq_domain: IRQ domain of the bank.
 * @gpio_chip: GPIO chip of the bank.
 * @grange: linux gpio pin range supported by this bank.
 * @slock: spinlock protecting bank registers
 * @pm_save: saved register values during suspend
 */
struct nexell_pin_bank {
	int index;
	struct nexell_pin_bank_type *type;
	u32				pctl_offset;
	u32				pin_base;
	u8				nr_pins;
	u8				eint_func;
	enum eint_type	eint_type;
	u32				eint_mask;
	u32				eint_offset;
	char			*name;
	void			*soc_priv;
	struct device_node *of_node;
	struct nexell_pinctrl_drv_data *drvdata;
	struct irq_domain *irq_domain;
	struct gpio_chip gpio_chip;
	struct pinctrl_gpio_range grange;
	spinlock_t slock;

	u32 pm_save[PINCFG_TYPE_NUM + 1]; /* +1 to handle double CON registers*/
};

/**
 * struct nexell_pin_ctrl: represent a pin controller.
 * @pin_banks: list of pin banks included in this controller.
 * @nr_banks: number of pin banks.
 * @base: starting system wide pin number.
 * @nr_pins: number of pins supported by the controller.
 * @eint_gpio_init: platform specific callback to setup the external gpio
 *	interrupts for the controller.
 * @eint_wkup_init: platform specific callback to setup the external wakeup
 *	interrupts for the controller.
 * @label: for debug information.
 */
struct nexell_pin_ctrl {
	struct nexell_pin_bank	*pin_banks;
	u32		nr_banks;

	u32		base;
	u32		nr_pins;

	int		(*eint_gpio_init)(struct nexell_pinctrl_drv_data *);
	int		(*eint_wkup_init)(struct nexell_pinctrl_drv_data *);
	void		(*suspend)(struct nexell_pinctrl_drv_data *);
	void		(*resume)(struct nexell_pinctrl_drv_data *);

	char		*label;
};

/**
 * struct nexell_pinctrl_drv_data: wrapper for holding driver data together.
 * @node: global list node
 * @virt_base: register base address of the controller.
 * @dev: device instance representing the controller.
 * @irq: interrpt number used by the controller to notify gpio interrupts.
 * @ctrl: pin controller instance managed by the driver.
 * @pctl: pin controller descriptor registered with the pinctrl subsystem.
 * @pctl_dev: cookie representing pinctrl device instance.
 * @pin_groups: list of pin groups available to the driver.
 * @nr_groups: number of such pin groups.
 * @pmx_functions: list of pin functions available to the driver.
 * @nr_function: number of such pin functions.
 */
struct nexell_pinctrl_drv_data {
	struct list_head		node;
	void __iomem			*virt_base;
	struct device			*dev;
	int				irq;

	struct nexell_pin_ctrl		*ctrl;
	struct pinctrl_desc		pctl;
	struct pinctrl_dev		*pctl_dev;

	const struct nexell_pin_group	*pin_groups;
	unsigned int			nr_groups;
	const struct nexell_pmx_func	*pmx_functions;
	unsigned int			nr_functions;
};

/**
 * struct nexell_pin_group: represent group of pins of a pinmux function.
 * @name: name of the pin group, used to lookup the group.
 * @pins: the pins included in this group.
 * @num_pins: number of pins included in this group.
 * @func: the function number to be programmed when selected.
 */
struct nexell_pin_group {
	const char		*name;
	const unsigned int	*pins;
	u8			num_pins;
	u8			func;
};

/**
 * struct nexell_pmx_func: represent a pin function.
 * @name: name of the pin function, used to lookup the function.
 * @groups: one or more names of pin groups that provide this function.
 * @num_groups: number of groups included in @groups.
 */
struct nexell_pmx_func {
	const char		*name;
	const char		**groups;
	u8			num_groups;
	u32			val;
};

/* list of all exported SoC specific data */
extern struct nexell_pin_ctrl s5p6818_pin_ctrl[];

#endif	/* __PINCTRL_NXP_H__ */
