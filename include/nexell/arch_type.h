#ifndef __ARCH_TYPE_H__
#define __ARCH_TYPE_H__

#define PAD_MD_POS    	0
#define PAD_MD_MASK     0xF

#define PAD_FN_POS    	4
#define PAD_FN_MASK     0xF

#define PAD_LV_POS      8
#define PAD_LV_MASK     0xF

#define PAD_PU_POS      12
#define PAD_PU_MASK     0xF

#define PAD_ST_POS      16
#define PAD_ST_MASK     0xF

#define	PAD_MODE_ALT    		((0 & PAD_MD_MASK) << PAD_MD_POS)
#define	PAD_MODE_IN     		((1 & PAD_MD_MASK) << PAD_MD_POS)
#define	PAD_MODE_OUT    		((2 & PAD_MD_MASK) << PAD_MD_POS)
#define	PAD_MODE_INT    		((3 & PAD_MD_MASK) << PAD_MD_POS)

#define	PAD_FUNC_ALT0  			((0 & PAD_FN_MASK) << PAD_FN_POS)
#define	PAD_FUNC_ALT1  			((1 & PAD_FN_MASK) << PAD_FN_POS)
#define	PAD_FUNC_ALT2  			((2 & PAD_FN_MASK) << PAD_FN_POS)
#define	PAD_FUNC_ALT3  			((3 & PAD_FN_MASK) << PAD_FN_POS)

#define PAD_LEVEL_LOW         	((0 & PAD_LV_MASK) << PAD_LV_POS)	/* if alive, async lowlevel */
#define PAD_LEVEL_HIGH        	((1 & PAD_LV_MASK) << PAD_LV_POS)   /* if alive, async highlevel */
#define PAD_LEVEL_FALLINGEDGE 	((2 & PAD_LV_MASK) << PAD_LV_POS)   /* if alive, async fallingedge */
#define PAD_LEVEL_RISINGEDGE  	((3 & PAD_LV_MASK) << PAD_LV_POS)   /* if alive, async eisingedge */
#define PAD_LEVEL_LOW_SYNC    	((4 & PAD_LV_MASK) << PAD_LV_POS)   /* if gpio , not support */
#define PAD_LEVEL_HIGH_SYNC   	((5 & PAD_LV_MASK) << PAD_LV_POS)   /* if gpio , not support */
#define PAD_LEVEL_BOTHEDGE    	((4 & PAD_LV_MASK) << PAD_LV_POS)   /* if alive, not support */
#define PAD_LEVEL_ALT         	((6 & PAD_LV_MASK) << PAD_LV_POS)   /* if pad function is alt, not set */

#define PAD_PULL_DN     		((0 & PAD_PU_MASK) << PAD_PU_POS)   /* Do not support Alive-GPIO */
#define PAD_PULL_UP     		((1 & PAD_PU_MASK) << PAD_PU_POS)
#define PAD_PULL_OFF    		((2 & PAD_PU_MASK) << PAD_PU_POS)

#define PAD_STRENGTH_0 			((0 & PAD_ST_MASK) << PAD_ST_POS)
#define PAD_STRENGTH_1 			((1 & PAD_ST_MASK) << PAD_ST_POS)
#define PAD_STRENGTH_2 			((2 & PAD_ST_MASK) << PAD_ST_POS)
#define PAD_STRENGTH_3 			((3 & PAD_ST_MASK) << PAD_ST_POS)

#define PAD_GET_GROUP(pin)      ((pin >> 0x5) & 0x07)       /* Divide 32 */
#define PAD_GET_BITNO(pin)      (pin & 0x1F)
#define PAD_GET_FUNC(pin)       ((pin >> PAD_FN_POS) & PAD_FN_MASK)
#define PAD_GET_MODE(pin)       ((pin >> PAD_MD_POS) & PAD_MD_MASK)
#define PAD_GET_LEVEL(pin)      ((pin >> PAD_LV_POS) & PAD_LV_MASK)
#define PAD_GET_PULLUP(pin)     ((pin >> PAD_PU_POS) & PAD_PU_MASK)
#define PAD_GET_STRENGTH(pin)   ((pin >> PAD_ST_POS) & PAD_ST_MASK)

#define PAD_GPIO_A      		(0 * 32)
#define PAD_GPIO_B      		(1 * 32)
#define PAD_GPIO_C      		(2 * 32)
#define PAD_GPIO_D      		(3 * 32)
#define PAD_GPIO_E      		(4 * 32)
#define PAD_GPIO_ALV    		(5 * 32)

/* Alive wakeup detect mode */
#define	PM_DECT_ASYNC_LOWLEVEL 	(0)
#define	PM_DECT_ASYNC_HIGHLEVEL	(1)
#define	PM_DECT_FALLINGEDGE 	(2)
#define	PM_DECT_RISINGEDGE  	(3)
#define	PM_DECT_SYNC_LOWLEVEL 	(4)
#define	PM_DECT_SYNC_HIGHLEVEL 	(5)
#define	PM_DECT_BOTHEDGE  		(6)

/*	the data output format. */
#define	DPC_FORMAT_RGB555     	0	///< RGB555 Format
#define	DPC_FORMAT_RGB565     	1	///< RGB565 Format
#define	DPC_FORMAT_RGB666     	2	///< RGB666 Format
#define	DPC_FORMAT_RGB888     	3	///< RGB888 Format
#define	DPC_FORMAT_MRGB555A   	4	///< MRGB555A Format
#define	DPC_FORMAT_MRGB555B   	5	///< MRGB555B Format
#define	DPC_FORMAT_MRGB565    	6	///< MRGB565 Format
#define	DPC_FORMAT_MRGB666    	7	///< MRGB666 Format
#define	DPC_FORMAT_MRGB888A   	8	///< MRGB888A Format
#define	DPC_FORMAT_MRGB888B   	9	///< MRGB888B Format
#define	DPC_FORMAT_CCIR656    	10	///< ITU-R BT.656 / 601(8-bit)
#define	DPC_FORMAT_CCIR601A		12	///< ITU-R BT.601A
#define	DPC_FORMAT_CCIR601B		13	///< ITU-R BT.601B
#define DPC_FORMAT_4096COLOR    1  	///< 4096 Color Format
#define DPC_FORMAT_16GRAY       3  	///< 16 Level Gray Format

/*	RGB layer pixel format. */
#define	MLC_RGBFMT_R5G6B5    	0x44320000	///< 16bpp { R5, G6, B5 }.
#define	MLC_RGBFMT_B5G6R5    	0xC4320000	///< 16bpp { B5, G6, R5 }.
#define	MLC_RGBFMT_X1R5G5B5  	0x43420000	///< 16bpp { X1, R5, G5, B5 }.
#define	MLC_RGBFMT_X1B5G5R5  	0xC3420000	///< 16bpp { X1, B5, G5, R5 }.
#define	MLC_RGBFMT_X4R4G4B4  	0x42110000	///< 16bpp { X4, R4, G4, B4 }.
#define	MLC_RGBFMT_X4B4G4R4  	0xC2110000	///< 16bpp { X4, B4, G4, R4 }.
#define	MLC_RGBFMT_X8R3G3B2  	0x41200000	///< 16bpp { X8, R3, G3, B2 }.
#define	MLC_RGBFMT_X8B3G3R2  	0xC1200000	///< 16bpp { X8, B3, G3, R2 }.
#define	MLC_RGBFMT_A1R5G5B5  	0x33420000	///< 16bpp { A1, R5, G5, B5 }.
#define	MLC_RGBFMT_A1B5G5R5  	0xB3420000	///< 16bpp { A1, B5, G5, R5 }.
#define	MLC_RGBFMT_A4R4G4B4  	0x22110000	///< 16bpp { A4, R4, G4, B4 }.
#define	MLC_RGBFMT_A4B4G4R4  	0xA2110000	///< 16bpp { A4, B4, G4, R4 }.
#define	MLC_RGBFMT_A8R3G3B2  	0x11200000	///< 16bpp { A8, R3, G3, B2 }.
#define	MLC_RGBFMT_A8B3G3R2  	0x91200000	///< 16bpp { A8, B3, G3, R2 }.
#define	MLC_RGBFMT_R8G8B8    	0x46530000	///< 24bpp { R8, G8, B8 }.
#define	MLC_RGBFMT_B8G8R8    	0xC6530000	///< 24bpp { B8, G8, R8 }.
#define	MLC_RGBFMT_X8R8G8B8  	0x46530000	///< 32bpp { X8, R8, G8, B8 }.
#define	MLC_RGBFMT_X8B8G8R8  	0xC6530000	///< 32bpp { X8, B8, G8, R8 }.
#define	MLC_RGBFMT_A8R8G8B8  	0x06530000	///< 32bpp { A8, R8, G8, B8 }.
#define	MLC_RGBFMT_A8B8G8R8  	0x86530000	///< 32bpp { A8, B8, G8, R8 }.

/*	the data output order in case of ITU-R BT.656 / 601. */
#define	DPC_YCORDER_CbYCrY		0	///< Cb, Y, Cr, Y
#define	DPC_YCORDER_CrYCbY		1	///< Cr, Y, Cb, Y
#define	DPC_YCORDER_YCbYCr		2	///< Y, Cb, Y, Cr
#define	DPC_YCORDER_YCrYCb		3	///< Y, Cr, Y, Cb

/* the PAD output clock. */
#define	DPC_PADCLKSEL_VCLK		0	///< VCLK
#define	DPC_PADCLKSEL_VCLK2		1	///< VCLK2

/*  LVDS output format. */
#define LVDS_LCDFORMAT_VESA  	0
#define LVDS_LCDFORMAT_JEIDA 	1
#define LVDS_LCDFORMAT_LOC   	2

/*f Alive wakeup detect mode */
#define	PWR_DECT_ASYNC_LOWLEVEL 	0
#define	PWR_DECT_ASYNC_HIGHLEVEL 	1
#define	PWR_DECT_FALLINGEDGE 		2
#define	PWR_DECT_RISINGEDGE  		3
#define	PWR_DECT_SYNC_LOWLEVEL 		4
#define	PWR_DECT_SYNC_HIGHLEVEL 	5
#define	PWR_DECT_BOTHEDGE  			6

#endif
