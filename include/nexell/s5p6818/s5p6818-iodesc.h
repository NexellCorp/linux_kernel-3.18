/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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
#ifndef _S5P6818_IO_DESC_H
#define _S5P6818_IO_DESC_H

/*
 * gpio descriptor
 */
#define	IO_ALT_0 	(0)
#define IO_ALT_1 	(1)
#define IO_ALT_2 	(2)
#define	IO_ALT_3 	(3)

#define	ALT_NO_GPIO_A	{ \
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	}

#define	ALT_NO_GPIO_B	{ \
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_2, IO_ALT_2, IO_ALT_1, IO_ALT_2, IO_ALT_1,	\
	IO_ALT_2, IO_ALT_1, IO_ALT_2, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1,	\
	IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1,	\
	}

#define	ALT_NO_GPIO_C	{ \
	IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1,	\
	IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1,	\
	IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1,	\
	IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	}

#define	ALT_NO_GPIO_D	{ \
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	}

#define	ALT_NO_GPIO_E	{ \
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1, IO_ALT_1,	\
	}

#define	ALT_NO_ALIVE	{ \
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0, IO_ALT_0,	\
	}

#define	GPIO_NUM_PER_BANK	(32)

extern const unsigned char pio_fn_no[][GPIO_NUM_PER_BANK];
#define GET_GPIO_ALTFUNC(idx,io)    (pio_fn_no[idx][io])

#endif