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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/amba/serial.h>

#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

#define	CFG_UART_DEBUG_CH			(0)
#define	CFG_UART_DEBUG_BAUDRATE		(115200)
#define	CFG_UART_CLKGEN_CLOCK_HZ	(14750000)

/*
 * Macro
 */
#define	UART_DEBUG_HZ				CFG_UART_CLKGEN_CLOCK_HZ
#define	UART_DEBUG_BAUDRATE			CFG_UART_DEBUG_BAUDRATE

#if	  (0 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		(void*)IO_ADDRESS(PHYS_BASE_UART0)
	#define	UART_CLKG_BASE		(void*)IO_ADDRESS(PHYS_BASE_CLK_22)
	#define	RESET_UART_ID		RESET_ID_UART0
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART0_USESMC		/* Use UART for SmartCard Interface */
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART0_SMCTXENB	/* SmartCard Interface TX mode enable */
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART0_SMCRXENB	/* SmartCard Interface RX mode enable */
#elif (1 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		(void*)IO_ADDRESS(PHYS_BASE_UART1)
	#define	UART_CLKG_BASE		(void*)IO_ADDRESS(PHYS_BASE_CLK_24)
	#define	RESET_UART_ID		RESET_ID_UART1
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_MODEM0_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_MODEM0_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_MODEM0_SMCRXENB
#elif (2 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		(void*)IO_ADDRESS(PHYS_BASE_UART2)
	#define	UART_CLKG_BASE		(void*)IO_ADDRESS(PHYS_BASE_CLK_23)
	#define	RESET_UART_ID		RESET_ID_UART2
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART1_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART1_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART1_SMCRXENB
#elif (3 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		(void*)IO_ADDRESS(PHYS_BASE_UART3)
	#define	UART_CLKG_BASE		(void*)IO_ADDRESS(PHYS_BASE_CLK_25)
	#define	RESET_UART_ID		RESET_ID_UART3
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_NODMA0_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_NODMA0_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_NODMA0_SMCRXENB
#elif (4 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		(void*)IO_ADDRESS(PHYS_BASE_UART4)
	#define	UART_CLKG_BASE		(void*)IO_ADDRESS(PHYS_BASE_CLK_26)
	#define	RESET_UART_ID		RESET_ID_UART4
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_NODMA1_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_NODMA1_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_NODMA1_SMCRXENB
#elif (5 == CFG_UART_DEBUG_CH)
	#define	UART_PHYS_BASE		(void*)IO_ADDRESS(PHYS_BASE_UART5)
	#define	UART_CLKG_BASE		(void*)IO_ADDRESS(PHYS_BASE_CLK_27)
	#define	RESET_UART_ID		RESET_ID_UART5
	#define	TIEOFF_USESMC		TIEOFFINDEX_OF_UART_NODMA2_USESMC
	#define	TIEOFF_SMCTXENB		TIEOFFINDEX_OF_UART_NODMA2_SMCTXENB
	#define	TIEOFF_SMCRXENB		TIEOFFINDEX_OF_UART_NODMA2_SMCRXENB
#else
	#error not support low debug out uart port (0 ~ 5)
#endif

/*
 * Calculate clock
 */
static unsigned int ref_clk = 24000000;
static unsigned int ref_clk_base = IO_ADDRESS(PHYS_BASE_CLKPWR);
#define	getquotient(v, d)	(v/d)

static inline unsigned int pll_rate(unsigned int pllN, unsigned int xtal)
{
	u32 *PLLSETREG = (u32 *)(ref_clk_base + 0x8);
	u32 *PLLSETREG_SSCG = (u32 *)(ref_clk_base + 0x48);

    u32 val, val1, nP, nM, nS, nK;
    u32 temp = 0;
    val   = PLLSETREG[pllN];
    val1  = PLLSETREG_SSCG[pllN];
	xtal /= 1000;	/* Unit Khz */

    nP = (val >> 18) & 0x03F;
    nM = (val >>  8) & 0x3FF;
    nS = (val >>  0) & 0x0FF;
    nK = (val1>> 16) & 0xFFFF;

    if ((pllN > 1) && nK)
        temp = (unsigned int)(getquotient((getquotient((nK * 1000), 65536) * xtal), nP)>>nS);

    return (unsigned int)((getquotient((nM * xtal), nP)>>nS)*1000) + temp;
}

/*
 * Registers
 */
#define RX_FIFO_COUNT_MASK	(0xff)
#define RX_FIFO_FULL_MASK	(1 << 8)
#define TX_FIFO_FULL_MASK	(1 << 24)

union br_rest {
    unsigned short  slot;       /* udivslot */
    unsigned char   value;      /* ufracval */
};

struct s5p_uart {
    unsigned int    ulcon;
    unsigned int    ucon;
    unsigned int    ufcon;
    unsigned int    umcon;
    unsigned int    utrstat;
    unsigned int    uerstat;
    unsigned int    ufstat;
    unsigned int    umstat;
    unsigned char   utxh;
    unsigned char   res1[3];
    unsigned char   urxh;
    unsigned char   res2[3];
    unsigned int    ubrdiv;
    union br_rest   rest;
    unsigned char   res3[0x3d0];
};

static const int udivslot[] = {
	0, 0x0080, 0x0808, 0x0888, 0x2222, 0x4924, 0x4a52, 0x54aa,
	0x5555, 0xd555, 0xd5d5, 0xddd5, 0xdddd, 0xdfdd, 0xdfdf, 0xffdf,
};

struct uart_data {
	/* clkgen */
	int pll, div;
	long rate;
	/* uart */
	unsigned int ubrdiv;
 	unsigned int udivslot;
 	unsigned int ulcon;
 	unsigned int ufcon;
};

/*
 * Low level debug function.
 * default debug port is '0'
 */
static struct uart_data udata = { 0 ,};

#define	MAX_DIVIDER			((1<<8) - 1)	// 256, align 2
#define	DIVIDER_ALIGN		(2)

static long calc_uart_clock(long request, int *pllsel, int *plldiv)
{
	unsigned long rate = 0, clkhz[3], freqhz = 0, pllhz;
	int pll = 0, div = 0, divide, maxdiv, align, n;

	clkhz[0] = pll_rate(0, ref_clk);
	clkhz[1] = pll_rate(1, ref_clk);
	clkhz[2] = pll_rate(2, ref_clk);

	for (n = 0; ARRAY_SIZE(clkhz) > n; n++) {
	#ifdef  CONFIG_ARM_NXP_CPUFREQ
		if (n == CONFIG_NXP_CPUFREQ_PLLDEV)
			continue;
	#endif
		pllhz = clkhz[n];
		divide = (pllhz/request);
		maxdiv = MAX_DIVIDER & ~(DIVIDER_ALIGN-1);
		align = (divide & ~(DIVIDER_ALIGN-1)) + DIVIDER_ALIGN;

		if (!divide) {
			divide = 1;
		} else {
			if (1 != divide)
				divide &= ~(DIVIDER_ALIGN-1);

			if (divide != align &&
				abs(request - pllhz/divide) >
				abs(request - pllhz/align))
				divide = align;

			divide = (divide > maxdiv ? maxdiv : divide);
		}
		freqhz = pllhz / divide;

		if (rate && (abs(freqhz-request) > abs(rate-request)))
			continue;

		rate = freqhz;
		div = divide;
		pll = n;
	}

	if (pllsel)
		*pllsel = pll;

	if (plldiv)
		*plldiv = div;

	return rate;
}

inline static void uart_init(void)
{
	void *CLKENB = UART_CLKG_BASE;
	void *CLKGEN = UART_CLKG_BASE + 0x04;
	struct uart_data *pdat = &udata;
	struct s5p_uart *uart = (struct s5p_uart *)UART_PHYS_BASE;
	unsigned int baudrate = UART_DEBUG_BAUDRATE;
	unsigned int clkval;

	/* Clock Generotor & reset */
	if (0 == pdat->rate) {
		u32 val = UART_DEBUG_HZ / baudrate;
		pdat->rate = calc_uart_clock(UART_DEBUG_HZ, &pdat->pll, &pdat->div);
		pdat->ubrdiv = (val/16) - 1;
		pdat->udivslot = udivslot[val % 16];
		/* NORMAL | No parity | 1 stop | 8bit */
		pdat->ulcon = (((0 & 0x1)<<6) | ((0 & 0x3)<<3) | ((0 & 0x1)<<2) | ((3 & 0x3)<<0));
		/* Tx FIFO clr | Rx FIFO clr | FIFOs EN */
		pdat->ufcon = (((1 & 0x1)<<1) | ((1 & 0x1)<<0));
	}

	/* check reset */
	if (!nxp_soc_peri_reset_status(RESET_UART_ID)) {
		NX_TIEOFF_Set(TIEOFF_USESMC  , 0);
		NX_TIEOFF_Set(TIEOFF_SMCTXENB, 0);
		NX_TIEOFF_Set(TIEOFF_SMCRXENB, 0);
		nxp_soc_peri_reset_set(RESET_UART_ID);
	}

	/* check pll : alaway enable clkgen */
	clkval = readl(CLKGEN) & ~(0x07<<2) & ~(0xFF<<5);
	writel((clkval|(pdat->pll<<2)|((pdat->div-1)<<5)), CLKGEN);
	writel((readl(CLKENB)|(1<<2)), CLKENB);

	/* Uart Register */
	writel(pdat->ufcon, &uart->ufcon);
	writel(pdat->ulcon, &uart->ulcon);
	writel(pdat->ubrdiv, &uart->ubrdiv);
	writew(pdat->udivslot, &uart->rest.slot);
}

inline static void uart_putc(char ch)
{
	struct s5p_uart *uart = (struct s5p_uart *)UART_PHYS_BASE;
	unsigned int mask = 0x8;

	/* wait for room in the tx FIFO */
	while ((readl(&uart->ufstat) & TX_FIFO_FULL_MASK)) {
		if (readl(&uart->uerstat) & mask)
			return;
	}

	writeb(ch, &uart->utxh);
}

inline static char uart_getc(void)
{
	struct s5p_uart *uart = (struct s5p_uart *)UART_PHYS_BASE;
	unsigned int mask = 0xf;

	/* wait for character to arrive */
	while (!(readl(&uart->ufstat) & (RX_FIFO_COUNT_MASK |
					 RX_FIFO_FULL_MASK))) {
		if (readl(&uart->uerstat) & mask)
			return 0;
	}

	return (int)(readb(&uart->urxh) & 0xff);
}

inline static int uart_tstc(void)
{
	struct s5p_uart *uart = (struct s5p_uart *)UART_PHYS_BASE;
	return (int)(readl(&uart->utrstat) & 0x1);
}

/*
 * Low level uart interface
 */
void lldebug_init(void)
{
	uart_init();
}

void lldebug_putc(const char ch)
{
   /* If \n, also do \r */
	if (ch == '\n')
    	uart_putc('\r');
	uart_putc(ch);
}

int lldebug_getc(void)
{
	return uart_getc();
}

void lldebug_puts(const char *str)
{
	while (*str)
		lldebug_putc(*str++);
}

int lldebug_tstc(void)
{
	return uart_tstc();
}

/*
 * Low level debug interface.
 */
static DEFINE_SPINLOCK(lld_lock);

void lldebugout(const char *fmt, ...)
{
	va_list va;
	char buff[256];
	u_long flags;

	spin_lock_irqsave(&lld_lock, flags);

	lldebug_init();

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	/* direct debug out */
	lldebug_puts(buff);

	spin_unlock_irqrestore(&lld_lock, flags);
}
EXPORT_SYMBOL_GPL(lldebugout);
