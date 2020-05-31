/*
 * MUSB OTG driver defines
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MUSBFSH_CORE_H__
#define __MUSBFSH_CORE_H__

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/otg.h>

struct musbfsh;
struct musbfsh_hw_ep;
struct musbfsh_ep;

/* Helper defines for struct musbfsh->hwvers */
#define MUSBFSH_HWVERS_MAJOR(x)	((x >> 10) & 0x1f)
#define MUSBFSH_HWVERS_MINOR(x)	(x & 0x3ff)
#define MUSBFSH_HWVERS_RC		0x8000
#define MUSBFSH_HWVERS_1300	0x52C
#define MUSBFSH_HWVERS_1400	0x590
#define MUSBFSH_HWVERS_1800	0x720
#define MUSBFSH_HWVERS_1900	0x784
#define MUSBFSH_HWVERS_2000	0x800

#include <linux/musbfsh.h>
#include "musbfsh_io.h"
#include "musbfsh_regs.h"
#include "musbfsh_debug.h"

#include <linux/xlog.h>
#include <linux/switch.h>
#include <linux/i2c.h>
#include <mach/irqs.h>
#include <mach/eint.h>


#include <linux/platform_device.h>

#include <cust_gpio_usage.h>
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_clkmgr.h>
#include <mach/emi_mpu.h>
#include <mach/mt_irq.h>
#include <mach/mt_reg_base.h>

//#define MT_USB1_IRQ_ID                      (32 + 33)
//#define USB1_BASE                   0xF1270000


/* NOTE:  otg and peripheral-only state machines start at B_IDLE.
 * OTG or host-only go to A_IDLE when ID is sensed.
 */
#define is_host_active(m)		((m)->is_host)

/****************************** HOST ROLE ***********************************/

#define	is_host_capable()	(1)

#define host_hc_driver_flag() (HCD_USB2 | HCD_MEMORY)

extern irqreturn_t musbfsh_h_ep0_irq(struct musbfsh *);
extern void musbfsh_host_tx(struct musbfsh *, u8);
extern void musbfsh_host_rx(struct musbfsh *, u8);

/****************************** CONSTANTS ********************************/

#ifndef MUSBFSH_C_NUM_EPS
#define MUSBFSH_C_NUM_EPS ((u8)16)
#endif

#ifndef MUSBFSH_MAX_END0_PACKET
#define MUSBFSH_MAX_END0_PACKET ((u16)MUSBFSH_EP0_FIFOSIZE)
#endif

/* host side ep0 states */
enum musbfsh_h_ep0_state {
	MUSBFSH_EP0_IDLE,
	MUSBFSH_EP0_START,			/* expect ack of setup */
	MUSBFSH_EP0_IN,			/* expect IN DATA */
	MUSBFSH_EP0_OUT,			/* expect ack of OUT DATA */
	MUSBFSH_EP0_STATUS,		/* expect ack of STATUS */
} __attribute__ ((packed));

/*************************** REGISTER ACCESS ********************************/

/* "indexed" mapping: INDEX register controls register bank select */
#define musbfsh_ep_select(_mbase, _epnum) \
	musbfsh_writeb((_mbase), MUSBFSH_INDEX, (_epnum))
#define	MUSBFSH_EP_OFFSET			MUSBFSH_INDEXED_OFFSET

/****************************** FUNCTIONS ********************************/

#define test_devctl_hst_mode(_x) \
	(musbfsh_readb((_x)->mregs, MUSBFSH_DEVCTL)&MUSBFSH_DEVCTL_HM)

/******************************** TYPES *************************************/
struct dma_channel;

/**
 * struct musb_platform_ops - Operations passed to musb_core by HW glue layer
 * @init:	turns on clocks, sets up platform-specific registers, etc
 * @exit:	undoes @init
 * @set_mode:	forcefully changes operating mode
 * @try_ilde:	tries to idle the IP
 * @vbus_status: returns vbus status if possible
 * @set_vbus:	forces vbus status
 * @adjust_channel_params: pre check for standard dma channel_program func
 */
struct musbfsh_platform_ops {
	int	(*init)(struct musbfsh *musbfsh);
	int	(*exit)(struct musbfsh *musbfsh);

	void	(*enable)(struct musbfsh *musbfsh);
	void	(*disable)(struct musbfsh *musbfsh);

	int	(*set_mode)(struct musbfsh *musbfsh, u8 mode);
	void	(*try_idle)(struct musbfsh *musbfsh, unsigned long timeout);

	int	(*vbus_status)(struct musbfsh *musbfsh);
	void	(*set_vbus)(struct musbfsh *musbfsh, int on);
	void	(*set_power)(struct musbfsh *musbfsh, int action);

	int	(*adjust_channel_params)(struct dma_channel *channel,
				u16 packet_sz, u8 *mode,
				dma_addr_t *dma_addr, u32 *len);
};

/*
 * struct musbfsh_hw_ep - endpoint hardware (bidirectional)
 *
 * Ordered slightly for better cacheline locality.
 */
struct musbfsh_hw_ep {
	struct musbfsh		*musbfsh;
    void __iomem		*fifo;
	void __iomem		*regs;
	/* index in musbfsh->endpoints[]  */
	u8			epnum;
	/* hardware configuration, possibly dynamic */
	bool			is_shared_fifo;
	bool			tx_double_buffered;
	bool			rx_double_buffered;
	u16			max_packet_sz_tx;
	u16			max_packet_sz_rx;
	struct dma_channel	*tx_channel;
	struct dma_channel	*rx_channel;
    void __iomem		*target_regs;
	/* currently scheduled peripheral endpoint */
	struct musbfsh_qh		*in_qh;
	struct musbfsh_qh		*out_qh;
	u8			rx_reinit;
	u8			tx_reinit;
};


struct musbfsh_csr_regs {
	/* FIFO registers */
	u16 txmaxp, txcsr, rxmaxp, rxcsr;
	u16 rxfifoadd, txfifoadd;
	u8 txtype, txinterval, rxtype, rxinterval;
	u8 rxfifosz, txfifosz;
	u8 txfunaddr, txhubaddr, txhubport;
	u8 rxfunaddr, rxhubaddr, rxhubport;
};

struct musbfsh_context_registers {

	u8 power;
	u16 intrtxe, intrrxe;
	u8 intrusbe;
	u16 frame;
	u8 index, testmode;

	u8 devctl, busctl, misc;
	u32 otg_interfsel;
	u32 l1_int;

	struct musbfsh_csr_regs index_regs[MUSBFSH_C_NUM_EPS];
};

/*
 * struct musb - Driver instance data.
 */
struct musbfsh {
	/* device lock */
	spinlock_t		lock;
	struct musbfsh_context_registers context;
	const struct musbfsh_platform_ops *ops;
	irqreturn_t		(*isr)(int, void *);

/* this hub status bit is reserved by USB 2.0 and not seen by usbcore */
#define MUSBFSH_PORT_STAT_RESUME	(1 << 31)

	u32			port1_status;

	unsigned long		rh_timer;
	enum musbfsh_h_ep0_state	ep0_stage;

	/* bulk traffic normally dedicates endpoint hardware, and each
	 * direction has its own ring of host side endpoints.
	 * we try to progress the transfer at the head of each endpoint's
	 * queue until it completes or NAKs too much; then we try the next
	 * endpoint.
	 */
	struct musbfsh_hw_ep	*bulk_ep;

	struct list_head	control;	/* of musbfsh_qh */
	struct list_head	in_bulk;	/* of musbfsh_qh */
	struct list_head	out_bulk;	/* of musbfsh_qh */

	struct dma_controller	*dma_controller;
	struct musbfsh_dma_controller *musbfsh_dma_controller;

	struct device		*controller;
	void __iomem		*ctrl_base;
	void __iomem		*mregs;

	/* passed down from chip/board specific irq handlers */
	u8			int_usb;
	u16			int_rx;
	u16			int_tx;
	u8			int_dma;

	int nIrq;
	unsigned		irq_wake:1;

	struct musbfsh_hw_ep	 endpoints[MUSBFSH_C_NUM_EPS];
#define control_ep		endpoints

#define VBUSERR_RETRY_COUNT	3
	u16			vbuserr_retry;
	u16 epmask;
	u8 nr_endpoints;

	u8 board_mode;		/* enum musbfsh_mode */
	int			(*board_set_power)(int state);
	bool			is_host;

	unsigned		is_multipoint:1; 

	unsigned long		idle_timeout;	/* Next timeout in jiffies */

	/* active means connected and not suspended */
	unsigned		is_active:1;
	unsigned ignore_disconnect:1;	/* during bus resets */

	unsigned		hb_iso_rx:1;	/* high bandwidth iso rx? */
	unsigned		hb_iso_tx:1;	/* high bandwidth iso tx? */
	unsigned		dyn_fifo:1;	/* dynamic FIFO supported? */

	unsigned		bulk_split:1;
#define	can_bulk_split(musb,type) \
	(((type) == USB_ENDPOINT_XFER_BULK) && (musb)->bulk_split)

	unsigned		bulk_combine:1;
#define	can_bulk_combine(musb,type) \
	(((type) == USB_ENDPOINT_XFER_BULK) && (musb)->bulk_combine)

	/*
	 * FIXME: Remove this flag.
	 *
	 * This is only added to allow Blackfin to work
	 * with current driver. For some unknown reason
	 * Blackfin doesn't work with double buffering
	 * and that's enabled by default.
	 *
	 * We added this flag to forcefully disable double
	 * buffering until we get it working.
	 */
	unsigned                double_buffer_not_ok:1;

	struct musbfsh_hdrc_config	*config;

};

static inline void musbfsh_configure_ep0(struct musbfsh *musbfsh)
{
	musbfsh->endpoints[0].max_packet_sz_tx = MUSBFSH_EP0_FIFOSIZE;
	musbfsh->endpoints[0].max_packet_sz_rx = MUSBFSH_EP0_FIFOSIZE;
	musbfsh->endpoints[0].is_shared_fifo = true;
}

/***************************** Glue it together *****************************/

extern const char musbfsh_driver_name[];

extern void musbfsh_start(struct musbfsh *musbfsh);
extern void musbfsh_stop(struct musbfsh *musbfsh);
extern int musbfsh_get_id(struct device *dev, gfp_t gfp_mask);
extern void musbfsh_put_id(struct device *dev, int id);

extern void musbfsh_write_fifo(struct musbfsh_hw_ep *ep, u16 len, const u8 *src);
extern void musbfsh_read_fifo(struct musbfsh_hw_ep *ep, u16 len, u8 *dst);

extern void musbfsh_load_testpacket(struct musbfsh *);

extern irqreturn_t musbfsh_interrupt(struct musbfsh *);

static inline void musbfsh_platform_set_power(struct musbfsh *musbfsh, int action)
{
	if (musbfsh->ops->set_power)
		musbfsh->ops->set_power(musbfsh, action);
}

static inline void musbfsh_platform_set_vbus(struct musbfsh *musbfsh, int is_on)
{
	if (musbfsh->ops->set_vbus)
		musbfsh->ops->set_vbus(musbfsh, is_on);
}

/* to conform original api interface */
static inline void musbfsh_set_vbus(struct musbfsh *musbfsh, int is_on)
{
	musbfsh_platform_set_vbus(musbfsh, is_on);
}

static inline void musbfsh_platform_enable(struct musbfsh *musbfsh)
{
	if (musbfsh->ops->enable)
		musbfsh->ops->enable(musbfsh);
}

static inline void musbfsh_platform_disable(struct musbfsh *musbfsh)
{
	if (musbfsh->ops->disable)
		musbfsh->ops->disable(musbfsh);
}

static inline int musbfsh_platform_set_mode(struct musbfsh *musbfsh, u8 mode)
{
	if (!musbfsh->ops->set_mode)
		return 0;

	return musbfsh->ops->set_mode(musbfsh, mode);
}

static inline void musbfsh_platform_try_idle(struct musbfsh *musbfsh,
		unsigned long timeout)
{
	if (musbfsh->ops->try_idle)
		musbfsh->ops->try_idle(musbfsh, timeout);
}

static inline int musbfsh_platform_get_vbus_status(struct musbfsh *musbfsh)
{
	if (!musbfsh->ops->vbus_status)
		return 0;

	return musbfsh->ops->vbus_status(musbfsh);
}

static inline int musbfsh_platform_init(struct musbfsh *musbfsh)
{
	if (!musbfsh->ops->init)
		return -EINVAL;

	return musbfsh->ops->init(musbfsh);
}

static inline int musbfsh_platform_exit(struct musbfsh *musbfsh)
{
	if (!musbfsh->ops->exit)
		return -EINVAL;

	return musbfsh->ops->exit(musbfsh);
}

#endif	/* __MUSBFSH_CORE_H__ */
