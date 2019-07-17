// SPDX-License-Identifier: GPL-2.0
/****************************************************************************ble
 * Copyright (C) 2001-2010  PEAK System-Technik GmbH
 *
 * linux@peak-system.com
 * www.peak-system.com
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
 *
 * Major contributions by:
 *                Edouard Tisserant (edouard.tisserant@lolitech.fr) XENOMAI
 *                Laurent Bessard   (laurent.bessard@lolitech.fr)   XENOMAI
 *                Oliver Hartkopp   (oliver.hartkopp@volkswagen.de) socketCAN
 *                Klaus Hitschler   (klaus.hitschler@gmx.de)
 *
 * Contributions: Philipp Baer (philipp.baer@informatik.uni-ulm.de)
 *                Armin Bauer (armin.bauer@desscon.com)
 ****************************************************************************/

/****************************************************************************
 *
 * all parts to handle the interface specific parts of pcan-pci
 *
 * $Id$
 *
 ****************************************************************************/
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"	/* must always be the 1st include */

#include <linux/ioport.h>
#include <linux/pci.h>		/* all about pci */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "src/pcan_pci.h"
#include "src/pcan_sja1000.h"
#include "src/pcan_filter.h"

#ifdef PCIEC_SUPPORT
#include "src/pcan_pciec.h"
#endif

/* Since PCAN_PCI_ENABLE_MSI is defined in pcan_pci.h, undefining
 * PCAN_PCI_ENABLE_MSI here enables to exclusively work with INTA
 * for the boards handled by this driver */
//#undef PCAN_PCI_ENABLE_MSI

/* important PITA registers */
#define PITA_ICR         0x00        /* interrupt control register */
#define PITA_GPIOICR     0x18        /* GPIO interface control register */
#define PITA_MISC        0x1C        /* miscellaneous register */

#define PEAK_PCI_VENDOR_ID	0x001C	/* the PCI device and vendor IDs */

#define PCAN_PCI_ID		0x0001  /* PCI / PCIe Slot cards */
#define PCAN_EXPRESSCARD_ID	0x0002	/* PCAN-ExpressCard */
#define PCAN_PCIE_ID		0x0003	/* PCIe Slot cards */
#define PCAN_CPCI_ID		0x0004	/* cPCI */
#define PCAN_MINIPCI_ID		0x0005	/* miniPCI */
#define PCAN_PC104PLUSQUAD_ID	0x0006	/* new PC-104 Plus Quad */
#define PCAN_PCI104E_ID		0x0007	/* PCI-104 Express */
#define PCAN_MINIPCIE_ID	0x0008	/* miniPCIe Slot cards */
#define PCAN_CHIPPCIE_ID	0x0009	/* PCAN-Chip PCIe */
					/* (former PCAN-PCI Express OEM) */
#define PCAN_EXPRESSCARD34_ID	0x000a	/* PCAN-Express Card 34 */

/* CAN-FD devices id. range start */
#define PEAK_PCICANFD_ID	0x0010

#define PCAN_PCIEFD10_ID	(PEAK_PCICANFD_ID + 0)
#define PCAN_PCIEFD_ID		(PEAK_PCICANFD_ID + PCAN_PCIE_ID)
#define PCAN_CPCIEFD_ID		(PEAK_PCICANFD_ID + PCAN_CPCI_ID)
#define PCAN_PCI104EFD_ID	(PEAK_PCICANFD_ID + PCAN_PCI104E_ID)
#define PCAN_MINIPCIEFD_ID	(PEAK_PCICANFD_ID + PCAN_MINIPCIE_ID)
#define PCAN_CHIPPCIEFD_ID	(PEAK_PCICANFD_ID + PCAN_CHIPPCIE_ID)
#define PCAN_M2_ID		0x001a

#define PCI_CONFIG_PORT_SIZE	0x1000  /* size of the config io-memory */
#define PCI_PORT_SIZE		0x0400  /* size of a channel io-memory */

#ifdef LINUX_26
#define pci_find_device(v, d, x) pci_get_device(v, d, x)
#endif

#define VERSION_REG1		0x40
#define VERSION_REG2		0x44
#define VERSION_REG2_MASK	0xfff
#define VERSION_REG2_MSI	0x110

/*
 * GLOBALS
 */
static const struct pci_device_id pcan_pci_tbl[] = {
	{PEAK_PCI_VENDOR_ID, PCAN_PCI_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_PCIE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_CPCI_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_MINIPCI_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_PC104PLUSQUAD_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_PCI104E_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_MINIPCIE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
#ifdef PCIEC_SUPPORT
	{PEAK_PCI_VENDOR_ID, PCAN_EXPRESSCARD_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{PEAK_PCI_VENDOR_ID, PCAN_EXPRESSCARD34_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},
#endif
	{PEAK_PCI_VENDOR_ID, PCAN_CHIPPCIE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0},

	/* CAN-FD devices */
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_PCIEFD10_ID) },
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_PCIEFD_ID) },
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_CPCIEFD_ID) },
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_PCI104EFD_ID) },
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_MINIPCIEFD_ID) },
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_CHIPPCIEFD_ID) },
	{ PCI_DEVICE(PEAK_PCI_VENDOR_ID, PCAN_M2_ID) },

	{0, }
};

MODULE_DEVICE_TABLE(pci, pcan_pci_tbl);

#ifdef PCAN_PCI_ENABLE_MSI
static uint usemsi = PCAN_PCI_USEMSI_DEFAULT;
module_param(usemsi, uint, 0644);
MODULE_PARM_DESC(usemsi, " 0=INTA; 1=MSI (not shared); 2=MSI (shared) (def="
			__stringify(PCAN_PCI_USEMSI_DEFAULT) ")");
#endif

static const char *pcan_pci_adapter_name[] = {
	[PCAN_PCI_ID] = "PCAN-PCI",
	[PCAN_PCIE_ID] = "PCAN-PCI Express",
	[PCAN_CPCI_ID] = "PCAN-cPCI",
	[PCAN_MINIPCI_ID] = "PCAN-miniPCI",
	[PCAN_PC104PLUSQUAD_ID] = "PCAN-PC/104-Plus Quad",
	[PCAN_PCI104E_ID] = "PCAN-PCI/104-Express",
	[PCAN_MINIPCIE_ID] = "PCAN-miniPCIe",
#ifdef PCIEC_SUPPORT
	[PCAN_EXPRESSCARD_ID] = "PCAN-ExpressCard",
	[PCAN_EXPRESSCARD34_ID] = "PCAN-ExpressCard 34",
#endif
	[PCAN_CHIPPCIE_ID] = "PCAN-Chip PCIe",

	/* CAN-FD devices */
	[PCAN_PCIEFD10_ID] = "PCAN-PCIe FD (proto)",
	[PCAN_PCIEFD_ID] = "PCAN-PCIe FD",
	[PCAN_CPCIEFD_ID] = "PCAN-cPCIe FD",
	[PCAN_PCI104EFD_ID] = "PCAN-PCI/104-Express FD",
	[PCAN_MINIPCIEFD_ID] = "PCAN-miniPCIe FD",
	[PCAN_CHIPPCIEFD_ID] = "PCAN-Chip PCIe FD",
	[PCAN_M2_ID] = "PCAN-M.2",
};

/* non-SJA1000 PCI devices probing is external */
extern void pcan_pcifd_remove(struct pci_dev *dev);
extern int pcan_pcifd_probe(struct pci_dev *dev, u16 sub_system_id,
			       const char *adapter_name, int can_count);

/* ugly (but historical) global count of ALL the pci channel devices */
int _pci_devices = 0;

/* count of SJA1000 PCI devices */
static int pcan_pci_sja1000_adapters = 0;

/* count of SJA1000 channels devices */
static int pcan_pci_sja1000_devices = 0;

static void pcan_pci_unregister_driver(struct pci_driver *p_pci_drv);

/* read a register */
static u8 pcan_pci_readreg(struct pcandev *dev, u8 port)
{
	u32 lPort = (u32 )port << 2;
	return readb(dev->port.pci.can_port_addr + lPort);
}

/* write a register */
static void pcan_pci_writereg(struct pcandev *dev, u8 port, u8 data)
{
	u32 lPort = (u32 )port << 2;
	writeb(data, dev->port.pci.can_port_addr + lPort);
}

static const u16 pita_icr_masks[] = { 0x0002, 0x0001, 0x0040, 0x0080 };

/* enable interrupt in PITA */
static void pcan_pci_enable_pita_interrupt(struct pcandev *dev)
{
	u16 pita_icr_high = readw(dev->port.pci.bar0_cfg_addr + PITA_ICR + 2);

	DPRINTK(KERN_DEBUG "%s: %s(%u): PITA ICR=%04Xh\n",
			DEVICE_NAME, __func__, dev->nMinor, pita_icr_high);

	pita_icr_high |= pita_icr_masks[dev->nChannel];
	writew(pita_icr_high, dev->port.pci.bar0_cfg_addr + PITA_ICR + 2);
}

/* disable interrupt in PITA */
static void pcan_pci_disable_pita_interrupt(struct pcandev *dev)
{
	u16 pita_icr_high = readw(dev->port.pci.bar0_cfg_addr + PITA_ICR + 2);

	DPRINTK(KERN_DEBUG "%s: %s(%u): PITA ICR=%04Xh\n",
			DEVICE_NAME, __func__, dev->nMinor, pita_icr_high);

	pita_icr_high &= ~pita_icr_masks[dev->nChannel];
	writew(pita_icr_high, dev->port.pci.bar0_cfg_addr + PITA_ICR + 2);

	/* read it again, to wait for write command to complete */
	readw(dev->port.pci.bar0_cfg_addr + PITA_ICR + 2);
}

/* interface depended open and close */
static int pcan_pci_open(struct pcandev *dev)
{
#ifdef DEBUG
	pr_info("%s: %s()\n", DEVICE_NAME, __func__);
#endif
	dev->ucActivityState = ACTIVITY_IDLE;
	return 0;
}

/*
 *  Second callback called by close().
 */
static int pcan_pci_release(struct pcandev *dev)
{
#ifdef DEBUG
	pr_info("%s: %s()\n", DEVICE_NAME, __func__);
#endif

	dev->ucActivityState = ACTIVITY_INITIALIZED;
	return 0;
}

/*
 * Third (and last) callback called by close().
 */
static void pcan_pci_free_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	DPRINTK(KERN_DEBUG "%s: %s(%u)\n", DEVICE_NAME, __func__, dev->nMinor);

	/* disable interrupt in PITA */
	pcan_pci_disable_pita_interrupt(dev);

#ifdef PCAN_PCI_MSI_WORKAROUND
	if (usemsi == PCAN_PCI_USEMSI_INTA)
#endif
		pcan_free_irq(dev);

	dev->wInitStep = 5;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
#define pcan_pci_enable_msi_range(a, b, c)	\
	pci_alloc_irq_vectors_affinity(a, b, c, PCI_IRQ_MSI, NULL)

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
#define pcan_pci_enable_msi_range(a, b, c)	\
	pci_alloc_irq_vectors(a, b, c, PCI_IRQ_MSI)

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#define pcan_pci_enable_msi_range(a, b, c)	pci_enable_msi_range(a, b, c)

#else
/**
 * pci_enable_msi_range - configure device's MSI capability structure
 * @dev: device to configure
 * @minvec: minimal number of interrupts to configure
 * @maxvec: maximum number of interrupts to configure
 *
 * This function tries to allocate a maximum possible number of interrupts in a
 * range between @minvec and @maxvec. It returns a negative errno if an error
 * occurs. If it succeeds, it returns the actual number of interrupts allocated
 * and updates the @dev's irq member to the lowest new interrupt number;
 * the other interrupt numbers allocated to this device are consecutive.
 **/
static int pcan_pci_enable_msi_range(struct pci_dev *dev,
					int minvec, int maxvec)
{
	int nvec = maxvec;
	int rc;

	if (maxvec < minvec)
		return -ERANGE;

	do {
		rc = pci_enable_msi_block(dev, nvec);
		if (rc < 0) {
			return rc;
		} else if (rc > 0) {
			if (rc < minvec)
				return -ENOSPC;
			nvec = rc;
		}
	} while (rc);

	return nvec;
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0) */

void pcan_pci_disable_msi(struct pci_dev *pciDev)
{
#ifdef PCAN_PCI_ENABLE_MSI
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
	pci_free_irq_vectors(pciDev);
#else
	pci_disable_msi(pciDev);
#endif
#endif
}

int pcan_pci_enable_msi(struct pcan_pci_adapter *pa, int can_count, int irq_max)
{
	u16 msgctl;
	int maxvec, pos;
	int msi_max = irq_max;
	//int msi_max = can_count;

	pa->msi_count = 0;

	/* first of all, read board MSI capacity */
	pos = pci_find_capability(pa->dev, PCI_CAP_ID_MSI);
	if (!pos) {
		pr_warn(DEVICE_NAME
			": pci_find_capability() failure\n");
		return -EIO;
	}

	/* read real MSI maximum of vectors available */
	pci_read_config_word(pa->dev, pos + PCI_MSI_FLAGS, &msgctl);
	maxvec = 1 << ((msgctl & PCI_MSI_FLAGS_QMASK) >> 1);

#ifdef DEBUG_MSI
	pr_info(DEVICE_NAME
		": MSI flags=%04xh => maxvec=%d vs. msi_max=%d\n",
		msgctl, maxvec, msi_max);
#endif

	/* adjust maximum requested by driver against maximum proposed by
	 * card: always trust the card since 2x CAN PCIe FD might propose
	 * 4 MSI levels! Do this adjustment *ONLY* if MSI shouldn't be
	 * shared. */
	if ((msi_max > 1) && (maxvec > msi_max))
		msi_max = maxvec;

	/* enable MSI for the PCI device */

	/* in some PCIe arch and/or Kernel version:
	 * looks like with x86 APIC, Linux Kernel won't give us anything else
	 * than "1" (see "native_setup_msi_irqs()" in
	 * arch/x86/kernel/apic/io_apic.c */
	pa->msi_count = pcan_pci_enable_msi_range(pa->dev, 1, msi_max);

#ifdef DEBUG_MSI
	pr_info(DEVICE_NAME ": enabling [1..%u] MSI status: %d\n",
		msi_max, pa->msi_count);
#endif
	if ((pa->msi_count <= 0) || (pa->msi_count < msi_max)) {

		/* fallback to INTA mode if we can't allocate the requested
		 * number of IRQs */
		if (pa->msi_count > 0)
			pcan_pci_disable_msi(pa->dev);

		else if (pa->msi_count < 0) {
			pr_err(DEVICE_NAME
				": enabling MSI failed (err %d)\n",
				pa->msi_count);

			return pa->msi_count;
		}

		pr_info(DEVICE_NAME
			": fallback into INTA mode IRQ%u (err %d)\n",
			pa->dev->irq, pa->msi_count);

		pa->msi_count = 0;

#ifdef DEBUG_MSI
	} else {
	
		pr_info(DEVICE_NAME ": %u MSI enabled from INT%u\n",
			pa->msi_count, pa->dev->irq);
#endif
	}

	return 0;
}

#ifndef NO_RT
/* RT version of the IRQ handler */
static int pcan_pci_irqhandler(rtdm_irq_t *irq_context)
{
	struct pcandev *dev = rtdm_irq_get_arg(irq_context, struct pcandev);

#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t pcan_pci_irqhandler(int irq, void *arg, struct pt_regs *regs)
{
	struct pcandev *dev = (struct pcandev *)arg;
#else
static irqreturn_t pcan_pci_irqhandler(int irq, void *arg)
{
	struct pcandev *dev = (struct pcandev *)arg;
#endif
	const u16 pita_icr_mask = pita_icr_masks[dev->nChannel];
	irqreturn_t err = PCAN_IRQ_NONE;
#if 1
	/* check whether this INT is for the given 'dev' */
	u16 pita_icr = readw(dev->port.pci.bar0_cfg_addr + PITA_ICR);
	while (pita_icr & pita_icr_mask) {

		/* whatever it was a SJA1000 INT or not, this PITA INT has been
		 * handled (this avoids INTA "irq nobody care" issue) */
		err = PCAN_IRQ_HANDLED;

		/* handle INT into the SJA1000 */
		if (pcan_sja1000_irqhandler(dev) == PCAN_IRQ_NONE) {

			/* clear corresponding INTerrupt in PITA */
			writew(pita_icr_mask,
					dev->port.pci.bar0_cfg_addr + PITA_ICR);

			/* and exit (this) ISR */
			break;
		}

		pita_icr = readw(dev->port.pci.bar0_cfg_addr + PITA_ICR);
	}
#else

static const u16 pita_icr_b_masks[] = { 0x02, 0x01, 0x40, 0x80 };

	const u8 pita_icr_b_mask = pita_icr_b_masks[dev->nChannel];
	u8 pita_icr = readb(dev->port.pci.bar0_cfg_addr + PITA_ICR);

	/* clear corresponding INTerrupt in PITA */
	while (pita_icr & pita_icr_b_mask) {

		writeb(pita_icr_b_mask, dev->port.pci.bar0_cfg_addr + PITA_ICR);

		while (pcan_sja1000_irqhandler(dev) != PCAN_IRQ_NONE);

		err = PCAN_IRQ_HANDLED;
		pita_icr = readb(dev->port.pci.bar0_cfg_addr + PITA_ICR);
	}
#endif

	return err;
}

static int __pcan_pci_req_irq(struct pcandev *dev)
{
	int err, irq_flags = PCAN_IRQF_SHARED;

#ifdef PCAN_PCI_ENABLE_MSI
	/* if driver got the requested count of MSI, IRQ is not shared */
	if (pci_dev_msi_enabled(dev->port.pci.pciDev))
		if (!(dev->flags & PCAN_DEV_MSI_SHARED))
			irq_flags &= ~PCAN_IRQF_SHARED;
#endif /* PCAN_PCI_ENABLE_MSI */

#ifndef NO_RT
	/* RT irq requesting */
	err = rtdm_irq_request(&dev->irq_handle,
			dev->wIrq,
			pcan_pci_irqhandler,
			irq_flags | RTDM_IRQTYPE_EDGE,
#else
	/* using legacy interrupt mechanism */
	err = request_irq(dev->wIrq,
			pcan_pci_irqhandler,
			irq_flags,
#endif
			DEVICE_NAME,
			dev);

	if (err)
		pr_err(DEVICE_NAME ": %s(CAN%u): failed to request irq %d "
			"flags=%0xh (err %d)\n",
			dev->adapter->name, dev->nChannel+1,
			dev->wIrq, irq_flags, err);

	return err;
}

static int pcan_pci_req_irq(struct pcandev *dev, struct pcan_udata *dev_priv)
{
	int err = 0;

#ifdef PCAN_PCI_MSI_WORKAROUND
	if (usemsi == PCAN_PCI_USEMSI_INTA)
#endif
		if ((err = __pcan_pci_req_irq(dev)))
			return err;

	pcan_pci_enable_pita_interrupt(dev);

	dev->wInitStep = 6;

	return err;
}

static int pcan_pci_cleanup(struct pcandev *dev)
{
	if (pcan_pci_sja1000_devices <= 0) {
		pr_info(DEVICE_NAME
			": %s(): ABNORMAL pcan_pci_sja1000_devices=%d! "
			"Nothing done next (pcan_pci_sja1000_adapters=%d)\n",
			__func__, pcan_pci_sja1000_devices,
			pcan_pci_sja1000_adapters);
		return 0;
	}

	DPRINTK(KERN_DEBUG
		"%s: %s(%s CAN%u): "
		"_pci_devices=%d pcan_pci_sja1000_adapters=%d "
		"sja100 devices=%d wInitStep=%u\n",
		DEVICE_NAME, __func__, dev->adapter->name, dev->nChannel+1,
		_pci_devices,
		pcan_pci_sja1000_adapters, pcan_pci_sja1000_devices,
		dev->wInitStep);

	switch (dev->wInitStep) {
	case 6:
		pcan_pci_free_irq(dev, NULL);
	case 5:
#ifdef PCAN_PCI_MSI_WORKAROUND
		if (usemsi != PCAN_PCI_USEMSI_INTA)
			pcan_free_irq(dev);
#endif
#ifdef PCIEC_SUPPORT
		pcan_pciec_delete_card(dev);
#endif
	case 4:
		iounmap(dev->port.pci.can_port_addr);
	case 3:
		release_mem_region(dev->dwPort, PCI_PORT_SIZE);
	case 2:
		/* SGr note: DON'T call  pcan_remove_dev_from_list(dev) here
		 * because it uses the same mutex than
		 * pcan_pci_sja1000_remove() */
		if (!dev->nChannel)
			iounmap(dev->port.pci.bar0_cfg_addr);
	case 1:
		if (!dev->nChannel)
			release_mem_region(dev->port.pci.dwConfigPort,
							PCI_CONFIG_PORT_SIZE);
	case 0:
		dev->filter = pcan_delete_filter_chain(dev->filter);

		_pci_devices--;
		pcan_pci_sja1000_devices--;

#ifdef UDEV_SUPPORT
#ifndef PCAN_PCI_EVENT_DRIVEN
		if (!pcan_pci_sja1000_devices)
			pcan_pci_unregister_driver(&pcan_drv.pci_drv);
#endif
#endif
	}

	return 0;
}

static int pcan_pci_channel_init(struct pcan_pci_adapter *adapter,
				 struct pcandev *dev)
{
	struct pci_dev *pciDev = adapter->dev; /* shortcut */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,0,0)
	int err;
#endif

	DPRINTK(KERN_DEBUG "%s: %s(), _pci_devices = %d\n",
		DEVICE_NAME, __func__, _pci_devices);

	/* obsolete - will be removed soon */
	if (dev->nChannel)
		dev->props.ucMasterDevice = CHANNEL_SLAVE;
	else
		dev->props.ucMasterDevice = CHANNEL_MASTER;

	/* set this before any instructions */
	dev->wInitStep = 2;
	dev->readreg = pcan_pci_readreg;
	dev->writereg = pcan_pci_writereg;
	dev->cleanup = pcan_pci_cleanup;
	dev->req_irq = pcan_pci_req_irq;
	dev->free_irq = pcan_pci_free_irq;
	dev->open = pcan_pci_open;
	dev->release = pcan_pci_release;
	dev->nMajor = pcan_drv.nMajor;
	dev->nMinor = PCAN_PCI_MINOR_BASE + _pci_devices;
	dev->filter = pcan_create_filter_chain();

	dev->port.pci.dwConfigPort = (u32 )pciDev->resource[0].start;
	dev->dwPort = (u32 )pciDev->resource[1].start + dev->nChannel *
								PCI_PORT_SIZE;

	dev->port.pci.bar0_cfg_addr = adapter->bar0_addr;
	dev->wIrq = (u16 )pciDev->irq;

	/* adjust MSI/INTA irq from adapter device IRQ value */
	dev->flags &= ~PCAN_DEV_MSI_SHARED;

#ifdef PCAN_PCI_ENABLE_MSI
	if (pci_dev_msi_enabled(adapter->dev)) {

		if (usemsi != PCAN_PCI_USEMSI_SHARED)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
			dev->wIrq = pci_irq_vector(pciDev, dev->nChannel);
#else
			dev->wIrq += dev->nChannel;
#endif

		else
			dev->flags |= PCAN_DEV_MSI_SHARED;
	}
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,0,0)
	err = check_mem_region(dev->dwPort, PCI_PORT_SIZE);
	if (err) {
		pr_info(DEVICE_NAME
			": %s(%u) check_mem_region(%d, %d) err=%d\n",
			__func__, __LINE__, dev->dwPort, PCI_PORT_SIZE, err);

		return err;
	}
#endif

	if (!request_mem_region(dev->dwPort, PCI_PORT_SIZE, DEVICE_NAME)) {
		pr_info(DEVICE_NAME
			": %s(%u) request_mem_region(%d, %d) failed\n",
			__func__, __LINE__, dev->dwPort, PCI_PORT_SIZE);
		dev->wInitStep--;
		return -EBUSY;
	}

	dev->wInitStep = 3;

	dev->port.pci.can_port_addr = ioremap(dev->dwPort, PCI_PORT_SIZE);
	if (!dev->port.pci.can_port_addr) {
		pr_info(DEVICE_NAME
			": %s(%u) ioremap(%d, %d) failure\n",
			__func__, __LINE__, dev->dwPort, PCI_PORT_SIZE);
		release_mem_region(dev->dwPort, PCI_PORT_SIZE);
		dev->wInitStep--;
		return -ENODEV;
	}

	dev->wInitStep = 4;

	_pci_devices++;
	dev->wInitStep = 5;

	/* double link the device and the adapter */
	pcan_set_dev_adapter(dev, &adapter->adapter);
	pci_adapter_dev(adapter, dev->nChannel) = dev;

	pr_info(DEVICE_NAME ": pci device minor %d found\n", dev->nMinor);

	return 0;
}

/*
 * create one pci based devices from peak
 * - this may be one of multiple from a card
 */
static int create_one_pci_device(struct pcan_pci_adapter *adapter, int nChannel)
{
	struct pci_dev *pciDev = adapter->dev; /* shortcut */
	struct pcandev *local_dev;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s(nChannel=%d)\n",
		DEVICE_NAME, __func__, nChannel);

	/* make the first device on board */
	local_dev = pcan_alloc_dev("pci", HW_PCI, nChannel);
	if (!local_dev) {
		err = -ENOMEM;
		goto fail;
	}

	pcan_soft_init(local_dev);

	local_dev->device_open = sja1000_open;
	local_dev->device_write = sja1000_write;
	local_dev->device_release = sja1000_release;

	local_dev->props.ucExternalClock = 1;

#ifdef PCIEC_SUPPORT
	/* card pointer must be NULL for all but PCAN-Expresscard */
	local_dev->port.pci.card = NULL;
#endif

	err = pcan_pci_channel_init(adapter, local_dev);
	if (!err)
		err = sja1000_probe(local_dev);

	if (err) {
#ifndef PCIEC_SUPPORT
		/* Thanks Hardi! */
		local_dev->cleanup(local_dev);
#endif
		goto fail_free;
	}

	local_dev->port.pci.pciDev = pciDev;

#ifdef PCIEC_SUPPORT
	/* we have a card with i2c controlled blinking LED */
	if ((pciDev->device == PCAN_EXPRESSCARD_ID) ||
		(pciDev->device == PCAN_EXPRESSCARD34_ID)) {

		/* master channel */
		if (!local_dev->nChannel)
			local_dev->port.pci.card =
				pcan_pciec_create_card(pciDev, local_dev);
		else
			local_dev->port.pci.card =
				pcan_pciec_locate_card(pciDev, local_dev);
	}
#endif

	/* add this device to the list */
	pcan_add_dev_in_list(local_dev);

#ifdef PCAN_PCI_MSI_WORKAROUND
	if (usemsi != PCAN_PCI_USEMSI_INTA)
		__pcan_pci_req_irq(local_dev);
#endif

	return 0;

fail_free:
	pcan_free_dev(local_dev);
fail:
	pr_err(DEVICE_NAME ": %s(CAN%u) discarded: err %d\n",
		__func__, nChannel+1, err);

	return err;
}

/* move to event driven creation of devices, not for kernels 2.4.x */
static int pcan_pci_sja1000_probe(struct pci_dev *pciDev, u16 wSubSysID,
				  const char *adapter_name, int can_count)
{
	struct pcan_pci_adapter *adapter;
	int err, i;
	u32 v1;

	DPRINTK(KERN_DEBUG "%s: %s(%p)\n", DEVICE_NAME, __func__, pciDev);

#ifdef DEBUG
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		if (pciDev->resource[i].name)
			pr_info(DEVICE_NAME
				": resource[%d]: name=\"%s\" start=%d (%xh) "
				"end=%d (%xh) flags=%08xh\n",
				i,
				pciDev->resource[i].name,
				(int )pciDev->resource[i].start,
				(int )pciDev->resource[i].start,
				(int )pciDev->resource[i].end,
				(int )pciDev->resource[i].end,
				(int )pciDev->resource[i].flags);
	}
#endif

	/* configure the PCI chip, part 1 */
	err = pci_write_config_word(pciDev, PCI_COMMAND, 2);
	if (err)
		goto fail;

	err = pci_write_config_word(pciDev, 0x44, 0);
	if (err)
		goto fail;
	wmb();

#ifdef PCAN_USB_ALLOC_DEV
	adapter = pcan_alloc_adapter_ex(adapter_name,
					pcan_pci_sja1000_adapters,
					can_count, sizeof(*adapter));
#else
	adapter = kzalloc(sizeof(*adapter), GFP_KERNEL);
	pcan_init_adapter(&adapter->adapter,
			adapter_name, pcan_pci_sja1000_adapters, can_count);
#endif
	if (!adapter)
		goto fail;

	adapter->dev = pciDev;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,0,0)
	err = check_mem_region((u32 )pciDev->resource[0].start,
					PCI_CONFIG_PORT_SIZE);
	if (err) {
		pr_info(DEVICE_NAME ": %s() check_mem_region(%xh, %d) err=%d\n",
			__func__, (u32)pciDev->resource[0].start,
			PCI_CONFIG_PORT_SIZE, err);
		goto fail_free;
	}
#endif
	if (!request_mem_region((u32 )pciDev->resource[0].start,
					PCI_CONFIG_PORT_SIZE, DEVICE_NAME)) {
		pr_info(DEVICE_NAME
			": %s(%u) request_mem_region(%xh, %d) failure\n",
			__func__, __LINE__, (u32)pciDev->resource[0].start,
			PCI_CONFIG_PORT_SIZE);
		err = -EBUSY;
		goto fail_free;
	}

	adapter->bar0_addr = ioremap((u32 )pciDev->resource[0].start,
					PCI_CONFIG_PORT_SIZE);

	if (!adapter->bar0_addr) {
		pr_info(DEVICE_NAME ": %s() ioremap(%xh, %d) failure\n",
			__func__, (u32 )pciDev->resource[0].start,
			PCI_CONFIG_PORT_SIZE);
		err = -EIO;
		goto fail_release_regions;
	}

	/* configuration of the PCI chip, part 2: */

	/* set GPIO control register */
	writew(0x0005, adapter->bar0_addr + PITA_GPIOICR + 2);

	/* enable all channels */
	writeb(0x00, adapter->bar0_addr + PITA_GPIOICR);

	/* toggle reset */
	writeb(0x05, adapter->bar0_addr + PITA_MISC + 3);
	mdelay(5);

	/* leave parport mux mode */
	writeb(0x04, adapter->bar0_addr + PITA_MISC + 3);
	wmb();

	v1 = readl(adapter->bar0_addr + VERSION_REG1);
	if (v1) {

		/* this is an FPGA equipped board */
		u32 v2 = readl(adapter->bar0_addr + VERSION_REG2);

		adapter->adapter.hw_ver.major = (v2 & 0x0000f000) >> 12;
		adapter->adapter.hw_ver.minor = (v2 & 0x00000f00) >> 8;
		adapter->adapter.hw_ver.subminor = (v2 & 0x000000f0) >> 4;

#ifdef PCAN_PCI_ENABLE_MSI
		/* read MSI ability of the board */
		if (((v2 >> 4) & VERSION_REG2_MASK) >= VERSION_REG2_MSI) {

			switch (usemsi) {
			case PCAN_PCI_USEMSI_NOTSHARED:
				/* try 1 IRQ per CAN, donot accept anything
				 * else, that is, donot accept sharing MSI... */
				err = pcan_pci_enable_msi(adapter,
							can_count, can_count);
				break;
			case PCAN_PCI_USEMSI_SHARED:
				/* try MSI and accept to share INTerrupt(s) */
				err = pcan_pci_enable_msi(adapter,
							can_count, 1);
				break;
			default:
				err = -1;
				break;
			}

			/* change usemsi to control when IRQ will be
			 * requested/freed next */
			if (err)
				usemsi = PCAN_PCI_USEMSI_INTA;
		}
#endif
	}

	for (i = 0; i < can_count; i++) {
		err = create_one_pci_device(adapter, i);
		if (err)
			goto fail_all;
	}

	pcan_pci_sja1000_devices += can_count;
	pcan_pci_sja1000_adapters++;

	return 0;

fail_all:
	while (i-- > 0) {
		struct pcandev *dev = pci_adapter_dev(adapter, i);

		iounmap(dev->port.pci.can_port_addr);
		release_mem_region(dev->dwPort, PCI_PORT_SIZE);

		/* if device was in global devices list, then it has
		 * been initialized, then it can be destroyed */
		if (pcan_remove_dev_from_list(dev))
			pcan_destroy_dev(dev);
	}

	iounmap(adapter->bar0_addr);

fail_release_regions:
	release_mem_region((u32 )pciDev->resource[0].start,
				PCI_CONFIG_PORT_SIZE);
fail_free:
	for (i = 0; i < can_count; i++)
		pcan_free_dev(pci_adapter_dev(adapter, i));

	pcan_free_adapter(&adapter->adapter);

	pcan_pci_disable_msi(pciDev);
fail:
	return err;
}

static int pcan_pci_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	u16 sub_system_id;
	int err, can_count;

	DPRINTK(KERN_DEBUG "%s: %s(%p:id=%0xh)\n",
			DEVICE_NAME, __func__, dev, ent->device);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
	err = pci_enable_device(dev);
	if (err)
		goto fail;
#endif

	err = pci_read_config_word(dev, PCI_SUBSYSTEM_ID, &sub_system_id);
	if (err)
		goto fail_disable_pci;

	/* number of CAN channels depends on the sub-system id */
	if (sub_system_id < 0x0004)
		can_count = 1;
	else if (sub_system_id < 0x0010)
		can_count = 2;
	else if (sub_system_id < 0x0012)
		can_count = 3;
	else
		can_count = 4;

	/* consider that devid >= 0x10 => CAN-FD devices */
	if (ent->device >= PEAK_PCICANFD_ID) {
		err = pcan_pcifd_probe(dev, sub_system_id,
				pcan_pci_adapter_name[dev->device],
				can_count);
	} else {
		err = pcan_pci_sja1000_probe(dev, sub_system_id,
				pcan_pci_adapter_name[dev->device],
				can_count);
	}

	if (err)
		goto fail_disable_pci;

	return 0;

fail_disable_pci:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 10)
	pci_disable_device(dev);
fail:
#endif
	return err;
}

static int pcan_pci_register_driver(struct pci_driver *pci_drv)
{
#ifdef DEBUG
	pr_info("%s: %s(%p)\n", DEVICE_NAME, __func__, pci_drv);
#endif
	pci_drv->name = DEVICE_NAME;
	pci_drv->id_table = pcan_pci_tbl;

	return pci_register_driver(pci_drv);
}

static void pcan_pci_unregister_driver(struct pci_driver *pci_drv)
{
#ifdef DEBUG
	pr_info("%s: %s(%p)\n", DEVICE_NAME, __func__, pci_drv);
#endif
	pci_unregister_driver(pci_drv);
}

#ifdef PCAN_PCI_EVENT_DRIVEN

static void pcan_pci_sja1000_remove(struct pci_dev *pciDev)
{
	struct pcandev *dev;
	struct pcan_adapter *adapter = NULL;
	struct list_head *pos;
	struct list_head *n;

#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	pcan_lock_get_irqsave(&pcan_drv.devices_lock, flags);
#endif
	list_for_each_prev_safe(pos, n, &pcan_drv.devices) {
		dev = list_entry(pos, struct pcandev, list);
		if ((dev->wType == HW_PCI) &&
					(dev->port.pci.pciDev == pciDev)) {

			pcan_cleanup_dev(dev);
			pcan_destroy_dev(dev);

			list_del(&dev->list);
			pcan_drv.wDeviceCount--;

#if 1
			/* SGR Note: because of pcan_free(dev), is this really
			 * useful? */
#else
			/* TODO: a much better hack to address plugging out
			 * while a path to the device is open
			 */
			dev->is_plugged = 0;
#endif
			/* free all device allocated memory */
			if (!adapter)
				adapter = dev->adapter;

			pcan_free_dev(dev);
		}
	}

#ifdef HANDLE_HOTPLUG
	pcan_lock_put_irqrestore(&pcan_drv.devices_lock, flags);
#endif
	if (adapter)
		pcan_free_adapter(adapter);

	pcan_pci_disable_msi(pciDev);
}

static void pcan_pci_remove(struct pci_dev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s(%p:id=%0xh)\n",
			DEVICE_NAME, __func__, dev, dev->device);

	if (dev->device >= PEAK_PCICANFD_ID) {
		pcan_pcifd_remove(dev);
	} else {
		pcan_pci_sja1000_remove(dev);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
	pci_disable_device(dev);
#endif
}

int pcan_pci_init(void)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	pcan_drv.pci_drv.probe = pcan_pci_probe;
	pcan_drv.pci_drv.remove = pcan_pci_remove;

	return pcan_pci_register_driver(&pcan_drv.pci_drv);
}

void pcan_pci_deinit(void)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);
	pcan_pci_unregister_driver(&pcan_drv.pci_drv);
}

#else /* PCAN_PCI_EVENT_DRIVEN */

#ifdef LINUX_26
static inline int pci_present(void)
{
#ifdef CONFIG_PCI
	return 1;
#else
	return 0;
#endif
}
#endif

/* search all pci based devices from peak */
int pcan_search_and_create_pci_devices(void)
{
	const int n = sizeof(pcan_pci_tbl) / sizeof(pcan_pci_tbl[0]) - 1;
	int err = 0;
	int i;

	/* search pci devices */
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	if (!pci_present())
		return 0;

	/* for each device id... */
	for (i = 0; i < n; i++) {
		struct pci_dev *from = NULL;
		struct pci_dev *pciDev;

		/* ...loop looking for all the same adapters */
		do {
			pciDev = pci_find_device(pcan_pci_tbl[i].vendor,
						 pcan_pci_tbl[i].device,
						 from);

			/* no (more) device found with that device id.:
			 * break the current device loop to search for any
			 * other PEAK devices...  */
			if (!pciDev) {
				DPRINTK(KERN_DEBUG "%s: %s(): i=%d (%04x.%04x) "
					"pciDev=NULL from=%p\n",
					DEVICE_NAME, __func__, i,
					pcan_pci_tbl[i].vendor,
					pcan_pci_tbl[i].device,
					from);
				break;
			}

			/* a PCI device with PCAN_PCI_VENDOR_ID and
			 * PCAN_PCI_DEVICE_ID was found */
			from = pciDev;

			/* create all corresponding channel devices */
			err = pcan_pci_probe(pciDev, pcan_pci_tbl + i);

		} while (!err);
	}

	DPRINTK(KERN_DEBUG "%s: %s() status=%d\n", DEVICE_NAME, __func__, err);

#ifdef UDEV_SUPPORT
	/* register only if at least one SJA1000 channel has been found */
	if (pcan_pci_sja1000_devices > 0)
		pcan_pci_register_driver(&pcan_drv.pci_drv);
#endif

	return err;
}
#endif /* PCAN_PCI_EVENT_DRIVEN */
