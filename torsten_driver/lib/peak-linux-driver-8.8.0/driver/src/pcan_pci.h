/* SPDX-License-Identifier: GPL-2.0 */
/*****************************************************************************
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
 *****************************************************************************/

/*****************************************************************************
 *
 * all parts to handle device interface specific for pcan-pci
 *
 * $Id$
 *
 *****************************************************************************/

#ifndef __PCAN_PCI_H__
#define __PCAN_PCI_H__

#include "src/pcan_main.h"

#if 1 /* def PCIEC_SUPPORT */
/* History: in previous versions of pcan, supporting or not PCIEC was leading
 * to two ways of registering PCI devices...
 * From v8.0, PCI CAN devices creation is always event driven... */
#if 1//def NO_RT
/* v7.x RT: PCIEC_SUPPORT was NOT supported, thus PCAN_PCI_EVENT_DRIVEN was'nt
 * defined... */
#define PCAN_PCI_EVENT_DRIVEN
#endif
#endif

#ifdef CONFIG_PCI_MSI
/* If defined, driver will first try to enable MSI mode with the device. On
 * any error, it will fall back into normal INTx mode.
 * If not defined, normal INTx mode will be used, as usual.  */
#define PCAN_PCI_ENABLE_MSI
//#define PCAN_PCI_ENABLE_MSIX

#ifdef PCAN_PCI_ENABLE_MSI
/* if defined, MSI can be shared. */
#define PCAN_PCI_SHARE_MSI
//#define DEBUG_MSI

#ifdef CONFIG_X86
/* workaround for using MSI with x86 archs and to prevent from not having any
 * interrupt any more from the board, when can0 is closed before the other canX:
 * IRQ is requested at _probe() time and is released at _exit */
#define PCAN_PCI_MSI_WORKAROUND	
#endif

#define PCAN_PCI_USEMSI_INTA		0
#define PCAN_PCI_USEMSI_NOTSHARED	1	/* normal MSI mode */
#define PCAN_PCI_USEMSI_SHARED		2	/* dangerous, might loss INT */
#define PCAN_PCI_USEMSI_DEFAULT		PCAN_PCI_USEMSI_INTA

#endif
#endif /* CONFIG_PCI_MSI */

struct pcan_pci_adapter {
	struct pcan_adapter	adapter;
	struct pci_dev *	dev;
	void __iomem *		bar0_addr;
	int			msi_count;
	int			msi_step;
	struct pcandev *	pci_devs[0];
};

#define pci_adapter_dev(p, i)	((p)->pci_devs[i])

int pcan_pci_enable_msi(struct pcan_pci_adapter *pa, int can_count,
				int irq_min);

void pcan_pci_disable_msi(struct pci_dev *pciDev);

#ifdef PCAN_PCI_EVENT_DRIVEN
int pcan_pci_init(void);
void pcan_pci_deinit(void);
#else
int pcan_search_and_create_pci_devices(void);
#endif

#endif
