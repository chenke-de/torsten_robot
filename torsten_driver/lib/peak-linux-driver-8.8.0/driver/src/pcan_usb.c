// SPDX-License-Identifier: GPL-2.0
/*****************************************************************************
 * Copyright (C) 2003-2010  PEAK System-Technik GmbH
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
 *                Klaus Hitschler (klaus.hitschler@gmx.de)
 *                Oliver Hartkopp (oliver.hartkopp@volkswagen.de) socketCAN
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_usb.c - the inner parts for pcan-usb support
 *
 * $Id$
 *
 * 2011-08-22 SGr
 * - pcan_usb_cleanup(NEW): add the sending of the bus off request in cleanup
 *   callback since this is the last place we may submit urb to the device
 *   before driver removal.
 *
 *****************************************************************************/
/*#define DEBUG*/
/*#undef DEBUG*/

#include "src/pcan_common.h"

#ifdef USB_SUPPORT

#include <linux/sched.h>
#include <linux/net.h>
#include "src/pcan_main.h"
#include "src/pcan_fifo.h"
#include "src/pcan_usb.h"
#include <asm/byteorder.h>

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"     /* for hotplug pcan_netdev_register() */
#endif

#ifdef DEBUG
#define DEBUG_DECODE
#define DEBUG_TIMESTAMP
#else
//#define DEBUG_DECODE
//#define DEBUG_TIMESTAMP
#endif

/* if defined, takes into account that DLC field may be > 8.
 * in any case, DLC > 8 means DLC = 8. It also means that applications HAVE TO
 * handle the fact that C struct object lenght field MAY be greater than 8 !!!
 * Should be defined, since PCAN-USB is able to generate such values !
 */
#define HANDLE_DLC_GT_THAN_8

/* if defined, then bus_state is set to ACTIVE when the driver receives the
 * very first timestamp sync from the PCAN-USB. This enables to flush the tx
 * fifo quicker.
 * if not defined, then bus_state is set to ACTIVE when the driver receives the
 * first STATUS message. This one can be received up to 1 s after! */
#define BUS_STATE_ACTIVE_ON_1ST_TS

/* if defined, then the PCAN-USB generates ECC errors at 1 Mbps and goes into
 * passive mode when receiving frames. This might help for tests.
 * MUST obviously  NOT be defined.
 */
//#define DO_GENERATE_ECC_ERROR_AT_1MBPS

// bit masks for status/length field in a USB message
#define STLN_WITH_TIMESTAMP	0x80
#define STLN_INTERNAL_DATA	0x40
#define STLN_EXTENDED_ID	0x20
#define STLN_RTR		0x10
#define STLN_DATA_LENGTH	0x0F         // mask for length of data bytes

// Error-Flags for PCAN-USB
#define XMT_BUFFER_FULL		0x01
#define CAN_RECEIVE_OVERRUN	0x02
#define BUS_LIGHT		0x04
#define BUS_HEAVY		0x08
#define BUS_OFF			0x10
#define QUEUE_RECEIVE_EMPTY	0x20
#define QUEUE_OVERRUN		0x40
#define QUEUE_XMT_FULL		0x80

/* timestamp calculation stuff:
 * converting ticks into us is done by scaling the number of ticks per us:
 * as one tick duration is 42.666 us, use 42666 us for 1000 ticks.
 * two methods are proposed here:
 * 1. fast: instead of div, use 2^20 shift method:
 *    (tick_number * 44739) >> 20 <~> (tick_number * 42666) / 1000000
 *    this gives same result with a 10^-7 precision
 * 2. accurate: use the 64-bits division
 *    (tick_number * 1024) / 24000
 */
//#define PCAN_USB_TS_DIV_SHIFTER          20  // shift faster than div
#ifdef PCAN_USB_TS_DIV_SHIFTER
#define PCAN_USB_TS_US_PER_TICK		44739243
#else
//#define PCAN_USB_TS_SCALE_MULTIPLIER	128
#define PCAN_USB_TS_SCALE_MULTIPLIER	42667
//#define PCAN_USB_TS_SCALE_DIVISOR	3
#define PCAN_USB_TS_SCALE_DIVISOR	1000
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define TICKS(msec)			((msec * HZ) / 1000)
#endif

#define COMMAND_TIMEOUT			500  /* timeout for EP0 control urbs */

/* USB Mass Storage Mode command (FW >= 8.3.0) */
#define	PCAN_USB_SETCAN2FLASH		0xC8

/* pcan-usb parameter get an set function */
typedef struct {
	u8  Function;
	u8  Number;
	u8  Param[14];
} __attribute__ ((packed)) PCAN_USB_PARAM;

static int pcan_usb_devices = 0;

/* functions to handle time construction */
static void pcan_reset_timestamp(struct pcandev *dev)
{
	struct pcan_usb_time *t = &dev->port.usb.time;

	memset(t, '\0', sizeof(struct pcan_usb_time));
}

/* PCAN-USB syncs every ~1.050 s ~23437 ticks */
#define PCAN_USB_SYNC_PERIOD_US		USEC_PER_SEC
#define PCAN_USB_SYNC_PERIOD_TICK	((PCAN_USB_SYNC_PERIOD_US * \
					  PCAN_USB_TS_SCALE_DIVISOR) / \
						PCAN_USB_TS_SCALE_MULTIPLIER)
#ifdef DEBUG_TIMESTAMP
static int already_dbg_wrapp = 0;
#endif

static void pcan_ticks_to_ts(u32 ticks, u32 *ts_low, u32 *ts_high)
{
	u64 ts = ticks;

	ts *= PCAN_USB_TS_SCALE_MULTIPLIER;

	ts = DIV_ROUND_UP_SECTOR_T(ts, PCAN_USB_TS_SCALE_DIVISOR);

	*ts_high = (u32 )(ts >> 32);
	*ts_low = (u32 )ts;
}

static void pcan_updateTimeStampFromWord(struct pcandev *dev, u16 ticks,
						struct pcan_timeval *tv)
{
	struct pcan_usb_time *t = &dev->port.usb.time;
	u32 ts_low, ts_high;
	u32 ticks_count;

	if (!t->sync_ticks_init)
		return;

	ticks_count = t->sync_ticks_high;

	/* if received tick16 is smaller than the last sync tick low 16-bit,
	 * then the tick counter may have wrapped.
	 * Note: during this sync period, we are able to receive severa
	 * smaller ticks16 */
	if (ticks < t->sync_ticks_low) {

		/* if the new tick is smaller than the sync period,
		 * then the counter has wrapped. */
		if (ticks < PCAN_USB_SYNC_PERIOD_TICK) {

			/* Because the driver might receive event(s)
			 * AFTER having received the calibration msg
			 * *BUT* with a timestamp LOWER than the
			 * ticks of the calibration msg, we must
			 * consider wrapping only if event ticks is
			 * REALLY lower than calibration ticks:
			 * the counter has wrapped if NEXT calibration
			 * ticks will wrapp too */
			u16 next_ticks = t->sync_ticks_low +
						 PCAN_USB_SYNC_PERIOD_TICK;
			if (next_ticks < t->sync_ticks_low) {
				ticks_count += 0x10000;
#ifdef DEBUG_TIMESTAMP
				if (!already_dbg_wrapp) {
					already_dbg_wrapp++;
					pr_info(DEVICE_NAME
						": tick16 wrapped: %u < %u\n",
						ticks, t->sync_ticks_low);
				}
			} else already_dbg_wrapp = 0;
		} else already_dbg_wrapp = 0;
	} else already_dbg_wrapp = 0;
#else
			}
		}
	}
#endif

	/* keep in memory the low 16-bit of the current tick value, 
	 * to detect when 8-bit value wrapps */
	t->ticks16 = ticks;

	/* convert the tick value of this event into µs timestamp: */
	ticks_count += ticks;
	pcan_ticks_to_ts(ticks_count, &ts_low, &ts_high);

#ifdef DEBUG_TIMESTAMP
	pr_info(DEVICE_NAME ": "
		//"ticks=%u ticks_count=%u; "
		"decoding ts=%u-%08u µs\n",
		//ticks, ticks_count,
		ts_high, ts_low);
#endif
	/* then, decode this timestamp according to current clock drift */
	pcan_sync_decode(dev, ts_low, ts_high, tv);
}

static void pcan_updateTimeStampFromByte(struct pcandev *dev, u8 ticks8,
						struct pcan_timeval *tv)
{
	struct pcan_usb_time *t = &dev->port.usb.time;
	u8 hsb = t->ticks16 >> 8;

	/* convert this tick8 into tick16 */
	if (t->sync_ticks_init) {
		const u8 lsb = t->ticks16 & 0xff;

		if (ticks8 < lsb) {
			hsb++;
#ifdef DEBUG_TIMESTAMP
			pr_info(DEVICE_NAME
				": ticks8 wrapped: %u < %u: "
				"tick16=%u\n",
				ticks8, lsb, (u16 )hsb << 8 | ticks8);
#endif
		}

		pcan_updateTimeStampFromWord(dev, (u16 )hsb << 8 | ticks8, tv);
	}
}

static int pcan_usb_sync_times(struct pcandev *dev, u16 ticks, u32 dtv)
{
	struct pcan_usb_time *t = &dev->port.usb.time;
	u32 ts_low, ts_high;
	u32 ticks_count;

	if (t->sync_ticks_init) {

		/* with calibration message, we're certain that this test is
		 * enough to detect wrapping: */
		if (ticks < t->sync_ticks_low) {
			t->sync_ticks_high += 0x10000;
#ifdef DEBUG_TIMESTAMP
			pr_info(DEVICE_NAME ": sync: ticks wrapped: %u < %u: "
				"ticks_high=%u\n",
				ticks, t->sync_ticks_low,
				t->sync_ticks_high);
#endif
		}
	} else {
#if 0
		/* ts_low seems to wrapp after ~1430s <=>
		 * (1430.000.000 x 3) / 128 ticks =>
		 * ticks = 33.515.628 = 0x01FF.6869 */
		t->sync_ticks_high = 0x01Fe0000;
#endif
		t->sync_ticks_init = ticks;
		t->ticks16 = ticks;
	}

	t->sync_ticks_low = ticks;
	ticks_count = t->sync_ticks_high + t->sync_ticks_low;

	/* convert now ticks into µs and use common sync mechanism */
	pcan_ticks_to_ts(ticks_count, &ts_low, &ts_high);

#ifdef DEBUG_TIMESTAMP
	pr_info(DEVICE_NAME
		": sync: %u tks => ts_high=%u ts_low=%08u\n",
		ticks, ts_high, ts_low);
#endif

	return pcan_sync_times(dev, ts_low, ts_high, dtv);
}

static int pcan_usb_setcontrol_urb(struct pcandev *dev, u8 function, u8 number,
                                   u8 param0, u8 param1, u8 param2, u8 param3)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	PCAN_USB_PARAM *cmd = (PCAN_USB_PARAM *)dev->port.usb.cout_baddr;
	int err, l;

#ifdef DEBUG
	pr_err(DEVICE_NAME ": %s(): [func=%d num=%d %02X %02X %02X]->EP#%02X\n",
			__func__, function, number, param0, param1, param2,
			usb_if->pipe_cmd_out.ucNumber);
#endif
	/* don't do anything with non-existent hardware */
	if (!dev->is_plugged)
		return -ENODEV;

	memset(cmd, '\0', sizeof(PCAN_USB_PARAM));
	cmd->Function = function;
	cmd->Number = number;
	cmd->Param[0] = param0;
	cmd->Param[1] = param1;
	cmd->Param[2] = param2;
	cmd->Param[3] = param3;

	err = usb_bulk_msg(usb_if->usb_dev,
			   usb_sndbulkpipe(usb_if->usb_dev,
	                   		usb_if->pipe_cmd_out.ucNumber),
			   cmd, sizeof(PCAN_USB_PARAM), &l, COMMAND_TIMEOUT);
#ifndef DEBUG
	if (err)
#endif
		pr_err(DEVICE_NAME
		       ": err %d submitting cmd %d.%d to PCAN-USB\n",
		       err, function, number);

	return err;
}

static int pcan_usb_getcontrol_urb(struct pcandev *dev, u8 function, u8 number,
				u8 *param0, u8 *param1, u8 *param2, u8 *param3)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	PCAN_USB_PARAM *cmd = (PCAN_USB_PARAM *)dev->port.usb.cout_baddr;
	int err, l, i;

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(cmd %d.%d): <-EP#%02X\n",  __func__,
	        function, number, usb_if->pipe_cmd_in.ucNumber);
#endif

	/* sometimes, need to send the cmd more than once... */
	for (i = 0; i < 3; i++) {

		/* don't do anything with non-existent hardware */
		if (!dev->is_plugged) {
			err = -ENODEV;
			break;
		}

		/* first write function and number to device */
		err = pcan_usb_setcontrol_urb(dev, function, number,
					      0, 0, 0, 0);
		if (err)
			break;

		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_rcvbulkpipe(usb_if->usb_dev,
					   usb_if->pipe_cmd_in.ucNumber),
				   cmd, sizeof(PCAN_USB_PARAM), &l,
				   COMMAND_TIMEOUT);
		if (!err) {

			if (param0) {
				*param0 = cmd->Param[0];
				if (param1) {
					*param1 = cmd->Param[1];
					if (param2) {
						*param2 = cmd->Param[2];
						if (param3)
							*param3 = cmd->Param[3];
					}
				}
			}
			break;
		}

		/* retry only in case of timeout */
		if (err != -ETIMEDOUT)
			break;

#ifdef DEBUG
		pr_err(DEVICE_NAME
		       ": timeout reading cmd %d.%d result from PCAN-USB\n",
		       function, number);
#endif
	}

#ifndef DEBUG
	if (err)
#endif
		pr_err(DEVICE_NAME
		       ": err %d reading cmd %d.%d result from PCAN-USB\n",
		       err, function, number);

	return err;
};

#ifdef PCAN_USB_HANDLE_ERR_CNT

#define PCAN_USB_ERR_ECC	0x01
#define PCAN_USB_ERR_RXERR	0x02
#define PCAN_USB_ERR_TXERR	0x04
#define PCAN_USB_ERR_RXERR_CNT	0x08
#define PCAN_USB_ERR_TXERR_CNT	0x10

static int pcan_usb_set_err_frame(struct pcandev *dev, u8 err_mask)
{
	DPRINTK(KERN_DEBUG "%s: %s(0x%02x)\n",
		DEVICE_NAME, __func__, err_mask);

	return pcan_usb_setcontrol_urb(dev, 0x0b, 2, err_mask, 0, 0, 0);
}
#endif

static int pcan_usb_setBTR0BTR1(struct pcandev *dev, u16 wBTR0BTR1)
{
	u8 param0 = (u8)(wBTR0BTR1 & 0xff);
	u8 param1 = (u8)((wBTR0BTR1 >> 8) & 0xff);

	DPRINTK(KERN_DEBUG "%s: %s(0x%04x)\n",
		DEVICE_NAME, __func__, wBTR0BTR1);

	return pcan_usb_setcontrol_urb(dev, 1, 2, param0, param1, 0, 0);
}

static int pcan_usb_setCANOn(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* be sure to resync each time we open */
	pcan_sync_init(dev);

	return pcan_usb_setcontrol_urb(dev, 3, 2, 1, 0, 0, 0);
}

static int pcan_usb_setCANOff(struct pcandev *dev)
{
	int err;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	/* wait a bit for last data to be written on CAN bus:
	 * - 5 ms is not enough if any data buffer was almost filled
	 * - 10 ms is not enough for PCAN-USB (fw 2.8)
	 * - 20 ms is ok for PCAN-USB fw 2.8. */
	if (dev->tx_frames_counter > 0)
		msleep_interruptible(20);

	/* from WIN driver: fw <= 2.5 need IRQ enable off before setting CAN
	 * to Off */
	if (pcan_usb_get_if(dev)->ucRevision <= 25) {
		err = pcan_usb_setcontrol_urb(dev, 9, 2, 4, 0, 0, 0);
	}

	/* CAN to Off */
	err = pcan_usb_setcontrol_urb(dev, 3, 2, 0, 0, 0, 0);

#ifdef PCAN_USB_HANDLE_ERR_CNT
	err = pcan_usb_set_err_frame(dev, 0);
#endif

#ifndef DO_GENERATE_ECC_ERROR_AT_1MBPS
	/* code below means that fw 8.x don't need to put SJA1000 into init
	 * mode? WTF? */
#else
	/* according to WIN driver: put SJA1000 reg0 = control reset for
	 * fw in range [2.6...4.2] */
	if (pcan_usb_get_if(dev)->ucRevision >= 26 &&
			pcan_usb_get_if(dev)->ucRevision <= 42) {

		/* set SJA1000 in init mode */
		err = pcan_usb_setcontrol_urb(dev, 9, 2, 0, 1, 0, 0);
	}
#endif

	/* or if (err != -ENODEV) { */
	if (dev->is_plugged) {
		struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

		DPRINTK(KERN_DEBUG "%s: %s fw 2.8 workaround\n",
				DEVICE_NAME, dev->adapter->name);
		err = usb_clear_halt(usb_if->usb_dev,
				usb_sndbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_out.ucNumber));
#ifdef DEBUG
		if (err)
			pr_err("%s: %s(): usb_clear_halt(1) failed err %d\n",
				DEVICE_NAME, __func__, err);
#endif

		err = usb_clear_halt(usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_in.ucNumber));
		if (err) {
			DPRINTK(KERN_DEBUG
				"%s: %s(): usb_clear_halt(2) failed err %d\n",
				DEVICE_NAME, __func__, err);
			return err;
		}

		mdelay(50);
	}

#ifdef DO_GENERATE_ECC_ERROR_AT_1MBPS
	return err;
#else
	/* set SJA1000 in init mode */
	return pcan_usb_setcontrol_urb(dev, 9, 2, 0, 1, 0, 0);
#endif
}

static int pcan_usb_setCANSilentOn(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return pcan_usb_setcontrol_urb(dev, 3, 3, 1, 0, 0, 0);
}

static int pcan_usb_setCANSilentOff(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return pcan_usb_setcontrol_urb(dev, 3, 3, 0, 0, 0, 0);
}

#if 0
/* not used */
static int pcan_usb_getBTR0BTR1(struct pcandev *dev, u16 *pwBTR0BTR1)
{
	int err;
	u8 param0 = 0;
	u8 param1 = 0;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	err = pcan_usb_getcontrol_urb(dev, 1, 1, &param0, &param1, NULL, NULL);

	*pwBTR0BTR1 = param1;
	*pwBTR0BTR1 <<= 8;
	*pwBTR0BTR1 |= param0;

	DPRINTK(KERN_DEBUG "%s: BTR0BTR1 = 0x%04x\n", DEVICE_NAME, *pwBTR0BTR1);

	return err;
}

static int pcan_usb_getQuartz(struct pcandev *dev, u32 *pdwQuartzHz)
{
	int err = 0;
	u8 param0 = 0;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	err = pcan_usb_getcontrol_urb(dev, 2, 1, &param0, NULL, NULL, NULL);

	*pdwQuartzHz = param0;
	*pdwQuartzHz *= 1000000L;

	DPRINTK(KERN_DEBUG "%s: Frequenz = %u\n", DEVICE_NAME, *pdwQuartzHz);

	return err;
}

static int pcan_usb_setExtVCCOn(struct pcandev *dev)
{
	u8 dummy = 0;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return pcan_usb_setcontrol_urb(dev, 0xA, 2, 1, 0, 0, 0);
}
#endif

static int pcan_usb_getDeviceNr(struct pcandev *dev, u32 *pdwDeviceNr)
{
	int err;
	u8 dev_nr = 0;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	*pdwDeviceNr = 0;
	err = pcan_usb_getcontrol_urb(dev, 4, 1, &dev_nr, NULL, NULL, NULL);

	DPRINTK(KERN_DEBUG "%s: DeviceNr = 0x%x\n", DEVICE_NAME, dev_nr);

        if (!err && (dev_nr != 255)
		/* New versions of PCAN-USB Classic uses 0 as default value */
		 && (dev_nr != 0)
		 ) {

		*pdwDeviceNr = dev_nr;

		dev->device_alt_num = dev_nr;
		dev->flags |= PCAN_DEV_USES_ALT_NUM;
	}

	return err;
}

static int pcan_usb_setDeviceNr(struct pcandev *dev, u32 dwDeviceNr)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return pcan_usb_setcontrol_urb(dev, 4, 2, (u8 )dwDeviceNr, 0, 0, 0);
}

static int pcan_usb_setExtVCCOff(struct pcandev *dev)
{
	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	return pcan_usb_setcontrol_urb(dev, 0xA, 2, 0, 0, 0, 0);
}

static int pcan_usb_getSNR(struct pcan_usb_interface *usb_if, u32 *pdwSNR)
{
	struct pcandev *dev = usb_if_dev(usb_if, 0);
	ULCONV SNR;
	int err;

	DPRINTK(KERN_DEBUG "%s: %s()\n", DEVICE_NAME, __func__);

	memset(&SNR, 0, sizeof(SNR));
	err = pcan_usb_getcontrol_urb(dev, 6, 1,
				&SNR.uc[3], &SNR.uc[2], &SNR.uc[1], &SNR.uc[0]);

	*pdwSNR = SNR.ul;

	DPRINTK(KERN_DEBUG "%s: SNR = 0x%08x\n", DEVICE_NAME, *pdwSNR);

	return err;
}

static int pcan_usb_Init(struct pcandev *dev, u16 btr0btr1,
						u8 use_ext, u8 bListenOnly)
{
	int err = 0;

	DPRINTK(KERN_DEBUG "%s: %s(btr0btr1=%04xh use_ext=%u bListenOnly=%u)\n",
			DEVICE_NAME, __func__, btr0btr1, use_ext,
			bListenOnly);

	err = pcan_usb_setBTR0BTR1(dev, btr0btr1);
	if (err)
		goto fail;

#ifdef PCAN_USB_HANDLE_ERR_CNT
	err = pcan_usb_set_err_frame(dev,
				PCAN_USB_ERR_ECC|
				PCAN_USB_ERR_RXERR|PCAN_USB_ERR_TXERR|
				PCAN_USB_ERR_RXERR_CNT|PCAN_USB_ERR_TXERR_CNT);
	if (err)
		pr_info("%s: WARNING: unable to get Rx/Tx counters changes "
			"(err %d)\n", DEVICE_NAME, err);
#endif
	if (pcan_usb_get_if(dev)->ucRevision > 3) {

		/* set listen only */
		if (bListenOnly)
			err = pcan_usb_setCANSilentOn(dev);
		else
			err = pcan_usb_setCANSilentOff(dev);
		if (err)
			goto fail;
	} else {
		/* generate err if one tries to set bListenOnly */
		if (bListenOnly) {
			err = -EINVAL;
			goto fail;
		}
	}

	/* don't know how to handle - walk the save way */
	err = pcan_usb_setExtVCCOff(dev);

	/* prepare for new start of timestamp calculation */
	pcan_reset_timestamp(dev);

	dev->port.usb.dwTelegramCount = 0;

fail:
	return err;
}

/* takes USB-message frames out of ucMsgPtr, decodes and packs them into
 * readFifo
 */
static int pcan_usb_DecodeMessage(struct pcan_usb_interface *usb_if,
					u8 *ucMsgPtr, int lCurrentLength)
{
	int err = 0;
	int i, j;
	u8 ucMsgPrefix;
	u8 ucMsgLen;         /* number of frames in one USB packet */
	u8 ucStatusLen = 0;  /* status/length entry leading each data frame */
	u8 ucLen;            /* len in bytes of received (CAN) data */
	UWCONV wTimeStamp;
	u8 *ucMsgStart = ucMsgPtr; /* for overflow compare */
#ifndef NETDEV_SUPPORT
	int rwakeup = 0;
#endif
	u8 *org = ucMsgPtr;
	int ts8 = 0;
	struct pcandev *dev = usb_if_dev(usb_if, 0);

	DPRINTK(KERN_DEBUG "%s: %s(%p, %d)\n",
			DEVICE_NAME, __func__, ucMsgPtr, lCurrentLength);

#ifdef DEBUG_DECODE
	dump_mem("received buffer", ucMsgPtr, lCurrentLength);
#endif

	/* Note: PCAN-USB does not notify when ERROR-ACTIVE is reached so it's
	 * hard to reset BUS ERROR bits... */

	/* don't count interrupts - count packets */
	dev->dwInterruptCounter++;

	/* sometimes is nothing to do */
	if (!lCurrentLength)
		return err;

	/* get prefix of message and step over */
	ucMsgPrefix = *ucMsgPtr++;

	/* get length of message and step over */
	ucMsgLen = *ucMsgPtr++;

#ifdef DEBUG_DECODE
	pr_info("%s: MsgPrefix=%02Xh MsgLen=%u\n",
			DEVICE_NAME, ucMsgPrefix, ucMsgLen);
#endif
	for (i = 0; i < ucMsgLen; i++) {
		ULCONV localID;
		struct pcanfd_rxmsg rx;

		ucStatusLen = *ucMsgPtr++;

		ucLen = ucStatusLen & STLN_DATA_LENGTH;

		memset(&rx.msg, '\0', sizeof(rx.msg));

#ifndef HANDLE_DLC_GT_THAN_8
		if (ucLen > 8)
			ucLen = 8;
		rx.msg.data_len = ucLen;
#else
		/* temporary, MUST check if lib and user app support */
		//rx.msg.data_len = ucLen;
		rx.msg.data_len = (ucLen > 8) ? 8 : ucLen;
#endif

#ifdef DEBUG_DECODE
		pr_info("%s: - [%u]: StatusLen=%02X (Status=%d Len=%u) ",
				DEVICE_NAME, i, ucStatusLen,
				!!(ucStatusLen & STLN_INTERNAL_DATA),
				ucLen);
		if (ucStatusLen & STLN_INTERNAL_DATA) {
			printk("Function=%u Number=%u ",
						ucMsgPtr[0], ucMsgPtr[1]);

			if (ucStatusLen & STLN_WITH_TIMESTAMP) {
				if (!ts8)
					printk("Ts=%02x %02x ",
						ucMsgPtr[2], ucMsgPtr[3]);
				else
					printk("Ts=%02x ", ucMsgPtr[2]);
			}
		} else {
			printk("CAN Msg");
		}
		printk("\n");
#endif

		/* normal CAN messages are always with timestamp */
		if (!(ucStatusLen & STLN_INTERNAL_DATA)) {
			int nRtrFrame;

			rx.msg.type = PCANFD_TYPE_CAN20_MSG;
			rx.msg.flags = PCANFD_MSG_STD|PCANFD_HWTIMESTAMP|\
							PCANFD_TIMESTAMP;

			nRtrFrame = ucStatusLen & STLN_RTR;
			if (nRtrFrame)
				rx.msg.flags |= PCANFD_MSG_RTR;

			/* check fw version if SR is supported */
			if (usb_if->ucRevision >= 41) {
				if (*ucMsgPtr & 0x01)
					rx.msg.flags |= PCANFD_MSG_SLF;
			}

			if (ucStatusLen & STLN_EXTENDED_ID) {
				rx.msg.flags |= PCANFD_MSG_EXT;

#if defined(__LITTLE_ENDIAN)
				localID.uc[0] = *ucMsgPtr++;
				localID.uc[1] = *ucMsgPtr++;
				localID.uc[2] = *ucMsgPtr++;
				localID.uc[3] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				localID.uc[3] = *ucMsgPtr++;
				localID.uc[2] = *ucMsgPtr++;
				localID.uc[1] = *ucMsgPtr++;
				localID.uc[0] = *ucMsgPtr++;
#endif
				localID.ul >>= 3;
			} else {
				localID.ul = 0;

#if defined(__LITTLE_ENDIAN)
				localID.uc[0] = *ucMsgPtr++;
				localID.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				localID.uc[3] = *ucMsgPtr++;
				localID.uc[2] = *ucMsgPtr++;
#endif
				localID.ul >>= 5;
			}

			rx.msg.id = localID.ul;

			/* read timestamp, 1st timestamp in packet is 16 bit
			 * AND data, following timestamps are 8 bit in length */
			if (!ts8++) {
#if defined(__LITTLE_ENDIAN)
				wTimeStamp.uc[0] = *ucMsgPtr++;
				wTimeStamp.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				wTimeStamp.uc[1] = *ucMsgPtr++;
				wTimeStamp.uc[0] = *ucMsgPtr++;
#endif

				pcan_updateTimeStampFromWord(dev, wTimeStamp.uw,
							     &rx.hwtv);
			} else {
				pcan_updateTimeStampFromByte(dev, *ucMsgPtr++,
							     &rx.hwtv);
			}

#ifndef HANDLE_DLC_GT_THAN_8
#else
			/* SGr FIX:
			 * save in DATA array at most 8 bytes, ignore
			 * all others but ALWAYS increment msg ptr
			 * according to ucLen */
#endif
			/* read data */
			j = 0;
			if (!nRtrFrame) {
				while (ucLen--) {
					if (j < 8)
						rx.msg.data[j++] = *ucMsgPtr;
					ucMsgPtr++;
				}
			}

			/* there's another byte when SR flag is set
			 * (client handle) */
			if (rx.msg.flags & PCANFD_MSG_SLF)
				ucMsgPtr++;

			/* only for beauty */
			while (j < 8)
				rx.msg.data[j++] = 0;

			err = pcan_xxxdev_rx(dev, &rx);
			if (err < 0) {
				if (net_ratelimit())
					pr_err("%s: pcan_xxxdev_rx(1) failed "
						" err=%d\n",
					       DEVICE_NAME, err);
				//dump_mem(DEVICE_NAME, frame.data, j);
				goto fail;
			}

#ifndef NETDEV_SUPPORT
			/* successfully enqueued into chardev FIFO */
			if (err > 0)
				rwakeup++;
#endif

		/* Status message */
		} else {
			u8 ucFunction;
			u8 ucNumber;
			u8 dummy;
			u8 ecc;

			/* get function and number */
			ucFunction = *ucMsgPtr++;
			ucNumber = *ucMsgPtr++;

			if (ucStatusLen & STLN_WITH_TIMESTAMP) {

				rx.msg.flags |= PCANFD_HWTIMESTAMP|PCANFD_TIMESTAMP;

				/* only the 1st packet supplies a word
				 * timestamp */
				if (!ts8++) {
#if defined(__LITTLE_ENDIAN)
					wTimeStamp.uc[0] = *ucMsgPtr++;
					wTimeStamp.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
					wTimeStamp.uc[1] = *ucMsgPtr++;
					wTimeStamp.uc[0] = *ucMsgPtr++;
#endif

					pcan_updateTimeStampFromWord(dev,
							wTimeStamp.uw,
							&rx.hwtv);
				} else
					pcan_updateTimeStampFromByte(dev,
							*ucMsgPtr++,
							&rx.hwtv);
			}

			switch (ucFunction) {

			case 1:
				if ((ucNumber & CAN_RECEIVE_OVERRUN) ||
						(ucNumber & QUEUE_OVERRUN)) {
					pcan_handle_error_ctrl(dev, &rx,
							PCANFD_RX_OVERFLOW);
				}

				if ((ucNumber & XMT_BUFFER_FULL) ||
						(ucNumber & QUEUE_XMT_FULL)) {
					pcan_handle_error_ctrl(dev, &rx,
							PCANFD_TX_OVERFLOW);
				}

				if (ucNumber & BUS_OFF) {
					pcan_handle_busoff(dev, &rx);
				} else if (!pcan_handle_error_status(dev, &rx,
							ucNumber & BUS_LIGHT,
							ucNumber & BUS_HEAVY)) {
					/* no error bit (so, no error, back to
					 * active state) */
					pcan_handle_error_active(dev, &rx);
				}

				/* version 3: sometimes the telegram carries 3
				 * additional data without note in ucStatusLen.
				 */
#ifndef HANDLE_DLC_GT_THAN_8
				/* Don't know what to do ?? */
#else
				/* SGr FIX:
				 * save in DATA array at most 8 bytes, ignore
				 * all others but ALWAYS increment msg ptr
				 * according to ucLen */
#endif
				for (j = 0; ucLen--; ucMsgPtr++)
					if (j < 8)
						rx.msg.data[j++] = *ucMsgPtr;
				break;

			case 2: /* get_analog_value, remove bytes */
				dummy = *ucMsgPtr++;
				dummy = *ucMsgPtr++;
				break;

			case 3: /* get_bus_load, remove byte */
				dev->bus_load = *ucMsgPtr++;
				break;

			case 4: /* only timestamp */
#if defined(__LITTLE_ENDIAN)
				wTimeStamp.uc[0] = *ucMsgPtr++;
				wTimeStamp.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				wTimeStamp.uc[1] = *ucMsgPtr++;
				wTimeStamp.uc[0] = *ucMsgPtr++;
#endif

				/* this specific timestamp is periodically 
				 * sent by the PCAN-USB (more than every 1s):
				 * use it to sync times. */
				pcan_usb_sync_times(dev, wTimeStamp.uw, -50);

#ifdef BUS_STATE_ACTIVE_ON_1ST_TS
				/* consider 1st sync as STATUS=ACTIVE */
				if (dev->bus_state == PCANFD_UNKNOWN)
					pcan_handle_error_active(dev, &rx);
#endif
				break;

			case 5:
				ecc = ucMsgPtr[0];
#ifdef DEBUG_DECODE
				/* Looks like QUEUE_XMT_FULL is not set with
				 * function=5 (ErrorFrame) */
				pr_info("%s: got err_frame len=%u number=%u "
					"[ECC=%02x RXERR=%02x TXERR=%02x]\n",
					DEVICE_NAME, ucLen, ucNumber,
					ucMsgPtr[0],
					ucMsgPtr[1],
					ucMsgPtr[2]);
#endif
				switch (ucNumber) {
				case 0x80:
					dev->rx_error_counter = ucMsgPtr[1];
					dev->tx_error_counter = ucMsgPtr[2];
					break;
				case 0x00:
					/* decrement ? */
					break;
				}

				/* sync msg */
				while (ucLen--)
					ucMsgPtr++;

				pcan_handle_error_msg(dev, &rx,
					(ecc & 0xc0) >> 6, (ecc & 0x1f),
					(ecc & 0x20), 0);
				break;

			case 10: /* prepared for future */
				break;

			default:
				printk(KERN_ERR
					"%s: unexpected function %d, i=%d, "
					"ucStatusLen=0x%02x\n",
					DEVICE_NAME, ucFunction, i,
					ucStatusLen);
				dump_mem("unexpected function", org, 64);

				/* unsync? stop the analysis of the entire
				 * packet */
				err = -EILSEQ;
				goto fail;
			}

			/* if an error condition occurred, send an error frame
			 * to the userspace */
			if (rx.msg.type != PCANFD_TYPE_NOP) {

				err = pcan_xxxdev_rx(dev, &rx);
				if (err < 0) {
					if (net_ratelimit())
						pr_err("%s: pcan_xxxdev_rx(2) "
						      "failure (err=%d)\n",
						      DEVICE_NAME, err);

					//dump_mem(DEVICE_NAME, msg.Msg.DATA,
					//			msg.Msg.LEN);
					goto fail;
				}

#ifndef NETDEV_SUPPORT
				/* successfully enqueued into chardev FIFO */
				if (err > 0)
					rwakeup++;
#endif
			}
		}

		/* check for 'read from'-buffer overrun
		 * must be <= dev->port.usb.pipe_read.wDataSz) */
		if ((ucMsgPtr - ucMsgStart) > lCurrentLength) {

			/* sometimes v 3 overrides the buffer by 1 byte */
			if ((usb_if->ucRevision > 3) ||
				((usb_if->ucRevision <= 3)
			&& ((ucMsgPtr - ucMsgStart) > (lCurrentLength + 1)))) {
				err = -EFAULT;
#ifdef __LP64__
				printk(KERN_ERR
					"%s: Internal Error = %d (%ld, %d)\n",
					DEVICE_NAME, err,
					ucMsgPtr - ucMsgStart,
					lCurrentLength);
#else
				printk(KERN_ERR
					"%s: Internal Error = %d (%d, %d)\n",
					DEVICE_NAME, err,
					ucMsgPtr - ucMsgStart,
					lCurrentLength);
#endif
				dump_mem("internal error", org, 64);
				goto fail;
			}
		}
	}

#ifndef NETDEV_SUPPORT
	if (rwakeup) {
		pcan_event_signal(&dev->in_event);
	}
#endif
	return 0;
fail:
	dev->nLastError = err;
	dev->dwErrorCounter++;

	return err;
}

/* gets messages out of write-fifo, encodes and puts them into USB buffer
 * ucMsgPtr
 * returns -ENODATA and *pnDataLength > 0  if I made a telegram and no more data
 *                                         are available
 *         -ENODATA and *pnDataLength == 0 if I made no telegram and no more
 *                                         data are available
 *         any ERROR else if something happend
 *         no ERROR if I made a telegram and there are more data available
 */
static int pcan_usb_EncodeMessage(struct pcandev *dev, u8 *ucMsgPtr,
							int *pnDataLength)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int nDataLength = *pnDataLength;
	int nBufferTop = nDataLength - 14;  /* buffer fill high water mark */
	u8 *ptr = ucMsgPtr;
	int nMsgCounter = 0;
	u8 bFinish = 0;
	int err = 0;
	u8 ucLen;
	ULCONV localID;
	u8 *pucStatusLen;
	u8 *pucMsgCountPtr;
	int j;

	/* indicate no packet */
	*pnDataLength = 0;

	/* put packet type information */
	*ptr++ = 2;
	pucMsgCountPtr = ptr++;  /* fill later the count of messages */

	while (!bFinish && ((ptr - ucMsgPtr) < nBufferTop)) {
		int nRtrFrame;
		struct pcanfd_txmsg tx;

		/* release fifo buffer and step forward in fifo */
		err = pcan_fifo_get(&dev->writeFifo, &tx);
		if (err) {
			bFinish = 1;

			if (err != -ENODATA) {
				pr_info(DEVICE_NAME
					": can't get data out of writeFifo, "
					"avail data: %d, err: %d\n",
					dev->writeFifo.nStored,
					err);
			}

			continue;
		}

		/* check fw version if SR is supported */
		if (usb_if->ucRevision < 41)
			tx.msg.flags &= ~MSGTYPE_SELFRECEIVE;

		/* get ptr to ucStatusLen byte */
		pucStatusLen = ptr++;

		*pucStatusLen = ucLen = tx.msg.data_len & STLN_DATA_LENGTH;

		nRtrFrame = tx.msg.flags & MSGTYPE_RTR;
		if (nRtrFrame)
			*pucStatusLen |= STLN_RTR;

		j = 0;
		localID.ul = tx.msg.id;

		//dump_mem("sending Msg:", &tx.msg, sizeof(tx.msg));

		if (tx.msg.flags & MSGTYPE_EXTENDED) {
			*pucStatusLen |= STLN_EXTENDED_ID;
			localID.ul <<= 3;

			if (tx.msg.flags & MSGTYPE_SELFRECEIVE)
				localID.ul |= 0x01;	/* SRR flag SJA1000 */
#if defined(__LITTLE_ENDIAN)
			*ptr++ = localID.uc[0];
			*ptr++ = localID.uc[1];
			*ptr++ = localID.uc[2];
			*ptr++ = localID.uc[3];
#elif defined(__BIG_ENDIAN)
			*ptr++ = localID.uc[3];
			*ptr++ = localID.uc[2];
			*ptr++ = localID.uc[1];
			*ptr++ = localID.uc[0];
#endif
		} else {
			localID.ul <<= 5;

			if (tx.msg.flags & MSGTYPE_SELFRECEIVE)
				localID.ul |= 0x01;	/* SRR flag SJA1000 */
#if defined(__LITTLE_ENDIAN)
			*ptr++ = localID.uc[0];
			*ptr++ = localID.uc[1];
#elif defined(__BIG_ENDIAN)
			*ptr++ = localID.uc[3];
			*ptr++ = localID.uc[2];
#endif
		}

		if (!nRtrFrame) {
#ifdef HANDLE_DLC_GT_THAN_8
			/* SGr NOTE: copy *ONLY* 8 bytes max */
#endif
			j = 0;
			while (ucLen--)
				*ptr++ = (j < 8) ? tx.msg.data[j++] : 0;
		}

		if (tx.msg.flags & MSGTYPE_SELFRECEIVE)
			*ptr++ = 0x80;

		nMsgCounter++;
	}

	/* generate external nDataLength if I carry payload */
	if ((ptr - ucMsgPtr) > 2) {
		*pnDataLength = nDataLength;

		/* set count of telegrams */
		ptr = ucMsgPtr + nDataLength - 1;
		*ptr = (u8)(dev->port.usb.dwTelegramCount++ & 0xff);

		/* last to do: put count of messages */
		*pucMsgCountPtr = nMsgCounter;

		dev->tx_frames_counter += nMsgCounter;
	} else {
		*pnDataLength = 0;
		*pucMsgCountPtr = 0;
	}

	//dump_mem("Tx buffer:", ucMsgPtr, *pnDataLength);
	return err;
}

static int pcan_usb_set_mass_storage_mode(struct pcan_usb_interface *usb_if)
{
	return pcan_usb_setcontrol_urb(usb_if_dev(usb_if, 0),
					PCAN_USB_SETCAN2FLASH, 0,
					0, 0, 0, 0);
}

/*
 * void pcan_usb_cleanup(struct pcandev *dev)
 *
 * Last chance to submit URB before driver removal.
  * Note: the device is always plugged!
 */
static void pcan_usb_cleanup(struct pcandev *dev)
{
	/* Set CAN bus off here now since we're sure that the request will be */
	/* sent to the usb module */
	pcan_usb_setCANOff(dev);
}

static int pcan_usb_ctrl_init(struct pcandev *dev)
{
	pr_info(DEVICE_NAME ": %s channel device number=%u\n",
		dev->adapter->name, dev->device_alt_num);

	return 0;
}

static void pcan_usb_free(struct pcan_usb_interface *usb_if)
{
	usb_if->adapter = pcan_free_adapter(usb_if->adapter);
	pcan_usb_devices--;
}

/*
 * int pcan_usb_init(struct pcan_usb_interface *usb_if)
 *
 * Do the initialization part of a PCAN-USB adapter.
 */
int pcan_usb_init(struct pcan_usb_interface *usb_if)
{
	usb_if->hw_ver.major = (usb_if->ucRevision / 10);
	usb_if->hw_ver.minor = usb_if->ucRevision % 10;

	usb_if->adapter = pcan_alloc_adapter("PCAN-USB", pcan_usb_devices++, 1);
	if (!usb_if->adapter)
		return -ENOMEM;

	usb_if->adapter->hw_ver = usb_if->hw_ver;

	pr_info(DEVICE_NAME ": %s fw v%u.%u\n",
		usb_if->adapter->name, usb_if->adapter->hw_ver.major,
		usb_if->adapter->hw_ver.minor);

	/* Note: customers version starts from 8.3 but some devices
	 * might includes development versions starting from 7.x */
	if (VER_NUM(usb_if->adapter->hw_ver.major,
		    usb_if->adapter->hw_ver.minor, 0) >= VER_NUM(7, 0, 0))

		usb_if->device_set_mass_storage_mode =
			pcan_usb_set_mass_storage_mode;

	usb_if->device_ctrl_init = pcan_usb_ctrl_init;
	usb_if->device_ctrl_cleanup = pcan_usb_cleanup;
	usb_if->device_ctrl_open = pcan_usb_Init;
	usb_if->device_ctrl_set_bus_on = pcan_usb_setCANOn;
	usb_if->device_ctrl_set_bus_off = pcan_usb_setCANOff;
	usb_if->device_get_snr = pcan_usb_getSNR;
	usb_if->device_ctrl_set_dnr = pcan_usb_setDeviceNr;
	usb_if->device_ctrl_get_dnr = pcan_usb_getDeviceNr;
	usb_if->device_ctrl_msg_encode = pcan_usb_EncodeMessage;
	usb_if->device_msg_decode = pcan_usb_DecodeMessage;
	usb_if->device_free = pcan_usb_free;

	return 0;
}
#endif /* USB_SUPPORT */
