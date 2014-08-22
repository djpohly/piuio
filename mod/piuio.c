/*
 * PIUIO interface driver
 *
 * Copyright (C) 2012-2014 Devin J. Pohly (djpohly+linux@gmail.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This code is based on the Linux USB HID keyboard driver by Vojtech Pavlik.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/usb/input.h>


/*
 * Device and protocol definitions
 */
#define USB_VENDOR_ID_ANCHOR 0x0547
#define USB_PRODUCT_ID_PYTHON2 0x1002

/* USB message used to communicate with the device */
#define PIUIO_MSG_REQ 0xae
#define PIUIO_MSG_VAL 0
#define PIUIO_MSG_IDX 0

#define PIUIO_MSG_SZ 8
#define PIUIO_MSG_LONGS (PIUIO_MSG_SZ / sizeof(unsigned long))

/* Number of usable inputs per set */
#define PIUIO_INPUTS 48

/* Number of sets of inputs multiplexed together */
#define PIUIO_MULTIPLEX 4


/**
 * struct piuio - state of each attached PIUIO
 * @dev:	input device associated with this PIUIO
 * @phys:	Physical path of the device. @dev's phys field points to this
 *		buffer
 * @usbdev:	usb device associated with this PIUIO
 * @in:		URB for requesting the current state of one set of inputs
 * @out:	URB for sending data to outputs and multiplexer
 * @cr_in:	Setup packet for @new URB
 * @cr_out:	Setup packet for @out URB
 * @old:	previous state of input pins from the @in URB for each of the
 *		input sets.  These are used to determine when a press or release
 *		has happened for a group of correlated inputs.
 * @new:	Buffer for the @in URB
 * @lights:	Buffer for the @out URB
 * @new_lights:	Staging for the @lights buffer
 * @set:	current set of inputs to read, (0 .. PIUIO_MULTIPLEX - 1)
 */
struct piuio {
	struct input_dev *dev;
	char phys[64];

	struct usb_device *usbdev;
	struct urb *in, *out;
	struct usb_ctrlrequest cr_in, cr_out;
	wait_queue_head_t shutdown_wait;

	unsigned long old[PIUIO_MULTIPLEX][PIUIO_MSG_LONGS];
	unsigned long new[PIUIO_MSG_LONGS];
	unsigned char lights[PIUIO_MSG_SZ];
	unsigned char new_lights[PIUIO_MSG_SZ];

	int set;
};


/*
 * Auxiliary functions for reporting input events
 */
static int keycode(unsigned int pin)
{
	/* Use joystick buttons first, then the extra "trigger happy" range. */
	if (pin >= PIUIO_INPUTS)
		return KEY_RESERVED;
	if (pin < 16)
		return BTN_JOYSTICK + pin;
	pin -= 16;
	return BTN_TRIGGER_HAPPY + pin;
}

static void report_key(struct input_dev *dev, unsigned int pin, int press)
{
	int code = keycode(pin);

	if (code == KEY_RESERVED)
		return;

	input_event(dev, EV_MSC, MSC_SCAN, pin + 1);
	input_report_key(dev, keycode(pin), press);
}


/*
 * URB completion handlers
 */
static void piuio_in_completed(struct urb *urb)
{
	struct piuio *piu = urb->context;
	unsigned long changed[PIUIO_MSG_LONGS];
	unsigned long b;
	int i, s;
	int cur_set;

	switch (urb->status) {
		case 0:			/* success */
			break;
		case -ECONNRESET:	/* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
			goto in_finished;
		default:		/* error */
			dev_warn(&piu->dev->dev, "in urb status %d received\n",
					urb->status);
			goto resubmit;
	}

	/* Get the index of the previous input set */
	cur_set = (piu->set + PIUIO_MULTIPLEX - 1) % PIUIO_MULTIPLEX;

	/* Note what has changed in this input set, then store the inputs for
	 * next time */
	for (i = 0; i < PIUIO_MSG_LONGS; i++) {
		changed[i] = piu->new[i] ^ piu->old[cur_set][i];
		piu->old[cur_set][i] = piu->new[i];
	}

	/* Changes only count when none of the corresponding inputs in other
	 * sets are pressed.  Since "pressed" reads as 0, we can use & to knock
	 * those bits out of the changes. */
	for (s = 0; s < PIUIO_MULTIPLEX; s++) {
		if (s == cur_set)
			continue;
		for (i = 0; i < PIUIO_MSG_LONGS; i++)
			changed[i] &= piu->old[s][i];
	}

	/* Find and report any inputs which have changed state */
	for (i = 0; i < PIUIO_MSG_LONGS; i++) {
		/* As long as some bit is still set... */
		while (changed[i]) {
			/* find the index of the first set bit and clear it */
			b = __ffs(changed[i]);
			clear_bit(b, &changed[i]);
			/* and report the corresponding press or release. */
			report_key(piu->dev, i * BITS_PER_LONG + b,
					!test_bit(b, &piu->new[i]));
		}
	}

	/* Done reporting input events */
	input_sync(piu->dev);

resubmit:
	i = usb_submit_urb(urb, GFP_ATOMIC);
	if (i) {
		dev_err(&piu->dev->dev,
				"usb_submit_urb(new) failed, status %d", i);
	}

in_finished:
	/* Let any waiting threads know we're done here */
	wake_up(&piu->shutdown_wait);
}

static void piuio_out_completed(struct urb *urb)
{
	struct piuio *piu = urb->context;
	int ret = urb->status;

	switch (ret) {
		case 0:			/* success */
			break;
		case -ECONNRESET:	/* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
			goto out_finished;
		default:		/* error */
			dev_warn(&piu->dev->dev, "out urb status %d received\n",
					ret);
			break;
	}

/* The code below assumes that PIUIO_MULTIPLEX is 4.  It could be made more
 * general if anyone happens to have a setup where that isn't the case. */
#if PIUIO_MULTIPLEX != 4
#error The PIUIO driver only works when PIUIO_MULTIPLEX is 4.
#endif

	/* Switch to the next input set in rotation */
	piu->set = (piu->set + 1) % PIUIO_MULTIPLEX;

	/* Copy in the new lights and set multiplexer bits */
	memcpy(piu->lights, piu->new_lights, PIUIO_MSG_SZ);
	piu->lights[0] &= ~3;
	piu->lights[0] |= piu->set;
	piu->lights[2] &= ~3;
	piu->lights[2] |= piu->set;
	
	ret = usb_submit_urb(piu->out, GFP_ATOMIC);
	if (ret) {
		dev_err(&piu->dev->dev,
				"usb_submit_urb(lights) failed, status %d\n",
				ret);
	}

out_finished:
	/* Let any waiting threads know we're done here */
	wake_up(&piu->shutdown_wait);
}


/*
 * Input device events
 */
static int piuio_open(struct input_dev *dev)
{
	struct piuio *piu = input_get_drvdata(dev);

	/* Kick off the polling */
	if (usb_submit_urb(piu->out, GFP_KERNEL))
		return -EIO;
	if (usb_submit_urb(piu->in, GFP_KERNEL)) {
		usb_kill_urb(piu->out);
		return -EIO;
	}

	return 0;
}

static void piuio_close(struct input_dev *dev)
{
	struct piuio *piu = input_get_drvdata(dev);

	/* Stop polling, but wait for the last requests to complete */
	usb_block_urb(piu->in);
	usb_block_urb(piu->out);
	wait_event_timeout(piu->shutdown_wait,
			atomic_read(&piu->in->use_count) == 0 &&
			atomic_read(&piu->out->use_count) == 0,
			msecs_to_jiffies(5));
	usb_unblock_urb(piu->in);
	usb_unblock_urb(piu->out);

	/* XXX Kill the lights! */
	/* XXX Re-initialize parts of piuio struct */
}


/*
 * Structure initialization and destruction
 */
static void piuio_input_init(struct piuio *piu, struct device *parent)
{
	struct input_dev *dev = piu->dev;
	int i;

	/* Fill in basic fields */
	dev->name = "PIUIO input";
	dev->phys = piu->phys;
	usb_to_input_id(piu->usbdev, &dev->id);
	dev->dev.parent = parent;

	/* HACK: Buttons are sufficient to trigger a /dev/input/js* device, but
	 * for systemd (and consequently udev and Xorg) to consider us a
	 * joystick, we have to have a set of XY absolute axes. */
	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);

	/* Configure buttons */
	for (i = 0; i < PIUIO_INPUTS; i++)
		set_bit(keycode(i), dev->keybit);
	clear_bit(0, dev->keybit);

	/* Configure fake axes */
	set_bit(ABS_X, dev->absbit);
	set_bit(ABS_Y, dev->absbit);
	input_set_abs_params(dev, ABS_X, 0, 0, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, 0, 0, 0);

	/* Set device callbacks */
	dev->open = piuio_open;
	dev->close = piuio_close;

	/* Link input device back to PIUIO */
	input_set_drvdata(dev, piu);
}

static int piuio_init(struct piuio *piu, struct input_dev *dev,
		struct usb_device *usbdev)
{
	/* If this function returns an error, call piuio_destroy */
	if (!(piu->in = usb_alloc_urb(0, GFP_KERNEL)))
		return -ENOMEM;
	if (!(piu->out = usb_alloc_urb(0, GFP_KERNEL)))
		return -ENOMEM;

	init_waitqueue_head(&piu->shutdown_wait);

	piu->dev = dev;
	piu->usbdev = usbdev;
	usb_make_path(usbdev, piu->phys, sizeof(piu->phys));
	strlcat(piu->phys, "/input0", sizeof(piu->phys));

	/* Prepare URB for multiplexer and lights */
	piu->cr_out.bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	piu->cr_out.bRequest = cpu_to_le16(PIUIO_MSG_REQ);
	piu->cr_out.wValue = cpu_to_le16(PIUIO_MSG_VAL);
	piu->cr_out.wIndex = cpu_to_le16(PIUIO_MSG_IDX);
	piu->cr_out.wLength = cpu_to_le16(PIUIO_MSG_SZ);
	usb_fill_control_urb(piu->out, usbdev, usb_sndctrlpipe(usbdev, 0),
			(void *) &piu->cr_out, piu->lights, PIUIO_MSG_SZ,
			piuio_out_completed, piu);

	/* Prepare URB for inputs */
	piu->cr_in.bRequestType = USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	piu->cr_in.bRequest = cpu_to_le16(PIUIO_MSG_REQ);
	piu->cr_in.wValue = cpu_to_le16(PIUIO_MSG_VAL);
	piu->cr_in.wIndex = cpu_to_le16(PIUIO_MSG_IDX);
	piu->cr_in.wLength = cpu_to_le16(PIUIO_MSG_SZ);
	usb_fill_control_urb(piu->in, usbdev, usb_rcvctrlpipe(usbdev, 0),
			(void *) &piu->cr_in, piu->new, PIUIO_MSG_SZ,
			piuio_in_completed, piu);

	return 0;
}

static void piuio_destroy(struct piuio *piu)
{
	/* These handle NULL gracefully, so we can call this if init fails */
	usb_free_urb(piu->out);
	usb_free_urb(piu->in);
}


/*
 * USB connect and disconnect events
 */
static int piuio_probe(struct usb_interface *iface,
			 const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(iface);
	struct piuio *piu;
	struct input_dev *dev;
	int ret = -ENOMEM;

	/* Allocate PIUIO state and input device */
	piu = kzalloc(sizeof(struct piuio), GFP_KERNEL);
	if (!piu)
		return ret;

	dev = input_allocate_device();
	if (!dev) {
		kfree(piu);
		return ret;
	}

	/* Initialize PIUIO state and input device */
	ret = piuio_init(piu, dev, usbdev);
	if (ret)
		goto err;

	piuio_input_init(piu, &iface->dev);

	/* Register input device */
	ret = input_register_device(piu->dev);
	if (ret)
		goto err;

	/* Final USB setup */
	usb_set_intfdata(iface, piu);
	return 0;

err:
	piuio_destroy(piu);
	input_free_device(dev);
	kfree(piu);
	return ret;
}

static void piuio_disconnect(struct usb_interface *intf)
{
	struct piuio *piu = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	if (!piu) {
		dev_err(&intf->dev, "disconnecting non-existent PIUIO\n");
		return;
	}

	usb_kill_urb(piu->in);
	usb_kill_urb(piu->out);
	input_unregister_device(piu->dev);
	piuio_destroy(piu);
	kfree(piu);
}


/*
 * USB driver and module definitions
 */
static struct usb_device_id piuio_id_table [] = {
	/* Python WDM2 Encoder used for PIUIO boards */
	{ USB_DEVICE(USB_VENDOR_ID_ANCHOR, USB_PRODUCT_ID_PYTHON2) },
	{},
};

MODULE_DEVICE_TABLE(usb, piuio_id_table);

static struct usb_driver piuio_driver = {
	.name =		"piuio",
	.probe =	piuio_probe,
	.disconnect =	piuio_disconnect,
	.id_table =	piuio_id_table,
};

MODULE_AUTHOR("Devin J. Pohly");
MODULE_DESCRIPTION("PIUIO input/output driver");
MODULE_LICENSE("GPL");

module_usb_driver(piuio_driver);
