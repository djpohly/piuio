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
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <linux/leds.h>
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
#define USB_VENDOR_ID_BTNBOARD 0x0d2f
#define USB_PRODUCT_ID_BTNBOARD 0x1010

/* USB message used to communicate with the device */
#define PIUIO_MSG_REQ 0xae
#define PIUIO_MSG_VAL 0
#define PIUIO_MSG_IDX 0

#define PIUIO_MSG_SZ 8
#define PIUIO_MSG_LONGS (PIUIO_MSG_SZ / sizeof(unsigned long))

/* Number of usable inputs and outputs */
#define PIUIO_INPUTS 48
#define PIUIO_OUTPUTS 48

/* Number of sets of inputs multiplexed together */
#define PIUIO_MULTIPLEX 4


/**
 * struct piuio_led - auxiliary struct for led devices
 * @piu:	Pointer back to the enclosing structure
 * @dev:	Actual led device
 */
struct piuio_led {
	struct piuio *piu;
	struct led_classdev dev;
};

/**
 * struct piuio - state of each attached PIUIO
 * @idev:	Input device associated with this PIUIO
 * @phys:	Physical path of the device. @idev's phys field points to this
 *		buffer
 * @udev:	USB device associated with this PIUIO
 * @in:		URB for requesting the current state of one set of inputs
 * @out:	URB for sending data to outputs and multiplexer
 * @cr_in:	Setup packet for @in URB
 * @cr_out:	Setup packet for @out URB
 * @old_inputs:	Previous state of input pins from the @in URB for each of the
 *		input sets.  These are used to determine when a press or release
 *		has happened for a group of correlated inputs.
 * @inputs:	Buffer for the @in URB
 * @outputs:	Buffer for the @out URB
 * @new_outputs:
 * 		Staging for the @outputs buffer
 * @set:	Current set of inputs to read (0 .. PIUIO_MULTIPLEX - 1) or -1
 *              to disable multiplexing
 */
struct piuio {
	struct input_dev *idev;
	char phys[64];

	struct usb_device *udev;
	struct urb *in, *out;
	struct usb_ctrlrequest cr_in, cr_out;
	wait_queue_head_t shutdown_wait;

	unsigned long old_inputs[PIUIO_MULTIPLEX][PIUIO_MSG_LONGS];
	unsigned long inputs[PIUIO_MSG_LONGS];
	unsigned char outputs[PIUIO_MSG_SZ];
	unsigned char new_outputs[PIUIO_MSG_SZ];

	struct piuio_led led[PIUIO_OUTPUTS];

	int set;
};

static const char *led_names[] = {
	"piuio::output0",
	"piuio::output1",
	"piuio::output2",
	"piuio::output3",
	"piuio::output4",
	"piuio::output5",
	"piuio::output6",
	"piuio::output7",
	"piuio::output8",
	"piuio::output9",
	"piuio::output10",
	"piuio::output11",
	"piuio::output12",
	"piuio::output13",
	"piuio::output14",
	"piuio::output15",
	"piuio::output16",
	"piuio::output17",
	"piuio::output18",
	"piuio::output19",
	"piuio::output20",
	"piuio::output21",
	"piuio::output22",
	"piuio::output23",
	"piuio::output24",
	"piuio::output25",
	"piuio::output26",
	"piuio::output27",
	"piuio::output28",
	"piuio::output29",
	"piuio::output30",
	"piuio::output31",
	"piuio::output32",
	"piuio::output33",
	"piuio::output34",
	"piuio::output35",
	"piuio::output36",
	"piuio::output37",
	"piuio::output38",
	"piuio::output39",
	"piuio::output40",
	"piuio::output41",
	"piuio::output42",
	"piuio::output43",
	"piuio::output44",
	"piuio::output45",
	"piuio::output46",
	"piuio::output47",
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

static void report_key(struct input_dev *idev, unsigned int pin, int press)
{
	int code = keycode(pin);

	if (code == KEY_RESERVED)
		return;

	input_event(idev, EV_MSC, MSC_SCAN, pin + 1);
	input_report_key(idev, keycode(pin), press);
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
	int ret = urb->status;

	if (ret) {
		dev_warn(&piu->udev->dev, "piuio callback(in): error %d\n", ret);
		goto resubmit;
	}

	/* Get the index of the previous input set (always 0 if no multiplexer) */
	cur_set = piu->set < 0 ? 0 :
		(piu->set + PIUIO_MULTIPLEX - 1) % PIUIO_MULTIPLEX;

	/* Note what has changed in this input set, then store the inputs for
	 * next time */
	for (i = 0; i < PIUIO_MSG_LONGS; i++) {
		changed[i] = piu->inputs[i] ^ piu->old_inputs[cur_set][i];
		piu->old_inputs[cur_set][i] = piu->inputs[i];
	}

	/* If we are using a multiplexer, changes only count when none of the
	 * corresponding inputs in other sets are pressed.  Since "pressed"
	 * reads as 0, we can use & to knock those bits out of the changes. */
	if (piu->set >= 0) {
		for (s = 0; s < PIUIO_MULTIPLEX; s++) {
			if (s == cur_set)
				continue;
			for (i = 0; i < PIUIO_MSG_LONGS; i++)
				changed[i] &= piu->old_inputs[s][i];
		}
	}

	/* For each input which has changed state, report whether it was pressed
	 * or released based on the current value. */
	for_each_set_bit(b, changed, PIUIO_INPUTS)
		report_key(piu->idev, b, !test_bit(b, piu->inputs));

	/* Done reporting input events */
	input_sync(piu->idev);

resubmit:
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret == -EPERM)
		dev_info(&piu->udev->dev, "piuio resubmit(in): shutdown\n");
	else if (ret)
		dev_err(&piu->udev->dev, "piuio resubmit(in): error %d\n", ret);

	/* Let any waiting threads know we're done here */
	wake_up(&piu->shutdown_wait);
}

static void piuio_out_completed(struct urb *urb)
{
	struct piuio *piu = urb->context;
	int ret = urb->status;

	if (ret) {
		dev_warn(&piu->udev->dev, "piuio callback(out): error %d\n", ret);
		goto resubmit;
	}

	/* Copy in the new outputs */
	memcpy(piu->outputs, piu->new_outputs, PIUIO_MSG_SZ);

	/* If we have a multiplexer, switch to the next input set in rotation
	 * and set the appropriate output bits */
	if (piu->set >= 0) {
		piu->set = (piu->set + 1) % PIUIO_MULTIPLEX;

/* The code below assumes that PIUIO_MULTIPLEX is 4.  It could be made more
 * general if anyone happens to have a setup where that isn't the case. */
#if PIUIO_MULTIPLEX != 4
#error The PIUIO driver only works when PIUIO_MULTIPLEX is 4.
#endif

		/* Set multiplexer bits */
		piu->outputs[0] &= ~3;
		piu->outputs[0] |= piu->set;
		piu->outputs[2] &= ~3;
		piu->outputs[2] |= piu->set;
	}
	
resubmit:
	ret = usb_submit_urb(piu->out, GFP_ATOMIC);
	if (ret == -EPERM)
		dev_info(&piu->udev->dev, "piuio resubmit(out): shutdown\n");
	else if (ret)
		dev_err(&piu->udev->dev, "piuio resubmit(out): error %d\n", ret);

	/* Let any waiting threads know we're done here */
	wake_up(&piu->shutdown_wait);
}


/*
 * Input device events
 */
static int piuio_open(struct input_dev *idev)
{
	struct piuio *piu = input_get_drvdata(idev);
	int ret;

	/* Kick off the polling */
	ret = usb_submit_urb(piu->out, GFP_KERNEL);
	if (ret) {
		dev_err(&piu->udev->dev, "piuio submit(out): error %d\n", ret);
		return -EIO;
	}

	ret = usb_submit_urb(piu->in, GFP_KERNEL);
	if (ret) {
		dev_err(&piu->udev->dev, "piuio submit(in): error %d\n", ret);
		usb_kill_urb(piu->out);
		return -EIO;
	}

	return 0;
}

static void piuio_close(struct input_dev *idev)
{
	struct piuio *piu = input_get_drvdata(idev);
	long remaining;

	/* Stop polling, but wait for the last requests to complete */
	usb_block_urb(piu->in);
	usb_block_urb(piu->out);
	remaining = wait_event_timeout(piu->shutdown_wait,
			atomic_read(&piu->in->use_count) == 0 &&
			atomic_read(&piu->out->use_count) == 0,
			msecs_to_jiffies(5));
	usb_unblock_urb(piu->in);
	usb_unblock_urb(piu->out);

	if (!remaining) {
		// Timed out
		dev_warn(&piu->udev->dev, "piuio close: urb timeout\n");
		usb_kill_urb(piu->in);
		usb_kill_urb(piu->out);
	}

	/* XXX Reset the outputs? */
}


/*
 * Led device event
 */
static void piuio_led_set(struct led_classdev *dev, enum led_brightness b)
{
	struct piuio_led *led = container_of(dev, struct piuio_led, dev);
	struct piuio *piu = led->piu;
	int n;

	n = led - piu->led;
	if (n > PIUIO_OUTPUTS) {
		dev_err(&piu->udev->dev, "piuio led: bad number %d\n", n);
		return;
	}

	/* Meh, forget atomicity, these aren't super-important */
	if (b)
		__set_bit(n, (unsigned long *) piu->new_outputs);
	else
		__clear_bit(n, (unsigned long *) piu->new_outputs);
}


/*
 * Structure initialization and destruction
 */
static void piuio_input_init(struct piuio *piu, struct device *parent)
{
	struct input_dev *idev = piu->idev;
	int i;

	/* Fill in basic fields */
	idev->name = "PIUIO input";
	idev->phys = piu->phys;
	usb_to_input_id(piu->udev, &idev->id);
	idev->dev.parent = parent;

	/* HACK: Buttons are sufficient to trigger a /dev/input/js* device, but
	 * for systemd (and consequently udev and Xorg) to consider us a
	 * joystick, we have to have a set of XY absolute axes. */
	set_bit(EV_KEY, idev->evbit);
	set_bit(EV_ABS, idev->evbit);

	/* Configure buttons */
	for (i = 0; i < PIUIO_INPUTS; i++)
		set_bit(keycode(i), idev->keybit);
	clear_bit(0, idev->keybit);

	/* Configure fake axes */
	set_bit(ABS_X, idev->absbit);
	set_bit(ABS_Y, idev->absbit);
	input_set_abs_params(idev, ABS_X, 0, 0, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 0, 0, 0);

	/* Set device callbacks */
	idev->open = piuio_open;
	idev->close = piuio_close;

	/* Link input device back to PIUIO */
	input_set_drvdata(idev, piu);
}

static int piuio_leds_init(struct piuio *piu)
{
	int i;
	const struct attribute_group **ag;
	struct attribute **attr;
	int ret;

	for (i = 0; i < PIUIO_OUTPUTS; i++) {
		/* Initialize led device and point back to piuio struct */
		piu->led[i].dev.name = led_names[i];
		piu->led[i].dev.brightness_set = piuio_led_set;
		piu->led[i].piu = piu;

		/* Register led device */
		ret = led_classdev_register(&piu->udev->dev, &piu->led[i].dev);
		if (ret)
			goto out_unregister;

		/* Relax permissions on led attributes */
		for (ag = piu->led[i].dev.dev->class->dev_groups; *ag; ag++) {
			for (attr = (*ag)->attrs; *attr; attr++) {
				ret = sysfs_chmod_file(&piu->led[i].dev.dev->kobj,
						*attr, S_IRUGO | S_IWUGO);
				if (ret) {
					led_classdev_unregister(&piu->led[i].dev);
					goto out_unregister;
				}
			}
		}
	}

	return 0;

out_unregister:
	for (--i; i >= 0; i--)
		led_classdev_unregister(&piu->led[i].dev);
	return ret;
}

static void piuio_leds_destroy(struct piuio *piu)
{
	int i;
	for (i = 0; i < PIUIO_OUTPUTS; i++)
		led_classdev_unregister(&piu->led[i].dev);
}

static int piuio_init(struct piuio *piu, struct input_dev *idev,
		struct usb_device *udev)
{
	/* If this function returns an error, piuio_destroy will be called */
	piu->in = usb_alloc_urb(0, GFP_KERNEL);
	piu->out = usb_alloc_urb(0, GFP_KERNEL);
	if (!piu->in || !piu->out) {
		dev_err(&udev->dev, "piuio init: failed to allocate URBs\n");
		return -ENOMEM;
	}

	init_waitqueue_head(&piu->shutdown_wait);

	piu->idev = idev;
	piu->udev = udev;
	usb_make_path(udev, piu->phys, sizeof(piu->phys));
	strlcat(piu->phys, "/input0", sizeof(piu->phys));

	/* Prepare URB for multiplexer and outputs */
	piu->cr_out.bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	piu->cr_out.bRequest = cpu_to_le16(PIUIO_MSG_REQ);
	piu->cr_out.wValue = cpu_to_le16(PIUIO_MSG_VAL);
	piu->cr_out.wIndex = cpu_to_le16(PIUIO_MSG_IDX);
	piu->cr_out.wLength = cpu_to_le16(PIUIO_MSG_SZ);
	usb_fill_control_urb(piu->out, udev, usb_sndctrlpipe(udev, 0),
			(void *) &piu->cr_out, piu->outputs, PIUIO_MSG_SZ,
			piuio_out_completed, piu);

	/* Prepare URB for inputs */
	piu->cr_in.bRequestType = USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	piu->cr_in.bRequest = cpu_to_le16(PIUIO_MSG_REQ);
	piu->cr_in.wValue = cpu_to_le16(PIUIO_MSG_VAL);
	piu->cr_in.wIndex = cpu_to_le16(PIUIO_MSG_IDX);
	piu->cr_in.wLength = cpu_to_le16(PIUIO_MSG_SZ);
	usb_fill_control_urb(piu->in, udev, usb_rcvctrlpipe(udev, 0),
			(void *) &piu->cr_in, piu->inputs, PIUIO_MSG_SZ,
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
static int piuio_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct piuio *piu;
	struct usb_device *udev = interface_to_usbdev(intf);
	struct input_dev *idev;
	int ret = -ENOMEM;

	/* Allocate PIUIO state and input device */
	piu = kzalloc(sizeof(struct piuio), GFP_KERNEL);
	if (!piu) {
		dev_err(&intf->dev, "piuio probe: failed to allocate state\n");
		return ret;
	}

	idev = input_allocate_device();
	if (!idev) {
		dev_err(&intf->dev, "piuio probe: failed to allocate input dev\n");
		kfree(piu);
		return ret;
	}

	/* Initialize PIUIO state and input device */
	ret = piuio_init(piu, idev, udev);
	if (ret)
		goto err;

	if (id->idVendor == USB_VENDOR_ID_BTNBOARD &&
			id->idProduct == USB_PRODUCT_ID_BTNBOARD) {
		/* Disable multiplexing */
		piu->set = -1;
	}

	piuio_input_init(piu, &intf->dev);

	/* Initialize and register led devices */
	ret = piuio_leds_init(piu);
	if (ret)
		goto err;

	/* Register input device */
	ret = input_register_device(piu->idev);
	if (ret) {
		dev_err(&intf->dev, "piuio probe: failed to register input dev\n");
		piuio_leds_destroy(piu);
		goto err;
	}

	/* Final USB setup */
	usb_set_intfdata(intf, piu);
	return 0;

err:
	piuio_destroy(piu);
	input_free_device(idev);
	kfree(piu);
	return ret;
}

static void piuio_disconnect(struct usb_interface *intf)
{
	struct piuio *piu = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	if (!piu) {
		dev_err(&intf->dev, "piuio disconnect: uninitialized device?\n");
		return;
	}

	usb_kill_urb(piu->in);
	usb_kill_urb(piu->out);
	piuio_leds_destroy(piu);
	input_unregister_device(piu->idev);
	piuio_destroy(piu);
	kfree(piu);
}


/*
 * USB driver and module definitions
 */
static struct usb_device_id piuio_id_table[] = {
	/* Python WDM2 Encoder used for PIUIO boards */
	{ USB_DEVICE(USB_VENDOR_ID_ANCHOR, USB_PRODUCT_ID_PYTHON2) },
	/* Special USB ID for button board devices */
	{ USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD) },
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
MODULE_VERSION("0.3.1");
MODULE_LICENSE("GPL");

module_usb_driver(piuio_driver);
