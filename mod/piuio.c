/*
 * PIUIO interface driver
 *
 * Copyright (C) 2012-2014 Devin J. Pohly (djpohly+linux@gmail.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This code is based on the USB skeleton driver by Greg Kroah-Hartman.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/usb/input.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/errno.h>


/* Module and driver info */
MODULE_AUTHOR("Devin J. Pohly");
MODULE_DESCRIPTION("PIUIO input/output driver");
MODULE_LICENSE("GPL");

/* Vendor/product ID table */
static const struct usb_device_id piuio_ids[] = {
	{ USB_DEVICE(0x0547, 0x1002) },
	{},
};
MODULE_DEVICE_TABLE(usb, piuio_ids);

/* Module parameters */
static int timeout_ms = 10;
module_param(timeout_ms, int, 0644);
MODULE_PARM_DESC(timeout_ms, "Timeout for PIUIO USB messages in ms"
		" (default 10)");

static bool batch_output = true;
module_param(batch_output, bool, 0644);
MODULE_PARM_DESC(batch_output, "Batch output messages with next input request"
		" (default true)");


/* Protocol-specific parameters */
#define PIUIO_MSG_REQ 0xae
#define PIUIO_MSG_VAL 0
#define PIUIO_MSG_IDX 0

/* Size of input and output packets */
#define PIUIO_NUM_INPUTS 64
#define PIUIO_INPUT_SZ ((PIUIO_NUM_INPUTS + 7) / 8)
#define PIUIO_NUM_OUTPUTS 64
#define PIUIO_OUTPUT_SZ ((PIUIO_NUM_OUTPUTS + 7) / 8)

/* XXX: The piuio_read code currently expects this to be 4.  Until we know more
 * about how the device works, it will have to stay that way. */
#define PIUIO_MULTIPLEX 4


/* Represents the current state of a device */
struct piuio_state {
	/* USB device and interface */
	struct usb_device *dev;	
	struct usb_interface *intf;

	/* Input device */
	struct input_polled_dev *ipdev;
	char input_phys[64];

	/* Protects intf and outputs */
	struct mutex lock;

	/* Refcount for state struct, incremented by probe and open, decremented
	 * by release and disconnect; needed because last release may happen
	 * after disconnect */
	struct kref kref;

	/* Current state of outputs */
	u8 outputs[PIUIO_OUTPUT_SZ];
};


/* Allocates driver state and initializes fields */
static struct piuio_state *state_create(struct usb_interface *intf)
{
	struct piuio_state *st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	/* Hold a reference to the USB device */
	st->dev = usb_get_dev(interface_to_usbdev(intf));
	st->intf = intf;

	/* Construct a physical path for the input device */
	usb_make_path(st->dev, st->input_phys, sizeof(st->input_phys));
	strlcat(st->input_phys, "/input0", sizeof(st->input_phys));

	/* Initialize synchro primitives */
	mutex_init(&st->lock);
	kref_init(&st->kref);

	return st;
}

/* Cleans up and frees driver state; opposite of state_create */
static void state_destroy(struct kref *kref)
{
	struct piuio_state *st = container_of(kref, struct piuio_state, kref);

	mutex_destroy(&st->lock);

	/* Drop our reference to the USB device */
	usb_put_dev(st->dev);
	kfree(st);
}


/* Forward-declared for use in functions */
static struct usb_driver piuio_driver;


/* Reading a packet from /dev/piuioN returns the state of all the sensors */
static ssize_t piuio_read(struct file *filp, char __user *ubuf, size_t sz,
		loff_t *pofs)
{
	struct piuio_state *st;
	char buf[PIUIO_INPUT_SZ * PIUIO_MULTIPLEX];
	int i;
	int rv = 0;

	if (sz != sizeof(buf))
		return -EINVAL;

	st = filp->private_data;

	mutex_lock(&st->lock);

	/* Error if the device has been disconnected */
	if (!st->intf) {
		rv = -ENODEV;
		goto out;
	}

	for (i = 0; i < PIUIO_MULTIPLEX; i++) {
		/* First select which set of inputs to get */
		st->outputs[0] = (st->outputs[0] & ~3) | i;
		st->outputs[2] = (st->outputs[2] & ~3) | i;
		rv = usb_control_msg(st->dev, usb_sndctrlpipe(st->dev, 0),
				PIUIO_MSG_REQ,
				USB_DIR_OUT|USB_TYPE_VENDOR|USB_RECIP_DEVICE,
				PIUIO_MSG_VAL, PIUIO_MSG_IDX,
				&st->outputs, sizeof(st->outputs), timeout_ms);
		if (rv < 0)
			break;

		/* Then request the status of the inputs */
		rv = usb_control_msg(st->dev, usb_rcvctrlpipe(st->dev, 0),
				PIUIO_MSG_REQ,
				USB_DIR_IN|USB_TYPE_VENDOR|USB_RECIP_DEVICE,
				PIUIO_MSG_VAL, PIUIO_MSG_IDX,
				&buf[i * PIUIO_INPUT_SZ], PIUIO_INPUT_SZ,
				timeout_ms);
		if (rv < 0)
			break;
	}

out:
	mutex_unlock(&st->lock);
	if (rv < 0)
		return rv;
	if (copy_to_user(ubuf, buf, sizeof(buf)))
		return -EFAULT;
	return sizeof(buf);
}

/* Performs the write after the buffer is copied to kernelspace */
static ssize_t do_piuio_write(struct file *filp)
{
	struct piuio_state *st = filp->private_data;
	int rv;

	/* Error if the device has been closed */
	if (!st->intf)
		return -ENODEV;

	/* Otherwise, update the lights right away */
	rv = usb_control_msg(st->dev, usb_sndctrlpipe(st->dev, 0),
			PIUIO_MSG_REQ,
			USB_DIR_OUT|USB_TYPE_VENDOR|USB_RECIP_DEVICE,
			PIUIO_MSG_VAL, PIUIO_MSG_IDX,
			&st->outputs, sizeof(st->outputs), timeout_ms);
	return rv ? rv : sizeof(st->outputs);
}

/* Writing a packet to /dev/piuioN controls the lights and other outputs */
static ssize_t piuio_write(struct file *filp, const char __user *ubuf,
		size_t sz, loff_t *pofs)
{
	struct piuio_state *st;
	unsigned char buf[PIUIO_OUTPUT_SZ];
	int rv;

	if (sz != sizeof(st->outputs))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, sizeof(buf)))
		return -EFAULT;

	st = filp->private_data;

	/* Save the desired outputs */
	memcpy(st->outputs, buf, sizeof(st->outputs));

	/* Batching with the next input request?  If so, return now. */
	if (batch_output)
		return 0;

	/* XXX Late lock - ignoring race conditions on st->outputs for now for
	 * performance reasons */
	mutex_lock(&st->lock);
	rv = do_piuio_write(filp);
	mutex_unlock(&st->lock);

	return rv;
}

/* Handles open() for /dev/piuioN */
static int piuio_open(struct inode *inode, struct file *filp)
{
	struct usb_interface *intf;
	struct piuio_state *st;
	int rv;

	/* Get the corresponding interface and state */
	intf = usb_find_interface(&piuio_driver, iminor(inode));
	if (!intf)
		return -ENODEV;
	st = usb_get_intfdata(intf);
	if (!st)
		return -ENODEV;

	/* Pick up a reference to the interface */
	kref_get(&st->kref);

	/* Ensure the device isn't suspended while in use */
	rv = usb_autopm_get_interface(intf);
	if (rv) {
		kref_put(&st->kref, state_destroy);
		return rv;
	}

	/* Attach our state to the file */
	filp->private_data = st;
	return 0;
}

/* Cleans up after the last close() on a descriptor for /dev/piuioN */
static int piuio_release(struct inode *inode, struct file *filp)
{
	struct piuio_state *st;

	st = filp->private_data;
	if (st == NULL)
		return -ENODEV;

	/* Reset lights */
	memset(st->outputs, 0, sizeof(st->outputs));
	do_piuio_write(filp);

	if (st->intf)
		usb_autopm_put_interface(st->intf);

	/* Drop reference */
	kref_put(&st->kref, state_destroy);
	return 0;
}

/* File operations for /dev/piuioN */
static const struct file_operations piuio_fops = {
	.owner =	THIS_MODULE,
	.read =		piuio_read,
	.write =	piuio_write,
	.open =		piuio_open,
	.release =	piuio_release,
};

/* Class driver, for creating device files */
static struct usb_class_driver piuio_class = {
	.name =		"piuio%d",
	.fops =		&piuio_fops,
};


/* Use the joystick buttons first, then the extra "trigger happy" range. */
static int keycode_for_pin(int pin)
{
	if (pin < 0x10)
		return BTN_JOYSTICK + pin;
	pin -= 0x10;
	if (pin < 0x40)
		return BTN_TRIGGER_HAPPY + pin;

	return KEY_RESERVED;
}

static void piuio_input_poll(struct input_polled_dev *ipdev)
{
	// XXX Actually poll the input here :)
}


/* Set up a device being connected to this driver */
static int piuio_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct piuio_state *st;
	struct input_polled_dev *ipdev;
	struct input_dev *idev;
	int i;
	int rv = -ENOMEM;

	/* Set up state structure */
	st = state_create(intf);
	if (!st) {
		dev_err(&intf->dev, "Failed to allocate state\n");
		return -ENOMEM;
	}

	/* Allocate and initialize the polled input device */
	ipdev = input_allocate_polled_device();
	if (!ipdev) {
		dev_err(&intf->dev, "Failed to allocate polled input device\n");
		goto err_allocating_ipdev;
	}
	st->ipdev = ipdev;

	ipdev->private = st;
	ipdev->poll = piuio_input_poll;
	/* XXX Testing value - reduce this! */
	ipdev->poll_interval = 1000;

	/* Initialize the underlying input device */
	idev = ipdev->input;
	idev->name = "PIUIO input";
	idev->phys = st->input_phys;
	usb_to_input_id(st->dev, &idev->id);
	idev->dev.parent = &intf->dev;

	/* Configure our buttons */
	set_bit(EV_KEY, idev->evbit);
	for (i = 0; i < PIUIO_NUM_INPUTS; i++)
		set_bit(keycode_for_pin(i), idev->keybit);

	/* HACK: Buttons are sufficient to trigger a /dev/input/js* device, but
	 * for systemd (and consequently udev and Xorg) to consider us a
	 * joystick, we have to have a set of XY absolute axes. */
	set_bit(EV_ABS, idev->evbit);
	set_bit(ABS_X, idev->absbit);
	set_bit(ABS_Y, idev->absbit);
	input_set_abs_params(idev, ABS_X, 0, 0, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 0, 0, 0);

	/* Register the polled input device */
	rv = input_register_polled_device(ipdev);
	if (rv) {
		dev_err(&intf->dev, "Failed to register polled input device\n");
		goto err_registering_input;
	}

	/* Register the USB device */
	usb_set_intfdata(intf, st);
	rv = usb_register_dev(intf, &piuio_class);
	if (rv) {
		dev_err(&intf->dev, "Failed to register USB device\n");
		goto err_registering_usb;
	}

	return rv;

err_registering_usb:
	usb_set_intfdata(intf, NULL);
	input_unregister_polled_device(ipdev);
err_registering_input:
	input_free_polled_device(ipdev);
err_allocating_ipdev:
	kref_put(&st->kref, state_destroy);
	return rv;
}

/* Clean up when a device is disconnected */
static void piuio_disconnect(struct usb_interface *intf)
{
	struct piuio_state *st = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	usb_deregister_dev(intf, &piuio_class);
	input_unregister_polled_device(st->ipdev);
	input_free_polled_device(st->ipdev);

	/* Signal to any stragglers that the device is gone */
	mutex_lock(&st->lock);
	st->intf = NULL;
	mutex_unlock(&st->lock);

	kref_put(&st->kref, state_destroy);
}

/* Device driver handlers */
static struct usb_driver piuio_driver = {
	.name =		"piuio",
	.probe =	piuio_probe,
	.disconnect =	piuio_disconnect,
	.id_table =	piuio_ids,
	.supports_autosuspend = 1,
};

/* This line requires kernel 3.3 or higher */
module_usb_driver(piuio_driver);
