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


/* Protocol-specific parameters */
#define PIUIO_MSG_REQ 0xae
#define PIUIO_MSG_VAL 0
#define PIUIO_MSG_IDX 0

/* Size of input and output packets */
#define PIUIO_NUM_INPUTS 64
#define PIUIO_INPUT_SZ ((PIUIO_NUM_INPUTS + 7) / 8)
#define PIUIO_NUM_OUTPUTS 64
#define PIUIO_OUTPUT_SZ ((PIUIO_NUM_OUTPUTS + 7) / 8)

/* XXX: The do_piuio_read code currently expects this to be 4.  Until we know
 * more about how the device works, it will have to stay that way. */
#define PIUIO_MULTIPLEX 4


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
MODULE_PARM_DESC(timeout_ms, "Timeout for PIUIO USB messages in ms");

static unsigned int poll_interval_ms = 3;
module_param(poll_interval_ms, uint, 0644);
MODULE_PARM_DESC(poll_interval_ms, "Input device polling interval");


/* Represents the current state of a device */
struct piuio_state {
	/* USB device and interface */
	struct usb_device *dev;	
	struct usb_interface *intf;

	/* Input device */
	struct input_polled_dev *ipdev;
	char input_phys[64];

	/* Protects intf, inputs, and outputs */
	struct mutex lock;

	/* Refcount for state struct, incremented by probe and open, decremented
	 * by release and disconnect; needed because last release may happen
	 * after disconnect */
	struct kref kref;

	/* Current state of inputs and outputs */
	u8 inputs[PIUIO_INPUT_SZ * PIUIO_MULTIPLEX];
	u8 last_inputs[PIUIO_INPUT_SZ * PIUIO_MULTIPLEX];
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


/* Perform the read in kernelspace.  Must be called with st->lock held. */
static ssize_t do_piuio_read(struct piuio_state *st, u8 *buf, int group)
{
	int rv;

	/* First select which set of inputs to get */
	st->outputs[0] = (st->outputs[0] & ~3) | group;
	st->outputs[2] = (st->outputs[2] & ~3) | group;
	rv = usb_control_msg(st->dev, usb_sndctrlpipe(st->dev, 0),
			PIUIO_MSG_REQ,
			USB_DIR_OUT|USB_TYPE_VENDOR|USB_RECIP_DEVICE,
			PIUIO_MSG_VAL, PIUIO_MSG_IDX,
			&st->outputs, sizeof(st->outputs), timeout_ms);
	if (rv < 0)
		return rv;

	/* Then request the status of the inputs */
	rv = usb_control_msg(st->dev, usb_rcvctrlpipe(st->dev, 0),
			PIUIO_MSG_REQ,
			USB_DIR_IN|USB_TYPE_VENDOR|USB_RECIP_DEVICE,
			PIUIO_MSG_VAL, PIUIO_MSG_IDX,
			buf, PIUIO_INPUT_SZ, timeout_ms);
	return rv;
}


/* Called when the input device is opened for polling */
static void piuio_input_open(struct input_polled_dev *ipdev)
{
	struct piuio_state *st = ipdev->private;
	int rv;

	/* Pick up a reference to the interface */
	kref_get(&st->kref);

	/* Ensure the device isn't suspended while in use */
	rv = usb_autopm_get_interface(st->intf);
	if (rv) {
		dev_err(&st->intf->dev, "couldn't get autopm reference\n");
		kref_put(&st->kref, state_destroy);
		return;
	}
}

/* Called when polling stops on the input device */
static void piuio_input_close(struct input_polled_dev *ipdev)
{
	struct piuio_state *st = ipdev->private;

	if (st->intf)
		usb_autopm_put_interface(st->intf);

	/* Drop reference */
	kref_put(&st->kref, state_destroy);
}

/* Use the joystick buttons first, then the extra "trigger happy" range. */
static int keycode_for_pin(int pin)
{
	if (pin < 16)
		return BTN_JOYSTICK + pin;
	pin -= 0x10;
	if (pin < 48)
		return BTN_TRIGGER_HAPPY + pin;

	return KEY_RESERVED;
}

/* Submit a keypress to the input subsystem.  Remember to sync after this. */
static void report_key(struct input_polled_dev *ipdev, int pin, int release)
{
	struct input_dev *input = ipdev->input;
	int code = keycode_for_pin(pin);

	if (code == KEY_RESERVED)
		return;

	input_event(input, EV_MSC, MSC_SCAN, pin + 1);
	input_report_key(input, keycode_for_pin(pin), !release);
}

/* Update the device state and generate input events based on the changes */
static void piuio_input_poll(struct input_polled_dev *ipdev)
{
	struct piuio_state *st = ipdev->private;
	u8 inputs[PIUIO_INPUT_SZ];
	u8 changed[PIUIO_INPUT_SZ];
	int i, j;
	int update;
	int rv;

	mutex_lock(&st->lock);

	/* Error if the device has been disconnected */
	if (!st->intf) {
		mutex_unlock(&st->lock);
		dev_warn(&st->intf->dev, "poll after device disconnected\n");
		return;
	}

	/* Keep these around for later reference */
	memcpy(st->last_inputs, st->inputs, sizeof(st->last_inputs));

	/* Poll the device */
	for (i = 0; i < PIUIO_MULTIPLEX; i++) {
		rv = do_piuio_read(st, &st->inputs[i * PIUIO_INPUT_SZ], i);
		if (rv < 0) {
			mutex_unlock(&st->lock);
			dev_err(&st->intf->dev, "read failed in poll: %d\n", rv);
			return;
		}
	}

	/* Consolidate the inputs (0 means pressed, so we AND them) */
	memcpy(inputs, st->inputs, PIUIO_INPUT_SZ);
	memcpy(changed, st->last_inputs, PIUIO_INPUT_SZ);
	for (i = 1; i < PIUIO_MULTIPLEX; i++)
		for (j = 0; j < PIUIO_INPUT_SZ; j++) {
			inputs[j] &= st->inputs[i * PIUIO_INPUT_SZ + j];
			changed[j] &= st->last_inputs[i * PIUIO_INPUT_SZ + j];
		}

	/* Done with st->inputs/last_inputs */
	mutex_unlock(&st->lock);

	/* Find the inputs which have changed state and report them */
	update = 0;
	for (i = 0; i < PIUIO_INPUT_SZ; i++) {
		changed[i] ^= inputs[i];
		for (j = 0; j < 8 * sizeof(*changed); j++) {
			if (changed[i] & (1 << j)) {
				update = 1;
				report_key(ipdev, i * 8 + j, inputs[i] & (1 << j));
			}
		}
	}

	/* If we reported anything, flush our input events */
	if (update)
		input_sync(ipdev->input);
}

static void setup_input_device(struct input_dev *idev, struct piuio_state *st)
{
	int i;

	/* Fill in basic fields */
	idev->name = "PIUIO input";
	idev->phys = st->input_phys;
	usb_to_input_id(st->dev, &idev->id);
	idev->dev.parent = &st->intf->dev;

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
}


/* Set up a device being connected to this driver */
static int piuio_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct piuio_state *st;
	struct input_polled_dev *ipdev;
	int rv = -ENOMEM;

	/* Set up state structure and save a handle in the USB interface */
	st = state_create(intf);
	if (!st) {
		dev_err(&intf->dev, "failed to allocate state\n");
		return -ENOMEM;
	}
	usb_set_intfdata(intf, st);

	/* Allocate and initialize the polled input device */
	ipdev = input_allocate_polled_device();
	if (!ipdev) {
		dev_err(&intf->dev, "failed to allocate polled input device\n");
		goto err_allocating_ipdev;
	}
	st->ipdev = ipdev;

	ipdev->private = st;
	ipdev->poll = piuio_input_poll;
	ipdev->open = piuio_input_open;
	ipdev->close = piuio_input_close;
	ipdev->poll_interval = poll_interval_ms;

	/* Set up the underlying input device */
	setup_input_device(ipdev->input, st);

	/* Register the polled input device */
	rv = input_register_polled_device(ipdev);
	if (rv) {
		dev_err(&intf->dev, "failed to register polled input device\n");
		goto err_registering_input;
	}

	return rv;

err_registering_input:
	input_free_polled_device(ipdev);
	usb_set_intfdata(intf, NULL);
err_allocating_ipdev:
	st->intf = NULL;
	kref_put(&st->kref, state_destroy);
	return rv;
}

/* Clean up when a device is disconnected */
static void piuio_disconnect(struct usb_interface *intf)
{
	struct piuio_state *st = usb_get_intfdata(intf);

	input_unregister_polled_device(st->ipdev);
	input_free_polled_device(st->ipdev);
	usb_set_intfdata(intf, NULL);

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
