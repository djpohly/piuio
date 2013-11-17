PIUIO driver for Linux
======================

This is a simple device driver for the PIUIO arcade input board that allows for
reading inputs and controlling outputs from userspace programs.  It is coded
assuming the default configuration where four sets of inputs are attached to a
multiplexer, and it fetches all four sets for a single read operation.  (In some
circles, this is referred to as the "r16 kernel hack.")


Module parameters
-----------------

There are two parameters for this module:
* `timeout_ms`: Milliseconds to wait for a USB message to complete before timing
  out and returning an error.  Default is 10.
* `batch_output`: Batch output messages with the next input request rather than
  sending them immediately.  Since the multiplexer requires outputs to be sent
  when requesting input, this can increase performance when input is frequently
  polled anyway.  Default is true (batching is on).


Compiling and installing
------------------------

To compile the kernel module, run `make` inside the `mod` directory.  This
should build a kernel object file `piuio.ko` against your currently installed
kernel.  To install this module, place it in your distribution's directory for
kernel module updates.  This often looks lomething like:
    /lib/modules/your-kernel-version/extra
    /lib/modules/your-kernel-version/updates
or similar.


PIU IO Internal Work
------------------------

There is a document called `INNER_WORKINGS.txt` that tells exactly how the PIUIO Works/


Tools
-----

Two tools are provided for testing I/O through this device.  These tools are
found in the `tools` directory.

* `intest`: Display input live from the I/O card.
* `outtest`: Read any of the characters [RLDUrldu1234B] from standard input and
  toggle the corresponding outputs.  (Note that this will not happen immediately
  unless you turn the `batch_output` option off.
* `pump.h`: The bit masks for PIUIO Inputs and Outputs
