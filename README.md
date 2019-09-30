PIUIO input driver for Linux
============================

This is a driver for the PIUIO arcade I/O board that maps panels and buttons to
a standard Linux event interface which typically appears as a joystick.


Compiling and installing
------------------------

To compile the kernel module, run `make` inside the `mod` directory.  This will
build a kernel object file `piuio.ko` which is compatible with your currently
installed kernel.  To build against different kernel headers, you may specify
the path to a source tree using the `KDIR` variable when building, e.g.:

    make KDIR=~/src/linux-3.15.7

To install this module once you have built it, use the `make install` command.
By default this will attempt to install the module directly to your system;
note that doing so will require root privileges.  To specify a path under
which to install (e.g. for packaging), you can specify a `DESTDIR`:

    make DESTDIR="$pkgdir" install

Note: during this step, you may see the following error:

    At main.c:160:
    - SSL error:02001002:system library:fopen:No such file or directory: crypto/bio/bss_file.c:72
    - SSL error:2006D080:BIO routines:BIO_new_file:no such file: crypto/bio/bss_file.c:79
    sign-file: certs/signing_key.pem: No such file or directory

Unless you are trying to build a cryptographically signed module, you may
safely ignore these messages.


Tools for testing
-----------------

Since this module uses the input subsystem, you can use standard tools such as
[evemu](http://cgit.freedesktop.org/evemu/) to test it.  Outputs are
implemented using the leds subsystem and are consequently located in the
`/sys/class/leds` directory.  You can test these by echoing zero/nonzero values
to the `brightness` file in a given led directory:

    echo 1 > /sys/class/leds/piuio::output3/brightness


Troubleshooting
---------------

### PIUIO does not load on boot ###

If PIUIO was installed via `make` as in the installation section, it should
automatically be loaded on boot. PIUIO unloads if it cannot communicate with the
arcade I/O board, so its absence may be a sign that something is wrong there
(it's not connected to the computer, there's an issue with the power supply,
etc.)  You can load PIUIO after boot with `insmod` or `modprobe`:

    insmod /path/to/piuio.ko

Implementation and accuracy
---------------------------

This driver is designed to provide the fastest possible response to
inputs, and thereby the best possible timing accuracy.  Below are some of
the considerations made in reducing the time from triggering an input
sensor to generating the input event.


### Polled input devices ###

The Linux kernel provides the `input_polldev` framework for input devices
such as the PIUIO which must be polled rather than generating interrupts.
This framework allows the driver to define a `poll` function which is
called at regular intervals to generate input events.

Unfortunately, this framework is designed for general input devices, such
as keyboards, where accurate event timing is not critical.  As a result
the mechanics behind its implementation can lead to extra time between
polls.  This driver implements polling itself to avoid the extra delay
(which is compounded by the use of a multiplexer as described below).


### USB protocol and multiplexer ###

PIUIO units are typically attached to a four-way multiplexer so that four
sensors can be connected to each input.  If any one sensor is triggered, the
input is considered to be pressed.  The current implementation assumes that
such a multiplexer is in use.  At a low level, the driver must send a USB
output message to select one set of sensors from the multiplexer, then an
input message to read their values.

However, the USB standard requires that each message consist of a request
from the host and a response from the device, and only one of those may
carry data.  Because of this, the output message to set up the multiplexer
and the input message to request the sensor state cannot be combined in
one round-trip from host to device and back; two round-trips are
necessary.

To get the best possible timing accuracy, these internal mechanics have to
be taken into account.


### Staggered input reporting ###

The first accuracy improvement that this driver makes is taking a different
approach with respect to when input events are reported.  Previous drivers,
as well as previous versions of this driver, have used a "block" strategy
when polling multiple sets of sensors.  With this approach, each set is
polled once, the results are combined, and then any changes in the input are
reported as press or release events.

Remember that polling one set of sensors requires two USB messages.  This
means that, in a typical setup with four sensor sets, eight round-trips take
place between consecutive polls of the same sensor.  So the delay between a
change in the state of any given sensor and the corresponding input event
will range from 0 to 8 RTTs, with an average of 4.

This driver instead staggers the input reporting.  After each set of sensors
is polled, input events are generated, using the most recent polled values
from the other sets.

At first, this does not seem to change anything, since there are still eight
round-trips between consecutive polls of a single sensor.  However, consider
the case in which more than one related sensor is triggered at the same time
(for example, if there are multiple sensors under a single panel).  The time
from trigger to input event will be that of the sensor which will be polled
next, i.e. the one with the least delay.

With the block strategy, the average delay is always 4 RTTs, even if
multiple sensors are triggered.  With the staggered strategy, hitting one
sensor will average 4 RTTs, two sensors 2.33 RTTs, three sensors 1.5 RTTs,
and hitting all four related sensors will bring the average delay to 1 RTT.

Given that the typical round-trip response time for a PIUIO device is 250
μs, this means this is the first PIUIO driver capable of sub-millisecond
accuracy on average.


### Message queueing ###

However, there is something else that can be done to reduce delay and
improve accuracy.  USB messages can be queued asynchronously so that they
are sent as quickly as possible, one after the other.  There is no need to
wait for an output message to complete before sending the following input
message, and vice versa.  This means that one response can be processed
while the next request is already being sent, using the USB link much more
efficiently.  In testing, the asynchronous driver was able to handle
requests 35–40% faster than synchronous drivers.
