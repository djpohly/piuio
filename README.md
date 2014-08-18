PIUIO input driver for Linux
============================

This is a driver for the PIUIO arcade I/O board that maps panels and buttons to
a standard Linux event interface which typically appears as a joystick.  It is
coded assuming the default configuration where four sets of inputs are attached
to a multiplexer.


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


Tools
-----

Since this module uses the input subsystem, you can use standard tools such as
[evemu](http://cgit.freedesktop.org/evemu/) to test it.  Outputs are currently
not implemented in the driver.


How it works
------------

### USB protocol ###

The PIUIO device uses USB control messages to communicate with a driver: one
message to set the outputs, and another to retrieve the inputs.  However,
every USB control message consists of a request and a response, only one of
which may carry data.  So we cannot combine the output and input messages
into a single round-trip to and from the device; two round-trips are required.


### Multiplexer ###

PIUIO units are typically attached to a four-way multiplexer so that four
sensors can be connected to each input.  If any one sensor is triggered, the
input is considered to be pressed.  The current implementation assumes that
such a multiplexer is in use.

At a low level, the driver must use an output message to select one set of
sensors from the multiplexer, then an input message to read their values.
The reason this is significant is that the PIUIO card is a USB device.
Every USB message consists of a request from the host and a response from the
device, only one of which may carry data.  Therefore sending an output
message to set up the multiplexer followed by an input message to request the
sensor state will require *two* round-trips from host to device.

To get the best possible timing accuracy (i.e. the least amount of delay
between triggering a sensor and registering the input event), these internal
mechanics have to be taken into account.


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
