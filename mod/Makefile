ifneq ($(PATCHLEVEL),)

# Kbuild configuration
obj-m := piuio.o

else

# Default to building for the currently running kernel
KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

# If DESTDIR is given, output there
INSTALL_MOD_PATH ?= $(DESTDIR)
export INSTALL_MOD_PATH

# This would otherwise default to "extra", which isn't in the depmod
# configuration on either Arch or Ubuntu.  Both have "updates" though.  You can
# override by setting this when running make install.
INSTALL_MOD_DIR ?= updates
export INSTALL_MOD_DIR

all: modules ;

install: modules_install ;

%:
	$(MAKE) -C $(KDIR) M=$(shell pwd) $@

endif
