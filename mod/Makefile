ifneq ($(KERNELRELEASE),)

# Kbuild configuration
obj-m := piuio.o

else

# Default to building for the currently running kernel
KDIR ?= /lib/modules/$(shell uname -r)/build

# If DESTDIR is given, output there
INSTALL_MOD_PATH ?= $(DESTDIR)
export INSTALL_MOD_PATH

all: modules ;

install: all modules_install ;

%:
	make -C $(KDIR) M=$(PWD) $@

endif
