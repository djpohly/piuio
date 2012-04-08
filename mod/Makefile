obj-m += piuio.o

modules:

%:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) $@
