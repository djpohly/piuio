#ifndef PIUIO_LEGACY_H
#define PIUIO_LEGACY_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
#define module_usb_driver(__driver) \
static int __init __driver##_init(void) \
{ \
	return usb_register(&(__driver)); \
} \
module_init(__driver##_init); \
static void __exit __driver##_exit(void) \
{ \
	usb_deregister(&(__driver)); \
} \
module_exit(__driver##_exit);
#endif

#endif
