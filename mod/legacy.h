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

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
static inline void usb_block_urb(struct urb *urb)
{
	if (!urb)
		return;
	atomic_inc(&urb->reject);
}
static inline void usb_unblock_urb(struct urb *urb)
{
	if (!urb)
		return;
	atomic_dec(&urb->reject);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
#define class_for_each_attr(__attr, __pattr, __agrp, __dattr, __cls) \
	for ((__dattr) = (__cls)->dev_attrs; ((__attr) = &(__dattr)->attr) && (__attr)->name; (__dattr)++)
#else
#define class_for_each_attr(__attr, __pattr, __agrp, __dattr, __cls) \
	for ((__agrp) = (__cls)->dev_groups; *(__agrp); (__agrp)++) \
		for ((__pattr) = (*(__agrp))->attrs; ((__attr) = *(__pattr)); (__pattr)++)
#endif

#endif
