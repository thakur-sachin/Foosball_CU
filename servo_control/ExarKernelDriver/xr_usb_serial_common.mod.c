#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xc8d01d53, "module_layout" },
	{ 0xe180ce57, "usb_deregister" },
	{ 0x6d40fcba, "tty_driver_kref_put" },
	{ 0x2f3a5006, "tty_unregister_driver" },
	{ 0x942410fb, "usb_register_driver" },
	{ 0x8887ba02, "tty_register_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x7ba0cb41, "__tty_alloc_driver" },
	{ 0xac7286b9, "gpiochip_add_data_with_key" },
	{ 0xc365003e, "tty_port_register_device" },
	{ 0xb1e6b6e1, "usb_get_intf" },
	{ 0x9cfbb611, "usb_driver_claim_interface" },
	{ 0xfdff3e35, "_dev_info" },
	{ 0x19665244, "_dev_warn" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x30d4ff22, "device_create_file" },
	{ 0x437b7175, "usb_alloc_urb" },
	{ 0xb5571d08, "usb_alloc_coherent" },
	{ 0x1f00d418, "tty_port_init" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x2aff387f, "usb_ifnum_to_if" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xb2fd5ceb, "__put_user_4" },
	{ 0xc6cbbc89, "capable" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x6729d3df, "__get_user_4" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0x545cbc05, "tty_port_open" },
	{ 0x2a8ca7d2, "usb_autopm_put_interface" },
	{ 0xe9de6a72, "usb_autopm_get_interface" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x92997ed8, "_printk" },
	{ 0xaf88e69b, "kmem_cache_alloc_trace" },
	{ 0xde310d05, "kmalloc_caches" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x71038ac7, "pv_ops" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0xe4defd1f, "tty_insert_flip_string_fixed_flag" },
	{ 0x507e74e3, "tty_flip_buffer_push" },
	{ 0xc0960fd3, "__tty_insert_flip_char" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x8ee5488b, "tty_standard_install" },
	{ 0xf621d045, "usb_driver_release_interface" },
	{ 0x2f886acb, "usb_free_urb" },
	{ 0x1da20e2b, "tty_unregister_device" },
	{ 0xb0640cf, "tty_kref_put" },
	{ 0x8a696c33, "tty_vhangup" },
	{ 0x4d4fc137, "tty_port_tty_get" },
	{ 0x8fcd0aa3, "device_remove_file" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x510508ea, "usb_kill_urb" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x40727f4b, "tty_port_close" },
	{ 0x2b0ae722, "usb_autopm_get_interface_async" },
	{ 0xa4f86fcf, "tty_port_hangup" },
	{ 0xda4a8e61, "tty_port_tty_wakeup" },
	{ 0x633424a0, "gpiochip_remove" },
	{ 0x37a0cba, "kfree" },
	{ 0x86136269, "usb_put_intf" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x4118597, "tty_port_tty_hangup" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x20505320, "tty_port_put" },
	{ 0x54af84da, "usb_free_coherent" },
	{ 0xd17e3596, "usb_autopm_put_interface_async" },
	{ 0x1ab02dc2, "_dev_err" },
	{ 0xb20c3797, "usb_submit_urb" },
	{ 0xe3713a3e, "__dynamic_dev_dbg" },
	{ 0x3d7e65af, "usb_control_msg" },
	{ 0x69acdf38, "memcpy" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v04E2p1410d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1411d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1412d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1414d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1420d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1421d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1422d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1424d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1400d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1401d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1402d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1403d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v2890p0213d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "043F42E5357B9E124424CD1");
