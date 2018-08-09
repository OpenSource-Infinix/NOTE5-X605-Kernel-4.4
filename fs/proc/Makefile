#
# Makefile for the Linux proc filesystem routines.
#

obj-y   += proc.o

proc-y			:= nommu.o task_nommu.o
proc-$(CONFIG_MMU)	:= task_mmu.o

proc-y       += inode.o root.o base.o generic.o array.o \
		fd.o
proc-$(CONFIG_TTY)      += proc_tty.o
proc-y	+= cmdline.o
proc-y	+= consoles.o
proc-y	+= cpuinfo.o
proc-y	+= devices.o
proc-y	+= interrupts.o
proc-y	+= loadavg.o
proc-y	+= meminfo.o
proc-y	+= stat.o
proc-y	+= uptime.o
proc-y	+= version.o
proc-y	+= softirqs.o
proc-y	+= namespaces.o
proc-y	+= self.o
proc-y	+= thread_self.o
proc-$(CONFIG_PROC_SYSCTL)	+= proc_sysctl.o
proc-$(CONFIG_NET)		+= proc_net.o
proc-$(CONFIG_PROC_KCORE)	+= kcore.o
proc-$(CONFIG_PROC_VMCORE)	+= vmcore.o
proc-$(CONFIG_PRINTK)	+= kmsg.o
proc-$(CONFIG_PROC_PAGE_MONITOR)	+= page.o

#add XLLLGOBLES-6 by lei.ni for devinfo 20171120 start
ifneq ($(CONFIG_CUSTOM_TRAN_PROJECT),)
TRAN_KERNEL_FLAGS := $(addprefix -D, $(shell echo \
	             $(CONFIG_CUSTOM_TRAN_PROJECT) |  tr a-z A-Z))
subdir-ccflags-y += $(TRAN_KERNEL_FLAGS)
endif
ifneq ($(CONFIG_CUSTOM_TRAN_BOM),)
TRAN_BOM_KERNEL_FLAGS := $(addprefix -D, $(shell echo \
	             $(CONFIG_CUSTOM_TRAN_BOM) |  tr a-z A-Z))
subdir-ccflags-y += $(TRAN_BOM_KERNEL_FLAGS)
endif
#add XLLLGOBLES-6 by lei.ni for devinfo 20171120 end

