# This builds a linux kernel module for modifying the control module registers

# The name of the kernel module
obj-m += controlmodule.o

# Location of the kernel source
KERNELDIR := /lib/modules/$(shell uname -r)/build

# Current working directory
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
