obj-m := mpu.o
KDIR := /home/louis/Documents/cross-compilation/linux-rpi-4.9.y/
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean


