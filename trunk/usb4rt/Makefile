
ifndef RTAIDIR
RTAIDIR := /usr/realtime/fusion
endif

KDIR := $(shell $(RTAIDIR)/bin/rtai-config --linux-dir)
PWD  := $(shell pwd)

obj-y += core/
obj-y += host/
obj-y += prog/


modules:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) RTAIDIR=$(RTAIDIR) modules

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) RTAIDIR=$(RTAIDIR) clean
