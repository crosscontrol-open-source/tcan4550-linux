#
# Makefile for tcan4550 module
#

target = tcan4550

obj-m += $(target).o
$(target)-objs := tcan4550-main.o

SRC := $(shell pwd)

#
# Compile module -rule
#
module:
	KCPPFLAGS="-DDRV_VERSION=\"\"$(DRV_VERSION)\"\"" $(MAKE) V=2 -C ${KERNEL_DIR}  M=$(SRC) modules 

#
# Module install -rule
# 
modules_install:
	$(MAKE) -C $(KERNEL_DIR) M=$(SRC) modules_install

#
# Default - rule
#
all: version_info module

#
# Clean - rule
#
clean:
	rm -rf modules.order .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd *~ 