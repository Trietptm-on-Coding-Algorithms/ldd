obj-m := globalmem.o   

KERNELBUILD := /lib/modules/`uname -r`/build  
default:  
	make -C $(KERNELBUILD) M=$(shell pwd) modules
clean:  
	make -C $(KERNELBUILD) M=$(shell pwd) clean
