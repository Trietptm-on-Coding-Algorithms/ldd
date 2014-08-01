obj-m := globalfifo.o   

KERNELBUILD := /lib/modules/`uname -r`/build  
default:  
	make -C $(KERNELBUILD) M=$(shell pwd) modules
	gcc pollmonitor.c -o pm
	gcc read.c -o rd
	gcc write.c -o wt
clean:  
	make -C $(KERNELBUILD) M=$(shell pwd) clean
	rm pm rd wt
