KERN_DIR = /work/projects/linux-3.4.2_alsa

all:
	make -C $(KERN_DIR) M=`pwd` modules 

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order

obj-m	+= machine/
obj-m	+= platform/
obj-m	+= codec/
