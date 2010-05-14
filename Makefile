obj-m    += m41st85w.o

all    :
		make -C /lib/modules/2.6.32/build M=$(PWD) modules

clean:
		make -C /lib/modules/2.6.32/build M=$(PWD) clean
