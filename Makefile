obj-m    += m41st.o

all    :
		make -C /home/armbuild/arm/linux-2.6 M=$(PWD) modules

clean:
		make -C /home/armbuild/arm/linux-2.6 M=$(PWD) clean
