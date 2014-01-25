all:
	gcc -o rtl_acars_ng rtl_acars_ng.c `pkg-config --cflags --libs librtlsdr libusb` -lpthread -lm -O2