default: hex2dfu

hex2dfu.o: hex2dfu.c
	gcc -c hex2dfu.c -o hex2dfu.o

hex2dfu: hex2dfu.o
	gcc hex2dfu.o -o hex2dfu

clean:
	-rm -f hex2dfu.o
	-rm -f hex2dfu