CC ?= gcc
CFLAGS ?= -std=gnu11 -Wall -Wextra -O1 -g -Wno-unused-function
SRC := ../../../src

# Compiles the real src/host_serial.c against a small FreeRTOS/IDF shim.
test: test_hostlink
	./test_hostlink

test_hostlink: test_hostlink.c $(SRC)/host_serial.c $(SRC)/host_serial.h $(wildcard shim/*.h shim/*/*.h)
	$(CC) $(CFLAGS) -Ishim -I$(SRC) -o $@ test_hostlink.c

clean:
	rm -rf test_hostlink test_hostlink.dSYM

.PHONY: test clean
