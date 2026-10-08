CC ?= gcc
CFLAGS ?= -std=gnu11 -Wall -Wextra -O1 -g
SRC := ../../../src

test: test_gcode
	./test_gcode

test_gcode: test_gcode.c $(SRC)/gcode_parse.c $(SRC)/gcode_parse.h
	$(CC) $(CFLAGS) -I$(SRC) -o $@ test_gcode.c $(SRC)/gcode_parse.c -lm

clean:
	rm -f test_gcode

.PHONY: test clean
