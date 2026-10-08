CC ?= gcc
CFLAGS = -std=c99 -Wall -Wextra -Werror -O2 -I../../src

all: run

test_motion: test_motion.c ../../src/motion.c ../../src/motion.h
	$(CC) $(CFLAGS) -o $@ test_motion.c ../../src/motion.c -lm

run: test_motion
	./test_motion

clean:
	rm -f test_motion

.PHONY: all run clean
