CC ?= gcc
CFLAGS = -std=c99 -Wall -Wextra -Werror -O2 -I../../../CompanionZ/main
SRC = ../../../CompanionZ/main/zb_math.c ../../../CompanionZ/main/zb_setcore.c

all: run

test_zbro_motion: test_zbro_motion.c $(SRC) ../../../CompanionZ/main/zb_math.h ../../../CompanionZ/main/zb_setcore.h
	$(CC) $(CFLAGS) -o $@ test_zbro_motion.c $(SRC) -lm

run: test_zbro_motion
	./test_zbro_motion

clean:
	rm -f test_zbro_motion

.PHONY: all run clean
