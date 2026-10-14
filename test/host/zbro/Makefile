CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O1 -g
SHARED := ../../../shared

test: test_zbro
	./test_zbro

test_zbro: test_zbro.c $(SHARED)/zbro_proto.c $(SHARED)/zbro_proto.h
	$(CC) $(CFLAGS) -I$(SHARED) -o $@ test_zbro.c $(SHARED)/zbro_proto.c -lm

clean:
	rm -rf test_zbro test_zbro.dSYM

.PHONY: test clean
