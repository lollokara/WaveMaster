CC ?= gcc
CFLAGS = -std=c99 -Wall -Wextra -Werror -O2 -I../../../CompanionZ/main

all: run

test_cal_search: test_cal_search.c ../../../CompanionZ/main/zb_cal_search.c ../../../CompanionZ/main/zb_cal_search.h
	$(CC) $(CFLAGS) -o $@ test_cal_search.c ../../../CompanionZ/main/zb_cal_search.c -lm

run: test_cal_search
	./test_cal_search

clean:
	rm -f test_cal_search

.PHONY: all run clean
